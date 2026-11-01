#!/usr/bin/env python3
"""
PX4 Gazebo HITL Bridge

Bridges modern Gazebo (gz-sim) sensor data to PX4 running on real Pixhawk
hardware in HITL mode via MAVLink HIL_* messages.

Architecture:
  Gazebo (gz-sim)  <--gz-transport-->  HITLBridge  <--MAVLink/Serial-->  Pixhawk
       |                                    |                               |
   Sensors                            Conversion                     PX4 Flight
   (IMU, GPS,                         HIL_SENSOR                       Stack
   Baro, Mag,                         HIL_GPS
   Airspeed)                              |
       ^                                  v
   Actuators  <-----------------  HIL_ACTUATOR_CONTROLS
"""

from __future__ import annotations

import importlib
import time
import math
import threading
from dataclasses import dataclass, field
from typing import Optional, Callable, Dict, Any
from enum import IntFlag

# Check for required dependencies
try:
    from pymavlink import mavutil
    from pymavlink.dialects.v20 import common as mavlink
    PYMAVLINK_AVAILABLE = True
except ImportError:
    PYMAVLINK_AVAILABLE = False



def _first_module(*names):
    """The first of the modules that imports."""
    for name in names:
        try:
            return importlib.import_module(name)
        except ImportError:
            pass
    raise ImportError(f"none of {', '.join(names)} found")


GZ_IMPORT_ERROR = ""
try:
    # The bindings' modules are unversioned from Gazebo Jetty on, versioned
    # before (Harmonic: gz.transport13 / gz.msgs10, Ionic: 14 / 11)
    Node = _first_module("gz.transport", "gz.transport14", "gz.transport13").Node
    _msgs = _first_module("gz.msgs", "gz.msgs11", "gz.msgs10").__name__
    IMU = importlib.import_module(f"{_msgs}.imu_pb2").IMU
    Magnetometer = importlib.import_module(f"{_msgs}.magnetometer_pb2").Magnetometer
    FluidPressure = importlib.import_module(f"{_msgs}.fluid_pressure_pb2").FluidPressure
    NavSat = importlib.import_module(f"{_msgs}.navsat_pb2").NavSat
    AirSpeedSensor = importlib.import_module(f"{_msgs}.air_speed_sensor_pb2").AirSpeedSensor
    Actuators = importlib.import_module(f"{_msgs}.actuators_pb2").Actuators
    Double = importlib.import_module(f"{_msgs}.double_pb2").Double
    GZ_TRANSPORT_AVAILABLE = True
except ImportError as e:
    GZ_TRANSPORT_AVAILABLE = False
    GZ_IMPORT_ERROR = str(e)

GZ_INSTALL_HINT = ("sudo apt install python3-gz-transport13 (Gazebo Harmonic), "
                   "python3-gz-transport14 (Ionic) or python3-gz-transport15 (Jetty)")


def missing_dependencies() -> list:
    """What the bridge can't run without, as messages; empty when all is there."""
    missing = []
    if not PYMAVLINK_AVAILABLE:
        missing.append("pymavlink: pip install pymavlink")
    if not GZ_TRANSPORT_AVAILABLE:
        missing.append(f"Gazebo's Python bindings ({GZ_IMPORT_ERROR}): {GZ_INSTALL_HINT}")
    return missing


class HILSensorFields(IntFlag):
    """Bitmask for HIL_SENSOR fields_updated field.

    Each axis has its own bit per MAVLink spec. PX4 checks for all
    axes of each sensor type to be set together.

    MAVLink bit positions:
      0-2: xacc, yacc, zacc
      3-5: xgyro, ygyro, zgyro
      6-8: xmag, ymag, zmag
      9: abs_pressure
      10: diff_pressure
      11: pressure_alt
      12: temperature
      31: full reset
    """
    # Individual axis bits
    XACC = 1 << 0
    YACC = 1 << 1
    ZACC = 1 << 2
    XGYRO = 1 << 3
    YGYRO = 1 << 4
    ZGYRO = 1 << 5
    XMAG = 1 << 6
    YMAG = 1 << 7
    ZMAG = 1 << 8
    ABS_PRESSURE = 1 << 9
    DIFF_PRESSURE = 1 << 10
    PRESSURE_ALT = 1 << 11
    TEMPERATURE = 1 << 12
    FULL_RESET = 1 << 31

    # Convenience combinations (matching PX4 SensorSource enum)
    ACC = XACC | YACC | ZACC           # bits 0,1,2 = 0b111 = 7
    GYRO = XGYRO | YGYRO | ZGYRO       # bits 3,4,5 = 0b111000 = 56
    MAG = XMAG | YMAG | ZMAG           # bits 6,7,8 = 0b111000000 = 448
    BARO = ABS_PRESSURE | PRESSURE_ALT | TEMPERATURE  # bits 9,11,12 = 6656


@dataclass
class SensorState:
    """Current state of all sensors."""
    # IMU
    xacc: float = 0.0  # m/s^2
    yacc: float = 0.0
    zacc: float = 0.0
    xgyro: float = 0.0  # rad/s
    ygyro: float = 0.0
    zgyro: float = 0.0

    # Magnetometer (Gauss)
    xmag: float = 0.0
    ymag: float = 0.0
    zmag: float = 0.0

    # Barometer
    abs_pressure: float = 1013.25  # mbar (sea level default)
    temperature: float = 20.0  # Celsius

    # Airspeed
    diff_pressure: float = 0.0  # mbar

    # Timestamps for freshness check
    imu_timestamp: float = 0.0
    mag_timestamp: float = 0.0
    baro_timestamp: float = 0.0
    airspeed_timestamp: float = 0.0

    # Update flags
    fields_updated: int = 0


@dataclass
class GPSState:
    """Current GPS state."""
    fix_type: int = 0  # 0=no fix, 3=3D fix
    lat: int = 0  # 1e7 degrees
    lon: int = 0  # 1e7 degrees
    alt: int = 0  # mm above MSL
    eph: int = 65535  # cm (65535 = unknown)
    epv: int = 65535  # cm
    vel: int = 0  # cm/s ground speed
    vn: int = 0  # cm/s north
    ve: int = 0  # cm/s east
    vd: int = 0  # cm/s down
    cog: int = 0  # 1e2 degrees course over ground
    satellites_visible: int = 0
    timestamp: float = 0.0


@dataclass
class HITLConfig:
    """HITL Bridge configuration."""
    # MAVLink connection - can be serial or UDP
    # Serial: "/dev/ttyACM0" or "/dev/ttyUSB0"
    # UDP: "udp:127.0.0.1:14551" or "udpin:0.0.0.0:14551"
    connection: str = "/dev/ttyACM0"
    baudrate: int = 921600  # Only used for serial connections

    # Gazebo world and model
    world_name: str = "default"
    model_name: str = "standard_vtol"

    # Sensor update rates (Hz)
    hil_sensor_rate: float = 250.0  # HIL_SENSOR rate
    hil_gps_rate: float = 10.0  # HIL_GPS rate

    # MAVLink settings
    # Use system_id=255 (GCS) to avoid being filtered as loopback by Pixhawk
    system_id: int = 255
    component_id: int = 190  # MAV_COMP_ID_MISSIONPLANNER
    target_system: int = 1
    target_component: int = 1

    # Timeouts
    heartbeat_interval: float = 1.0
    connection_timeout: float = 5.0

    # Actuator configuration
    num_motors: int = 5  # Number of motors (standard_vtol has 5)
    num_servos: int = 4  # Number of servos (aileron, elevator, rudder, etc.)
    motor_max_rpm: float = 1000.0  # Maximum motor RPM for simulation
    servo_angle_max: float = 0.5236  # 30 degrees in radians
    servo_angle_min: float = -0.5236


class HITLBridge:
    """
    Bridge between Gazebo simulation and PX4 on real Pixhawk hardware.

    Subscribes to Gazebo sensor topics, converts to MAVLink HIL messages,
    and sends to Pixhawk. Receives actuator commands from Pixhawk and
    publishes to Gazebo.
    """

    def __init__(self, config: Optional[HITLConfig] = None):
        if not PYMAVLINK_AVAILABLE:
            raise ImportError("pymavlink is required: pip install pymavlink")
        if not GZ_TRANSPORT_AVAILABLE:
            raise ImportError(f"gz-transport Python bindings required: {GZ_INSTALL_HINT}")

        self.config = config or HITLConfig()
        self._running = False
        self._connected = False

        # Sensor state
        self._sensor_state = SensorState()
        self._gps_state = GPSState()
        self._state_lock = threading.Lock()

        # MAVLink connection
        self._mav: Optional[mavutil.mavlink_connection] = None

        # Gazebo node
        self._gz_node: Optional[Node] = None

        # Gazebo publishers for actuators
        self._motor_pub = None  # Publisher for motor speeds
        self._servo_pubs = []  # Publishers for servos

        # Threads
        self._sensor_thread: Optional[threading.Thread] = None
        self._gps_thread: Optional[threading.Thread] = None
        self._receive_thread: Optional[threading.Thread] = None
        self._heartbeat_thread: Optional[threading.Thread] = None

        # Callbacks
        self._actuator_callback: Optional[Callable[[Dict[str, Any]], None]] = None
        self._status_callback: Optional[Callable[[str], None]] = None

        # Statistics
        self._stats = {
            'hil_sensor_sent': 0,
            'hil_gps_sent': 0,
            'actuator_received': 0,
            'heartbeats_sent': 0,
            'errors': 0
        }

    def connect(self) -> bool:
        """Connect to Pixhawk and Gazebo."""
        try:
            conn = self.config.connection

            # Determine connection type
            if conn.startswith('udp:') or conn.startswith('udpin:') or conn.startswith('udpout:'):
                self._log(f"Connecting to Pixhawk via UDP: {conn}")
                self._mav = mavutil.mavlink_connection(
                    conn,
                    source_system=self.config.system_id,
                    source_component=self.config.component_id
                )
            elif conn.startswith('tcp:'):
                self._log(f"Connecting to Pixhawk via TCP: {conn}")
                self._mav = mavutil.mavlink_connection(
                    conn,
                    source_system=self.config.system_id,
                    source_component=self.config.component_id
                )
            else:
                # Assume serial port
                self._log(f"Connecting to Pixhawk on {conn} at {self.config.baudrate} baud...")
                self._mav = mavutil.mavlink_connection(
                    conn,
                    baud=self.config.baudrate,
                    source_system=self.config.system_id,
                    source_component=self.config.component_id
                )

            # Wait for heartbeat from Pixhawk
            self._log("Waiting for Pixhawk heartbeat...")
            msg = self._mav.wait_heartbeat(timeout=self.config.connection_timeout)
            if msg is None:
                self._log("ERROR: No heartbeat from Pixhawk")
                return False

            self._log(f"Connected to Pixhawk (system {msg.get_srcSystem()}, "
                     f"component {msg.get_srcComponent()})")
            self.config.target_system = msg.get_srcSystem()
            self.config.target_component = msg.get_srcComponent()

            # Initialize Gazebo node
            self._log("Initializing Gazebo transport...")
            self._gz_node = Node()
            self._setup_gz_subscriptions()

            self._connected = True
            self._log("HITL Bridge connected successfully")
            return True

        except Exception as e:
            self._log(f"ERROR: Connection failed: {e}")
            self._stats['errors'] += 1
            return False

    def _setup_gz_subscriptions(self):
        """Subscribe to Gazebo sensor topics and setup actuator publishers."""
        world = self.config.world_name
        model = self.config.model_name

        # Note: gz-transport Python subscribe signature is (msg_type, topic, callback)

        # IMU topic
        imu_topic = f"/world/{world}/model/{model}/link/base_link/sensor/imu_sensor/imu"
        self._gz_node.subscribe(IMU, imu_topic, self._on_imu)
        self._log(f"Subscribed to IMU: {imu_topic}")

        # Magnetometer topic
        mag_topic = f"/world/{world}/model/{model}/link/base_link/sensor/magnetometer_sensor/magnetometer"
        self._gz_node.subscribe(Magnetometer, mag_topic, self._on_magnetometer)
        self._log(f"Subscribed to Magnetometer: {mag_topic}")

        # Barometer topic
        baro_topic = f"/world/{world}/model/{model}/link/base_link/sensor/air_pressure_sensor/air_pressure"
        self._gz_node.subscribe(FluidPressure, baro_topic, self._on_barometer)
        self._log(f"Subscribed to Barometer: {baro_topic}")

        # GPS topic
        gps_topic = f"/world/{world}/model/{model}/link/base_link/sensor/navsat_sensor/navsat"
        self._gz_node.subscribe(NavSat, gps_topic, self._on_gps)
        self._log(f"Subscribed to GPS: {gps_topic}")

        # Airspeed topic
        airspeed_topic = f"/world/{world}/model/{model}/link/airspeed_link/sensor/air_speed/air_speed"
        self._gz_node.subscribe(AirSpeedSensor, airspeed_topic, self._on_airspeed)
        self._log(f"Subscribed to Airspeed: {airspeed_topic}")

        # Setup actuator publishers
        self._setup_actuator_publishers(model)

    def _setup_actuator_publishers(self, model: str):
        """Setup publishers for motors and servos."""
        # Motor speed publisher
        # Topic: /{model}/command/motor_speed
        motor_topic = f"/{model}/command/motor_speed"
        self._motor_pub = self._gz_node.advertise(motor_topic, Actuators)
        self._log(f"Publishing motors to: {motor_topic}")

        # Servo publishers (one per servo)
        # Topic: /model/{model}/servo_{i}
        self._servo_pubs = []
        for i in range(self.config.num_servos):
            servo_topic = f"/model/{model}/servo_{i}"
            pub = self._gz_node.advertise(servo_topic, Double)
            self._servo_pubs.append(pub)
            self._log(f"Publishing servo {i} to: {servo_topic}")

    def _on_imu(self, msg: IMU):
        """Handle IMU message from Gazebo."""
        with self._state_lock:
            # Transform from Gazebo FLU (Forward-Left-Up) to PX4 FRD (Forward-Right-Down)
            # FLU->FRD: x' = x, y' = -y, z' = -z

            # Linear acceleration (m/s^2)
            self._sensor_state.xacc = msg.linear_acceleration.x
            self._sensor_state.yacc = -msg.linear_acceleration.y
            self._sensor_state.zacc = -msg.linear_acceleration.z

            # Angular velocity (rad/s)
            self._sensor_state.xgyro = msg.angular_velocity.x
            self._sensor_state.ygyro = -msg.angular_velocity.y
            self._sensor_state.zgyro = -msg.angular_velocity.z

            self._sensor_state.imu_timestamp = time.time()
            self._sensor_state.fields_updated |= (HILSensorFields.ACC | HILSensorFields.GYRO)

    def _on_magnetometer(self, msg: Magnetometer):
        """Handle magnetometer message from Gazebo."""
        with self._state_lock:
            # Transform from Gazebo FLU to PX4 FRD
            # Note: Despite the protobuf field being named 'field_tesla',
            # Gazebo Harmonic actually outputs values in Gauss due to a bug.
            # See: https://github.com/gazebosim/gz-sim/pull/2460
            # HIL_SENSOR expects Gauss, so use values directly without conversion.
            self._sensor_state.xmag = msg.field_tesla.x
            self._sensor_state.ymag = -msg.field_tesla.y
            self._sensor_state.zmag = -msg.field_tesla.z

            self._sensor_state.mag_timestamp = time.time()
            self._sensor_state.fields_updated |= HILSensorFields.MAG

    def _on_barometer(self, msg: FluidPressure):
        """Handle barometer message from Gazebo."""
        with self._state_lock:
            # Convert Pa to mbar (1 Pa = 0.01 mbar)
            self._sensor_state.abs_pressure = msg.pressure * 0.01

            # Estimate temperature if available (default 20C)
            # Gazebo FluidPressure doesn't include temperature

            self._sensor_state.baro_timestamp = time.time()
            self._sensor_state.fields_updated |= HILSensorFields.BARO

    def _on_gps(self, msg: NavSat):
        """Handle GPS message from Gazebo."""
        with self._state_lock:
            self._gps_state.fix_type = 3  # 3D fix
            self._gps_state.lat = int(msg.latitude_deg * 1e7)
            self._gps_state.lon = int(msg.longitude_deg * 1e7)
            self._gps_state.alt = int(msg.altitude * 1000)  # m to mm

            # Velocity from NavSat (if available)
            if hasattr(msg, 'velocity_east') and hasattr(msg, 'velocity_north'):
                self._gps_state.ve = int(msg.velocity_east * 100)  # m/s to cm/s
                self._gps_state.vn = int(msg.velocity_north * 100)
                self._gps_state.vd = int(msg.velocity_up * -100)  # up to down

                # Ground speed
                self._gps_state.vel = int(math.sqrt(
                    msg.velocity_east**2 + msg.velocity_north**2
                ) * 100)

                # Course over ground
                if self._gps_state.vel > 50:  # > 0.5 m/s
                    cog = math.atan2(msg.velocity_east, msg.velocity_north)
                    self._gps_state.cog = int(math.degrees(cog) * 100) % 36000

            # Accuracy (simulate reasonable GPS accuracy)
            self._gps_state.eph = 100  # 1m horizontal
            self._gps_state.epv = 150  # 1.5m vertical
            self._gps_state.satellites_visible = 12

            self._gps_state.timestamp = time.time()

    def _on_airspeed(self, msg: AirSpeedSensor):
        """Handle airspeed message from Gazebo."""
        with self._state_lock:
            # Convert differential pressure to mbar
            # AirSpeedSensor provides diff_pressure in Pa
            if hasattr(msg, 'diff_pressure'):
                self._sensor_state.diff_pressure = msg.diff_pressure * 0.01
            else:
                # Calculate from airspeed using Bernoulli equation
                # diff_pressure = 0.5 * rho * v^2
                # At sea level, rho ≈ 1.225 kg/m^3
                rho = 1.225
                v = msg.speed if hasattr(msg, 'speed') else 0.0
                self._sensor_state.diff_pressure = 0.5 * rho * v * v * 0.01  # Pa to mbar

            self._sensor_state.airspeed_timestamp = time.time()
            self._sensor_state.fields_updated |= HILSensorFields.DIFF_PRESSURE

    def start(self):
        """Start the HITL bridge."""
        if not self._connected:
            raise RuntimeError("Not connected. Call connect() first.")

        self._running = True

        # Start HIL_SENSOR send thread
        self._sensor_thread = threading.Thread(
            target=self._sensor_send_loop,
            name="HIL_SENSOR",
            daemon=True
        )
        self._sensor_thread.start()

        # Start HIL_GPS send thread
        self._gps_thread = threading.Thread(
            target=self._gps_send_loop,
            name="HIL_GPS",
            daemon=True
        )
        self._gps_thread.start()

        # Start receive thread for actuator commands
        self._receive_thread = threading.Thread(
            target=self._receive_loop,
            name="MAVLink_Receive",
            daemon=True
        )
        self._receive_thread.start()

        # Start heartbeat thread
        self._heartbeat_thread = threading.Thread(
            target=self._heartbeat_loop,
            name="Heartbeat",
            daemon=True
        )
        self._heartbeat_thread.start()

        self._log("HITL Bridge started")

    def stop(self):
        """Stop the HITL bridge."""
        self._running = False

        # Wait for threads to finish
        for thread in [self._sensor_thread, self._gps_thread,
                       self._receive_thread, self._heartbeat_thread]:
            if thread and thread.is_alive():
                thread.join(timeout=1.0)

        # Close MAVLink connection
        if self._mav:
            self._mav.close()
            self._mav = None

        self._connected = False
        self._log("HITL Bridge stopped")

    def _sensor_send_loop(self):
        """Send HIL_SENSOR messages at configured rate."""
        interval = 1.0 / self.config.hil_sensor_rate

        while self._running:
            try:
                with self._state_lock:
                    # Build fields_updated based on which sensors have valid data
                    # (have been updated at least once)
                    fields = 0
                    if self._sensor_state.imu_timestamp > 0:
                        fields |= HILSensorFields.ACC | HILSensorFields.GYRO
                    if self._sensor_state.mag_timestamp > 0:
                        fields |= HILSensorFields.MAG
                    if self._sensor_state.baro_timestamp > 0:
                        fields |= HILSensorFields.BARO
                    if self._sensor_state.airspeed_timestamp > 0:
                        fields |= HILSensorFields.DIFF_PRESSURE

                    state = SensorState(
                        xacc=self._sensor_state.xacc,
                        yacc=self._sensor_state.yacc,
                        zacc=self._sensor_state.zacc,
                        xgyro=self._sensor_state.xgyro,
                        ygyro=self._sensor_state.ygyro,
                        zgyro=self._sensor_state.zgyro,
                        xmag=self._sensor_state.xmag,
                        ymag=self._sensor_state.ymag,
                        zmag=self._sensor_state.zmag,
                        abs_pressure=self._sensor_state.abs_pressure,
                        diff_pressure=self._sensor_state.diff_pressure,
                        temperature=self._sensor_state.temperature,
                        fields_updated=fields
                    )

                # Calculate pressure altitude from barometric formula
                # h = 44330 * (1 - (P/P0)^0.1903)
                pressure_alt = 44330.0 * (1.0 - pow(
                    state.abs_pressure / 1013.25, 0.1903
                ))

                # Send HIL_SENSOR
                self._mav.mav.hil_sensor_send(
                    int(time.time() * 1e6),  # time_usec
                    state.xacc,
                    state.yacc,
                    state.zacc,
                    state.xgyro,
                    state.ygyro,
                    state.zgyro,
                    state.xmag,
                    state.ymag,
                    state.zmag,
                    state.abs_pressure,
                    state.diff_pressure,
                    pressure_alt,
                    state.temperature,
                    state.fields_updated,
                    0  # id (sensor instance)
                )

                self._stats['hil_sensor_sent'] += 1

            except Exception as e:
                self._log(f"ERROR sending HIL_SENSOR: {e}")
                self._stats['errors'] += 1

            time.sleep(interval)

    def _gps_send_loop(self):
        """Send HIL_GPS messages at configured rate."""
        interval = 1.0 / self.config.hil_gps_rate

        while self._running:
            try:
                with self._state_lock:
                    gps = GPSState(
                        fix_type=self._gps_state.fix_type,
                        lat=self._gps_state.lat,
                        lon=self._gps_state.lon,
                        alt=self._gps_state.alt,
                        eph=self._gps_state.eph,
                        epv=self._gps_state.epv,
                        vel=self._gps_state.vel,
                        vn=self._gps_state.vn,
                        ve=self._gps_state.ve,
                        vd=self._gps_state.vd,
                        cog=self._gps_state.cog,
                        satellites_visible=self._gps_state.satellites_visible
                    )

                # Send HIL_GPS
                # Note: Some pymavlink versions don't have id/yaw fields
                self._mav.mav.hil_gps_send(
                    int(time.time() * 1e6),  # time_usec
                    gps.fix_type,
                    gps.lat,
                    gps.lon,
                    gps.alt,
                    gps.eph,
                    gps.epv,
                    gps.vel,
                    gps.vn,
                    gps.ve,
                    gps.vd,
                    gps.cog,
                    gps.satellites_visible
                )

                self._stats['hil_gps_sent'] += 1

            except Exception as e:
                self._log(f"ERROR sending HIL_GPS: {e}")
                self._stats['errors'] += 1

            time.sleep(interval)

    def _receive_loop(self):
        """Receive MAVLink messages from Pixhawk."""
        while self._running:
            try:
                msg = self._mav.recv_match(blocking=True, timeout=0.1)
                if msg is None:
                    continue

                msg_type = msg.get_type()

                if msg_type == 'HIL_ACTUATOR_CONTROLS':
                    self._handle_actuator_controls(msg)
                elif msg_type == 'HEARTBEAT':
                    pass  # Normal heartbeat
                elif msg_type == 'STATUSTEXT':
                    self._log(f"PX4: {msg.text}")

            except Exception as e:
                if self._running:  # Don't log if we're stopping
                    self._log(f"ERROR in receive loop: {e}")
                    self._stats['errors'] += 1

    def _handle_actuator_controls(self, msg):
        """Handle HIL_ACTUATOR_CONTROLS from Pixhawk."""
        self._stats['actuator_received'] += 1

        # Extract actuator values (normalized -1 to 1 or 0 to 1)
        controls = {
            'time_usec': msg.time_usec,
            'mode': msg.mode,
            'flags': msg.flags,
            'controls': list(msg.controls)  # 16 channels
        }

        # Call user callback if registered
        if self._actuator_callback:
            try:
                self._actuator_callback(controls)
            except Exception as e:
                self._log(f"ERROR in actuator callback: {e}")

        # Publish actuator commands to Gazebo
        self._publish_actuators_to_gazebo(msg.controls)

    def _publish_actuators_to_gazebo(self, controls):
        """Publish actuator commands to Gazebo topics.

        HIL_ACTUATOR_CONTROLS layout for standard_vtol:
        - controls[0-4]: Motors (normalized 0-1, where 0=off, 1=max)
        - controls[5-8]: Servos (normalized -1 to 1)

        This matches the PX4 mixer output ordering.
        """
        try:
            # Publish motor speeds
            if self._motor_pub:
                motor_msg = Actuators()

                # Convert normalized controls to velocity (rad/s or RPM)
                # Motors are typically in controls[0:num_motors]
                for i in range(self.config.num_motors):
                    # Normalize 0-1 to 0-max_rpm (convert to rad/s for Gazebo)
                    value = max(0.0, min(1.0, controls[i]))
                    # Convert to angular velocity (rad/s) assuming max_rpm
                    velocity = value * self.config.motor_max_rpm * (2 * math.pi / 60)
                    motor_msg.velocity.append(velocity)

                self._motor_pub.publish(motor_msg)

            # Publish servo positions
            # Servos are in controls[num_motors:num_motors+num_servos]
            for i, servo_pub in enumerate(self._servo_pubs):
                if servo_pub:
                    servo_idx = self.config.num_motors + i
                    if servo_idx < len(controls):
                        # Convert -1 to 1 range to servo angle range
                        value = max(-1.0, min(1.0, controls[servo_idx]))
                        # Linear interpolation between min and max angle
                        angle = (value + 1.0) / 2.0 * (
                            self.config.servo_angle_max - self.config.servo_angle_min
                        ) + self.config.servo_angle_min

                        servo_msg = Double()
                        servo_msg.data = angle
                        servo_pub.publish(servo_msg)

        except Exception as e:
            self._log(f"ERROR publishing actuators to Gazebo: {e}")
            self._stats['errors'] += 1

    def _heartbeat_loop(self):
        """Send heartbeats to Pixhawk."""
        while self._running:
            try:
                self._mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                    0, 0, 0
                )
                self._stats['heartbeats_sent'] += 1
            except Exception as e:
                self._log(f"ERROR sending heartbeat: {e}")
                self._stats['errors'] += 1

            time.sleep(self.config.heartbeat_interval)

    def set_actuator_callback(self, callback: Callable[[Dict[str, Any]], None]):
        """Set callback for actuator control messages."""
        self._actuator_callback = callback

    def set_status_callback(self, callback: Callable[[str], None]):
        """Set callback for status messages."""
        self._status_callback = callback

    def get_stats(self) -> Dict[str, int]:
        """Get bridge statistics."""
        return dict(self._stats)

    def _log(self, message: str):
        """Log a message."""
        timestamp = time.strftime("%H:%M:%S")
        log_msg = f"[{timestamp}] {message}"
        print(log_msg)
        if self._status_callback:
            self._status_callback(log_msg)


def main():
    """Command-line interface for HITL bridge."""
    import argparse

    parser = argparse.ArgumentParser(
        description="PX4 Gazebo HITL Bridge - Connect gz-sim to real Pixhawk",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Connection examples:
  Serial (direct):     --connection /dev/ttyACM0 --baud 921600
  UDP (via router):    --connection udp:127.0.0.1:14551
  UDP (listen):        --connection udpin:0.0.0.0:14551

Typical setup with mavlink-router:
  1. Start router:  mavlink-routerd -e 127.0.0.1:14550 -e 127.0.0.1:14551 /dev/ttyACM0:921600
  2. QGC connects to UDP 14550
  3. HITL bridge:   python3 hitl_bridge.py --connection udp:127.0.0.1:14551
"""
    )
    parser.add_argument(
        "--connection", "-c", type=str, default="/dev/ttyACM0",
        help="MAVLink connection string. Serial: /dev/ttyACM0, UDP: udp:127.0.0.1:14551 (default: /dev/ttyACM0)"
    )
    parser.add_argument(
        "--baud", type=int, default=921600,
        help="Baud rate for serial connections (default: 921600)"
    )
    parser.add_argument(
        "--world", type=str, default="default",
        help="Gazebo world name (default: default)"
    )
    parser.add_argument(
        "--model", type=str, default="standard_vtol",
        help="Model name in Gazebo (default: standard_vtol)"
    )
    parser.add_argument(
        "--sensor-rate", type=float, default=250.0,
        help="HIL_SENSOR rate in Hz (default: 250)"
    )
    parser.add_argument(
        "--gps-rate", type=float, default=10.0,
        help="HIL_GPS rate in Hz (default: 10)"
    )
    parser.add_argument(
        "--num-motors", type=int, default=5,
        help="Number of motors (default: 5 for standard_vtol)"
    )
    parser.add_argument(
        "--num-servos", type=int, default=4,
        help="Number of servos (default: 4)"
    )
    parser.add_argument(
        "--motor-rpm", type=float, default=1000.0,
        help="Maximum motor RPM (default: 1000)"
    )
    parser.add_argument(
        "--component-id", type=int, default=190,
        help="MAVLink component ID of the bridge (default: 190). Through a MAVLink router, pick one QGC doesn't use "
             "(QGC is 255/190), e.g. 25"
    )
    parser.add_argument(
        "--check", action="store_true",
        help="Only check that the dependencies are installed; exit status 1 if not"
    )

    args = parser.parse_args()

    if args.check:
        missing = missing_dependencies()
        for message in missing:
            print(message)
        return 1 if missing else 0

    # Create config
    config = HITLConfig(
        connection=args.connection,
        baudrate=args.baud,
        world_name=args.world,
        model_name=args.model,
        hil_sensor_rate=args.sensor_rate,
        hil_gps_rate=args.gps_rate,
        num_motors=args.num_motors,
        num_servos=args.num_servos,
        motor_max_rpm=args.motor_rpm,
        component_id=args.component_id,
    )

    # Create and start bridge
    bridge = HITLBridge(config)

    def on_actuators(controls):
        print(f"Actuators: {controls['controls'][:8]}")

    bridge.set_actuator_callback(on_actuators)

    if not bridge.connect():
        print("Failed to connect")
        return 1

    bridge.start()

    print("\nHITL Bridge running. Press Ctrl+C to stop.\n")

    try:
        while True:
            time.sleep(5)
            stats = bridge.get_stats()
            print(f"Stats: HIL_SENSOR={stats['hil_sensor_sent']}, "
                  f"HIL_GPS={stats['hil_gps_sent']}, "
                  f"Actuators={stats['actuator_received']}, "
                  f"Errors={stats['errors']}")
    except KeyboardInterrupt:
        print("\nStopping...")

    bridge.stop()
    return 0


if __name__ == "__main__":
    import sys
    sys.exit(main())
