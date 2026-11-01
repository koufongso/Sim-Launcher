"""Set up a Pixhawk for HITL over its USB port.

1. Select the vehicle's HIL airframe (SYS_AUTOSTART) and reboot. The airframe
   enables SYS_HITL and maps the actuators; PX4 resets most parameters when the
   airframe changes, so nothing else is set before this reboot.
2. Set what the gz HITL bridge also needs but the airframe leaves out, then
   reboot again. Parameters the airframe sets itself are never touched.
"""

import os
import struct
import time
from typing import Callable, List, Optional, Tuple

from .px4 import HilAirframe

# Needed for HITL with the gz bridge, in groups that belong together: a group
# is only set if the airframe sets none of it (e.g. an airframe that sets up
# its airspeed sensor keeps its airspeed checks)
EXTRA_PARAMS: List[List[Tuple[str, float]]] = [
    # Simulated sensor IDs (what PX4's mavlink_receiver reports for HIL_SENSOR)
    [("CAL_ACC0_ID", 1310988), ("CAL_GYRO0_ID", 1310988), ("CAL_MAG0_ID", 197388), ("CAL_BARO0_ID", 6620172)],
    # Simulated sensors have no offsets
    [("CAL_ACC0_XOFF", 0.0), ("CAL_ACC0_YOFF", 0.0), ("CAL_ACC0_ZOFF", 0.0)],
    [("CAL_GYRO0_XOFF", 0.0), ("CAL_GYRO0_YOFF", 0.0), ("CAL_GYRO0_ZOFF", 0.0)],
    [("CAL_MAG0_XOFF", 0.0), ("CAL_MAG0_YOFF", 0.0), ("CAL_MAG0_ZOFF", 0.0)],
    # GPS comes from HIL_GPS, not a driver
    [("SENS_EN_GPSSIM", 0), ("GPS_1_CONFIG", 0)],
    [("UAVCAN_ENABLE", 0)],
    # No battery, airspeed sensor, safety switch or ESC telemetry on the bench
    [("CBRK_SUPPLY_CHK", 894281)],
    [("CBRK_AIRSPD_CHK", 162128), ("SYS_HAS_NUM_ASPD", 0), ("ASPD_DO_CHECKS", 0)],
    [("CBRK_IO_SAFETY", 22027)],
    [("COM_ARM_CHK_ESCS", 0)],
    # Unlimited rate on the USB link the bridge uses
    [("MAV_0_RATE", 0)],
    [("COM_RC_IN_MODE", 1)],
    [("EKF2_MAG_TYPE", 1)],
    [("EKF2_HGT_REF", 0)],
]

MAV_PARAM_TYPE_REAL32 = 9
REBOOT_TIMEOUT_S = 40.0


class SetupError(Exception):
    pass


def extra_params(airframe: HilAirframe) -> List[Tuple[str, float]]:
    return [param for group in EXTRA_PARAMS if not any(name in airframe.params for name, _ in group)
            for param in group]


class PixhawkSetup:
    def __init__(self, device: str, baud: int, airframe: HilAirframe, log: Callable[[str], None]):
        self._device = device
        self._baud = baud
        self._airframe = airframe
        self._log = log
        self._conn = None

    def run(self):
        """Raises SetupError with a message for the user if a step fails."""
        try:
            self._connect()
            target = self._airframe.autostart
            current = self._read("SYS_AUTOSTART")
            if current is None:
                raise SetupError("Could not read SYS_AUTOSTART from the Pixhawk.")
            if int(current[0]) != target:
                self._log(f"Selecting airframe {target} ({self._airframe.model} HIL), was {int(current[0])}")
                self._set("SYS_AUTOSTART", target)
                self._reboot_and_reconnect()
                if int(self._read("SYS_AUTOSTART")[0]) != target:
                    raise SetupError(f"The Pixhawk did not keep SYS_AUTOSTART = {target}.")
            hitl = self._read("SYS_HITL")
            if hitl is None or int(hitl[0]) != 1:
                raise SetupError(
                    f"Airframe {target} did not enable SYS_HITL. The Pixhawk firmware probably doesn't "
                    f"include {os.path.basename(self._airframe.path)}; flash firmware built from the "
                    "same PX4 tree.")

            skipped = []
            for name, value in extra_params(self._airframe):
                if self._read(name) is None:
                    skipped.append(name)  # not in this firmware
                    continue
                self._set(name, value)
            if skipped:
                self._log(f"Not in this firmware, skipped: {', '.join(skipped)}")
            self._reboot()
            self._log("Pixhawk is set up for HITL and rebooting.")
        finally:
            self._close()

    # --- MAVLink --------------------------------------------------------------

    def _connect(self, timeout: float = 10.0):
        from pymavlink import mavutil

        self._log(f"Connecting to {self._device}")
        self._conn = mavutil.mavlink_connection(self._device, baud=self._baud, source_system=255)
        if not self._conn.wait_heartbeat(timeout=timeout):
            raise SetupError(f"No heartbeat from the Pixhawk on {self._device}.")
        self._log(f"Connected to system {self._conn.target_system}")

    def _close(self):
        if self._conn is not None:
            try:
                self._conn.close()
            except OSError:
                pass
            self._conn = None

    def _wait_param(self, name: str, timeout: float = 2.0):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            msg = self._conn.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if msg is not None and msg.param_id.rstrip("\x00") == name:
                return msg
        return None

    def _read(self, name: str, tries: int = 3) -> Optional[Tuple[float, int]]:
        """(value, MAV_PARAM_TYPE) of a parameter; None if the firmware doesn't have it."""
        for _ in range(tries):
            self._conn.mav.param_request_read_send(
                self._conn.target_system, self._conn.target_component, name.encode(), -1)
            msg = self._wait_param(name)
            if msg is not None:
                return self._decode(msg.param_value, msg.param_type), msg.param_type
        return None

    def _set(self, name: str, value: float, tries: int = 3):
        param_type = self._read(name)[1]
        if param_type == MAV_PARAM_TYPE_REAL32:
            wire = float(value)
        else:
            # PX4 sends integer parameters bit-for-bit in the float field
            wire = struct.unpack("<f", struct.pack("<i", int(value)))[0]
        for _ in range(tries):
            self._conn.mav.param_set_send(
                self._conn.target_system, self._conn.target_component, name.encode(), wire, param_type)
            msg = self._wait_param(name)
            if msg is not None and abs(self._decode(msg.param_value, msg.param_type) - value) < 1e-3:
                self._log(f"  {name} = {value:g}")
                return
        raise SetupError(f"The Pixhawk did not accept {name} = {value:g}.")

    @staticmethod
    def _decode(raw: float, param_type: int) -> float:
        if param_type == MAV_PARAM_TYPE_REAL32:
            return raw
        return struct.unpack("<i", struct.pack("<f", raw))[0]

    def _reboot(self):
        from pymavlink import mavutil

        self._conn.mav.command_long_send(
            self._conn.target_system, self._conn.target_component,
            mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)

    def _reboot_and_reconnect(self):
        self._log("Rebooting the Pixhawk")
        self._reboot()
        self._close()
        # The USB port goes away while it reboots
        deadline = time.monotonic() + 5
        while os.path.exists(self._device) and time.monotonic() < deadline:
            time.sleep(0.2)
        deadline = time.monotonic() + REBOOT_TIMEOUT_S
        while not os.path.exists(self._device):
            if time.monotonic() > deadline:
                raise SetupError(f"The Pixhawk did not come back on {self._device} after rebooting.")
            time.sleep(0.5)
        time.sleep(2)  # udev permissions, and PX4 starting its USB link
        self._connect(timeout=20.0)
