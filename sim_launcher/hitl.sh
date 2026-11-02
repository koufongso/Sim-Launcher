#!/bin/sh
# Runs Gazebo with the vehicle for HITL and keeps the HITL bridge between
# Gazebo and the Pixhawk running: the bridge exits when the Pixhawk reboots or
# is unplugged, so it waits for the device and starts the bridge again.
#
# <device> is the Pixhawk USB, or a MAVLink connection string such as
# udpin:127.0.0.1:14551 when the MAVLink Router has the USB.
#
# usage: hitl.sh <px4 dir> <build dir> <world> <model> <gazebo window: 0|1> <python> <hitl_bridge.py> <device> <baud>
#                [bridge args...]
#
# <build dir>: the SITL build folder (relative to <px4 dir>) whose Gazebo environment to use, if it's built.
#
# An empty <python> runs the bridge with the app's own Python environment (pyenv.sh).

PX4_DIR="$1"
BUILD_DIR="$2"
WORLD="$3"
MODEL="$4"
GUI="$5"
PYTHON="$6"
BRIDGE="$7"
DEVICE="$8"
BAUD="$9"
shift 9

GZ_DIR="$PX4_DIR/Tools/simulation/gz"
ENV_FILE="$PX4_DIR/$BUILD_DIR/rootfs/gz_env.sh"

stamp() { date '+%Y-%m-%d %H:%M:%S'; }
# Take Gazebo down with us; a half-started simulation is no use
fail() { echo "=== $(stamp) $1"; kill 0; exit 1; }

if [ -z "$PYTHON" ]; then
    . "$(dirname "$0")/pyenv.sh"
    PYTHON="$VENV_PYTHON"
fi

# Same Gazebo environment PX4 SITL uses (models, worlds, sensor plugins)
if [ -f "$ENV_FILE" ]; then
    . "$ENV_FILE"
else
    export GZ_SIM_RESOURCE_PATH="$GZ_SIM_RESOURCE_PATH:$GZ_DIR/models:$GZ_DIR/worlds"
    export GZ_SIM_SERVER_CONFIG_PATH="$PX4_DIR/src/modules/simulation/gz_bridge/server.config"
fi

echo "=== $(stamp) starting Gazebo ($WORLD)"
gz sim --verbose=1 -r -s "$GZ_DIR/worlds/$WORLD.sdf" &
if [ "$GUI" = 1 ]; then
    gz sim -g >/dev/null 2>&1 &
fi

echo "=== $(stamp) waiting for Gazebo"
tries=0
until gz service -i --service "/world/$WORLD/create" 2>/dev/null | grep -q "Service providers"; do
    tries=$((tries + 1))
    [ "$tries" -ge 60 ] && fail "Gazebo did not start"
    sleep 1
done

echo "=== $(stamp) spawning $MODEL"
SDF="<sdf version=\"1.6\"> <include> <uri>file://$GZ_DIR/models/$MODEL/model.sdf</uri> </include> </sdf>"
gz service -s "/world/$WORLD/create" --reqtype gz.msgs.EntityFactory --reptype gz.msgs.Boolean \
    --timeout 5000 --req "name: \"$MODEL\", allow_renaming: false, sdf: '$SDF'" | grep -q true \
    || fail "could not spawn $MODEL"

# The bridge's subscriptions and publishers can stay unconnected for good when
# they're made while Gazebo is still creating the vehicle's sensors, motors and
# servos (the vehicle then sits on the ground at full throttle). Start it only
# once the vehicle is up, as the groundstation app's bridge is started by hand.
echo "=== $(stamp) waiting for $MODEL's sensors"
IMU_TOPIC="/world/$WORLD/model/$MODEL/link/base_link/sensor/imu_sensor/imu"
tries=0
until timeout 5 gz topic -e -n 1 -t "$IMU_TOPIC" >/dev/null 2>&1; do
    tries=$((tries + 1))
    [ "$tries" -ge 12 ] && fail "$MODEL's IMU does not publish ($IMU_TOPIC)"
done
sleep 3

# Retry delay: doubles while the bridge keeps failing right away, back to
# 1 s once a run lasted a while (e.g. after the Pixhawk rebooted)
delay=1
while true; do
    # Only a device can be waited for; through the router, the bridge waits for its heartbeat
    if [ "${DEVICE#/}" != "$DEVICE" ] && [ ! -e "$DEVICE" ]; then
        echo "=== $(stamp) waiting for Pixhawk on $DEVICE"
        while [ ! -e "$DEVICE" ]; do sleep 0.5; done
        # Give udev a moment to set the device permissions
        sleep 1
    fi

    echo "=== $(stamp) starting HITL bridge on $DEVICE"
    started=$(date +%s)
    # The bridge prints every actuator message; keep those out of the log
    "$PYTHON" -u "$BRIDGE" --connection "$DEVICE" --baud "$BAUD" --world "$WORLD" --model "$MODEL" "$@" 2>&1 \
        | grep --line-buffered -v '^Actuators: '
    if [ $(( $(date +%s) - started )) -ge 10 ]; then
        delay=1
    else
        delay=$(( delay * 2 > 5 ? 5 : delay * 2 ))
    fi
    echo "=== $(stamp) HITL bridge exited, restarting in ${delay} s"
    sleep "$delay"
done
