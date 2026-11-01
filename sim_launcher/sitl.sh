#!/bin/sh
# Builds (optionally) and runs PX4 SITL. PX4 starts Gazebo itself
# (px4-rc.gzsim), so PX4, the Gazebo server and its window all stay in this
# process group and the app can stop exactly them.
#
# usage: sitl.sh <px4 dir> <build dir> <model> <world> <build: 0|1> <gazebo window: 0|1> [docker image]
#
# <build dir> is relative to <px4 dir>, e.g. build/px4_sitl_default. With a
# Docker image, PX4 is built in a container and still runs here.

PX4_DIR="$1"
BUILD_DIR="$2"
MODEL="$3"
WORLD="$4"
BUILD="$5"
GUI="$6"
IMAGE="$7"

stamp() { date '+%Y-%m-%d %H:%M:%S'; }

build() {
    # e.g. build/px4_sitl_default_launcher -> BUILD_DIR_SUFFIX=_launcher
    suffix="${BUILD_DIR#build/px4_sitl_default}"
    if [ -z "$IMAGE" ]; then
        . "$(dirname "$0")/pyenv.sh"
        echo "=== $(stamp) building PX4 SITL"
        PATH="$VENV/bin:$PATH" make px4_sitl PYTHON_EXECUTABLE="$VENV_PYTHON" BUILD_DIR_SUFFIX="$suffix"
        return
    fi

    # Mount the outermost repo (PX4 may be a submodule; git describe needs the
    # superproject's .git) at the same path as here, so what the build records
    # (gz_env.sh, the CMake cache) is right outside the container too
    root="$PX4_DIR"
    while up=$(git -C "$root" rev-parse --show-superproject-working-tree 2>/dev/null) && [ -n "$up" ]; do
        root="$up"
    done
    # Named, so a build left running by a killed client is removed next time;
    # --init passes Stop's SIGTERM on to make
    name="sim-launcher-build-$(id -u)"
    docker rm -f "$name" >/dev/null 2>&1
    echo "=== $(stamp) building PX4 SITL in Docker ($IMAGE)"
    docker run --rm --init --name "$name" \
        --user "$(id -u):$(id -g)" \
        -v "$root:$root" -w "$PX4_DIR" \
        -e GIT_CONFIG_COUNT=1 -e GIT_CONFIG_KEY_0=safe.directory -e GIT_CONFIG_VALUE_0='*' \
        "$IMAGE" make px4_sitl BUILD_DIR_SUFFIX="$suffix"
}

cd "$PX4_DIR" || exit 1
if [ "$BUILD" = 1 ]; then
    if ! build; then
        echo "=== $(stamp) build failed"
        exit 1
    fi
fi

if ! cd "$BUILD_DIR/rootfs"; then
    echo "=== $(stamp) PX4 SITL is not built"
    exit 1
fi

export PX4_SIM_MODEL="gz_$MODEL"
export PX4_GZ_WORLD="$WORLD"
[ "$GUI" = 1 ] || export HEADLESS=1

echo "=== $(stamp) starting PX4 ($MODEL in $WORLD)"
# -d: no interactive shell; use QGC's MAVLink console or bin/px4-<command>
exec ../bin/px4 -d
