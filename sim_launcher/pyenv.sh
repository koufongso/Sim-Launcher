# Sourced by sitl.sh and hitl.sh: makes sure the app's own Python environment
# has what PX4's build and the HITL bridge need (PX4's
# Tools/setup/requirements.txt), and sets VENV_PYTHON. It's a venv with the
# system site packages, so Gazebo's Python bindings (apt only) stay importable.
#
# needs: PX4_DIR, stamp()

VENV="$(dirname "$0")/venv"
VENV_PYTHON="$VENV/bin/python"
REQUIREMENTS="$PX4_DIR/Tools/setup/requirements.txt"
# Copy of the requirements last installed; reinstall when PX4's change
INSTALLED="$VENV/px4-requirements.txt"

if [ ! -x "$VENV_PYTHON" ]; then
    echo "=== $(stamp) creating Python environment in $VENV"
    if ! /usr/bin/python3 -m venv --system-site-packages "$VENV"; then
        rm -rf "$VENV"
        echo "=== $(stamp) could not create the Python environment (sudo apt install python3-venv)"
        exit 1
    fi
fi

if ! cmp -s "$REQUIREMENTS" "$INSTALLED"; then
    echo "=== $(stamp) installing PX4's Python packages (the first time takes a few minutes)"
    if ! "$VENV_PYTHON" -m pip install --disable-pip-version-check -r "$REQUIREMENTS"; then
        echo "=== $(stamp) installing PX4's Python packages failed"
        exit 1
    fi
    cp "$REQUIREMENTS" "$INSTALLED"
fi
