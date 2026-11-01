"""Persisted settings."""

import copy
import json
import os
from pathlib import Path

CONFIG_DIR = Path(os.environ.get("XDG_CONFIG_HOME", Path.home() / ".config")) / "sim-launcher-app"
STATE_DIR = Path(os.environ.get("XDG_STATE_HOME", Path.home() / ".local" / "state")) / "sim-launcher-app"
SETTINGS_FILE = CONFIG_DIR / "settings.json"

MODES = {"sitl": "SITL", "hitl": "HITL"}

# PX4 SITL sends its GCS link here (QGC, or the MAVLink Router in SITL mode)
SITL_GCS_PORT = 14550

# How the HITL bridge reaches the Pixhawk: its USB directly, or through the
# MAVLink Router app (Bench mode) as one of the router's UDP endpoints
HITL_LINKS = {"usb": "Pixhawk USB", "router": "MAVLink Router"}
# The MAVLink Router app's settings, to check it has an endpoint for the bridge
ROUTER_SETTINGS_FILE = CONFIG_DIR.parent / "mavlink-router-app" / "settings.json"
# The bridge's MAVLink component ID through the router; QGC already is 255/190
ROUTER_BRIDGE_COMPONENT_ID = 25  # MAV_COMP_ID_USER1

DEFAULT_SETTINGS = {
    "mode": "sitl",
    # Found automatically when empty (see px4.find_px4_dir)
    "px4_dir": "",
    # PX4 folder -> Docker image it's built in; not listed: built on this machine
    "build_images": {},
    # Empty: the app's own Python environment (pyenv.sh), which sees the system
    # Python's packages, so Gazebo's Python bindings
    "bridge_python": "",
    "sitl": {"model": "standard_vtol", "world": "default", "build": True, "gui": True},
    "hitl": {"model": "standard_vtol", "world": "default", "device": "/dev/ttyACM0", "baud": 921600, "gui": True,
             "link": "usb", "router_port": 14551},
}


def build_image(settings: dict, px4_dir: str) -> str:
    """Docker image the PX4 folder is built in; empty when it's built on this machine."""
    return settings["build_images"].get(os.path.realpath(px4_dir), "") if px4_dir else ""


def set_build_image(settings: dict, px4_dir: str, image: str):
    key = os.path.realpath(px4_dir)
    if image:
        settings["build_images"][key] = image
    else:
        settings["build_images"].pop(key, None)


def router_has_endpoint(port: int) -> bool:
    """Whether the MAVLink Router app sends to this UDP port; True when its settings can't be read."""
    try:
        with open(ROUTER_SETTINGS_FILE) as f:
            endpoints = json.load(f).get("endpoints", [])
    except (OSError, ValueError):
        return True
    return any(int(ep.get("port", 0)) == port and ep.get("address", "127.0.0.1") in ("127.0.0.1", "localhost")
               for ep in endpoints)


def load_settings() -> dict:
    """Load settings, filling in defaults; writes the defaults on first run."""
    settings = copy.deepcopy(DEFAULT_SETTINGS)
    try:
        with open(SETTINGS_FILE) as f:
            user = json.load(f)
        for key, value in user.items():
            if isinstance(value, dict) and isinstance(settings.get(key), dict):
                settings[key].update(value)
            else:
                settings[key] = value
    except FileNotFoundError:
        save_settings(settings)
    except (OSError, ValueError) as e:
        print(f"[settings] ignoring unreadable {SETTINGS_FILE}: {e}")
    return settings


def save_settings(settings: dict):
    CONFIG_DIR.mkdir(parents=True, exist_ok=True)
    with open(SETTINGS_FILE, "w") as f:
        json.dump(settings, f, indent=2)
        f.write("\n")
