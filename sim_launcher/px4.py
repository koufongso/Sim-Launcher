"""What a PX4-Autopilot checkout offers: SITL models, HIL airframes, worlds."""

import glob
import os
import re
from dataclasses import dataclass, field
from pathlib import Path
from typing import Dict, List, Optional

SITL_BUILD = "build/px4_sitl_default"
# Builds in Docker go to a folder of their own: a build folder records absolute
# paths, and a dev container's build of the same tree has the repo at another path
DOCKER_BUILD_SUFFIX = "_launcher"
GZ_DIR = "Tools/simulation/gz"
POSIX_AIRFRAMES = "ROMFS/px4fmu_common/init.d-posix/airframes"
# Airframes of the real (NuttX) firmware; the *.hil ones are for HITL
AIRFRAMES = "ROMFS/px4fmu_common/init.d/airframes"

_PARAM_LINE = re.compile(r"^\s*param\s+(?:set|set-default)\s+(\w+)\s+(\S+)")


@dataclass
class HilAirframe:
    model: str
    autostart: int
    path: str
    # Parameters the airframe sets itself; HITL setup leaves these alone
    params: Dict[str, str] = field(default_factory=dict)

    def _functions(self) -> List[int]:
        return [int(v) for k, v in self.params.items() if k.startswith("HIL_ACT_FUNC") and v.isdigit()]

    @property
    def motors(self) -> int:
        return sum(101 <= f <= 199 for f in self._functions())

    @property
    def servos(self) -> int:
        return sum(201 <= f <= 299 for f in self._functions())


def is_px4_dir(path: str) -> bool:
    return bool(path) and os.path.isfile(os.path.join(path, "Makefile")) and \
        os.path.isdir(os.path.join(path, GZ_DIR))


def _search_home(pattern: str) -> List[str]:
    """Matches of pattern up to three directories below home."""
    home = Path.home()
    found = []
    for depth in range(4):
        found += glob.glob(str(home.joinpath(*(["*"] * depth), pattern)))
    return found


def find_px4_dir(saved: str = "") -> Optional[str]:
    if is_px4_dir(saved):
        return saved
    env = os.environ.get("PX4_DIR", "")
    if is_px4_dir(env):
        return env
    # Prefer a plain "PX4-Autopilot" over forks like "PX4-Autopilot_montecarlo"
    candidates = sorted(_search_home("PX4-Autopilot*"), key=lambda p: (os.path.basename(p) != "PX4-Autopilot", p))
    return next((p for p in candidates if is_px4_dir(p)), None)


def sitl_build_dir(docker: bool) -> str:
    """The SITL build folder, relative to the PX4 folder."""
    return SITL_BUILD + (DOCKER_BUILD_SUFFIX if docker else "")


def sitl_built(px4_dir: str, build_dir: str) -> bool:
    return os.path.isfile(os.path.join(px4_dir, build_dir, "bin", "px4"))


def sitl_models(px4_dir: str) -> List[str]:
    """Models PX4 SITL can start with Gazebo: airframes named <id>_gz_<model>."""
    models = set()
    for name in _listdir(os.path.join(px4_dir, POSIX_AIRFRAMES)):
        match = re.fullmatch(r"\d+_gz_(\w+)", name)
        if match:
            models.add(match.group(1))
    return sorted(models)


def hil_airframes(px4_dir: str) -> Dict[str, HilAirframe]:
    """HIL airframes (<id>_<model>.hil) that have a Gazebo model of the same name."""
    found = {}
    models_dir = os.path.join(px4_dir, GZ_DIR, "models")
    for name in _listdir(os.path.join(px4_dir, AIRFRAMES)):
        match = re.fullmatch(r"(\d+)_(\w+)\.hil", name)
        if not match or not os.path.isfile(os.path.join(models_dir, match.group(2), "model.sdf")):
            continue
        path = os.path.join(px4_dir, AIRFRAMES, name)
        found[match.group(2)] = HilAirframe(match.group(2), int(match.group(1)), path, _airframe_params(path))
    return found


def _airframe_params(path: str) -> Dict[str, str]:
    params = {}
    try:
        with open(path) as f:
            for line in f:
                match = _PARAM_LINE.match(line)
                if match:
                    params[match.group(1)] = match.group(2)
    except OSError:
        pass
    return params


def worlds(px4_dir: str) -> List[str]:
    return sorted(n[:-4] for n in _listdir(os.path.join(px4_dir, GZ_DIR, "worlds")) if n.endswith(".sdf"))


def _listdir(path: str) -> List[str]:
    try:
        return os.listdir(path)
    except OSError:
        return []
