"""Start, stop and inspect the detached simulation (SITL or HITL).

The simulation is one process group, started from sitl.sh or hitl.sh in its
own session. Being detached, it outlives the GUI; and Stop signals exactly that
group (PX4, Gazebo server and window, HITL bridge, build), nothing started
elsewhere.
"""

import os
import shutil
import signal
import subprocess
import time
from pathlib import Path
from typing import Dict, List, Optional, Tuple

from .common import cmdline, pgid, pids
from .profiles import STATE_DIR

PID_FILE = STATE_DIR / "sim.pid"  # "<pid> <mode>"
LOG_FILE = STATE_DIR / "sim.log"
# Copied out of the package so they keep working after an AppImage exits
SCRIPTS = {"sitl": STATE_DIR / "sitl.sh", "hitl": STATE_DIR / "hitl.sh"}
PYENV = STATE_DIR / "pyenv.sh"
BRIDGE = STATE_DIR / "hitl_bridge.py"
# The app's own Python environment, made by pyenv.sh
VENV_PYTHON = STATE_DIR / "venv" / "bin" / "python"
# A /proc scan for simulations started elsewhere runs at most this often
EXTERNAL_SCAN_INTERVAL_S = 2.0
# Kinds of simulation processes, as shown in the status
KINDS = ("build", "px4", "gazebo", "gazebo_gui", "bridge")


def process_kind(pid: int) -> Optional[str]:
    argv = cmdline(pid)
    if not argv:
        return None
    exe = os.path.basename(argv[0])
    if exe == "px4":
        return "px4"
    # "gz sim ..." (Ruby rewrites its argv) or "ruby /usr/bin/gz sim ..."
    args = argv[1:] if exe == "gz" else argv[2:] if len(argv) > 1 and os.path.basename(argv[1]) == "gz" else None
    if args and args[0] == "sim":
        return "gazebo_gui" if "-g" in args else "gazebo"
    if any(a.endswith("hitl_bridge.py") for a in argv[:3]):
        return "bridge"
    # docker: the client of a build in a container (sitl.sh)
    if exe in ("make", "ninja", "cmake", "docker"):
        return "build"
    # Setting up the app's Python environment (pyenv.sh) belongs to the build
    if argv[1:3] in (["-m", "pip"], ["-m", "venv"]):
        return "build"
    return None


def _copy_if_changed(src: Path, dst: Path):
    if dst.exists() and dst.read_bytes() == src.read_bytes():
        return
    dst.parent.mkdir(parents=True, exist_ok=True)
    tmp = dst.with_suffix(".tmp")
    shutil.copy2(src, tmp)
    os.replace(tmp, dst)


def _trim_log(max_bytes: int = 5_000_000, keep_bytes: int = 1_000_000):
    """Keep the log bounded; a PX4 build alone writes a lot."""
    try:
        if LOG_FILE.stat().st_size > max_bytes:
            with open(LOG_FILE, "rb") as f:
                f.seek(-keep_bytes, os.SEEK_END)
                tail = f.read()
            LOG_FILE.write_bytes(tail[tail.find(b"\n") + 1:])
    except OSError:
        pass


class Sim:
    def __init__(self):
        STATE_DIR.mkdir(parents=True, exist_ok=True)
        # Set only when started in this session, so the child gets reaped
        self._proc: Optional[subprocess.Popen] = None
        self._external_scan: Tuple[float, Dict[int, str]] = (float("-inf"), {})
        # Start at the end so output of earlier runs isn't shown again
        self._log_offset = LOG_FILE.stat().st_size if LOG_FILE.exists() else 0

    def running(self) -> Optional[Tuple[int, str]]:
        """(process group, mode) of the simulation this app started, if any of it is still running."""
        self._reap()
        try:
            pid, mode = PID_FILE.read_text().split()
            pid = int(pid)
        except (OSError, ValueError):
            return None
        return (pid, mode) if self.members(pid) else None

    @staticmethod
    def members(group: int) -> List[int]:
        return [p for p in pids() if pgid(p) == group]

    def components(self, group: int) -> Dict[str, List[int]]:
        """Pids of each kind of process (see KINDS) in the simulation's group."""
        found = {kind: [] for kind in KINDS}
        for pid in self.members(group):
            kind = process_kind(pid)
            if kind:
                found[kind].append(pid)
        return found

    def external(self, fresh: bool = False) -> Dict[int, str]:
        """PX4 SITL, Gazebo and HITL bridge processes not started by this app: {pid: kind}."""
        now = time.monotonic()
        if fresh or now - self._external_scan[0] > EXTERNAL_SCAN_INTERVAL_S:
            ours = self.running()
            found = {}
            for pid in pids():
                kind = process_kind(pid)
                if kind and kind != "build" and (ours is None or pgid(pid) != ours[0]):
                    found[pid] = kind
            self._external_scan = (now, found)
        return {p: k for p, k in self._external_scan[1].items() if os.path.exists(f"/proc/{p}")}

    def start_sitl(self, px4_dir: str, build_dir: str, model: str, world: str, build: bool, gui: bool,
                   image: str = "") -> int:
        return self._start("sitl", [px4_dir, build_dir, model, world, str(int(build)), str(int(gui)), image])

    def start_hitl(self, px4_dir: str, build_dir: str, world: str, model: str, gui: bool, python: str,
                   device: str, baud: int, bridge_args: List[str]) -> int:
        _copy_if_changed(Path(__file__).parent / BRIDGE.name, BRIDGE)
        return self._start("hitl", [px4_dir, build_dir, world, model, str(int(gui)), python, str(BRIDGE), device,
                                    str(baud)] + bridge_args)

    def _start(self, mode: str, args: List[str]) -> int:
        script = SCRIPTS[mode]
        for path in (script, PYENV):
            _copy_if_changed(Path(__file__).parent / path.name, path)
        _trim_log()
        self._log_offset = LOG_FILE.stat().st_size if LOG_FILE.exists() else 0
        with open(LOG_FILE, "a") as log:
            log.write(f"\n=== {time.strftime('%Y-%m-%d %H:%M:%S')} {mode.upper()}: {' '.join(args)}\n")
            log.flush()
            self._proc = subprocess.Popen(
                ["/bin/sh", str(script)] + args,
                stdin=subprocess.DEVNULL,
                stdout=log,
                stderr=subprocess.STDOUT,
                start_new_session=True,
                close_fds=True,
            )
        PID_FILE.write_text(f"{self._proc.pid} {mode}\n")
        return self._proc.pid

    def stop(self, timeout: float = 5.0):
        """Stop the whole simulation group: TERM, then KILL what's left."""
        current = self.running()
        if current is not None:
            group = current[0]
            self._signal_group(group, signal.SIGTERM)
            deadline = time.monotonic() + timeout
            while self.members(group) and time.monotonic() < deadline:
                self._reap()
                time.sleep(0.1)
            self._signal_group(group, signal.SIGKILL)
            self._reap()
        PID_FILE.unlink(missing_ok=True)

    def stop_external(self, pids_: List[int], timeout: float = 5.0):
        for pid in pids_:
            try:
                os.kill(pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
        deadline = time.monotonic() + timeout
        while any(os.path.exists(f"/proc/{p}") for p in pids_) and time.monotonic() < deadline:
            time.sleep(0.1)
        for pid in pids_:
            try:
                os.kill(pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        self._external_scan = (float("-inf"), {})

    @staticmethod
    def _signal_group(group: int, sig: int):
        try:
            os.killpg(group, sig)
        except ProcessLookupError:
            pass

    def _reap(self):
        if self._proc is not None and self._proc.poll() is not None:
            self._proc = None

    def new_output(self) -> List[str]:
        """Simulation output lines written since the last call."""
        try:
            with open(LOG_FILE, "rb") as f:
                f.seek(0, os.SEEK_END)
                if f.tell() < self._log_offset:  # trimmed
                    self._log_offset = 0
                f.seek(self._log_offset)
                data = f.read()
        except OSError:
            return []
        # Only consume complete lines
        end = data.rfind(b"\n") + 1
        self._log_offset += end
        return data[:end].decode(errors="replace").splitlines()
