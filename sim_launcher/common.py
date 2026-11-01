"""Qt-free helpers: serial ports, /proc, UDP listeners, formatting."""

import glob
import os
from typing import List, Optional, Tuple

BAUD_RATES = ["57600", "115200", "230400", "460800", "921600"]


def serial_ports() -> list:
    """(label, path) for serial devices; stable /dev/serial/by-id paths first.

    by-id paths survive a replug (ttyACM0 may come back as ttyACM1, e.g. after
    the Pixhawk reboots), so they are the ones to save and to wait for.
    """
    ports = []
    linked = set()
    for path in sorted(glob.glob("/dev/serial/by-id/*")):
        real = os.path.realpath(path)
        linked.add(real)
        ports.append((f"{by_id_label(path)} ({os.path.basename(real)})", path))
    for path in sorted(glob.glob("/dev/ttyUSB*") + glob.glob("/dev/ttyACM*")):
        if path not in linked:
            ports.append((path, path))
    return ports


def by_id_label(path: str) -> str:
    """'usb-Holybro_PX4_FMU_v6C.x_3A0029-if00' -> 'Holybro PX4 FMU v6C.x'."""
    name = os.path.basename(path)
    if name.startswith("usb-"):
        name = name[4:]
    name = name.rsplit("-if", 1)[0]
    parts = name.split("_")
    if len(parts) > 1 and len(parts[-1]) >= 6 and parts[-1].isalnum():
        parts = parts[:-1]  # trailing serial number
    return " ".join(parts)


def udp_listen_ports() -> set:
    """Local UDP ports something on this machine is bound to."""
    ports = set()
    for path in ("/proc/net/udp", "/proc/net/udp6"):
        try:
            with open(path) as f:
                next(f)
                for line in f:
                    local = line.split()[1]
                    ports.add(int(local.rsplit(":", 1)[1], 16))
        except (OSError, StopIteration, ValueError, IndexError):
            pass
    return ports


# --- /proc helpers -----------------------------------------------------------

def pids() -> List[int]:
    return [int(e) for e in os.listdir("/proc") if e.isdigit()]


def cmdline(pid: int) -> List[str]:
    """argv of a process, split on spaces too.

    Gazebo's `gz` is a Ruby script that rewrites its argv into one string
    ("gz sim -s -r default.sdf"), so splitting is the only way to read it.
    """
    try:
        with open(f"/proc/{pid}/cmdline", "rb") as f:
            raw = f.read()
    except OSError:
        return []
    return raw.replace(b"\0", b" ").decode(errors="replace").split()


def pgid(pid: int) -> Optional[int]:
    try:
        with open(f"/proc/{pid}/stat") as f:
            return int(f.read().rsplit(")", 1)[1].split()[3])
    except (OSError, ValueError, IndexError):
        return None


def process_name(pid: int) -> str:
    try:
        with open(f"/proc/{pid}/comm") as f:
            return f.read().strip()
    except OSError:
        return "?"


def open_files(pid: int) -> List[str]:
    """Paths of the files a process has open (empty if not permitted)."""
    paths = []
    try:
        for fd in os.listdir(f"/proc/{pid}/fd"):
            try:
                paths.append(os.readlink(f"/proc/{pid}/fd/{fd}"))
            except OSError:
                pass
    except OSError:
        pass
    return paths


def device_holders(device: str) -> List[Tuple[int, str]]:
    """(pid, name) of processes that have the device open (own processes only)."""
    target = os.path.realpath(device)
    return [(pid, process_name(pid)) for pid in pids() if target in open_files(pid)]


def uptime_s(pid: int) -> Optional[float]:
    try:
        with open(f"/proc/{pid}/stat") as f:
            # Field 22 (starttime) counted after the ")" that ends the comm field
            start_ticks = int(f.read().rsplit(")", 1)[1].split()[19])
        with open("/proc/uptime") as f:
            system_uptime = float(f.read().split()[0])
        return system_uptime - start_ticks / os.sysconf("SC_CLK_TCK")
    except (OSError, ValueError, IndexError):
        return None


def describe(holders) -> str:
    return ", ".join(f"{name} (pid {pid})" for pid, name in holders)


def format_duration(seconds: float) -> str:
    seconds = int(seconds)
    return f"{seconds // 3600:02d}:{seconds % 3600 // 60:02d}:{seconds % 60:02d}"
