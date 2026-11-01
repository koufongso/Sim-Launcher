# Sim Launcher

A small window for starting PX4 simulations with Gazebo: pick **SITL** or **HITL**, a vehicle and a world, press
**Start**. **Stop** takes down exactly what was started (PX4, Gazebo and its window, the HITL bridge, a running build)
and nothing else.

The simulation runs **detached**, so closing or crashing the app doesn't stop it; start the app again and it picks the
running simulation up. Quitting from the app asks, then stops it.

MAVLink fan-out (QGC, other tools) is the job of the [MAVLink Router](../Mavlink-Router) app, not this one.

## SITL

```
make px4_sitl (optional) ──► px4 -d ──► starts Gazebo (server + window) and spawns the vehicle
                               └──► MAVLink to UDP 14550 ──► QGC, or the MAVLink Router in SITL mode
```

- **Vehicle**: every PX4 SITL airframe named `<id>_gz_<model>`.
- **Build PX4 before starting** runs `make px4_sitl` first, so your code changes are in; it's quick when nothing
  changed. The first start builds anyway. The build uses the app's own Python environment (see below), so PX4's
  Python packages don't need to be installed system-wide.
- PX4 runs without its interactive shell; use QGC's MAVLink console, or `build/px4_sitl_default/bin/px4-<command>`
  (e.g. `px4-commander arm`) from the PX4 folder.

## HITL

**Connect via** picks how the bridge reaches the Pixhawk:

**Pixhawk USB**: the bridge opens the USB itself; QGC uses a telemetry radio.

```
Gazebo (vehicle) ⇄ hitl_bridge ⇄ Pixhawk USB (/dev/ttyACM0)
Pixhawk TELEM1 ~ SiK radio ──► MAVLink Router (Flight mode) ──► QGC
```

**MAVLink Router**: no radio needed. The [MAVLink Router](../Mavlink-Router) app in **Bench** mode has the USB and
shares it; the bridge is one of its UDP endpoints, next to QGC and other tools.

```
Gazebo (vehicle) ⇄ hitl_bridge (udpin:127.0.0.1:14551) ⇄ mavlink-routerd ⇄ Pixhawk USB (/dev/ttyACM0)
                                                               └──► QGC (14550), other tools
```

Once, add the bridge's endpoint to the router's `endpoints` (`~/.config/mavlink-router-app/settings.json`), with the
port set under **Router UDP port** (default 14551), and restart the router:

```json
{"name": "HITL bridge", "port": 14551}
```

Start warns when the router has no endpoint for that port. Through the router the bridge uses MAVLink component ID 25,
so it doesn't clash with QGC (255/190). **Set up Pixhawk for HITL…** still talks to the USB directly: stop the router
for it, start it again after.

1. Once per Pixhawk and vehicle: **Set up Pixhawk for HITL…**. It selects the vehicle's HIL airframe
   (`<id>_<model>.hil`, e.g. `1002_standard_vtol.hil`), reboots, sets what the simulated sensors need and the airframe
   doesn't already set (sensor IDs, GPS driver off, circuit breakers for battery/safety switch/ESCs), and reboots again.
   The Pixhawk firmware must be built from the same PX4 tree, so it has that airframe.
2. **Start**: Gazebo starts, the vehicle is spawned, then the bridge connects to the Pixhawk. When the Pixhawk reboots
   or is unplugged, the bridge waits for it and reconnects. Motor and servo counts come from the HIL airframe.

**Vehicle** lists the HIL airframes that have a Gazebo model of the same name.

The bridge is part of this app ([sim_launcher/hitl_bridge.py](sim_launcher/hitl_bridge.py)). It is copied to the
runtime folder on start and run with the app's own Python environment, which has `pymavlink`; Gazebo's Python bindings
(`gz.transport`, or `gz.transport13` on Harmonic) come from the system Python (`sudo apt install python3-gz-transport13` for Gazebo Harmonic). The app
checks this before starting and tells you what's missing. It also runs on its own:
`python3 sim_launcher/hitl_bridge.py --help`.

Connecting via Pixhawk USB, QGC and the MAVLink Router must not open the USB; the bridge needs it to itself. The app
shows who holds the port if it's taken.

## Status

- **Simulation**: what it's doing (building, waiting for Gazebo, waiting for the Pixhawk, running + uptime), or what
  failed. Orange "running outside this app" when PX4 SITL or Gazebo was started some other way; **Stop** offers to
  stop those too.
- **Gazebo**: server running, and whether its window is open.
- **PX4** (SITL): the PX4 process. **Bridge** (HITL): sensor messages sent to and actuator messages received from the
  Pixhawk, per second.
- **MAVLink** (SITL): whether something listens on UDP 14550. **Pixhawk** (HITL): connected, open by the bridge, or
  held by another program; via the router: whether the bridge hears the Pixhawk through it.

The log view shows the simulation's output; **Open log** opens the whole file.

## Configuration

**Paths…** sets the PX4-Autopilot folder, the Docker image to build it in (see below) and the Python that runs the
bridge (empty: the app's own environment). The PX4 folder is found automatically up
to three levels below your home folder (a folder named exactly `PX4-Autopilot` is preferred), or from `$PX4_DIR`.

Settings: `~/.config/sim-launcher-app/settings.json`. Runtime files (log, pid file, the launch scripts, the bridge and the Python environment): 
`~/.local/state/sim-launcher-app/`.

## Building in Docker

A PX4 folder whose toolchain lives in a container (e.g. a dev container image) can be built there: set **Build in
Docker image** under **Paths…** (saved per PX4 folder; empty builds on this machine). Only the build runs in the
container; PX4, Gazebo and the bridge still run here, so the image must match this machine's OS and Gazebo closely
enough for the built PX4 to run.

The build runs `docker run --rm --init --user <you> -v <repo>:<repo> -w <PX4 folder> <image> make px4_sitl`. The
outermost git repo (PX4 can be a submodule) is mounted at the same path as on this machine, because the build folder
records absolute paths. Docker builds go to `build/px4_sitl_default_launcher`, so they never mix with a dev container's
own `build/px4_sitl_default`, which has the repo at another path (e.g. `/workspaces/...`). HITL uses the same folder's
Gazebo environment.

## Python environment

A PX4 build on this machine and the HITL bridge run with a venv the app keeps in `~/.local/state/sim-launcher-app/venv`, made from
`/usr/bin/python3` with `--system-site-packages` so the apt-installed Gazebo bindings stay visible. The first start
creates it and installs PX4's `Tools/setup/requirements.txt` into it (a few minutes, needs internet); it's reinstalled
when that file changes. Needs `python3-venv`. Delete the folder to start over.

## Run from source

```bash
pip install -r requirements.txt   # PyQt6, pymavlink
python3 -m sim_launcher
```

Needs Gazebo Harmonic (`gz`) and a PX4-Autopilot checkout.
