"""Main window: pick SITL or HITL, a vehicle and a world, press Start."""

import os
import re
import subprocess
import threading
import time
from concurrent.futures import ThreadPoolExecutor

from PyQt6.QtCore import QObject, Qt, QTimer, pyqtSignal
from PyQt6.QtGui import QFontDatabase, QKeySequence, QShortcut
from PyQt6.QtWidgets import (
    QApplication, QButtonGroup, QCheckBox, QComboBox, QDialog, QDialogButtonBox, QFileDialog, QFormLayout,
    QGridLayout, QGroupBox, QHBoxLayout, QLabel, QLineEdit, QMainWindow, QMessageBox, QPlainTextEdit,
    QPushButton, QSpinBox, QStackedWidget, QToolButton, QVBoxLayout, QWidget,
)

from . import hitl_setup, profiles, px4
from .common import BAUD_RATES, describe, device_holders, format_duration, open_files, serial_ports, \
    udp_listen_ports, uptime_s
from .launcher import LOG_FILE, VENV_PYTHON, Sim

GREEN, ORANGE, RED, GRAY = "#2e9d4f", "#d98a1a", "#d0393b", "#8a8a8a"
HOLDERS_CACHE_S = 2.0
WINDOW_WIDTH = 600
# Stage lines the scripts write, e.g. "=== 2026-11-01 12:00:00 building PX4 SITL"
STAGE = re.compile(r"^=== (?:\d{4}-\d\d-\d\d \d\d:\d\d:\d\d )?(.*)$")
FAILED = ("failed", "did not", "could not", "not built")
# The bridge prints these every 5 s
BRIDGE_STATS = re.compile(r"Stats: HIL_SENSOR=(\d+), HIL_GPS=(\d+), Actuators=(\d+), Errors=(\d+)")
# Gazebo and PX4 color their output
ANSI = re.compile(r"\x1b\[[0-9;]*m")


class StatusRow:
    """A colored dot, a label and a detail text."""

    def __init__(self, grid: QGridLayout, row: int, label: str):
        self.dot = QLabel("●")
        self._state = None
        self.label = QLabel(label)
        self.detail = QLabel()
        self.detail.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
        self.detail.setWordWrap(True)
        grid.addWidget(self.dot, row, 0)
        grid.addWidget(self.label, row, 1)
        grid.addWidget(self.detail, row, 2)
        self.set(GRAY, "")  # not the default black until the first refresh

    def set(self, color: str, text: str, tooltip: str = ""):
        # Called every refresh; only touch the widgets when something changed
        state = (color, text, tooltip)
        if state == self._state:
            return
        self._state = state
        self.dot.setStyleSheet(f"color: {color}; font-size: 16px;")
        self.dot.setToolTip(tooltip)
        self.detail.setText(text)


class SetupSignals(QObject):
    log = pyqtSignal(str)
    done = pyqtSignal(str)  # error message, empty on success


class PathsDialog(QDialog):
    def __init__(self, settings: dict, parent=None):
        super().__init__(parent)
        self.setWindowTitle("Paths")
        form = QFormLayout(self)
        self._settings = settings
        self._px4 = self._dir_row(form, "PX4-Autopilot", settings["px4_dir"] or px4.find_px4_dir() or "")
        self._image = QLineEdit()
        self._image.setPlaceholderText("none: build on this machine")
        self._image.setToolTip("Docker image with PX4's build tools, for a PX4 folder that's built in a container. "
                               "PX4 and Gazebo still run on this machine. Saved per PX4 folder.")
        form.addRow("Build in Docker image", self._image)
        self._px4.textChanged.connect(self._load_image)
        self._load_image()
        self._python = QLineEdit(settings["bridge_python"])
        self._python.setPlaceholderText("the app's own Python environment")
        self._python.setToolTip("Python that runs the HITL bridge; needs pymavlink and Gazebo's Python bindings "
                                "(gz.transport). Leave empty for the app's own environment.")
        form.addRow("Bridge Python", self._python)
        buttons = QDialogButtonBox(QDialogButtonBox.StandardButton.Ok | QDialogButtonBox.StandardButton.Cancel)
        buttons.accepted.connect(self.accept)
        buttons.rejected.connect(self.reject)
        form.addRow(buttons)
        self.setMinimumWidth(520)

    def _dir_row(self, form: QFormLayout, label: str, value: str) -> QLineEdit:
        edit = QLineEdit(value)
        browse = QToolButton()
        browse.setText("…")
        browse.clicked.connect(lambda: self._browse(edit, label))
        row = QHBoxLayout()
        row.addWidget(edit)
        row.addWidget(browse)
        form.addRow(label, row)
        return edit

    def _browse(self, edit: QLineEdit, label: str):
        path = QFileDialog.getExistingDirectory(self, label, edit.text() or os.path.expanduser("~"))
        if path:
            edit.setText(path)

    def _load_image(self):
        self._image.setText(profiles.build_image(self._settings, self._px4.text().strip()))

    def apply(self, settings: dict):
        settings["px4_dir"] = self._px4.text().strip()
        if settings["px4_dir"]:
            profiles.set_build_image(settings, settings["px4_dir"], self._image.text().strip())
        settings["bridge_python"] = self._python.text().strip()


class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Sim Launcher")
        self._quitting = False
        self._settings = profiles.load_settings()
        self._sim = Sim()
        self._px4_dir = None
        self._airframes = {}
        self._stage = ""
        self._bridge_stats = None  # (time, counts) of the last stats line
        self._bridge_rates = ""
        self._python_ok = {}  # bridge python -> error message ("" if fine)
        self._setup_running = False
        self._holders_cache = ("", float("-inf"), [])
        self._holders_scan = None  # Future of a background device_holders() scan
        self._scanner = ThreadPoolExecutor(max_workers=1)
        self._setup_signals = SetupSignals()
        self._setup_signals.log.connect(self._append_log)
        self._setup_signals.done.connect(self._on_setup_done)

        central = QWidget()
        layout = QVBoxLayout(central)
        layout.addLayout(self._build_mode_row())
        layout.addWidget(self._build_sim_box())
        layout.addWidget(self._build_status_box())
        self._log_view = QPlainTextEdit()
        self._log_view.setReadOnly(True)
        self._log_view.setMaximumBlockCount(500)
        self._log_view.setLineWrapMode(QPlainTextEdit.LineWrapMode.NoWrap)
        self._log_view.setFont(QFontDatabase.systemFont(QFontDatabase.SystemFont.FixedFont))
        self._log_view.setMinimumHeight(160)
        layout.addWidget(self._log_view, 1)
        layout.addLayout(self._build_button_row())
        QShortcut(QKeySequence.StandardKey.Quit, self, activated=self.close)
        self.setCentralWidget(central)
        self.resize(WINDOW_WIDTH, self.sizeHint().height())

        self._load_px4()
        self._load_mode(self._settings["mode"])
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._refresh)
        self._timer.start(1000)
        self._refresh()

    # --- layout -------------------------------------------------------------

    def _build_mode_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self._mode_group = QButtonGroup(self)
        self._mode_buttons = {}
        for key, label in profiles.MODES.items():
            button = QPushButton(label)
            button.setCheckable(True)
            button.setMinimumHeight(32)
            self._mode_group.addButton(button)
            self._mode_buttons[key] = button
            row.addWidget(button)
        self._mode_group.buttonClicked.connect(lambda _: self._load_mode(self._mode()))
        return row

    def _build_sim_box(self) -> QGroupBox:
        box = QGroupBox("Simulation")
        grid = QGridLayout(box)
        self._model_combo = QComboBox()
        self._world_combo = QComboBox()
        grid.addWidget(QLabel("Vehicle"), 0, 0)
        grid.addWidget(self._model_combo, 0, 1)
        grid.addWidget(QLabel("World"), 1, 0)
        grid.addWidget(self._world_combo, 1, 1)

        # Mode-specific inputs on stacked pages: the stack is as tall as its
        # tallest page, so nothing below moves when the mode changes
        self._sitl_page = QWidget()
        sitl = QGridLayout(self._sitl_page)
        sitl.setContentsMargins(0, 0, 0, 0)
        self._build_check = QCheckBox("Build PX4 before starting")
        self._build_check.setToolTip("make px4_sitl: picks up your code changes; quick when nothing changed")
        sitl.addWidget(self._build_check, 0, 0)
        sitl.setRowStretch(1, 1)

        self._hitl_page = QWidget()
        hitl = QGridLayout(self._hitl_page)
        hitl.setContentsMargins(0, 0, 0, 0)
        self._link_combo = QComboBox()
        for key, label in profiles.HITL_LINKS.items():
            self._link_combo.addItem(label, key)
        self._link_combo.setToolTip("Pixhawk USB: the bridge opens the USB itself.\n"
                                    "MAVLink Router: the router (Bench mode) has the USB and shares it with QGC "
                                    "and other tools; the bridge is one of its UDP endpoints.")
        self._link_combo.currentIndexChanged.connect(self._show_link_inputs)
        self._device_combo = QComboBox()
        # Long by-id names shouldn't dictate the window width
        self._device_combo.setSizeAdjustPolicy(QComboBox.SizeAdjustPolicy.AdjustToMinimumContentsLengthWithIcon)
        self._device_combo.setMinimumContentsLength(20)
        self._rescan_btn = QToolButton()
        self._rescan_btn.setText("⟳")
        self._rescan_btn.setToolTip("Rescan serial ports")
        self._rescan_btn.clicked.connect(lambda: self._rescan_ports())
        self._baud_combo = QComboBox()
        self._baud_combo.setEditable(True)
        self._baud_combo.addItems(BAUD_RATES)
        self._router_port_spin = QSpinBox()
        self._router_port_spin.setRange(1, 65535)
        self._router_port_spin.setToolTip("UDP port the MAVLink Router sends to for the bridge: add an endpoint "
                                          "with this port in the router's settings")
        # Baud for the USB or the router's port, whichever applies: one row with
        # one widget, so it's spaced like the others in both cases
        self._link_label = QLabel("Router UDP port")  # the longer text, for the label width below
        self._link_inputs = QStackedWidget()
        self._link_inputs.addWidget(self._baud_combo)
        self._link_inputs.addWidget(self._router_port_spin)
        self._setup_btn = QPushButton("Set up Pixhawk for HITL…")
        self._setup_btn.setToolTip("Once per Pixhawk and vehicle: selects the HIL airframe and the HITL parameters")
        self._setup_btn.clicked.connect(self._on_setup)
        hitl.addWidget(QLabel("Connect via"), 0, 0)
        hitl.addWidget(self._link_combo, 0, 1, 1, 2)
        self._device_label = QLabel("Pixhawk USB")
        hitl.addWidget(self._device_label, 1, 0)
        hitl.addWidget(self._device_combo, 1, 1)
        hitl.addWidget(self._rescan_btn, 1, 2)
        hitl.addWidget(self._link_label, 2, 0)
        hitl.addWidget(self._link_inputs, 2, 1, 1, 2)
        hitl.addWidget(self._setup_btn, 3, 1, 1, 2)
        hitl.setColumnStretch(1, 1)

        self._pages = QStackedWidget()
        self._pages.addWidget(self._sitl_page)
        self._pages.addWidget(self._hitl_page)
        grid.addWidget(self._pages, 2, 0, 1, 2)

        self._gui_check = QCheckBox("Show Gazebo window")
        grid.addWidget(self._gui_check, 3, 0, 1, 2)
        grid.setColumnStretch(1, 1)
        # One label width on both pages, so the inputs line up and don't shift between modes
        labels = [grid.itemAtPosition(r, 0).widget() for r in (0, 1)] + \
            [hitl.itemAtPosition(r, 0).widget() for r in (0, 1, 2)]
        width = max(label.sizeHint().width() for label in labels)
        for label in labels:
            label.setFixedWidth(width)
        return box

    def _build_status_box(self) -> QGroupBox:
        box = QGroupBox("Status")
        grid = QGridLayout(box)
        grid.setColumnStretch(2, 1)
        self._sim_row = StatusRow(grid, 0, "Simulation")
        self._gazebo_row = StatusRow(grid, 1, "Gazebo")
        self._autopilot_row = StatusRow(grid, 2, "PX4")  # "Bridge" in HITL
        self._link_row = StatusRow(grid, 3, "MAVLink")  # "Pixhawk" in HITL
        # Same label width in both modes, so the details don't shift
        width = max(QLabel(t).sizeHint().width() for t in ("Simulation", "Pixhawk", "MAVLink", "Bridge"))
        for row in (self._sim_row, self._gazebo_row, self._autopilot_row, self._link_row):
            row.label.setFixedWidth(width)
        return box

    def _build_button_row(self) -> QHBoxLayout:
        row = QHBoxLayout()
        self._start_btn = QPushButton("Start")
        self._start_btn.clicked.connect(self._on_start)
        self._stop_btn = QPushButton("Stop")
        self._stop_btn.clicked.connect(self._on_stop)
        paths_btn = QPushButton("Paths…")
        paths_btn.clicked.connect(self._on_paths)
        log_btn = QPushButton("Open log")
        log_btn.setToolTip(str(LOG_FILE))
        log_btn.clicked.connect(lambda: subprocess.Popen(["xdg-open", str(LOG_FILE)]))
        row.addWidget(self._start_btn)
        row.addWidget(self._stop_btn)
        row.addStretch()
        row.addWidget(paths_btn)
        row.addWidget(log_btn)
        return row

    # --- mode / inputs ------------------------------------------------------

    def _mode(self) -> str:
        return next(key for key, button in self._mode_buttons.items() if button.isChecked())

    def _load_px4(self):
        self._px4_dir = px4.find_px4_dir(self._settings["px4_dir"])
        self._airframes = px4.hil_airframes(self._px4_dir) if self._px4_dir else {}

    def _load_mode(self, mode: str):
        # Keep what was picked in the mode we leave
        shown = getattr(self, "_shown_mode", None)
        if shown is not None and shown != mode:
            self._save_inputs(shown, mode)
        self._shown_mode = mode
        self._mode_buttons[mode].setChecked(True)
        cfg = self._settings[mode]
        hitl = mode == "hitl"
        self._pages.setCurrentWidget(self._hitl_page if hitl else self._sitl_page)
        self._autopilot_row.label.setText("Bridge" if hitl else "PX4")
        self._link_row.label.setText("Pixhawk" if hitl else "MAVLink")

        if self._px4_dir:
            models = sorted(self._airframes) if hitl else px4.sitl_models(self._px4_dir)
            worlds = px4.worlds(self._px4_dir)
        else:
            models, worlds = [], []
        self._fill(self._model_combo, models, cfg["model"])
        self._fill(self._world_combo, worlds, cfg["world"])
        if hitl:
            self._model_combo.setToolTip("Vehicles with a HIL airframe (<id>_<model>.hil) and a Gazebo model")
            self._rescan_ports(selected=cfg["device"])
            self._baud_combo.setCurrentText(str(cfg["baud"]))
            self._link_combo.setCurrentIndex(max(0, self._link_combo.findData(cfg["link"])))
            self._router_port_spin.setValue(int(cfg["router_port"]))
            self._show_link_inputs()
        else:
            self._model_combo.setToolTip("PX4 SITL airframes (<id>_gz_<model>)")
            self._build_check.setChecked(cfg["build"])
        self._gui_check.setChecked(cfg["gui"])
        self._refresh()

    @staticmethod
    def _fill(combo: QComboBox, items, selected: str):
        combo.clear()
        combo.addItems(items)
        index = combo.findText(selected)
        if index >= 0:
            combo.setCurrentIndex(index)

    def _via_router(self) -> bool:
        return self._link_combo.currentData() == "router"

    def _show_link_inputs(self):
        router = self._via_router()
        self._link_label.setText("Router UDP port" if router else "Baud")
        self._link_inputs.setCurrentWidget(self._router_port_spin if router else self._baud_combo)
        self._setup_btn.setToolTip(
            "Talks to the Pixhawk USB directly: connect via Pixhawk USB (and stop the router) to set up" if router
            else "Once per Pixhawk and vehicle: selects the HIL airframe and the HITL parameters")
        self._refresh()  # the USB inputs lock or unlock

    def _device(self) -> str:
        return self._device_combo.currentData() or ""

    def _rescan_ports(self, selected: str = ""):
        current = selected or self._device()
        self._device_combo.clear()
        for label, path in serial_ports():
            self._device_combo.addItem(label, path)
        if current and self._device_combo.findData(current) < 0:
            # Keep the saved device selectable while unplugged; the bridge waits for it
            label = current if os.path.exists(current) else f"{current} (not connected)"
            self._device_combo.addItem(label, current)
        index = self._device_combo.findData(current)
        if index >= 0:
            self._device_combo.setCurrentIndex(index)

    def _save_inputs(self, mode: str, next_mode: str = ""):
        cfg = self._settings[mode]
        if self._model_combo.currentText():  # empty while PX4 isn't found; keep the saved choice
            cfg["model"] = self._model_combo.currentText()
            cfg["world"] = self._world_combo.currentText()
        cfg["gui"] = self._gui_check.isChecked()
        if mode == "hitl":
            cfg["device"] = self._device() or cfg["device"]
            cfg["link"] = self._link_combo.currentData()
            cfg["router_port"] = self._router_port_spin.value()
            try:
                cfg["baud"] = int(self._baud_combo.currentText())
            except ValueError:
                pass
        else:
            cfg["build"] = self._build_check.isChecked()
        self._settings["mode"] = next_mode or mode
        profiles.save_settings(self._settings)

    # --- actions ------------------------------------------------------------

    def _on_start(self):
        mode = self._mode()
        if not self._px4_dir:
            QMessageBox.warning(self, "PX4 not found", "Set the PX4-Autopilot folder under Paths.")
            self._on_paths()
            return
        if not self._model_combo.currentText() or not self._world_combo.currentText():
            QMessageBox.warning(self, "Nothing to start", "Pick a vehicle and a world.")
            return
        if mode == "hitl":
            try:
                int(self._baud_combo.currentText())
            except ValueError:
                QMessageBox.warning(self, "Invalid baud rate", "Baud rate must be a number.")
                return
        if not self._stop_external_first():
            return

        if mode == "sitl":
            self._save_inputs(mode)
            cfg = self._settings["sitl"]
            image = profiles.build_image(self._settings, self._px4_dir)
            build_dir = px4.sitl_build_dir(bool(image))
            build = cfg["build"] or not px4.sitl_built(self._px4_dir, build_dir)
            if build and image and not self._docker_image_ok(image):
                return
            if build and not cfg["build"]:
                self._append_log("PX4 SITL isn't built yet, building it first (takes a few minutes the first time)")
            self._begin()
            self._sim.start_sitl(self._px4_dir, build_dir, cfg["model"], cfg["world"], build, cfg["gui"], image)
        else:
            if not self._bridge_can_run():
                return
            if self._via_router():
                port = self._router_port_spin.value()
                if not profiles.router_has_endpoint(port) and not self._confirm(
                        "No router endpoint for the bridge",
                        f"The MAVLink Router doesn't send to UDP {port}, so the bridge won't hear the Pixhawk.\n\n"
                        f"Add an endpoint to {profiles.ROUTER_SETTINGS_FILE}, e.g.\n"
                        f'    {{"name": "HITL bridge", "port": {port}}}\n'
                        "then restart the router (Bench mode). Start anyway? The bridge keeps retrying."):
                    return
                # The router has the USB; the bridge listens for it like QGC does
                connection = f"udpin:127.0.0.1:{port}"
                link_args = ["--component-id", str(profiles.ROUTER_BRIDGE_COMPONENT_ID)]
            else:
                connection = self._device()
                if not connection:
                    QMessageBox.warning(self, "No port", "Select the Pixhawk's USB port first.")
                    return
                holders = device_holders(connection)
                if holders and not self._confirm(
                        "Port in use",
                        f"{connection} is open by {describe(holders)}.\n\n"
                        "Connecting via Pixhawk USB, the HITL bridge needs the USB to itself. To share it with QGC "
                        "and other tools, connect via the MAVLink Router instead. Start anyway? "
                        "The bridge keeps retrying."):
                    return
                link_args = []
            self._save_inputs(mode)
            cfg = self._settings["hitl"]
            airframe = self._airframes[cfg["model"]]
            self._begin()
            build_dir = px4.sitl_build_dir(bool(profiles.build_image(self._settings, self._px4_dir)))
            self._sim.start_hitl(
                self._px4_dir, build_dir, cfg["world"], cfg["model"], cfg["gui"], self._settings["bridge_python"],
                connection, cfg["baud"],
                ["--num-motors", str(airframe.motors), "--num-servos", str(airframe.servos)] + link_args)
        self._refresh()

    def _begin(self):
        self._stage = ""
        self._bridge_stats = None
        self._bridge_rates = ""
        self._log_view.clear()

    def _docker_image_ok(self, image: str) -> bool:
        """Whether Docker has the image to build PX4 in; tells the user why not."""
        try:
            result = subprocess.run(["docker", "image", "inspect", image], capture_output=True, text=True, timeout=15)
            error = "" if result.returncode == 0 else result.stderr.strip()
        except (OSError, subprocess.TimeoutExpired) as e:
            error = str(e)
        if error:
            QMessageBox.warning(self, "Can't build in Docker",
                                f"PX4 is set to build in Docker image {image}, but:\n{error}\n\n"
                                "Build the image first, or clear the image under Paths to build on this machine.")
        return not error

    def _bridge_can_run(self) -> bool:
        """Whether the bridge Python has what the bridge needs (the bridge's own --check); tells the user why not."""
        python = self._settings["bridge_python"]
        # Before the app's own environment exists, check the system Python it's made from;
        # setting it up installs pymavlink
        not_set_up = not python and not VENV_PYTHON.exists()
        if not python:
            python = "/usr/bin/python3" if not_set_up else str(VENV_PYTHON)
        if python not in self._python_ok:
            bridge = os.path.join(os.path.dirname(__file__), "hitl_bridge.py")
            try:
                result = subprocess.run([python, bridge, "--check"], capture_output=True, text=True, timeout=15)
                lines = (result.stdout + result.stderr).strip().splitlines()
                if not_set_up:
                    lines = [line for line in lines if not line.startswith("pymavlink")]
                self._python_ok[python] = "" if result.returncode == 0 or not lines else "\n".join(lines[-3:])
            except (OSError, subprocess.TimeoutExpired) as e:
                self._python_ok[python] = str(e)
        error = self._python_ok[python]
        if error:
            del self._python_ok[python]  # check again next time, e.g. after installing
            QMessageBox.warning(self, "HITL bridge can't run", f"{python} is missing what the bridge needs:\n\n{error}")
            return False
        return True

    def _stop_external_first(self) -> bool:
        """Offer to stop PX4/Gazebo started outside this app; they would clash. True to go on."""
        external = self._sim.external(fresh=True)
        if not external:
            return True
        listing = "\n".join(f"  {kind} (pid {pid})" for pid, kind in sorted(external.items()))
        if not self._confirm("Simulation already running",
                             f"Started outside this app:\n{listing}\n\n"
                             "A second simulation would fight over Gazebo and the MAVLink ports. Stop these "
                             "and start?"):
            return False
        self._sim.stop_external(list(external))
        return True

    def _on_stop(self):
        if self._sim.running() is not None:
            self._sim.stop()
        else:
            external = self._sim.external(fresh=True)
            if external and self._confirm(
                    "Stop external simulation?",
                    "A simulation not started from this app is running "
                    f"(pid {', '.join(map(str, sorted(external)))}). Stop it?"):
                self._sim.stop_external(list(external))
        self._refresh()

    def _on_paths(self):
        dialog = PathsDialog(self._settings, self)
        if dialog.exec() != QDialog.DialogCode.Accepted:
            return
        dialog.apply(self._settings)
        profiles.save_settings(self._settings)
        self._load_px4()
        self._load_mode(self._mode())

    def _on_setup(self):
        airframe = self._airframes.get(self._model_combo.currentText())
        device = self._device()
        if airframe is None or not device:
            QMessageBox.warning(self, "HITL setup", "Pick the vehicle and the Pixhawk's USB port first.")
            return
        holders = device_holders(device)
        if holders:
            QMessageBox.warning(self, "Port in use",
                                f"{device} is open by {describe(holders)}. Close it first: setup talks to the "
                                "Pixhawk USB directly, also when the simulation connects via the MAVLink Router "
                                "(stop the router for the setup, start it again after).")
            return
        extras = hitl_setup.extra_params(airframe)
        if not self._confirm(
                "Set up Pixhawk for HITL",
                f"This sets the Pixhawk on {os.path.basename(os.path.realpath(device))} up for HITL with "
                f"{airframe.model}:\n\n"
                f"1. Select airframe {os.path.basename(airframe.path)} (SYS_AUTOSTART = {airframe.autostart}) "
                "and reboot. PX4 resets most parameters when the airframe changes.\n"
                f"2. Set {len(extras)} more parameters for the simulated sensors, then reboot again:\n"
                f"{', '.join(name for name, _ in extras)}\n\n"
                "The Pixhawk firmware must include this airframe (built from the same PX4 tree). "
                "To fly the real vehicle again, select its normal airframe in QGC.\n\nContinue?"):
            return
        self._settings["hitl"]["device"] = device
        profiles.save_settings(self._settings)
        self._setup_running = True
        self._log_view.clear()
        baud = int(self._baud_combo.currentText())
        threading.Thread(target=self._run_setup, args=(device, baud, airframe), daemon=True).start()
        self._refresh()

    def _run_setup(self, device: str, baud: int, airframe):
        signals = self._setup_signals
        try:
            hitl_setup.PixhawkSetup(device, baud, airframe, signals.log.emit).run()
            signals.done.emit("")
        except hitl_setup.SetupError as e:
            signals.done.emit(str(e))
        except Exception as e:  # serial errors etc.; report rather than lose them in the thread
            signals.done.emit(f"{type(e).__name__}: {e}")

    def _on_setup_done(self, error: str):
        self._setup_running = False
        if error:
            self._append_log(f"Setup failed: {error}")
            QMessageBox.warning(self, "HITL setup failed", error)
        else:
            QMessageBox.information(self, "HITL setup", "Done. Once the Pixhawk has rebooted, press Start.")
        self._refresh()

    def _on_quit(self) -> bool:
        """Quit after confirmation; a running simulation is stopped. True if quitting."""
        if self._sim.running() is not None:
            if not self._ask_quit("The simulation is running. Stop it and quit?", "Stop simulation and quit"):
                return False
            self._sim.stop()
        elif not self._ask_quit("Quit Sim Launcher?", "Quit"):
            return False
        self._quitting = True
        self._scanner.shutdown(wait=False, cancel_futures=True)
        QApplication.quit()
        return True

    def _ask_quit(self, text: str, action: str) -> bool:
        """Cancel + one action button; same roles in both quit dialogs, so the same button order."""
        box = QMessageBox(QMessageBox.Icon.Question, "Quit", text, parent=self)
        go = box.addButton(action, QMessageBox.ButtonRole.AcceptRole)
        cancel = box.addButton("Cancel", QMessageBox.ButtonRole.RejectRole)
        box.setDefaultButton(cancel)
        box.exec()
        return box.clickedButton() == go

    def show_existing(self):
        """Bring the window back, e.g. when the app is launched a second time."""
        self.showNormal()
        self.raise_()
        self.activateWindow()

    def _confirm(self, title: str, text: str) -> bool:
        return QMessageBox.question(self, title, text) == QMessageBox.StandardButton.Yes

    def _append_log(self, line: str):
        self._log_view.appendPlainText(line)

    # --- status -------------------------------------------------------------

    def _read_output(self):
        for line in self._sim.new_output():
            line = ANSI.sub("", line)
            stage = STAGE.match(line)
            if stage:
                self._stage = stage.group(1)
            stats = BRIDGE_STATS.search(line)
            if stats:
                self._on_bridge_stats([int(n) for n in stats.groups()])
                continue  # shown in the status, not worth a log line every 5 s
            self._append_log(line)

    def _on_bridge_stats(self, counts):
        now = time.monotonic()
        if self._bridge_stats is not None:
            then, before = self._bridge_stats
            dt = max(now - then, 1e-3)
            sensor, _, actuators, errors = [(a - b) / dt for a, b in zip(counts, before)]
            self._bridge_rates = f"sensors {sensor:.0f}/s, actuators {actuators:.0f}/s" + \
                (f", {counts[3]} errors" if counts[3] else "")
        self._bridge_stats = (now, counts)

    def _refresh(self):
        self._read_output()
        current = self._sim.running()
        parts = self._sim.components(current[0]) if current else None
        external = {} if current else self._sim.external()
        mode = current[1] if current else self._mode()
        if current and self._mode() != mode:
            self._load_mode(mode)  # e.g. the app was restarted while a simulation runs
            return
        active = current is not None or bool(external) or self._setup_running

        self._refresh_sim(current, parts, external)
        if parts and parts["gazebo"]:
            self._gazebo_row.set(GREEN, "running" + (", window open" if parts["gazebo_gui"] else ""))
        elif any(k in external.values() for k in ("gazebo", "gazebo_gui")):
            self._gazebo_row.set(ORANGE, "running (external)")
        else:
            self._gazebo_row.set(GRAY, "-")
        if mode == "sitl":
            self._refresh_sitl(parts)
        else:
            self._refresh_hitl(current, parts)

        state = "running" if active else "stopped"
        title = f"Sim Launcher: {mode.upper()} {state}" if active else "Sim Launcher"
        if title != self.windowTitle():
            self.setWindowTitle(title)
        self._start_btn.setEnabled(not active and self._px4_dir is not None)
        self._stop_btn.setEnabled((current is not None or bool(external)) and not self._setup_running)
        for w in (*self._mode_buttons.values(), self._model_combo, self._world_combo, self._build_check,
                  self._baud_combo, self._gui_check, self._link_combo, self._router_port_spin):
            w.setEnabled(not active)
        # Through the router, the router has the USB
        usb = not self._via_router()
        for w in (self._device_label, self._device_combo, self._rescan_btn):
            w.setEnabled(not active and usb)
        self._setup_btn.setEnabled(not active and usb and bool(self._airframes))

    def _refresh_sim(self, current, parts, external):
        if self._setup_running:
            self._sim_row.set(ORANGE, "setting up the Pixhawk")
        elif not self._px4_dir:
            self._sim_row.set(RED, "PX4-Autopilot not found: set it under Paths")
        elif current is None:
            if external:
                self._sim_row.set(ORANGE, "running outside this app",
                                  ", ".join(f"{k} (pid {p})" for p, k in sorted(external.items())))
            elif any(word in self._stage for word in FAILED):
                self._sim_row.set(RED, f"{self._stage}, see the log")
            else:
                self._sim_row.set(GRAY, "stopped")
        else:
            mode = current[1]
            main = parts["px4"] if mode == "sitl" else parts["bridge"]
            if main and parts["gazebo"]:
                up = uptime_s(parts["gazebo"][0])
                self._sim_row.set(GREEN, "running" + (f", up {format_duration(up)}" if up else ""))
            elif mode == "sitl" and not parts["px4"] and not parts["build"]:
                self._sim_row.set(RED, "PX4 exited, see the log; press Stop to clean up")
            elif mode == "hitl" and not parts["gazebo"] and self._stage.startswith("starting HITL bridge"):
                self._sim_row.set(RED, "Gazebo exited; press Stop to clean up")
            else:
                self._sim_row.set(ORANGE, self._stage or "starting")

    def _refresh_sitl(self, parts):
        if parts and parts["px4"]:
            self._autopilot_row.set(GREEN, f"running, pid {parts['px4'][0]}")
        elif parts and parts["build"]:
            self._autopilot_row.set(ORANGE, "building")
        else:
            self._autopilot_row.set(GRAY, "-")
        port = profiles.SITL_GCS_PORT
        hint = "PX4 sends its GCS link here: open QGC, or start the MAVLink Router in SITL mode to share it"
        if port in udp_listen_ports():
            self._link_row.set(GREEN, f"UDP {port}, something is listening", hint)
        else:
            self._link_row.set(GRAY, f"UDP {port}, nothing listening", hint)

    def _refresh_hitl(self, current, parts):
        if parts and parts["bridge"]:
            self._autopilot_row.set(GREEN, self._bridge_rates or "running")
        elif current is not None:
            self._autopilot_row.set(ORANGE, "not running yet")
        else:
            self._autopilot_row.set(GRAY, "-")

        if (self._settings["hitl"]["link"] == "router") if current else self._via_router():
            self._refresh_router_link(current, parts)
            return
        device = self._settings["hitl"]["device"] if current else self._device()
        if not device:
            self._link_row.set(GRAY, "-")
            return
        name = os.path.basename(os.path.realpath(device))
        if not os.path.exists(device):
            color = ORANGE if current or self._setup_running else GRAY
            self._link_row.set(color, "not connected" + (", waiting" if current else ""))
            return
        if self._setup_running:
            self._link_row.set(ORANGE, f"{name}, being set up")
            return
        bridge = parts["bridge"] if parts else []
        if any(os.path.realpath(device) in open_files(pid) for pid in bridge):
            self._link_row.set(GREEN, f"{name} open by the bridge")
            return
        holders = [h for h in self._cached_holders(device) if h[0] not in bridge]
        if holders:
            self._link_row.set(RED if current else ORANGE, f"held by {holders[0][1]}",
                               f"{name} is open by {describe(holders)}")
        else:
            self._link_row.set(GRAY, f"{name} available")

    def _refresh_router_link(self, current, parts):
        port = self._settings["hitl"]["router_port"] if current else self._router_port_spin.value()
        via = f"via MAVLink Router, UDP {port}"
        if parts and parts["bridge"] and self._bridge_rates:
            self._link_row.set(GREEN, via)
        elif current is not None:
            self._link_row.set(ORANGE, f"{via}, waiting for the Pixhawk's heartbeat",
                               "Is the router running in Bench mode, with an endpoint for this port?")
        else:
            self._link_row.set(GRAY, via)

    def _cached_holders(self, device: str):
        """Last known holders of the device.

        device_holders() scans every process's open files (tens of ms), so it
        runs in the background at most every HOLDERS_CACHE_S; until a result
        for this device is in, report none.
        """
        scan = self._holders_scan
        if scan is not None and scan.done():
            self._holders_scan = None
            try:
                self._holders_cache = (scan.device, time.monotonic(), scan.result())
            except OSError:
                pass
        cached_device, stamp, holders = self._holders_cache
        stale = cached_device != device or time.monotonic() - stamp > HOLDERS_CACHE_S
        if stale and self._holders_scan is None:
            self._holders_scan = self._scanner.submit(device_holders, device)
            self._holders_scan.device = device
        return holders if cached_device == device else []

    def closeEvent(self, event):
        # Closing the window quits the app (with confirmation; stops the simulation)
        if self._quitting or self._on_quit():
            event.accept()
        else:
            event.ignore()
