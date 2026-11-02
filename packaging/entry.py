# PyInstaller entry point (the package's __main__ uses relative imports)
import os
import sys

# The bootloader and PyInstaller's runtime hooks point LD_LIBRARY_PATH,
# QT_PLUGIN_PATH, XDG_DATA_DIRS etc. at the bundle. Everything started from
# here (make, PX4, Gazebo's Qt5 window, Python) inherits the environment, so
# give it the original back. Qt's own plugin path is set before that, so the
# app keeps finding its plugins.
from PyQt6.QtCore import QCoreApplication

if "QT_PLUGIN_PATH" in os.environ:
    QCoreApplication.addLibraryPath(os.environ["QT_PLUGIN_PATH"])

_orig = os.environ.pop("LD_LIBRARY_PATH_ORIG", None)
if _orig is None:
    os.environ.pop("LD_LIBRARY_PATH", None)
else:
    os.environ["LD_LIBRARY_PATH"] = _orig

for _name, _value in list(os.environ.items()):
    _kept = [p for p in _value.split(os.pathsep) if not p.startswith(sys._MEIPASS)]
    if _kept != _value.split(os.pathsep):
        if any(_kept):
            os.environ[_name] = os.pathsep.join(_kept)
        else:
            del os.environ[_name]

from sim_launcher.__main__ import main

main()
