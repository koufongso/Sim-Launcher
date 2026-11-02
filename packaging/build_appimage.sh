#!/usr/bin/env bash
# Build Sim_Launcher-x86_64.AppImage: the GUI as a PyInstaller bundle, with the
# launch scripts and the HITL bridge it copies to the runtime folder.
#
# Needs: pyinstaller (pip install pyinstaller), appimagetool (on PATH or
# APPIMAGETOOL=/path/to/appimagetool).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BUILD="$ROOT/build"
APPDIR="$BUILD/AppDir"
APPIMAGETOOL="${APPIMAGETOOL:-appimagetool}"

command -v "$APPIMAGETOOL" >/dev/null || { echo "appimagetool not found; set APPIMAGETOOL=..." >&2; exit 1; }
command -v pyinstaller >/dev/null || { echo "pyinstaller not found; pip install pyinstaller" >&2; exit 1; }

rm -rf "$BUILD"
mkdir -p "$APPDIR/usr/lib"

cd "$ROOT"
pyinstaller --noconfirm --clean --windowed \
    --name sim-launcher-app \
    --distpath "$BUILD/dist" --workpath "$BUILD/work" --specpath "$BUILD" \
    --add-data "$ROOT/sim_launcher/icon.svg:sim_launcher" \
    --add-data "$ROOT/sim_launcher/sitl.sh:sim_launcher" \
    --add-data "$ROOT/sim_launcher/hitl.sh:sim_launcher" \
    --add-data "$ROOT/sim_launcher/pyenv.sh:sim_launcher" \
    `# run with the bridge Python, not imported` \
    --add-data "$ROOT/sim_launcher/hitl_bridge.py:sim_launcher" \
    --paths "$ROOT" \
    `# not used; keeps stray site-packages out of the bundle` \
    --exclude-module numpy --exclude-module tkinter --exclude-module matplotlib --exclude-module gi \
    packaging/entry.py

cp -r "$BUILD/dist/sim-launcher-app" "$APPDIR/usr/lib/"
cp packaging/sim-launcher.desktop "$APPDIR/"
# appimagetool needs a PNG icon (and .DirIcon); render the SVG with Qt
QT_QPA_PLATFORM=offscreen python3 - "$ROOT/sim_launcher/icon.svg" "$APPDIR/sim-launcher.png" <<'PY'
import sys
from PyQt6.QtGui import QGuiApplication, QImage, QPainter
from PyQt6.QtSvg import QSvgRenderer
app = QGuiApplication(sys.argv[:1])
image = QImage(256, 256, QImage.Format.Format_ARGB32)
image.fill(0)
painter = QPainter(image)
QSvgRenderer(sys.argv[1]).render(painter)
painter.end()
image.save(sys.argv[2])
PY
ln -sf sim-launcher.png "$APPDIR/.DirIcon"

cat > "$APPDIR/AppRun" <<'APPRUN'
#!/bin/sh
HERE="$(dirname "$(readlink -f "$0")")"
export APPDIR="${APPDIR:-$HERE}"
exec "$HERE/usr/lib/sim-launcher-app/sim-launcher-app" "$@"
APPRUN
chmod +x "$APPDIR/AppRun"

ARCH=x86_64 "$APPIMAGETOOL" "$APPDIR" "$ROOT/Sim_Launcher-x86_64.AppImage"
echo "Built $ROOT/Sim_Launcher-x86_64.AppImage"
