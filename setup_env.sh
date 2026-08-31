#!/bin/bash
WS_DIR="$HOME/ros2_ws"
PKG_DIR="$WS_DIR/src/ufvision_controller"
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"

[ -f "/opt/ros/jazzy/setup.bash" ] && source /opt/ros/jazzy/setup.bash
[ -f "/opt/ros/humble/setup.bash" ] && source /opt/ros/humble/setup.bash

mkdir -p "$PKG_DIR/models" "$PKG_DIR/launch" "$PKG_DIR/ufvision_controller/missions"

[ -f "$SCRIPT_DIR/models/best.pt" ] && cp "$SCRIPT_DIR/models/best.pt" "$PKG_DIR/models/best.pt"
[ -d "$SCRIPT_DIR/launch" ] && cp -r "$SCRIPT_DIR/launch/"* "$PKG_DIR/launch/" 2>/dev/null
[ -d "$SCRIPT_DIR/ufvision_controller" ] && cp -r "$SCRIPT_DIR/ufvision_controller/"* "$PKG_DIR/ufvision_controller/" 2>/dev/null
[ -f "$SCRIPT_DIR/package.xml" ] && cp "$SCRIPT_DIR/package.xml" "$PKG_DIR/"
[ -f "$SCRIPT_DIR/setup.py" ] && cp "$SCRIPT_DIR/setup.py" "$PKG_DIR/"

if [ -f "$SCRIPT_DIR/run_autonomo.py" ]; then
    cp "$SCRIPT_DIR/run_autonomo.py" "$WS_DIR/run_autonomo.py"
    chmod +x "$WS_DIR/run_autonomo.py"
fi

cd "$WS_DIR"
colcon build --packages-select ufvision_controller
source "$WS_DIR/install/setup.bash"

echo "Instalação concluída!"
