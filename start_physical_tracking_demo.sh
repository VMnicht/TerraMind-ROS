#!/usr/bin/env bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source scripts/native_gui_env.sh
source /opt/ros/humble/setup.bash
source install/setup.bash
exec /usr/bin/python3 demos/physical_tracking_demo/panel.py "$@"
