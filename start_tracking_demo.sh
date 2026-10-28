#!/usr/bin/env bash
set -euo pipefail
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"
source scripts/native_gui_env.sh
exec /usr/bin/python3 demos/tracking_demo/demo.py "$@"
