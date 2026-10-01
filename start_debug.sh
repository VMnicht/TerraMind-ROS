#!/usr/bin/env bash
# The launcher itself needs only system Python + PyQt5, even before colcon build.
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
export PYTHONPATH="$workspace_dir/src/terramind_panel${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m terramind_panel.debug_panel --workspace "$workspace_dir" "$@"
