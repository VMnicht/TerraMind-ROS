#!/usr/bin/env bash
# 总面板直接加载源码，只需系统 Python 和 PyQt5，首次 colcon 构建前也能打开。
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -eo pipefail
workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$workspace_dir/scripts/native_gui_env.sh"
export PYTHONPATH="$workspace_dir/src/terramind_panel${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m terramind_panel.debug_panel --workspace "$workspace_dir" "$@"
