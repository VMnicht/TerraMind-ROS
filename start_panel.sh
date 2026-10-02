#!/usr/bin/env bash
# Attach a panel to an already running simulation or hardware backend.
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$workspace_dir/scripts/native_gui_env.sh"
rebuild_requested=false
panel_arguments=()
while (($#)); do
  case "$1" in
    --build) rebuild_requested=true ;;
    -h|--help)
      cat <<'EOF'
用法：start_panel.sh [--build] [ROS 节点参数...]

打开控制面板，连接当前 ROS 域中已启动的仿真或实机后端。
不会打开串口或自动使能。请只运行一个控制输入源。
--build  先重新编译；缺少构建产物时自动编译。

完整仿真：./start_sim.sh capabilities:=63
串口回环：ros2 launch terramind_bringup serial_test.launch.py panel:=true
真实串口：ros2 launch terramind_bringup real.launch.py panel:=true（自动识别端口）
EOF
      exit 0 ;;
    *) panel_arguments+=("$1") ;;
  esac
  shift
done

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" && "${QT_QPA_PLATFORM:-}" != offscreen && "${QT_QPA_PLATFORM:-}" != minimal ]]; then
  printf '控制面板需要图形桌面，请在桌面终端运行。\n' >&2
  exit 1
fi
if [[ ! -r /opt/ros/humble/setup.bash ]]; then
  printf '未找到 ROS 2 Humble，请按 README.md 安装依赖。\n' >&2
  exit 1
fi
source /opt/ros/humble/setup.bash
cd "$workspace_dir"
if [[ ! -r install/local_setup.bash || ! -x install/terramind_panel/lib/terramind_panel/control_panel ]]; then
  rebuild_requested=true
fi
if [[ "$rebuild_requested" == true ]]; then
  MAKEFLAGS="${MAKEFLAGS:--j2 -l2}" colcon build --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
fi
source "$workspace_dir/install/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_dir/log/ros}"
mkdir -p "$ROS_LOG_DIR"
exec ros2 run terramind_panel control_panel "${panel_arguments[@]}"
