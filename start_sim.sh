#!/usr/bin/env bash
# Also support invoking this entry point with `sh start_sim.sh`.
[ -n "${BASH_VERSION:-}" ] || exec bash "$0" "$@"
set -eo pipefail

workspace_dir="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
source "$workspace_dir/scripts/native_gui_env.sh"
rebuild_requested=false
headless_requested=false
panel_requested=true
launch_arguments=()

while (($#)); do
  case "$1" in
    --headless)
      headless_requested=true
      ;;
    --build)
      rebuild_requested=true
      ;;
    --no-panel)
      panel_requested=false
      ;;
    -h|--help)
      cat <<'EOF'
用法：start_sim.sh [--headless] [--no-panel] [--build] [ROS launch 参数...]

  默认启动二维仿真、控制面板、机器人模型和 RViz。
  --headless  不打开 RViz 和控制面板，适用于无显示环境。
  --no-panel 不打开控制面板，可用其他控制工具。
  --build     启动前重新编译；缺少构建产物时会自动编译。
  --help      显示此帮助。

其他参数传给 sim.launch.py，例如 capabilities:=63（开启全部仿真装置）。
启动后保持停机，控制方法见 README.md。按 Ctrl+C 退出。
EOF
      exit 0
      ;;
    --)
      shift
      launch_arguments+=("$@")
      break
      ;;
    *)
      launch_arguments+=("$1")
      ;;
  esac
  shift
done

terramind_ros_setup=/opt/ros/humble/setup.bash
if [[ ! -r "$terramind_ros_setup" ]]; then
  printf '未找到 ROS 2 Humble：%s\n' "$terramind_ros_setup" >&2
  exit 1
fi
# ROS setup scripts may read unset variables, so do not enable nounset here.
source "$terramind_ros_setup"
cd "$workspace_dir"

required_artifacts=(
  install/local_setup.bash
  install/terramind_control/lib/terramind_control/control_manager_node
  install/terramind_sim/lib/terramind_sim/sim_interface_node
  install/terramind_bringup/share/terramind_bringup/launch/sim.launch.py
  install/terramind_description/share/terramind_description/urdf/terramind.urdf.in
  install/terramind_panel/lib/terramind_panel/control_panel
)
for artifact in "${required_artifacts[@]}"; do
  if [[ ! -r "$artifact" ]]; then
    rebuild_requested=true
    break
  fi
done

if [[ "$rebuild_requested" == true ]]; then
  if ! command -v colcon >/dev/null 2>&1; then
    printf '未找到 colcon，请按 README.md 安装构建依赖。\n' >&2
    exit 1
  fi
  printf '正在编译 TerraMind…\n'
  MAKEFLAGS="${MAKEFLAGS:--j2 -l2}" colcon build \
    --symlink-install --parallel-workers 2 \
    --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo
fi

source "$workspace_dir/install/local_setup.bash"
export ROS_LOG_DIR="${ROS_LOG_DIR:-$workspace_dir/log/ros}"
mkdir -p "$ROS_LOG_DIR"

if [[ -z "${DISPLAY:-}" && -z "${WAYLAND_DISPLAY:-}" ]]; then
  headless_requested=true
fi
if [[ "$headless_requested" == true ]]; then
  headless_arguments=(rviz:=false panel:=false)
  for argument in "${launch_arguments[@]}"; do
    if [[ "$argument" != rviz:=* && "$argument" != panel:=* ]]; then
      headless_arguments+=("$argument")
    fi
  done
  launch_arguments=("${headless_arguments[@]}")
  printf '启动二维仿真（无图形窗口）。按 Ctrl+C 退出。\n'
else
  panel_arguments=("panel:=$panel_requested")
  for argument in "${launch_arguments[@]}"; do
    if [[ "$panel_requested" != false || "$argument" != panel:=* ]]; then
      panel_arguments+=("$argument")
    fi
  done
  launch_arguments=("${panel_arguments[@]}")
  printf '启动二维仿真。可在控制面板中设置目标并开始发送。按 Ctrl+C 退出。\n'
fi

# Let ros2 launch receive terminal signals and shut down all its child nodes.
exec ros2 launch terramind_bringup sim.launch.py "${launch_arguments[@]}"
