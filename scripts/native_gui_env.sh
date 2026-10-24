#!/usr/bin/env bash
# 在 ROS setup 前 source；系统 Qt/ROS 程序不能混用 Snap 内的运行库。
# Snap 编辑器的集成终端可能继承 GTK/GIO/Qt 路径，这里仅过滤其中的 Snap 项。
terramind_clean_snap_paths() {
  local name value entry changed
  local -a entries kept cleaned=()
  for name in GTK_PATH GIO_MODULE_DIR GIO_EXTRA_MODULES QT_PLUGIN_PATH \
    QT_QPA_PLATFORM_PLUGIN_PATH LD_LIBRARY_PATH LD_PRELOAD; do
    value="${!name:-}"
    [[ -n "$value" ]] || continue
    # 动态加载器允许 LD_PRELOAD 同时使用空格和冒号分隔。
    if [[ "$name" == LD_PRELOAD ]]; then
      value="${value// /:}"
    fi
    IFS=: read -r -a entries <<< "$value"
    kept=()
    changed=false
    for entry in "${entries[@]}"; do
      case "$entry" in
        /snap/*|/var/lib/snapd/snap/*) changed=true ;;
        *) kept+=("$entry") ;;
      esac
    done
    if [[ "$changed" == true ]]; then
      local IFS=:
      value="${kept[*]}"
      if [[ -n "$value" ]]; then
        export "$name=$value"
      else
        unset "$name"
      fi
      cleaned+=("$name")
    fi
  done
  if ((${#cleaned[@]})); then
    printf '已过滤 Snap 库路径（仅影响本次启动）：%s\n' "${cleaned[*]}" >&2
  fi
}
terramind_clean_snap_paths
unset -f terramind_clean_snap_paths

# 本机 WSL 的 Fast DDS 进程发现不稳定；已安装 Cyclone DDS 时用于本工作区入口。
# 保留用户显式选择，不修改系统或其他终端的默认中间件。
if [[ -z "${RMW_IMPLEMENTATION:-}" ]] && grep -qi microsoft /proc/sys/kernel/osrelease \
  && [[ -f /opt/ros/humble/lib/librmw_cyclonedds_cpp.so ]]; then
  export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
fi
