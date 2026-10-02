#!/usr/bin/env bash
# Source before ROS setup: native Qt/ROS programs must not load Snap libraries.
# A Snap editor's integrated terminal can inherit its GTK/GIO/Qt search paths.
terramind_clean_snap_paths() {
  local name value entry changed
  local -a entries kept cleaned=()
  for name in GTK_PATH GIO_MODULE_DIR GIO_EXTRA_MODULES QT_PLUGIN_PATH \
    QT_QPA_PLATFORM_PLUGIN_PATH LD_LIBRARY_PATH LD_PRELOAD; do
    value="${!name:-}"
    [[ -n "$value" ]] || continue
    # The loader accepts both spaces and colons in LD_PRELOAD.
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
