#!/usr/bin/env bash
# Source this file to export ROS settings without changing the caller's shell options.
_kaixuan_load_environment() {
  local -  # Bash restores all `set` options when this function returns.
  set +u   # ROS setup scripts reference optional variables without defaults.
  local client_root client_requested_domain
  client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)" || return 1
  # Capture the caller's override before ROS setup/config files are sourced.
  client_requested_domain="${ROS_DOMAIN_ID:-${KAIXUAN_ROS_DOMAIN_ID:-}}"
  if [[ ! -f /opt/ros/humble/setup.bash ]]; then
    echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash" >&2
    return 1
  fi
  source /opt/ros/humble/setup.bash || return 1
  source "$client_root/network.env" || return 1
  client_requested_domain="${client_requested_domain:-$KAIXUAN_ROS_DOMAIN_ID}"
  if [[ ! "$client_requested_domain" =~ ^[0-9]{1,3}$ ]] || (( 10#$client_requested_domain > 232 )); then
    echo "ROS_DOMAIN_ID / KAIXUAN_ROS_DOMAIN_ID 必须为 0～232 的整数。" >&2
    return 1
  fi
  export ROS_DOMAIN_ID="$((10#$client_requested_domain))"
  export ROS_LOCALHOST_ONLY=0
  export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
  export FASTRTPS_DEFAULT_PROFILES_FILE="$client_root/config/dds_udp.xml"
  export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
  if [[ -n "$KAIXUAN_DISCOVERY_SERVER" ]]; then
    export ROS_DISCOVERY_SERVER="$KAIXUAN_DISCOVERY_SERVER"
  else
    unset ROS_DISCOVERY_SERVER
  fi
}

# Check status explicitly; startup wrappers may enable errexit in their own process.
if _kaixuan_load_environment; then
  unset -f _kaixuan_load_environment
else
  unset -f _kaixuan_load_environment
  return 1
fi
