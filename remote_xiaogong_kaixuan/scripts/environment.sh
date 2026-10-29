#!/usr/bin/env bash
# Source this file; ROS setup scripts are not safe with nounset enabled.
set -eo pipefail
client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
# Preserve the caller's standard ROS variable before sourcing setup/config files.
# In particular, an explicit ROS_DOMAIN_ID=0 is a valid override, not "unset".
client_requested_domain="${ROS_DOMAIN_ID:-${KAIXUAN_ROS_DOMAIN_ID:-}}"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash"
  return 1
fi
source /opt/ros/humble/setup.bash
source "$client_root/network.env"
client_requested_domain="${client_requested_domain:-$KAIXUAN_ROS_DOMAIN_ID}"
if [[ ! "$client_requested_domain" =~ ^[0-9]{1,3}$ ]] || (( 10#$client_requested_domain > 232 )); then
  echo "ROS_DOMAIN_ID / KAIXUAN_ROS_DOMAIN_ID 必须为 0～232 的整数。"
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
