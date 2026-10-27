#!/usr/bin/env bash
# Source this file; ROS setup scripts are not safe with nounset enabled.
set -eo pipefail
client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "未找到 ROS 2 Humble：/opt/ros/humble/setup.bash"
  return 1
fi
source /opt/ros/humble/setup.bash
source "$client_root/network.env"
if [[ ! "$KAIXUAN_ROS_DOMAIN_ID" =~ ^[0-9]+$ ]] || (( 10#$KAIXUAN_ROS_DOMAIN_ID > 232 )); then
  echo "KAIXUAN_ROS_DOMAIN_ID 必须为 0～232 的整数。"
  return 1
fi
export ROS_DOMAIN_ID="$KAIXUAN_ROS_DOMAIN_ID"
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE="$client_root/config/dds_udp.xml"
export FASTDDS_DEFAULT_PROFILES_FILE="$FASTRTPS_DEFAULT_PROFILES_FILE"
if [[ -n "$KAIXUAN_DISCOVERY_SERVER" ]]; then
  export ROS_DISCOVERY_SERVER="$KAIXUAN_DISCOVERY_SERVER"
else
  unset ROS_DISCOVERY_SERVER
fi
