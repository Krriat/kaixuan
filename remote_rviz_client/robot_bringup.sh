#!/usr/bin/env bash
# Run this script on the robot host, not on the RViz laptop.
set -eo pipefail
client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
robot_workspace="${KAIXUAN_ROBOT_WORKSPACE:-/home/agxorin/cxl/kaixuan}"
if [[ ! -f "$robot_workspace/install/setup.bash" ]]; then
  echo "此脚本应在机器人上运行。未找到 $robot_workspace/install/setup.bash"
  exit 1
fi
source "$client_root/scripts/environment.sh"
source "$robot_workspace/install/setup.bash"
echo "机器人端：ROS Domain $ROS_DOMAIN_ID；默认只启用左臂。"
exec ros2 launch kaixuan3_config moveit_bringup.launch.py \
  use_real_hardware:=true use_rviz:=false active_arm:=left "$@"
