#!/usr/bin/env bash
set -eo pipefail
client_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
source "$client_root/scripts/environment.sh"
python3 "$client_root/scripts/check_dependencies.py"
echo "本机只启动 RViz；连接 ROS Domain $ROS_DOMAIN_ID 的机器人节点。"
exec ros2 launch "$client_root/launch/remote_rviz.launch.py" "$@"
