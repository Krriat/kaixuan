#!/usr/bin/env bash
set -eo pipefail

if [[ -f /opt/ros/humble/setup.bash ]]; then
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash
fi

set -u

# Keep this standalone viewer isolated from robot_description/TF publishers
# belonging to other ROS projects on the same machine or LAN.
export ROS_DOMAIN_ID="${OVERALL_URDF_DOMAIN_ID:-77}"

package_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
model_tmp="$(mktemp --suffix=.urdf)"
rsp_pid=""
joint_gui_pid=""

cleanup() {
  [[ -z "$joint_gui_pid" ]] || kill "$joint_gui_pid" 2>/dev/null || true
  [[ -z "$rsp_pid" ]] || kill "$rsp_pid" 2>/dev/null || true
  rm -f "$model_tmp"
}
trap cleanup EXIT INT TERM

# Resolve mesh URIs locally so this standalone viewer works without installation.
sed "s#package://tilt_kaixuan/#file://${package_dir}/#g" \
  "$package_dir/urdf/整体urdf.urdf" > "$model_tmp"

/opt/ros/humble/lib/robot_state_publisher/robot_state_publisher "$model_tmp" &
rsp_pid=$!

python3 "$package_dir/scripts/joint_angle_gui.py" &
joint_gui_pid=$!

rviz2 -d "$package_dir/config/rviz2.rviz"
