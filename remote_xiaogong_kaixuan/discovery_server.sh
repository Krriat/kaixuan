#!/usr/bin/env bash
# Optional: run on the robot when the LAN blocks multicast discovery.
set -eo pipefail
source /opt/ros/humble/setup.bash
exec fastdds discovery --server-id 0 --udp-address 0.0.0.0 --udp-port 11811
