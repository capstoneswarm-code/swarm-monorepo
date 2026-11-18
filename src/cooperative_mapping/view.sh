#!/usr/bin/env bash
set -e
cd "$HOME/Documents/swarm-mono"
source /opt/ros/kilted/setup.bash
source install/setup.bash
robot="$(hostname -s)"
case "$robot" in
  swarm1) export ROS_DOMAIN_ID=79 ;;
  swarm2) export ROS_DOMAIN_ID=78 ;;
  *) echo "Expected hostname swarm1 or swarm2" >&2; exit 1 ;;
esac
exec python3 "src/cooperative_mapping/runtime/$robot/web_map.py" "$@"
