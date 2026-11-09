#!/usr/bin/env bash
set -e

workspace_dir="$HOME/Documents/swarm-mono"
cd -- "$workspace_dir"
source /opt/ros/kilted/setup.bash
if [[ ! -f "$workspace_dir/install/setup.bash" ]]; then
    printf 'Missing workspace setup: %s/install/setup.bash\n' "$workspace_dir" >&2
    exit 1
fi
source "$workspace_dir/install/setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-78}"

if ! ros2 pkg prefix sllidar_ros2 >/dev/null; then
    printf 'sllidar_ros2 is missing from the sourced workspace. Build it in %s first.\n' "$workspace_dir" >&2
    exit 1
fi
exec ros2 launch "$workspace_dir/src/wheel_ekf_demo/nav2.launch.py" "$@"
