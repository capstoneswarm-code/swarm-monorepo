#!/usr/bin/env bash
set -e
cd "$HOME/Documents/swarm-mono"
source /opt/ros/kilted/setup.bash
source install/setup.bash
exec python3 src/cooperative_mapping/run.py "$@"
