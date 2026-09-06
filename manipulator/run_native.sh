#!/usr/bin/env bash
set -eo pipefail

MANIPULATOR_ROOT="$(cd "$(dirname "$0")" && pwd)"

source /opt/ros/jazzy/setup.bash
if [[ -f /home/dongwoo/install/setup.bash ]]; then
  source /home/dongwoo/install/setup.bash
fi
# Source the biped workspace last so its current package is not shadowed by an
# older biped_bike_robot that may also exist in /home/dongwoo/install.
if [[ -f /home/dongwoo/biped_bike_ws/install/setup.bash ]]; then
  source /home/dongwoo/biped_bike_ws/install/setup.bash
fi
set -u

export ROS_DOMAIN_ID=30
export PYTHONPATH="$MANIPULATOR_ROOT${PYTHONPATH:+:$PYTHONPATH}"
export PYTHONDONTWRITEBYTECODE=1
export ROS_LOG_DIR="$MANIPULATOR_ROOT/data/ros_logs"
mkdir -p "$ROS_LOG_DIR"

exec python3 "$MANIPULATOR_ROOT/launch_native.py" "$@"
