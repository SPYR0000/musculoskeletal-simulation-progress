#!/usr/bin/env bash
# Sourced by ROS entry points. Keep ROS on system Python 3.10.
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo 'ROS 2 Humble missing: /opt/ros/humble/setup.bash' >&2
  exit 1
fi
# ROS setup scripts may reference unset variables.
set +u
source /opt/ros/humble/setup.bash
set -u
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"
export PYTHONDONTWRITEBYTECODE=1
export OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/lab-simulation-mpl}"
