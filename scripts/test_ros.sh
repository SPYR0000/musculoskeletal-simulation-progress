#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/common_ros.sh
# Use an independent DDS domain for automated validation.
export ROS_DOMAIN_ID="${MYOASSIST_TEST_DOMAIN_ID:-73}"
exec /usr/bin/python3 -m tools.test_exo_ros "$@"
