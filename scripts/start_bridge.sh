#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/common_ros.sh
exec /usr/bin/python3 -m tools.exo_ros_bridge "$@"
