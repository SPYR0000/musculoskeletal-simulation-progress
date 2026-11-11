#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
source scripts/common_ros.sh
exec /usr/bin/python3 -m tools.demo_exo_ros --cycles 3 --hold 1.5 "$@"
