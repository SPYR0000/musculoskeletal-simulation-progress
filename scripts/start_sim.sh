#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1
export MPLCONFIGDIR="${MPLCONFIGDIR:-/tmp/lab-simulation-mpl}"
export MESA_SHADER_CACHE_DIR="${MESA_SHADER_CACHE_DIR:-/tmp/lab-simulation-mesa}"
sim_python="${MYOASSIST_SIM_PYTHON:-$PWD/.my_venv/bin/python}"
[[ -x "$sim_python" ]] || { echo 'Run bash scripts/setup_sim.sh first.' >&2; exit 1; }
# Pass --no-window for headless use; remaining options go to the server.
if [[ "${1:-}" == '--no-window' ]]; then
  shift
  exec "$sim_python" -m tools.exo_tcp_server --watchdog 10 --log results/ros_live_server.jsonl "$@"
fi
exec "$sim_python" -m tools.exo_tcp_server --window --watchdog 10 --log results/ros_live_server.jsonl "$@"
