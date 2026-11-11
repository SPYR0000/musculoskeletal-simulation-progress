#!/usr/bin/env bash
# Install into this share's own environment. No changes to the source project.
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "${1:-}" == "--help" ]]; then
  echo 'Usage: bash scripts/setup_sim.sh [--resolve-only]'
  echo 'Requires uv. Uses Python 3.11 and a local .my_venv environment.'
  exit 0
fi
command -v uv >/dev/null || { echo 'uv not found. Install uv, then rerun this script.' >&2; exit 1; }
if [[ "${1:-}" == "--resolve-only" ]]; then
  uv pip compile pyproject.toml --python-version 3.11 --output-file /tmp/lab-simulation-resolved.txt
  exit 0
fi
if [[ ! -x .my_venv/bin/python ]]; then
  uv venv --python 3.11 .my_venv
fi
uv pip install --python .my_venv/bin/python -e .
.my_venv/bin/python -m scripts.check_environment sim
