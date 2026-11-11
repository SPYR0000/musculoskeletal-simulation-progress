#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "$0")/.."
if [[ "${1:-}" == '--help' ]]; then
  echo 'Usage: bash scripts/setup_ms700.sh'
  echo 'Requires uv, git and curl; creates workspace/ and .venv_ms700/.'
  exit 0
fi
for tool in uv git curl; do
  command -v "$tool" >/dev/null || { echo "Missing $tool" >&2; exit 1; }
done
source_dir=workspace/sources/msgym
source_commit=ad3aac166dab3577d7d16b4a03faccef94c7aac7
mkdir -p workspace/sources workspace/assets
if [[ ! -e "$source_dir" ]]; then
  git clone --no-checkout https://github.com/LNSGroup/msgym.git "$source_dir"
  git -C "$source_dir" checkout "$source_commit"
  git -C "$source_dir" submodule update --init --recursive
fi
[[ "$(git -C "$source_dir" rev-parse HEAD)" == "$source_commit" ]] || { echo 'msgym commit mismatch' >&2; exit 1; }
[[ -z "$(git -C "$source_dir" status --porcelain)" ]] || { echo 'msgym has local changes; refusing to continue' >&2; exit 1; }
[[ "$(git -C "$source_dir/msgym/MS-Human-700" rev-parse HEAD)" == 2d686957aefd5739cf4d2859a6acfd94c8f84400 ]] || { echo 'Model submodule mismatch' >&2; exit 1; }
for name in LocomotionFull LocomotionLegs; do
  target="workspace/assets/$name.zip"
  if [[ ! -f "$target" ]]; then
    curl -fL --retry 3 "https://github.com/LNSGroup/msgym/releases/download/Checkpoints/$name.zip" -o "$target.download"
    mv "$target.download" "$target"
  fi
done
if [[ ! -x .venv_ms700/bin/python ]]; then
  uv venv --python 3.12 .venv_ms700
fi
.venv_ms700/bin/python - <<'PY'
from pathlib import Path
import hashlib, json, zipfile
base = Path('workspace')
meta = json.loads(Path('results/ms700_summary.json').read_text())
for name in ('LocomotionFull', 'LocomotionLegs'):
    archive = base / 'assets' / (name + '.zip')
    expected = meta['asset_sha256']['assets/' + archive.name]
    if hashlib.sha256(archive.read_bytes()).hexdigest() != expected:
        raise SystemExit('SHA256 mismatch: ' + str(archive))
    if not (base / 'assets' / name).exists():
        with zipfile.ZipFile(archive) as z:
            z.extractall(base / 'assets')
for relative, expected in meta['asset_sha256'].items():
    path = base / relative
    if not path.is_file() or hashlib.sha256(path.read_bytes()).hexdigest() != expected:
        raise SystemExit('Asset verification failed: ' + relative)
print('Official model/normalization/config assets verified.')
PY
uv pip install --python .venv_ms700/bin/python torch==2.6.0+cpu --index-url https://download.pytorch.org/whl/cpu
uv pip install --python .venv_ms700/bin/python -r requirements-ms700.txt
uv pip install --python .venv_ms700/bin/python --no-deps -e "$source_dir"
.venv_ms700/bin/python -m scripts.check_environment ms700
