"""Check distributed file hashes; this does not rerun or validate the physics."""
from pathlib import Path
import hashlib
import json
import sys


def main():
    root = Path(__file__).resolve().parents[1]
    manifest = json.loads((root / 'MANIFEST.json').read_text(encoding='utf-8'))
    failures = []
    for name, expected in manifest['sha256'].items():
        path = root / name
        if not path.is_file():
            failures.append('Missing: ' + name)
        elif hashlib.sha256(path.read_bytes()).hexdigest() != expected:
            failures.append('Changed: ' + name)
    if failures:
        print('\n'.join(failures), file=sys.stderr)
        return 1
    summary = json.loads((root / 'results/ms700_summary.json').read_text(encoding='utf-8'))
    print(f"PASS: {len(manifest['sha256'])} distributed files match SHA256.")
    print('Stored results (not a new simulation):')
    for run in summary['runs']:
        print(f"  {run['experiment_id']}: {run['actual_time_s']:.2f} s; {run['reason']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
