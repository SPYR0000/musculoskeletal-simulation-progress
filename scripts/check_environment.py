"""Read-only environment and local module checks for sim / ros / ms700."""
import argparse
import importlib
import importlib.metadata
import json
import sys
from pathlib import Path


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('mode', choices=['sim', 'ros', 'ms700'])
    args = p.parse_args()
    packages = {
        'sim': ['numpy', 'mujoco', 'dm-control', 'gymnasium', 'myosuite', 'myo-sim', 'assist-sim', 'myoassist-terrains'],
        'ros': ['numpy', 'matplotlib'],
        'ms700': ['numpy', 'mujoco', 'torch', 'stable-baselines3', 'gymnasium', 'msgym'],
    }[args.mode]
    versions = {n: importlib.metadata.version(n) for n in packages}
    if args.mode == 'sim':
        assert sys.version_info[:2] == (3, 11), 'Simulation requires Python 3.11'
        for n in ('myoassist_utils.compose', 'rl_train.envs.myoassist_leg_base', 'tools.exo_device'):
            module = importlib.import_module(n)
            expected = Path(__file__).resolve().parents[1]
            assert Path(module.__file__).resolve().is_relative_to(expected), f'Loaded external runtime: {module.__file__}'
    elif args.mode == 'ros':
        import rclpy
        from std_msgs.msg import String
        assert sys.version_info[:2] == (3, 10), 'Humble bridge uses system Python 3.10'
    print(json.dumps({'mode': args.mode, 'python': sys.version.split()[0], 'versions': versions, 'imports_passed': True}, indent=2))


if __name__ == '__main__':
    main()
