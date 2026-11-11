"""Launch isolated real ROS/TCP processes, run and validate the matched comparison."""

import argparse
from datetime import datetime
import json
import os
from pathlib import Path
import signal
import subprocess
import time
from zoneinfo import ZoneInfo

from tools.test_exo_tcp import ROOT, RunningServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", action="store_true")
    parser.add_argument("--hold", type=float, default=0.02, help="wall seconds to observe each control step")
    parser.add_argument("--output", type=Path, default=ROOT / "results" / datetime.now(ZoneInfo("Asia/Shanghai")).strftime("exo_comparison_check_%Y%m%d_%H%M%S"))
    args = parser.parse_args()
    output = args.output.resolve()
    server = RunningServer(watchdog=3, window=args.window)
    bridge = comparison = None
    prefix = f"/exo_comparison_test_{os.getpid()}"
    bridge_log = (server.directory / "bridge.txt").open("w+")
    client_log = (server.directory / "comparison.txt").open("w+")
    try:
        bridge = subprocess.Popen(
            ["/usr/bin/python3", "-m", "tools.exo_ros_bridge", "--port", str(server.port), "--prefix", prefix],
            cwd=ROOT, stdout=bridge_log, stderr=subprocess.STDOUT)
        comparison = subprocess.Popen(
            ["/usr/bin/python3", "-m", "tools.compare_exo_ros", "--prefix", prefix,
             "--output", str(output), "--hold", str(args.hold)],
            cwd=ROOT, stdout=client_log, stderr=subprocess.STDOUT)
        comparison.wait(timeout=45 + 78 * max(0, args.hold))
        client_log.seek(0)
        if comparison.returncode != 0:
            raise AssertionError(client_log.read())
        summary = json.loads((output / "summary.json").read_text())
        assert summary["passed"]
        assert summary["trials"] == 6 and summary["steps_per_trial"] == 12
        assert summary["initial_state_identical"] and summary["actual_muscle_inputs_identical"]
        assert summary["pre_pulse_states_identical"] and summary["max_repeat_state_error"] <= 1e-12
        samples = json.loads((output / "samples.json").read_text())
        assert len(samples) == 6 * 13
        server_records = [json.loads(line) for line in server.log.read_text().splitlines()]
        responses = {r["response"]["seq"]: r["response"] for r in server_records if r["event"] == "request"}
        for sample in samples:
            state = sample["state"]
            assert state == responses[sample["tcp_seq"]]["state"]
            assert state["viewer"]["enabled"] == args.window
            assert not state["terminated"] and not state["truncated"]
        # Confirm a nonzero response exists, without declaring its sign beneficial.
        for case, side in (("right_pulse", "r"), ("left_pulse", "l")):
            assert summary["metrics"][case][f"ankle_angle_{side}"]["max_abs_angle_delta_rad"] > 1e-8
        for name in ("states.csv", "deltas.csv", "joint_deltas.png", "device_torques.png", "ros_feedback.jsonl"):
            assert (output / name).stat().st_size > 0
        (output / "server.jsonl").write_text(server.log.read_text())
        (output / "command_output.txt").write_text(client_log.read())
        (output / "integration_checks.json").write_text(json.dumps({
            "passed": True, "window_tested": args.window, "samples_checked_against_server": len(samples),
            "separate_ROS_and_simulation_processes": True,
            "equal_full_initial_state": True, "same_step_count": True,
            "identical_muscle_inputs": True, "deterministic_repeats": True,
            "not_an_assistance_efficacy_test": True,
        }, indent=2) + "\n")
        print("Three-arm ROS/MuJoCo comparison PASSED:", output)
        print("Repeat maximum full-state difference:", summary["max_repeat_state_error"])
        print(json.dumps(summary["metrics"], indent=2))
    finally:
        if comparison is not None and comparison.poll() is None:
            comparison.terminate()
            try:
                comparison.wait(timeout=5)
            except subprocess.TimeoutExpired:
                comparison.kill()
                comparison.wait()
        if bridge is not None and bridge.poll() is None:
            bridge.send_signal(signal.SIGINT)
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait()
        bridge_log.close()
        client_log.close()
        server.close()


if __name__ == "__main__":
    main()
