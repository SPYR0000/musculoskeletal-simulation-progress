"""System Python ROS topic -> bridge -> real TCP simulator -> ROS feedback test."""

import json
import argparse
import os
from pathlib import Path
import signal
import subprocess
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from tools.exo_tcp_client import ExoClient
from tools.test_exo_tcp import ROOT, RunningServer


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", action="store_true")
    parser.add_argument("--demo", action="store_true", help="also exercise the user-facing ROS demonstration")
    args = parser.parse_args()
    rclpy.init()
    server = RunningServer(watchdog=1.5, window=args.window)
    bridge_output = (server.directory / "bridge.txt").open("w+")
    bridge = None
    node = Node("myoassist_integration_probe")
    prefix = f"/myoassist_test_{os.getpid()}"
    received = []
    subscription = node.create_subscription(String, prefix + "/state", lambda m: received.append(json.loads(m.data)), 100)
    publisher = node.create_publisher(String, prefix + "/command", 10)
    evidence = []

    def wait_for(predicate, timeout=8):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            for msg in received:
                if predicate(msg):
                    return msg
            rclpy.spin_once(node, timeout_sec=0.05)
        raise AssertionError(f"ROS response timeout; last received={received[-2:]}")

    seq = 0

    def command(cmd, **fields):
        nonlocal seq
        seq += 1
        msg = String()
        msg.data = json.dumps({"version": 1, "seq": seq, "cmd": cmd, **fields})
        publisher.publish(msg)
        reply = wait_for(lambda r: r.get("source") == "command" and r.get("command_seq") == seq)
        assert reply["ok"], reply
        evidence.append(reply)
        return reply

    try:
        bridge = subprocess.Popen(
            ["/usr/bin/python3", "-m", "tools.exo_ros_bridge", "--port", str(server.port), "--prefix", prefix],
            cwd=ROOT, env=os.environ.copy(), stdout=bridge_output, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 12
        while publisher.get_subscription_count() == 0 and time.monotonic() < deadline:
            assert bridge.poll() is None, "ROS bridge exited"
            rclpy.spin_once(node, timeout_sec=0.05)
        assert publisher.get_subscription_count() > 0, "ROS command subscriber not discovered"
        wait_for(lambda r: r.get("source") == "poll" and r.get("ok"))
        for action, expected in (([0.8, 1], [-0.1, 0]), ([1, 0.6], [0, -0.2]),
                                 ([0, 1], [-0.5, 0]), ([-1, 1], [-1, 0]),
                                 ([2, -2], [0, -1]), ([1, 1], [0, 0])):
            reset = command("reset")
            reply = command("step", epoch=reset["state"]["epoch"], action=action, steps=1)
            state = reply["state"]
            if args.window:
                v = state["viewer"]
                assert v["enabled"] and v["shared_model_data"] and v["running"]
                shown = v["displayed_state"]
                for key in ("instance_id", "sim_time_s", "actual_ctrl", "device_joint_torque_Nm", "joints"):
                    assert shown[key] == state[key], (key, shown, state)
            assert abs(state["sim_time_s"] - 1 / 30) < 1e-9
            assert len(state["joints"]) == 6
            for actual, u, force in zip(state["actual_ctrl"], expected, state["actuator_force"]):
                assert abs(actual - u) < 1e-10
                assert abs(force - 100 * u) < 1e-10
        reset = command("reset")
        command("step", epoch=reset["state"]["epoch"], action=[0.8, 0.8], steps=1)
        timeout = wait_for(lambda r: r.get("source") == "poll" and r.get("state", {}).get("status") == "watchdog_timeout")
        assert timeout["state"]["actual_ctrl"] == [0, 0]
        assert timeout["state"]["actuator_force"] == [0, 0]
        evidence.append(timeout)

        # Compare ROS feedback against the simulator's logged response to the same TCP seq.
        logs = [json.loads(line) for line in server.log.read_text().splitlines()]
        responses = {r["response"]["seq"]: r["response"] for r in logs if r["event"] == "request"}
        for response in evidence:
            assert response["state"] == responses[response["seq"]]["state"]

        if args.demo:
            demo = subprocess.Popen(
                ["/usr/bin/python3", "-m", "tools.demo_exo_ros", "--prefix", prefix,
                 "--cycles", "1", "--hold", "0.2", "--output", str(ROOT / "results/ros_live_demo_test")],
                cwd=ROOT, env=os.environ.copy())
            deadline = time.monotonic() + 15
            while demo.poll() is None and time.monotonic() < deadline:
                rclpy.spin_once(node, timeout_sec=0.05)
            if demo.poll() is None:
                demo.terminate()
                demo.wait(timeout=5)
                raise AssertionError("Demonstration timed out")
            assert demo.returncode == 0, "ROS demonstration failed"
            import csv
            with (ROOT / "results/ros_live_demo_test.csv").open() as f:
                demo_rows = list(csv.DictReader(f))
            assert any(r["stage"].endswith("right_pulse") and abs(float(r["right_torque_Nm"]) + 5) < 1e-8
                       and float(r["left_torque_Nm"]) == 0 for r in demo_rows)
            assert any(r["stage"].endswith("left_pulse") and abs(float(r["left_torque_Nm"]) + 5) < 1e-8
                       and float(r["right_torque_Nm"]) == 0 for r in demo_rows)
            assert len({r["instance_id"] for r in demo_rows}) == 1

        # Node exit closes TCP, and the server independently applies mapped OFF.
        bridge.send_signal(signal.SIGINT)
        bridge.wait(timeout=5)
        with ExoClient(server.port) as client:
            disconnected = client.request("ping")
            assert disconnected["state"]["status"] == "disconnected"
            assert disconnected["state"]["actual_ctrl"] == [0, 0]
            assert client.request("stop")["ok"]
        server.process.wait(timeout=5)
        result = {
            "passed": True, "ros_python": "/usr/bin/python3 (3.10)",
            "simulation_python": os.environ.get("MYOASSIST_SIM_PYTHON", str(ROOT / ".my_venv/bin/python")),
            "ROS_DOMAIN_ID": os.environ.get("ROS_DOMAIN_ID"),
            "ROS_LOCALHOST_ONLY": os.environ.get("ROS_LOCALHOST_ONLY"), "topic_prefix": prefix,
            "checks": ["DDS topic discovery", "six independent/mapping/clipping probes",
                       "real simulation time and joint feedback", "watchdog zero output via ROS feedback",
                       "ROS feedback equals TCP simulator log", "bridge exit disconnect closure", "server stop"],
            "responses": evidence, "after_bridge_exit": disconnected,
            "viewer_tested": args.window, "user_demo_tested": args.demo,
        }
        output = ROOT / ("results/ros_live_validation.json" if args.window else "results/ros_tcp_validation.json")
        output.parent.mkdir(exist_ok=True)
        output.write_text(json.dumps(result, indent=2) + "\n")
        output.with_suffix(".server.jsonl").write_text(server.log.read_text())
        print(f"ROS -> TCP -> MuJoCo -> ROS validation passed: {output}")
    finally:
        if bridge is not None and bridge.poll() is None:
            bridge.terminate()
            try:
                bridge.wait(timeout=5)
            except subprocess.TimeoutExpired:
                bridge.kill()
                bridge.wait()
        bridge_output.flush()
        if bridge is not None and bridge.returncode not in (None, 0):
            bridge_output.seek(0)
            print(bridge_output.read())
        bridge_output.close()
        node.destroy_subscription(subscription)
        node.destroy_node()
        rclpy.shutdown()
        server.close()


if __name__ == "__main__":
    main()
