"""Operate the existing ROS/TCP instance: reset -> off -> right pulse -> off -> left pulse -> off."""

import argparse
import csv
import json
import math
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from tools.exo_device import off_actions, pulse_actions


class Demo(Node):
    def __init__(self, prefix, output, hold):
        super().__init__("myoassist_control_demo")
        self.publisher = self.create_publisher(String, prefix + "/command", 10)
        self.subscription = self.create_subscription(String, prefix + "/state", self.receive, 100)
        self.responses = {}
        self.seq = time.time_ns() // 1000  # strictly newer across ordinary restarts; within 2^53
        self.instance = None
        self.started = time.monotonic()
        self.stage = "discovery"
        self.hold = hold
        self.rows = []
        self.output = output
        output.parent.mkdir(parents=True, exist_ok=True)
        self.log = output.with_suffix(".jsonl").open("w")
        self.last_state = None
        self.error = None

    def receive(self, msg):
        reply = json.loads(msg.data)
        if reply.get("source") == "command":
            self.responses[reply.get("command_seq")] = reply
        if "state" not in reply:
            if not reply.get("ok"):
                self.error = reply.get("error")
            return
        s = reply["state"]
        if self.instance is not None and s["instance_id"] != self.instance:
            self.error = "Simulation instance changed; refusing to continue"
        self.last_state = s
        self.log.write(json.dumps({"wall_elapsed_s": time.monotonic() - self.started,
                                   "stage": self.stage, "response": reply}, allow_nan=False) + "\n")
        self.log.flush()

    def wait(self, condition, timeout=5):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.error:
                raise RuntimeError(str(self.error))
            if condition():
                return
        raise TimeoutError("No matching ROS feedback; check bridge, ROS_DOMAIN_ID and TCP server")

    def command(self, cmd, **fields):
        self.seq += 1
        msg = String()
        msg.data = json.dumps({"version": 1, "seq": self.seq, "cmd": cmd, **fields})
        self.publisher.publish(msg)
        self.wait(lambda: self.seq in self.responses)
        reply = self.responses.pop(self.seq)
        if not reply["ok"]:
            raise RuntimeError(str(reply["error"]))
        s = reply["state"]
        self.instance = self.instance or s["instance_id"]
        if s["instance_id"] != self.instance:
            raise RuntimeError("Simulation instance changed")
        row = {"stage": self.stage, "command_seq": reply["command_seq"], "tcp_seq": reply["seq"],
               "instance_id": s["instance_id"], "wall_elapsed_s": time.monotonic() - self.started,
               "sim_time_s": s["sim_time_s"], "status": s["status"]}
        for i, channel in enumerate(s["device_config"]["channels"]):
            name = channel["name"]
            row[name + "_ctrl"] = s["actual_ctrl"][i]
            row[name + "_torque_Nm"] = s["device_joint_torque_Nm"][i]
        for name, joint in s["joints"].items():
            row[name + "_rad"] = joint["angle_rad"]
            row[name + "_rad_s"] = joint["velocity_rad_s"]
        self.rows.append(row)
        angles = " ".join(f"{c['joint']}={s['joints'][c['joint']]['angle_rad']:+.4f}"
                          for c in s["device_config"]["channels"])
        print(f"{self.stage:16s} sim={s['sim_time_s']:.4f}s "
              f"ctrl={s['actual_ctrl']} torque_Nm={s['device_joint_torque_Nm']} {angles}", flush=True)
        return s

    def dwell(self):
        end = time.monotonic() + self.hold
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=min(0.02, max(0, end - time.monotonic())))
            if self.error:
                raise RuntimeError(str(self.error))
            if self.last_state and self.last_state["status"] != "ready":
                raise RuntimeError(f"Simulation paused/closed: {self.last_state['status']}")

    def run(self, cycles, torque):
        self.wait(lambda: self.publisher.get_subscription_count() > 0 and self.last_state is not None, 12)
        for cycle in range(cycles):
            self.stage = f"{cycle+1}:reset"
            s = self.command("reset")
            channels = s["device_config"]["channels"]
            if {c["name"] for c in channels} != {"right", "left"}:
                raise ValueError("Demo needs channels named right and left")
            config = s["device_config"]
            epoch = s["epoch"]
            for label, side in (("zero_assist", None), ("right_pulse", "right"),
                                ("right_off", None), ("left_pulse", "left"), ("final_off", None)):
                self.stage = f"{cycle+1}:{label}"
                action = pulse_actions(config, side, torque) if side else off_actions(config)
                # One physical control step per visible stage; holds pause physics, not fake slow steps.
                s = self.command("step", epoch=epoch, action=action, steps=1)
                if s["status"] != "ready" or s["terminated"] or s["truncated"]:
                    raise RuntimeError("Environment terminated; no further step commands sent")
                expected = [(c["demo_torque_nm"] if torque is None else torque) if c["name"] == side else 0 for c in channels]
                if any(abs(a-b) > 1e-8 for a, b in zip(s["device_joint_torque_Nm"], expected)):
                    raise RuntimeError("Actual joint torque differs from the command")
                viewer = s["viewer"]
                if viewer["enabled"]:
                    displayed = viewer["displayed_state"]
                    if (not viewer["shared_model_data"] or not viewer["running"]
                            or displayed["instance_id"] != self.instance
                            or displayed["sim_time_s"] != s["sim_time_s"]
                            or displayed["actual_ctrl"] != s["actual_ctrl"]):
                        raise RuntimeError("Viewer state is not synchronized with TCP state")
                self.dwell()
        print("Demo passed: right/left actual torques and mapped OFF verified; same instance:", self.instance)

    def save(self):
        self.log.close()
        if self.rows:
            with self.output.with_suffix(".csv").open("w", newline="") as f:
                writer = csv.DictWriter(f, fieldnames=list(self.rows[0]))
                writer.writeheader()
                writer.writerows(self.rows)
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            figure, axes = plt.subplots(3, 1, figsize=(10, 7), sharex=True, constrained_layout=True)
            x = [r["wall_elapsed_s"] for r in self.rows]
            for side in ("right", "left"):
                axes[0].step(x, [r[side + "_torque_Nm"] for r in self.rows], where="post", label=side)
            for channel in self.last_state["device_config"]["channels"]:
                name = channel["joint"]
                axes[1].step(x, [r[name + "_rad"] for r in self.rows], where="post", marker=".", label=name)
            axes[2].step(x, [r["sim_time_s"] for r in self.rows], where="post", marker=".")
            axes[0].set_ylabel("Device torque (Nm)")
            axes[1].set_ylabel("Joint angle (rad)")
            axes[2].set_ylabel("Simulation time (s)")
            axes[2].set_xlabel("Wall elapsed time (s); holds pause physics; reset restarts simulation time")
            axes[0].legend()
            axes[1].legend()
            for axis in axes:
                axis.grid(alpha=0.3)
            figure.suptitle("ROS -> TCP -> same MuJoCo instance | no gait controller")
            figure.savefig(self.output.with_suffix(".png"), dpi=150)
            plt.close(figure)
        print("Saved:", self.output.with_suffix(".jsonl"), self.output.with_suffix(".csv"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", default="/myoassist")
    parser.add_argument("--cycles", type=int, default=3)
    parser.add_argument("--hold", type=float, default=0.8, help="wall seconds to view each short physical step")
    parser.add_argument("--torque-nm", type=float, default=None, help="joint-coordinate Nm; default per-channel device configuration")
    parser.add_argument("--output", type=Path, default=Path("results/ros_live_demo"))
    args, ros_args = parser.parse_known_args()
    if args.cycles < 1 or not math.isfinite(args.hold) or args.hold < 0 or (args.torque_nm is not None and (not math.isfinite(args.torque_nm) or not 0 < abs(args.torque_nm) <= 10)):
        parser.error("Need cycles>=1, finite hold>=0, finite pulse 0<|torque-nm|<=10")
    rclpy.init(args=ros_args)
    node = Demo(args.prefix, args.output, args.hold)
    try:
        node.run(args.cycles, args.torque_nm)
    finally:
        # Explicit mapped closure without further integration, even after an aborted pulse.
        # Reset is safe and keeps the reusable server running; TCP watchdog remains the fallback.
        try:
            if node.last_state is not None and node.instance is not None and node.last_state["instance_id"] != node.instance:
                print("Instance changed: no cleanup command sent to the replacement simulation")
            else:
                node.stage = "cleanup_reset_off"
                node.error = None
                node.command("reset")
        except Exception as exc:
            print("Cleanup reset unavailable; server watchdog will close outputs:", exc)
        finally:
            node.save()
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        raise SystemExit(130)
