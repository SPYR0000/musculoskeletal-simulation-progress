"""Matched baseline/right/left short rollouts via the existing ROS/TCP control chain."""

import argparse
import csv
from datetime import datetime
import hashlib
import json
import math
from pathlib import Path
from zoneinfo import ZoneInfo

import numpy as np
import rclpy

from tools.demo_exo_ros import Demo
from tools.exo_device import pulse_actions, off_actions

CASES = ("baseline", "right_pulse", "left_pulse")


class Comparison(Demo):
    def __init__(self, args):
        args.output.mkdir(parents=True, exist_ok=True)
        super().__init__(args.prefix, args.output / "ros_feedback", args.hold)
        self.args = args
        self.samples = []
        self.reference_initial = None
        self.config = None
        self.dt = None
        self.summary = {"passed": False, "interpretation": "Short-time dynamical sensitivity; not assistance efficacy"}

    def capture(self, case, repetition, step, state, expected):
        if (state["status"] != "ready" or state["terminated"] or state["truncated"]
                or not math.isclose(state["sim_time_s"], step * self.dt, abs_tol=1e-9)):
            raise RuntimeError("Unequal/terminated horizon: comparison aborted; shorten the configured rollout")
        if state["device_config"] != self.config or state["control_dt_s"] != self.dt:
            raise RuntimeError("Configuration/timestep changed during the experiment")
        np.testing.assert_allclose(state["device_joint_torque_Nm"], expected, atol=1e-9, rtol=0)
        if step:
            device_ids = set(state["device_actuator_ids"])
            muscle_action = self.config["muscle_action"]
            if any(value != muscle_action for i, value in enumerate(state["env_step_action"]) if i not in device_ids):
                raise RuntimeError("Muscle input differs from the fixed baseline")
        if state["viewer"]["enabled"]:
            viewer = state["viewer"]
            if not viewer["shared_model_data"] or not viewer["running"]:
                raise RuntimeError("Viewer is not sharing the active simulation")
            for key in ("instance_id", "sim_time_s", "actual_ctrl", "joints"):
                if viewer["displayed_state"][key] != state[key]:
                    raise RuntimeError("Viewer and ROS response differ")
        self.samples.append({"case": case, "repetition": repetition, "step": step,
                             "command_seq": self.rows[-1]["command_seq"],
                             "tcp_seq": self.rows[-1]["tcp_seq"], "state": state})

    def run_comparison(self):
        self.wait(lambda: self.publisher.get_subscription_count() > 0 and self.last_state is not None, 12)
        for repetition in range(1, self.args.repeats + 1):
            for case in CASES:
                self.stage = f"{repetition}:{case}:reset"
                state = self.command("reset")
                if "model_state" not in state:
                    raise RuntimeError("Restart the TCP server: experiment needs current full-state feedback")
                if self.config is None:
                    self.config, self.dt = state["device_config"], state["control_dt_s"]
                channels = self.config["channels"]
                if {c["name"] for c in channels} != {"right", "left"}:
                    raise ValueError("Expected right/left named channels")
                initial = {"model_state": state["model_state"], "seed": state["seed"],
                           "sim_time_s": state["sim_time_s"], "config": state["device_config"]}
                if self.reference_initial is None:
                    self.reference_initial = initial
                if initial != self.reference_initial:
                    raise RuntimeError("Reset qpos/qvel/activation/ctrl/seed/config are not identical")
                epoch = state["epoch"]
                self.capture(case, repetition, 0, state, [0, 0])
                for step in range(1, self.args.steps + 1):
                    active = (case != "baseline" and self.args.pulse_first <= step
                              < self.args.pulse_first + self.args.pulse_steps)
                    side = case.removesuffix("_pulse") if active else None
                    expected = [(c["demo_torque_nm"] if self.args.torque_nm is None else self.args.torque_nm)
                                if c["name"] == side else 0 for c in channels]
                    action = pulse_actions(self.config, side, self.args.torque_nm) if active else off_actions(self.config)
                    self.stage = f"{repetition}:{case}:{step}"
                    state = self.command("step", epoch=epoch, action=action, steps=1)
                    self.capture(case, repetition, step, state, expected)
                    self.dwell()
        self.analyze()

    def analyze(self):
        trajectories = {(rep, case): [s for s in self.samples if s["case"] == case and s["repetition"] == rep]
                        for rep in range(1, self.args.repeats + 1) for case in CASES}
        if any(len(t) != self.args.steps + 1 for t in trajectories.values()):
            raise RuntimeError("Incomplete trials cannot form a matched comparison")
        max_repeat_error = 0.0
        for case in CASES:
            for rep in range(2, self.args.repeats + 1):
                for first, repeated in zip(trajectories[1, case], trajectories[rep, case]):
                    for key in ("qpos", "qvel", "activation", "ctrl"):
                        a, b = first["state"]["model_state"][key], repeated["state"]["model_state"][key]
                        np.testing.assert_allclose(a, b, atol=1e-12, rtol=0)
                        max_repeat_error = max(max_repeat_error, float(np.max(np.abs(np.array(a) - b))))
        flat, delta = [], []
        metrics = {}
        for (rep, case), trajectory in trajectories.items():
            baseline = trajectories[rep, "baseline"]
            for sample, base in zip(trajectory, baseline):
                s, b = sample["state"], base["state"]
                # Prior to the pulse all full integration state must equal the matched baseline.
                if sample["step"] < self.args.pulse_first:
                    for key in ("qpos", "qvel", "activation", "ctrl"):
                        np.testing.assert_allclose(s["model_state"][key], b["model_state"][key], atol=1e-12, rtol=0)
                device_ids = set(s["device_actuator_ids"])
                for i, (value, reference) in enumerate(zip(s["model_state"]["ctrl"], b["model_state"]["ctrl"])):
                    if i not in device_ids and value != reference:
                        raise RuntimeError("Actual muscle excitation differs between trials")
                row = {k: sample[k] for k in ("case", "repetition", "step", "command_seq", "tcp_seq")}
                row.update(sim_time_s=s["sim_time_s"], instance_id=s["instance_id"])
                d = {k: row[k] for k in ("case", "repetition", "step", "sim_time_s")}
                for i, c in enumerate(self.config["channels"]):
                    for name, key in (("ctrl", "actual_ctrl"), ("torque_Nm", "device_joint_torque_Nm"),
                                      ("actuator_force", "actuator_force")):
                        row[c["name"] + "_" + name] = s[key][i]
                for name in s["joints"]:
                    for key, unit in (("angle_rad", "angle_rad"), ("velocity_rad_s", "velocity_rad_s")):
                        row[name + "_" + unit] = s["joints"][name][key]
                        d[name + "_delta_" + unit] = s["joints"][name][key] - b["joints"][name][key]
                flat.append(row)
                delta.append(d)
        for case in CASES[1:]:
            metrics[case] = {}
            subset = [d for d in delta if d["case"] == case and d["repetition"] == 1]
            for name in self.config["state_joints"]:
                metrics[case][name] = {
                    "max_abs_angle_delta_rad": max(abs(d[name + "_delta_angle_rad"]) for d in subset),
                    "max_abs_velocity_delta_rad_s": max(abs(d[name + "_delta_velocity_rad_s"]) for d in subset),
                    "final_angle_delta_rad": subset[-1][name + "_delta_angle_rad"],
                    "final_velocity_delta_rad_s": subset[-1][name + "_delta_velocity_rad_s"],
                }
        self.flat, self.delta = flat, delta
        self.summary.update(
            passed=True, instance_id=self.instance, seed=self.reference_initial["seed"],
            config=self.config, steps_per_trial=self.args.steps, dt_s=self.dt,
            duration_per_trial_s=self.args.steps * self.dt, repeats=self.args.repeats,
            trials=3 * self.args.repeats, pulse_first_step=self.args.pulse_first,
            pulse_steps=self.args.pulse_steps,
            pulse_torque_Nm={c["name"]: c["demo_torque_nm"] if self.args.torque_nm is None else self.args.torque_nm
                             for c in self.config["channels"]},
            pulse_interval_s=[(self.args.pulse_first - 1) * self.dt,
                              (self.args.pulse_first - 1 + self.args.pulse_steps) * self.dt],
            initial_state_identical=True, actual_muscle_inputs_identical=True,
            pre_pulse_states_identical=True, max_repeat_state_error=max_repeat_error,
            repeated_deterministic_runs_are_not_independent_subjects=True,
            initial_state_sha256=hashlib.sha256(json.dumps(self.reference_initial, sort_keys=True).encode()).hexdigest(),
            metrics=metrics)

    def save(self):
        self.log.close()
        directory = self.args.output
        (directory / "samples.json").write_text(json.dumps(self.samples, indent=2) + "\n")
        (directory / "summary.json").write_text(json.dumps(self.summary, indent=2) + "\n")
        if self.summary["passed"]:
            for name, rows in (("states.csv", self.flat), ("deltas.csv", self.delta)):
                with (directory / name).open("w", newline="") as f:
                    writer = csv.DictWriter(f, fieldnames=list(rows[0]))
                    writer.writeheader()
                    writer.writerows(rows)
            self.plot()
        print("Comparison", "PASSED" if self.summary["passed"] else "INCOMPLETE", ":", directory.resolve())

    def plot(self):
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        joints = self.config["state_joints"]
        fig, axes = plt.subplots(len(joints), 2, figsize=(12, 2.1 * len(joints)), sharex=True, constrained_layout=True)
        for i, joint in enumerate(joints):
            for col, (field, unit) in enumerate((("angle_rad", "rad"), ("velocity_rad_s", "rad/s"))):
                ax = axes[i, col]
                for case in CASES:
                    rows = [r for r in self.delta if r["case"] == case and r["repetition"] == 1]
                    ax.plot([r["sim_time_s"] for r in rows], [r[joint + "_delta_" + field] for r in rows], label=case)
                ax.axvspan(*self.summary["pulse_interval_s"], color="gray", alpha=0.12)
                ax.set_ylabel(f"{joint}\nDelta ({unit})")
                ax.grid(alpha=0.3)
                ax.ticklabel_format(axis="y", style="sci", scilimits=(-2, 2))
        axes[0, 0].legend(fontsize=8)
        for ax in axes[-1]:
            ax.set_xlabel("Simulation time (s); gray = pulse interval")
        fig.suptitle("Matched short rollouts: intervention minus baseline | no assistance efficacy claim")
        fig.savefig(self.args.output / "joint_deltas.png", dpi=150)
        plt.close(fig)
        fig, axes = plt.subplots(2, 1, figsize=(9, 5), sharex=True, constrained_layout=True)
        for ax, side in zip(axes, ("right", "left")):
            for case in CASES:
                rows = [r for r in self.flat if r["case"] == case and r["repetition"] == 1 and r["step"] > 0]
                # Each output sample describes the control applied over the PRECEDING interval.
                x = [0] + [r["sim_time_s"] for r in rows]
                y = [rows[0][side + "_torque_Nm"]] + [r[side + "_torque_Nm"] for r in rows]
                ax.step(x, y, where="pre", label=case, alpha=0.8)
            ax.set_ylabel(side + " torque (Nm)")
            ax.legend(fontsize=8, loc="lower right")
            ax.grid(alpha=0.3)
        axes[-1].set_xlabel("Simulation time (s)")
        fig.suptitle("Measured device output; repeat 1")
        fig.savefig(self.args.output / "device_torques.png", dpi=150)
        plt.close(fig)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--prefix", default="/myoassist")
    parser.add_argument("--steps", type=int, default=12)
    parser.add_argument("--pulse-first", type=int, default=4, help="1-based first pulse step")
    parser.add_argument("--pulse-steps", type=int, default=3)
    parser.add_argument("--torque-nm", type=float, default=None, help="optional joint-coordinate Nm override")
    parser.add_argument("--repeats", type=int, default=2)
    parser.add_argument("--hold", type=float, default=0.05)
    parser.add_argument("--output", type=Path, default=Path("results") / datetime.now(ZoneInfo("Asia/Shanghai")).strftime("exo_comparison_%Y%m%d_%H%M%S"))
    args, ros_args = parser.parse_known_args()
    if (not 1 <= args.steps <= 60 or args.pulse_first < 1 or args.pulse_steps < 1
            or args.pulse_first + args.pulse_steps - 1 > args.steps or args.repeats < 1
            or (args.torque_nm is not None and (not math.isfinite(args.torque_nm) or not 0 < abs(args.torque_nm) <= 10))
            or not math.isfinite(args.hold) or args.hold < 0):
        parser.error("Invalid short horizon, pulse, repetition, torque or hold")
    if args.output.exists() and any(args.output.iterdir()):
        parser.error("Output directory is not empty; choose a new path to preserve existing data")
    rclpy.init(args=ros_args)
    node = Comparison(args)
    try:
        node.run_comparison()
    except BaseException as exc:
        node.summary.update(passed=False, error=f"{type(exc).__name__}: {exc}")
        raise
    finally:
        try:
            if node.instance is not None and node.last_state and node.last_state["instance_id"] == node.instance:
                node.stage = "cleanup_reset_off"
                node.error = None
                node.command("reset")
        except Exception as exc:
            print("Cleanup unavailable; server watchdog remains active:", exc)
        finally:
            try:
                node.save()
            finally:
                node.destroy_node()
                if rclpy.ok():
                    rclpy.shutdown()


if __name__ == "__main__":
    main()
