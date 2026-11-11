"""Loopback-only, single-owner MyoAssist simulation server (JSON lines protocol v1)."""

import argparse
import json
import math
import os
import random
import signal
import socket
import time
import uuid
from pathlib import Path
from tools.exo_device import DEFAULT_CONFIG, load_config, model_input, off_actions

VERSION = 1
MAX_FRAME = 16384


class ProtocolError(Exception):
    def __init__(self, code, message):
        super().__init__(message)
        self.code = code


def reject_constant(value):
    raise ValueError(f"Non-finite JSON constant: {value}")


def unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"Duplicate JSON field: {key}")
        result[key] = value
    return result


def clean(value):
    """Keep fault responses/logs valid JSON even if the simulator is non-finite."""
    if isinstance(value, float) and not math.isfinite(value):
        return None
    if isinstance(value, dict):
        return {k: clean(v) for k, v in value.items()}
    if isinstance(value, (list, tuple)):
        return [clean(v) for v in value]
    return value


class Simulation:
    def __init__(self, seed=0, config_path=DEFAULT_CONFIG):
        # Keep this import local so protocol/client tools need no scientific dependencies.
        import mujoco
        import numpy as np
        from rl_train.envs.myoassist_leg_base import MyoAssistLegBase
        from rl_train.train.train_configs.config import TrainSessionConfigBase

        self.np = np
        self.seed = seed
        self.config = load_config(config_path)
        channels = self.config["channels"]
        if len(channels) != 2 or len({c["actuator"] for c in channels}) != 2:
            raise ValueError("This adapter requires two distinct configured device channels")
        if self.config["muscle_action"] not in (-1, 0):
            raise ValueError("muscle_action must be -1 (zero excitation) or 0 (legacy test baseline)")
        random.seed(seed)
        np.random.seed(seed)
        self.env = MyoAssistLegBase(
            model_path=model_input(self.config),
            env_params=TrainSessionConfigBase.EnvParams(), seed=seed,
            is_evaluate_mode=True, is_hardware=False, normalize_act=True,
        )
        self.model, self.data = self.env.sim.model, self.env.sim.data
        self.ids = []
        self.joints = []
        self.terminated = self.truncated = False
        self.requested = self.clipped = self.last_step = None
        self.effective = None
        self.source = "startup"
        try:
            self.ids = [self.model.actuator(c["actuator"]).id for c in channels]
            self.joints = self.config["state_joints"]
            if self.env.robot.is_hardware or not self.env.normalize_act:
                raise RuntimeError("Unexpected environment/control mode")
            for channel, i in zip(channels, self.ids):
                joint = self.model.joint(channel["joint"]).id
                if (self.model.actuator_trntype[i] != mujoco.mjtTrn.mjTRN_JOINT
                        or self.model.actuator_trnid[i, 0] != joint
                        or self.model.actuator_dyntype[i] != mujoco.mjtDyn.mjDYN_NONE
                        or self.model.actuator_gaintype[i] != mujoco.mjtGain.mjGAIN_FIXED
                        or self.model.actuator_biastype[i] != mujoco.mjtBias.mjBIAS_NONE):
                    raise RuntimeError("Unexpected device transmission/dynamics")
                np.testing.assert_allclose(self.model.actuator_ctrlrange[i], channel["ctrlrange"])
                np.testing.assert_allclose(self.model.actuator_gainprm[i, 0], channel["gain"])
                np.testing.assert_allclose(self.model.actuator_gear[i], [channel["gear"], 0, 0, 0, 0, 0])
                if not self.model.actuator_ctrllimited[i] or self.model.actuator_forcelimited[i]:
                    raise RuntimeError("Adapter requires ctrl-limited devices without force clipping")
                if channel["off_ctrl"] != 0 or not -1 <= channel["off_action"] <= 1:
                    raise ValueError("Expected zero-ctrl safe state and valid normalized off_action")
            muscle_ids = {i for i in range(self.model.nu)
                          if self.model.actuator_dyntype[i] == mujoco.mjtDyn.mjDYN_MUSCLE}
            if muscle_ids | set(self.ids) != set(range(self.model.nu)) or muscle_ids & set(self.ids):
                raise RuntimeError("Unconfigured non-muscle actuators in model")
            for name in self.joints:
                j = self.model.joint(name).id
                if self.model.jnt_type[j] != mujoco.mjtJoint.mjJNT_HINGE:
                    raise RuntimeError(f"Expected scalar hinge joint: {name}")
                if not (0 <= self.model.jnt_qposadr[j] < self.model.nq
                        and 0 <= self.model.jnt_dofadr[j] < self.model.nv):
                    raise RuntimeError(f"Invalid joint address: {name}")
            self.reset()
        except BaseException:
            self.close()
            raise

    def close_devices(self, reason):
        """Map +1 via the SAME Robot mapping, apply without integrating a terminal body.

        This is not env.step(0), and does not advance time or hide the last env.step input.
        Only the two device controls change; existing muscle controls are preserved.
        """
        action = self.np.zeros(self.model.nu)
        off_action = off_actions(self.config)
        action[self.ids] = off_action
        mapped = self.env.robot.process_actuator(action, self.env.dt, normalized=True)
        self.data.ctrl[self.ids] = mapped[self.ids]
        self.effective = off_action
        self.source = reason
        self.env.sim.forward()
        self.np.testing.assert_allclose(self.data.ctrl[self.ids], 0, atol=1e-12)
        self.np.testing.assert_allclose(self.data.actuator_force[self.ids], 0, atol=1e-12)

    def reset(self):
        random.seed(self.seed)
        self.np.random.seed(self.seed)
        velocity = self.config["target_velocity_m_s"]
        self.env.set_target_velocity_mode_manually(self.env.VelocityMode.UNIFORM, 0, velocity, velocity, velocity)
        self.env.reset(seed=self.seed)
        self.terminated = bool(self.env._get_done())
        self.truncated = False
        self.requested = self.clipped = self.last_step = None
        self.close_devices("reset")
        self.check_finite()

    def check_finite(self):
        arrays = (self.data.qpos, self.data.qvel, self.data.act, self.data.ctrl, self.data.actuator_force)
        if not math.isfinite(float(self.data.time)) or any(not self.np.isfinite(a).all() for a in arrays):
            raise RuntimeError("Non-finite simulation state")

    def step(self, requested):
        self.requested = list(requested)
        self.clipped = self.np.clip(requested, -1, 1).tolist()
        action = self.np.full(self.model.nu, self.config["muscle_action"], dtype=float)
        action[self.ids] = self.clipped
        self.last_step = action.tolist()
        self.effective = list(self.clipped)
        self.source = "step"
        before = float(self.data.time)
        _, _, self.terminated, self.truncated, _ = self.env.step(action)
        self.check_finite()
        if not math.isclose(float(self.data.time) - before, self.env.dt, abs_tol=1e-9, rel_tol=1e-9):
            raise RuntimeError("Unexpected simulation time increment (possible backend reset)")

    def state(self):
        return clean({
            "device_config": self.config,
            "seed": self.seed,
            "model_state": {"qpos": self.data.qpos.tolist(), "qvel": self.data.qvel.tolist(),
                            "activation": self.data.act.tolist(), "ctrl": self.data.ctrl.tolist()},
            "actuator_names": [c["actuator"] for c in self.config["channels"]],
            "device_actuator_ids": self.ids,
            "sim_time_s": float(self.data.time), "control_dt_s": self.env.dt,
            "requested_action": self.requested, "clipped_action": self.clipped,
            "env_step_action": self.last_step, "effective_exo_action": self.effective,
            "control_source": self.source,
            "actual_ctrl": self.data.ctrl[self.ids].tolist(),
            "actuator_force": self.data.actuator_force[self.ids].tolist(),
            "device_joint_torque_Nm": [float(self.data.actuator_force[i]) * c["gear"]
                                       for i, c in zip(self.ids, self.config["channels"])],
            "joints": {name: {"angle_rad": float(self.data.joint(name).qpos[0]),
                               "velocity_rad_s": float(self.data.joint(name).qvel[0])}
                       for name in self.joints},
            "muscle_activation": self.data.act[self.env._muscle_act_indices].tolist(),
            "terminated": bool(self.terminated), "truncated": bool(self.truncated),
        })

    def close(self):
        try:
            self.close_devices("shutdown")
        finally:
            self.env.close()


class LiveView:
    """Passive viewer of the EXISTING model/data. Never creates or advances a simulation."""

    def __init__(self, sim):
        import mujoco
        from mujoco import viewer
        self.mujoco = mujoco
        self.handle = viewer.launch_passive(sim.model.ptr, sim.data.ptr,
                                            show_left_ui=False, show_right_ui=False)
        try:
            self.shared = (sim.np.shares_memory(self.handle.d.qpos, sim.data.qpos)
                           and sim.np.shares_memory(self.handle.m.actuator_gainprm, sim.model.actuator_gainprm))
            if not self.shared:
                raise RuntimeError("Viewer must share the TCP simulation model/data")
            with self.handle.lock():
                camera = sim.config["camera"]
                position = sim.data.body(camera["lookat_body"]).xpos
                self.handle.cam.lookat[:] = [position[0], position[1], camera["lookat_z"]]
                self.handle.cam.distance = camera["distance"]
                self.handle.cam.azimuth = camera["azimuth"]
                self.handle.cam.elevation = camera["elevation"]
                self.handle.opt.sitegroup[:] = 0
        except BaseException:
            self.close()
            raise
        self.last_update = 0.0
        self.last_frame = None
        self.updates = 0

    def update(self, state, force=False):
        if not self.handle.is_running():
            return False
        now = time.monotonic()
        if not force and now - self.last_update < 1 / 30:
            return True
        def number(value, signed=False):
            return "INVALID" if value is None else format(value, "+.4f" if signed else ".4f")
        left = ["ROS/TCP SAME SIMULATION", "Instance", "Status / epoch", "Simulation time (s)",
                "Requested action", "Applied ctrl", "Joint torque R/L (Nm)"]
        right = ["No gait controller", state["instance_id"][:12],
                 f"{state['status']} / {state['epoch']}", number(state["sim_time_s"]),
                 "none" if state["requested_action"] is None else " / ".join(number(v, True) for v in state["requested_action"]),
                 " / ".join(number(v, True) for v in state["actual_ctrl"]),
                 " / ".join(number(v, True) for v in state["device_joint_torque_Nm"])]
        for name, joint in state["joints"].items():
            left.append(name + " (rad, rad/s)")
            right.append(number(joint["angle_rad"], True) + ", " + number(joint["velocity_rad_s"], True))
        self.handle.set_texts((self.mujoco.mjtFontScale.mjFONTSCALE_100,
                               self.mujoco.mjtGridPos.mjGRID_TOPLEFT, "\n".join(left), "\n".join(right)))
        # state_only prevents UI controls/perturbations from becoming extra control inputs.
        self.handle.sync(state_only=True)
        self.last_update = now
        self.updates += 1
        self.last_frame = {k: state[k] for k in ("instance_id", "epoch", "sim_time_s", "actual_ctrl",
                                                "device_joint_torque_Nm", "joints")}
        return True

    def metadata(self):
        return {"enabled": True, "running": self.handle.is_running(), "shared_model_data": self.shared,
                "updates": self.updates, "displayed_state": self.last_frame}

    def close(self):
        self.handle.close()


class Server:
    def __init__(self, sim, log, watchdog=1.0, max_steps=10, view=None):
        self.sim, self.log = sim, log
        self.watchdog, self.max_steps = watchdog, max_steps
        self.run_id = uuid.uuid4().hex
        self.epoch = 0
        self.status = "reset_required"
        self.fault = None
        self.deadline = None
        self.stopping = False
        self.view = view

    def refresh_view(self, force=False):
        if self.view is not None and not self.view.update(self.state(), force):
            self.stopping = True
            self.safe("window_closed")

    def record(self, event, **payload):
        self.log.write(json.dumps(clean({"run_id": self.run_id, "event": event,
                                        "monotonic_s": time.monotonic(), "unix_time_s": time.time(),
                                        **payload}), allow_nan=False) + "\n")
        self.log.flush()

    def safe(self, reason):
        self.deadline = None
        self.epoch += 1
        self.status = reason
        try:
            self.sim.close_devices(reason)
        except Exception as exc:
            self.status = "fault"
            self.fault = f"Safety closure failed: {exc}"
            # Last-resort raw zero only when the verified mapping itself failed.
            self.sim.data.ctrl[self.sim.ids] = 0
        self.record("safety", reason=reason, state=self.state())
        if self.view is not None and self.view.handle.is_running():
            self.view.update(self.state(), force=True)

    def check_watchdog(self):
        if self.deadline is not None and time.monotonic() >= self.deadline:
            self.safe("watchdog_timeout")

    def state(self):
        return {"status": self.status, "epoch": self.epoch, "fault": self.fault,
                "instance_id": self.run_id,
                "viewer": self.view.metadata() if self.view is not None else {"enabled": False},
                **self.sim.state()}

    def response(self, seq, code=None, message="", **extra):
        return {"version": VERSION, "seq": seq, "ok": code is None,
                "error": None if code is None else {"code": code, "message": message},
                "state": self.state(), **extra}

    def request(self, raw, last_seq):
        self.check_watchdog()
        seq = None
        completed = 0
        try:
            try:
                req = json.loads(raw, parse_constant=reject_constant, object_pairs_hook=unique_object)
            except (ValueError, UnicodeError, RecursionError):
                raise ProtocolError("invalid_json", "Expected finite UTF-8 JSON with unique fields")
            if not isinstance(req, dict):
                raise ProtocolError("invalid_request", "Request must be an object")
            seq = req.get("seq")
            if type(seq) is not int or not 0 <= seq <= 2**53 - 1:
                seq = None
                raise ProtocolError("invalid_seq", "seq must be an integer in [0,2^53-1]")
            if seq <= last_seq:
                raise ProtocolError("stale_seq", "seq must strictly increase within this connection")
            last_seq = seq  # consume a syntactically valid seq even on a rejected request
            if type(req.get("version")) is not int or req["version"] != VERSION:
                raise ProtocolError("bad_version", "version must be integer 1")
            cmd = req.get("cmd")
            if not isinstance(cmd, str) or cmd not in ("ping", "reset", "step", "stop"):
                raise ProtocolError("unknown_command", "cmd must be ping, reset, step or stop")
            expected = {"version", "seq", "cmd"}
            if cmd == "step":
                expected |= {"epoch", "action", "steps"}
            if set(req) != expected:
                raise ProtocolError("invalid_fields", f"Expected exactly {sorted(expected)}")
            if cmd == "reset":
                self.sim.reset()
                self.epoch += 1
                self.fault = None
                self.status = "terminated" if self.sim.terminated else "ready"
                self.deadline = time.monotonic() + self.watchdog if self.status == "ready" else None
            elif cmd == "step":
                action = req["action"]
                if (not isinstance(action, list) or len(action) != 2
                        or any(type(v) not in (int, float) for v in action)):
                    raise ProtocolError("invalid_action", "action must contain two numbers [right,left]")
                try:
                    finite = all(math.isfinite(v) for v in action)
                except OverflowError:
                    finite = False
                if not finite:
                    raise ProtocolError("invalid_action", "action must be finite")
                if type(req["steps"]) is not int or not 1 <= req["steps"] <= self.max_steps:
                    raise ProtocolError("invalid_steps", f"steps must be integer 1..{self.max_steps}")
                if self.status != "ready":
                    raise ProtocolError("reset_required", "Explicit reset required before stepping")
                if type(req["epoch"]) is not int or req["epoch"] != self.epoch:
                    raise ProtocolError("stale_epoch", "Use epoch returned by the latest reset")
                self.deadline = time.monotonic() + self.watchdog
                for _ in range(req["steps"]):
                    if self.stopping:
                        self.safe("stopped")
                        break
                    self.check_watchdog()
                    if self.status != "ready":
                        break
                    self.sim.step(action)
                    completed += 1
                    self.refresh_view(force=True)
                    self.record("step", seq=seq, substep=completed, state=self.state())
                    if self.sim.terminated or self.sim.truncated:
                        self.safe("terminated")
                        break
                self.check_watchdog()
            elif cmd == "stop":
                self.safe("stopped")
                self.stopping = True
            code = "watchdog_timeout" if cmd == "step" and self.status == "watchdog_timeout" else None
            self.refresh_view(force=True)
            reply = self.response(seq, code, "Command watchdog expired" if code else "", steps_completed=completed)
        except ProtocolError as exc:
            reply = self.response(seq, exc.code, str(exc), steps_completed=completed)
        except Exception as exc:
            self.fault = f"{type(exc).__name__}: {exc}"
            self.safe("fault")
            reply = self.response(seq, "simulation_fault", self.fault, steps_completed=completed)
        self.record("request", raw=raw.decode("utf-8", errors="replace"), response=reply)
        return reply, last_seq

    def serve_connection(self, conn):
        conn.settimeout(min(0.02, self.watchdog / 4))
        buffer = b""
        last_seq = -1
        try:
            while not self.stopping:
                self.check_watchdog()
                self.refresh_view()
                if self.stopping:
                    break
                try:
                    chunk = conn.recv(4096)
                except socket.timeout:
                    continue
                if not chunk:
                    break
                buffer += chunk
                while b"\n" in buffer and not self.stopping:
                    raw, buffer = buffer.split(b"\n", 1)
                    if len(raw) > MAX_FRAME:
                        raise ProtocolError("frame_too_large", f"Maximum frame size is {MAX_FRAME} bytes")
                    reply, last_seq = self.request(raw, last_seq)
                    conn.sendall(json.dumps(reply, allow_nan=False).encode() + b"\n")
                if len(buffer) > MAX_FRAME:
                    raise ProtocolError("frame_too_large", f"Maximum frame size is {MAX_FRAME} bytes")
        except ProtocolError as exc:
            try:
                conn.sendall(json.dumps(self.response(None, exc.code, str(exc)), allow_nan=False).encode() + b"\n")
            except OSError:
                pass
        except (OSError, TimeoutError):
            pass
        finally:
            if not self.stopping:
                self.safe("disconnected")

    def serve(self, port):
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as listener:
            listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            listener.bind(("127.0.0.1", port))
            listener.listen(1)
            listener.settimeout(0.1)
            actual_port = listener.getsockname()[1]
            self.record("startup", port=actual_port, seed=self.sim.seed, watchdog_s=self.watchdog,
                        max_steps=self.max_steps, config=self.sim.config, state=self.state())
            print(json.dumps({"event": "listening", "host": "127.0.0.1", "port": actual_port}), flush=True)
            while not self.stopping:
                self.refresh_view()
                if self.stopping:
                    break
                try:
                    conn, _ = listener.accept()
                except socket.timeout:
                    continue
                with conn:
                    self.serve_connection(conn)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--watchdog", type=float, default=1.0, help="wall-clock seconds, not simulation seconds")
    parser.add_argument("--max-steps", type=int, default=10)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--config", type=Path, default=DEFAULT_CONFIG)
    parser.add_argument("--window", action="store_true", help="show the same simulation receiving TCP/ROS commands")
    parser.add_argument("--log", type=Path, default=Path("/tmp/myoassist-exo-tcp.jsonl"))
    args = parser.parse_args()
    if not 0 <= args.port <= 65535 or not math.isfinite(args.watchdog) or args.watchdog <= 0 or args.max_steps < 1:
        parser.error("Invalid port, watchdog or max-steps")
    if args.window:
        os.environ["MUJOCO_GL"] = "glfw"
        from tools.demo_exo_video import check_window
        check_window()
    else:
        os.environ.setdefault("MUJOCO_GL", "egl")
    sim = Simulation(args.seed, args.config)
    view = None
    try:
        if args.window:
            view = LiveView(sim)
        args.log.parent.mkdir(parents=True, exist_ok=True)
        with args.log.open("a", encoding="utf-8") as log:
            server = Server(sim, log, args.watchdog, args.max_steps, view)

            def stop_signal(signum, frame):
                # Do not re-enter MuJoCo from the signal handler; main thread closes in finally.
                server.stopping = True

            signal.signal(signal.SIGINT, stop_signal)
            signal.signal(signal.SIGTERM, stop_signal)
            try:
                server.serve(args.port)
            finally:
                server.safe("stopped")
    finally:
        try:
            if view is not None:
                view.close()
        finally:
            sim.close()


if __name__ == "__main__":
    main()
