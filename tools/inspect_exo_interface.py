"""Deterministic, simulation-only audit; run with .my_venv/bin/python -m tools.inspect_exo_interface."""

import argparse
import json
import random
import sys
from importlib.metadata import version
from pathlib import Path
import xml.etree.ElementTree as ET

import mujoco
import numpy as np

from myoassist_utils.compose import compose_env_model
from rl_train.envs.myoassist_leg_base import MyoAssistLegBase
from rl_train.train.train_configs.config import TrainSessionConfigBase


def audit():
    random.seed(0)
    np.random.seed(0)
    xml = compose_env_model("myolegs22", "DephyExoBoot_L1", terrain=None)
    env = MyoAssistLegBase(
        model_path=xml,
        env_params=TrainSessionConfigBase.EnvParams(),
        seed=0,
        is_evaluate_mode=True,
        is_hardware=False,
    )
    try:
        assert not env.robot.is_hardware
        model, data = env.sim.model, env.sim.data
        ids = [model.actuator(n).id for n in ("Exo_R", "Exo_L")]
        report = {
            "python": sys.version,
            "versions": {n: version(n) for n in ("myosuite", "mujoco", "assist-sim")},
            "model": {"nu": model.nu, "na": model.na, "nq": model.nq, "nv": model.nv},
            "is_hardware": env.robot.is_hardware,
            "normalize_act": env.normalize_act,
            "physics_dt_s": model.opt.timestep,
            "frame_skip": env.frame_skip,
            "control_dt_s": env.dt,
            "after_constructor": {"time_s": data.time, "exo_ctrl": data.ctrl[ids].tolist()},
            "actuators": [],
        }
        for i in ids:
            joint = int(model.actuator_trnid[i, 0])
            report["actuators"].append({
                "name": model.actuator(i).name, "id": i,
                "joint": model.joint(joint).name,
                "joint_type": int(model.jnt_type[joint]),
                "joint_axis": model.jnt_axis[joint].tolist(),
                "qpos_address": int(model.jnt_qposadr[joint]),
                "dof_address": int(model.jnt_dofadr[joint]),
                **{k: np.asarray(getattr(model, "actuator_" + k)[i]).tolist()
                   for k in ("trntype", "dyntype", "gaintype", "biastype", "gainprm",
                             "biasprm", "gear", "ctrlrange", "ctrllimited", "forcerange", "forcelimited")},
            })
        env.set_target_velocity_mode_manually(env.VelocityMode.UNIFORM, 0, 1, 1, 1)
        obs, _ = env.reset(seed=0)
        report["after_reset"] = {"time_s": data.time, "exo_ctrl": data.ctrl[ids].tolist()}
        report["observation_blocks"] = {}
        offset = 0
        for key in env.obs_keys:
            count = int(np.asarray(env.get_obs_dict(env.sim)[key]).size)
            report["observation_blocks"][key] = {"start": offset, "stop_exclusive": offset + count}
            offset += count
        report["activation_order"] = [model.actuator(i).name for i in range(model.nu)
                                       if model.actuator_dyntype[i] == mujoco.mjtDyn.mjDYN_MUSCLE]
        assert obs.shape == (23,) and offset == 23
        report["action_probes"] = []
        for right, left in ((1, 1), (0, 0), (-1, -1), (-0.5, 1), (1, -0.5), (2, -2)):
            env.reset(seed=0)
            action = np.full(model.nu, -1.0)  # zero muscle excitation under affine mapping
            action[ids] = [right, left]
            t0 = data.time
            _, _, terminated, truncated, _ = env.step(action)
            expected = (np.clip([right, left], -1, 1) - 1) / 2
            np.testing.assert_allclose(data.ctrl[ids], expected)
            np.testing.assert_allclose(data.actuator_force[ids], 100 * expected, atol=1e-10)
            np.testing.assert_allclose(data.time - t0, env.dt)
            report["action_probes"].append({
                "action_R_L": [right, left], "ctrl_R_L": data.ctrl[ids].tolist(),
                "actuator_force_R_L": data.actuator_force[ids].tolist(),
                "dt_s": data.time - t0, "terminated": terminated, "truncated": truncated,
                "device_effort": env._get_device_effort(),
            })
        # At identical qpos/qvel/activation, isolate each device contribution from all muscles.
        env.reset(seed=0)
        data.ctrl[:] = 0
        env.sim.forward()
        baseline = data.qfrc_actuator.copy()
        report["isolated_device_probes"] = []
        for i in ids:
            data.ctrl[:] = 0
            data.ctrl[i] = -0.25
            env.sim.forward()
            contribution = data.qfrc_actuator - baseline
            dof = int(model.jnt_dofadr[model.actuator_trnid[i, 0]])
            expected = np.zeros(model.nv)
            expected[dof] = -25
            np.testing.assert_allclose(contribution, expected, atol=1e-10)
            report["isolated_device_probes"].append({
                "name": model.actuator(i).name, "ctrl": float(data.ctrl[i]),
                "actuator_force": float(data.actuator_force[i]),
                "nonzero_dofs": np.flatnonzero(np.abs(contribution) > 1e-10).tolist(),
                "ankle_contribution_Nm": float(contribution[dof]),
            })
        report["joint_state_sample"] = {
            name: {"angle_rad": float(data.joint(name).qpos[0]),
                   "velocity_rad_s": float(data.joint(name).qvel[0]),
                   "qpos_address": int(model.jnt_qposadr[model.joint(name).id]),
                   "dof_address": int(model.jnt_dofadr[model.joint(name).id])}
            for side in ("r", "l") for name in
            (f"hip_flexion_{side}", f"knee_angle_{side}", f"ankle_angle_{side}")
        }
        root = ET.fromstring(xml)
        report["composed_exo_xml"] = [ET.tostring(a, encoding="unicode").strip()
                                       for a in root.find("actuator") if a.get("name") in ("Exo_R", "Exo_L")]
        report["checks_passed"] = True
        return report
    finally:
        env.sim.data.ctrl[:] = 0
        env.close()


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    result = audit()
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, allow_nan=False) + "\n", encoding="utf-8")
    print(f"Audit passed: {args.output}")
