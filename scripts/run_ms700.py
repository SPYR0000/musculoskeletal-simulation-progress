"""Run an official msgym DynSyn-SAC checkpoint without automatic episode resets."""
from __future__ import annotations
import argparse
import os
import hashlib
import json
import random
import sys
import time
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
BASE = Path(os.environ.get('MS700_WORKSPACE', str(ROOT / 'workspace'))).expanduser().resolve()
SOURCE = BASE / 'sources/msgym'


def sha(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--model', choices=['full', 'legs'], default='full')
    p.add_argument('--seconds', type=float, default=10)
    p.add_argument('--seed', type=int, default=0)
    p.add_argument('--stochastic', action='store_true')
    p.add_argument('--cycles', type=int, help='Override episode cycle limit only')
    p.add_argument('--from-start', action='store_true', help='Disable random reference phase at reset')
    p.add_argument('--amplitude', choices=['official', 'training-schedule'], default='official')
    p.add_argument('--window', action='store_true')
    p.add_argument('--video', action='store_true')
    p.add_argument('--output', type=Path)
    args = p.parse_args()
    if not 0 < args.seconds <= 120:
        p.error('seconds must be in (0,120]')
    out = args.output or ROOT / 'runs' / (args.model + '_' + time.strftime('%Y%m%d_%H%M%S'))
    out.mkdir(parents=True, exist_ok=False)
    sys.path.insert(0, str(SOURCE / 'DynSyn-SAC/SB3-Scripts'))
    import numpy as np
    import torch
    import mujoco
    import gymnasium as gym
    import msgym
    from DynSyn import SAC_DynSyn
    from wrapper import MuscleNormWrapper
    from stable_baselines3.common.vec_env import VecNormalize, DummyVecEnv
    torch.set_num_threads(1)
    torch.manual_seed(args.seed)
    random.seed(args.seed)
    np.random.seed(args.seed)
    asset = BASE / 'assets' / ('LocomotionFull' if args.model == 'full' else 'LocomotionLegs')
    config_path = next(asset.glob('*.json'))
    config = json.loads(config_path.read_text())
    if config['agent'] != 'SAC_DynSyn' or list(config['wrapper_list']) != ['MuscleNormWrapper']:
        raise ValueError('Unsupported checkpoint contract')
    model_path = asset / 'checkpoint/best_model.zip'
    norm_path = asset / 'checkpoint/best_env.zip'
    # Inference never uses replay storage. Avoid allocating the saved training buffer.
    policy = SAC_DynSyn.load(model_path, device='cpu', buffer_size=1)
    if args.amplitude == 'training-schedule':
        amp = policy.dynsyn_weight_amp
        if amp is None:
            amp = policy.get_dynsyn_weight_amp(policy.dynsyn_k, policy.dynsyn_a, policy.num_timesteps)
        policy.actor.dynsyn_layer.update_dynsyn_weight_amp(amp)
    kwargs = dict(config['single_env_kwargs'])
    if args.cycles is not None:
        kwargs['gait_cycles'] = args.cycles
    if args.from_start:
        kwargs['random_init'] = False
    kwargs['kinematic_play'] = False
    env = MuscleNormWrapper(gym.make(config['env_name'], **kwargs))
    u = env.unwrapped
    norm = None
    viewer = renderer = writer = None
    rows = []
    actor_outputs = []
    hook = policy.actor.register_forward_hook(lambda module, inputs, output: actor_outputs.append(output.detach().cpu().numpy().copy()))
    start_wall = time.monotonic()
    reason = 'requested_duration'
    terminated = truncated = False
    frames = 0
    try:
        assert not u.kinematic_play
        norm = VecNormalize.load(norm_path, DummyVecEnv([lambda: env]))
        norm.training = False
        norm.norm_reward = False
        obs, _ = env.reset(seed=args.seed)
        assert obs.shape == policy.observation_space.shape
        assert env.action_space.shape == policy.action_space.shape
        m, d = u.model, u.data
        names = [f'{j}_{side}' for side in ['r', 'l'] for j in ['hip_flexion', 'knee_angle', 'ankle_angle']]
        ids = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, name) for name in names]
        assert min(ids) >= 0 and all(m.jnt_type[i] == mujoco.mjtJoint.mjJNT_HINGE for i in ids)
        qi, vi = m.jnt_qposadr[ids], m.jnt_dofadr[ids]
        initial_qpos, initial_qvel, initial_act = d.qpos.copy(), d.qvel.copy(), d.act.copy()
        initial_pelvis = d.xpos[u.pelvis_id].copy()
        # Visual quality only: avoid the upstream 4K / 24x MSAA framebuffer.
        m.vis.global_.offwidth, m.vis.global_.offheight = 640, 480
        m.vis.quality.offsamples, m.vis.quality.shadowsize = 4, 2048
        cam = mujoco.MjvCamera()
        cam.distance, cam.azimuth, cam.elevation = 3.0, 135, -12
        if args.window:
            import mujoco.viewer
            viewer = mujoco.viewer.launch_passive(m, d)
            viewer.cam.distance, viewer.cam.azimuth, viewer.cam.elevation = 3.0, 135, -12
        if args.video:
            import imageio.v2 as imageio
            renderer = mujoco.Renderer(m, 480, 640)
            writer = imageio.get_writer(str(out / 'simulation.mp4'), fps=30, codec='libx264')
        steps = int(np.ceil(args.seconds / u.dt))
        for step in range(steps):
            if viewer is not None and not viewer.is_running():
                reason = 'viewer_closed'
                break
            tick = time.monotonic()
            obs_norm = norm.normalize_obs(obs.copy())
            ref = u.qpos_ref.copy()  # the target used by this step, before next-target update
            action, _ = policy.predict(obs_norm, deterministic=not args.stochastic)
            mapped = env.action(action)
            obs, reward, terminated, truncated, info = env.step(action)
            if not all(np.isfinite(x).all() for x in [d.qpos, d.qvel, d.act, action]):
                raise RuntimeError('Non-finite physical state/action')
            np.testing.assert_allclose(mapped, d.ctrl, rtol=0, atol=1e-7)
            contact_force = np.zeros(2)
            wrench = np.zeros(6)
            for c in range(d.ncon):
                contact = d.contact[c]
                geom_names = [m.geom(int(g)).name for g in contact.geom]
                if 'floor' not in geom_names:
                    continue
                other = int(contact.geom[0] if geom_names[1] == 'floor' else contact.geom[1])
                body_name = m.body(m.geom_bodyid[other]).name
                for side_index, side in enumerate(['r', 'l']):
                    if body_name in [f'calcn_{side}', f'toes_{side}', f'talus_{side}']:
                        mujoco.mj_contactForce(m, d, c, wrench)
                        contact_force[side_index] += max(float(wrench[0]), 0)
            rows.append(dict(time=float(d.time), observation_normalized=obs_norm.copy(),
                             actor_output=actor_outputs[-1].reshape(-1), action=action.copy(), mapped_action=mapped.copy(),
                             ctrl=d.ctrl.copy(), act=d.act.copy(), actuator_force=d.actuator_force.copy(),
                             qpos=d.qpos.copy(), qvel=d.qvel.copy(), reference_qpos=ref,
                             joint_angle=d.qpos[qi].copy(), joint_velocity=d.qvel[vi].copy(),
                             pelvis_position=d.xpos[u.pelvis_id].copy(), pelvis_quat=d.xquat[u.pelvis_id].copy(),
                             foot_normal_force=contact_force, reward=float(reward), terminated=bool(terminated), truncated=bool(truncated)))
            if renderer is not None and frames / 30 < d.time - 1e-9:
                cam.lookat[:] = d.xpos[u.pelvis_id]
                renderer.update_scene(d, camera=cam)
                writer.append_data(renderer.render())
                frames += 1
            if viewer is not None:
                viewer.cam.lookat[:] = d.xpos[u.pelvis_id]
                viewer.sync()
                time.sleep(max(0, u.dt - (time.monotonic() - tick)))
            if terminated or truncated:
                reason = 'cycle_limit' if truncated else 'tracking_error_termination'
                break
        if not rows:
            raise RuntimeError('No control step executed')
        arrays = {k: np.asarray([r[k] for r in rows]) for k in rows[0]}
        np.savez_compressed(out / 'states.npz', **arrays, initial_qpos=initial_qpos, initial_qvel=initial_qvel,
                            initial_act=initial_act, joint_names=np.array(names))
        np.savetxt(out / 'joints.csv', np.column_stack([arrays['time'], arrays['joint_angle'], arrays['joint_velocity'], arrays['pelvis_position'],arrays['foot_normal_force']]),
                   delimiter=',', header=','.join(['time_s']+[n+'_rad' for n in names]+[n+'_rad_s' for n in names]+['pelvis_x_m','pelvis_y_m','pelvis_z_m','foot_r_N','foot_l_N']), comments='')
        summary = dict(model=args.model, runner='official_msgym_DynSyn_SAC', actor_calls=len(actor_outputs),
                       deterministic=not args.stochastic, seed=args.seed, kinematic_play=False, resets_after_initialization=0,
                       environment=config['env_name'], env_kwargs=kwargs, ctrl_mapping='sigmoid(5*(action-0.5))',
                       dynsyn_weight_amp=policy.actor.dynsyn_layer.dynsyn_weight_amp, amplitude_mode=args.amplitude,
                       model_sha256=sha(model_path), normalization_sha256=sha(norm_path),
                       observation_dim=obs.size, action_dim=m.nu, nq=m.nq, nv=m.nv,
                       physics_dt=m.opt.timestep, control_dt=u.dt, reference_init_time=float(u.init_time),
                       reference_period_s=float(u.terminate_time), actual_time_s=float(d.time), steps=len(rows),
                       reason=reason, terminated=bool(terminated), truncated=bool(truncated),
                       pelvis_min_height_m=float(arrays['pelvis_position'][:,2].min()),
                       pelvis_displacement_m=(arrays['pelvis_position'][-1]-initial_pelvis).tolist(),
                       joint_tracking_rmse_deg=np.rad2deg(np.sqrt(np.mean((arrays['joint_angle']-arrays['reference_qpos'][:,qi])**2,axis=0))).tolist(),
                       joint_names=names, video_frames=frames, wall_seconds=time.monotonic()-start_wall,
                       training=False, health_gait_validated=False)
        (out / 'summary.json').write_text(json.dumps(summary,indent=2))
        print(json.dumps(summary,indent=2), flush=True)
    except BaseException as exc:
        (out / 'failure.json').write_text(json.dumps({'error':repr(exc),'steps_recorded':len(rows)},indent=2))
        if rows:
            np.savez_compressed(out / 'partial_states.npz', **{k:np.asarray([r[k] for r in rows]) for k in rows[0]})
        raise
    finally:
        hook.remove()
        if writer is not None: writer.close()
        if renderer is not None: renderer.close()
        if viewer is not None: viewer.close()
        if norm is not None: norm.close()
        else: env.close()


if __name__ == '__main__':
    main()
