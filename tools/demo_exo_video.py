"""Show/record the combined model with devices OFF; no walking controller.

Run from the repository root with .my_venv/bin/python -m tools.demo_exo_video.
"""

import argparse
import json
import math
import os
from pathlib import Path
import random
import time


def arguments():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--window", action="store_true", help="open a real-time MuJoCo viewer")
    parser.add_argument("--output", type=Path, help="MP4 path; default results/demo_exo.mp4 in headless mode")
    parser.add_argument("--seconds", type=float, default=8)
    parser.add_argument("--fps", type=int, default=30, help="must divide the 1200 Hz physics rate")
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--initial-hold", type=float, default=1, help="seconds to inspect the initial pose per episode")
    parser.add_argument("--on-terminal", choices=("hold", "reset"), default="hold",
                        help="freeze terminal state or explicitly restart short episodes; never step a done env")
    parser.add_argument("--azimuth", type=float, default=115)
    parser.add_argument("--elevation", type=float, default=-12)
    parser.add_argument("--distance", type=float, default=3.2)
    args = parser.parse_args()
    if args.output is None and not args.window:
        args.output = Path("results/demo_exo.mp4")
    if (not math.isfinite(args.seconds) or args.seconds <= 0
            or not math.isfinite(args.initial_hold) or args.initial_hold < 0
            or not math.isfinite(args.distance) or args.distance <= 0
            or not all(math.isfinite(x) for x in (args.azimuth, args.elevation))
            or not 1 <= args.fps <= 1200 or 1200 % args.fps
            or min(args.width, args.height) < 64 or args.width % 2 or args.height % 2):
        parser.error("Need positive finite duration/distance, nonnegative hold, even dimensions >=64, fps dividing 1200")
    if args.output and args.output.suffix.lower() != ".mp4":
        parser.error("--output must end in .mp4 (H.264/yuv420p)")
    return args


def check_window():
    # Test GLFW first: launch_passive can otherwise fail in its viewer thread.
    import glfw
    if not glfw.init():
        raise RuntimeError("Cannot initialize GLFW/display. Run in a graphical desktop with a valid DISPLAY, "
                           "or omit --window to record using EGL; no packages need installing.")
    window = None
    try:
        glfw.window_hint(glfw.VISIBLE, glfw.FALSE)
        window = glfw.create_window(64, 64, "MyoAssist display check", None, None)
        if window is None:
            raise RuntimeError("GLFW cannot create an OpenGL window. Check desktop DISPLAY/OpenGL; "
                               "use headless EGL recording if no graphical display is available.")
    finally:
        if window:
            glfw.destroy_window(window)
        glfw.terminate()


def run(args):
    # Must select the backend BEFORE importing MuJoCo or MyoSuite.
    if args.window:
        os.environ["MUJOCO_GL"] = "glfw"
        check_window()
    else:
        os.environ.setdefault("MUJOCO_GL", "egl")

    import imageio.v2 as imageio
    import imageio_ffmpeg
    import mujoco
    import numpy as np
    from PIL import Image, ImageDraw, ImageFont
    from myoassist_utils.compose import compose_env_model
    from rl_train.envs.myoassist_leg_base import MyoAssistLegBase
    from rl_train.train.train_configs.config import TrainSessionConfigBase

    if args.output:
        # Resolve existing bundled encoder now; encoding errors remain fatal, no silent fallback.
        encoder = imageio_ffmpeg.get_ffmpeg_exe()
        args.output = args.output.resolve()
        args.output.parent.mkdir(parents=True, exist_ok=True)
        partial = args.output.with_name(args.output.stem + ".partial.mp4")
    random.seed(args.seed)
    np.random.seed(args.seed)
    env = renderer = window = writer = None
    samples = []
    try:
        params = TrainSessionConfigBase.EnvParams(control_framerate=args.fps)
        env = MyoAssistLegBase(
            model_path=compose_env_model("myolegs22", "DephyExoBoot_L1", terrain=None),
            env_params=params, seed=args.seed, is_evaluate_mode=True,
            is_hardware=False, normalize_act=True,
        )
        env.mujoco_render_frames = False  # our viewer/clock owns visualization and pacing
        if env.robot.is_hardware:
            raise RuntimeError("Demo requires simulation mode")
        model, data = env.sim.model, env.sim.data
        native_model, native_data = model.ptr, data.ptr  # installed backend: dm_control wrappers
        ids = [model.actuator(n).id for n in ("Exo_R", "Exo_L")]
        np.testing.assert_allclose(model.actuator_ctrlrange[ids], [[-1, 0], [-1, 0]])
        action = np.full(model.nu, -1.0)  # muscle excitation OFF under the affine mapping
        action[ids] = 1.0  # device OFF: +1 -> raw ctrl=0
        mapped = env.robot.process_actuator(action, env.dt, normalized=True)
        np.testing.assert_allclose(mapped, 0, atol=1e-12)

        def reset():
            random.seed(args.seed)
            np.random.seed(args.seed)
            env.set_target_velocity_mode_manually(env.VelocityMode.UNIFORM, 0, 0, 0, 0)
            env.reset(seed=args.seed)
            # Apply the verified mapping without advancing physics, including startup/reset.
            data.ctrl[:] = mapped
            env.sim.forward()
            np.testing.assert_allclose(data.ctrl[ids], 0, atol=1e-12)
            np.testing.assert_allclose(data.actuator_force[ids], 0, atol=1e-12)

        reset()
        camera = mujoco.MjvCamera()
        camera.type = mujoco.mjtCamera.mjCAMERA_FREE
        camera.distance, camera.azimuth, camera.elevation = args.distance, args.azimuth, args.elevation
        pelvis = data.body("pelvis").xpos.copy()
        camera.lookat[:] = [pelvis[0], pelvis[1], 0.85]
        option = mujoco.MjvOption()
        option.sitegroup[:] = 0  # hide attachment/contact markers, keep actual device geoms
        if args.output:
            model.vis.global_.offwidth = max(model.vis.global_.offwidth, args.width)
            model.vis.global_.offheight = max(model.vis.global_.offheight, args.height)
            try:
                renderer = mujoco.Renderer(native_model, height=args.height, width=args.width)
            except Exception as exc:
                raise RuntimeError(f"Offscreen renderer failed with MUJOCO_GL={os.environ.get('MUJOCO_GL')}: {exc}. "
                                   "Check an available EGL/OpenGL driver; try MUJOCO_GL=osmesa only if OSMesa is installed.") from exc
            writer = imageio.get_writer(str(partial), format="FFMPEG", fps=args.fps,
                                       codec="libx264", pixelformat="yuv420p", macro_block_size=None)
        if args.window:
            from mujoco import viewer
            window = viewer.launch_passive(native_model, native_data)
            with window.lock():
                window.cam.distance = camera.distance
                window.cam.azimuth = camera.azimuth
                window.cam.elevation = camera.elevation
                window.cam.lookat[:] = camera.lookat
                window.opt.sitegroup[:] = 0

        font_path = Path("/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf")
        font = ImageFont.truetype(str(font_path), max(12, round(args.height / 40))) if font_path.exists() else ImageFont.load_default()
        frames = round(args.seconds * args.fps)
        if frames < 1:
            raise ValueError("Duration too short for one frame")
        episode = 1
        episode_frames = 0
        terminal = truncated = False
        physics_steps = 0
        first_terminal = None
        started = time.monotonic()
        print("Devices OFF; muscle excitation zero. No gait controller. Terminal policy:", args.on_terminal, flush=True)
        for frame_index in range(frames):
            if window is not None and not window.is_running():
                break
            if (terminal or truncated) and args.on_terminal == "reset":
                reset()
                episode += 1
                episode_frames = 0
                terminal = truncated = False
            phase = "INITIAL POSE - physics paused"
            if episode_frames >= round(args.initial_hold * args.fps) and not (terminal or truncated):
                before = float(data.time)
                _, _, terminal, truncated, _ = env.step(action.copy())
                physics_steps += 1
                if not math.isclose(float(data.time) - before, env.dt, abs_tol=1e-9):
                    raise RuntimeError("Unexpected simulation time increment; backend may have reset")
                if not all(np.isfinite(a).all() for a in (data.qpos, data.qvel, data.act, data.actuator_force)):
                    raise RuntimeError("Non-finite simulation state")
                phase = "FREE DYNAMICS - no gait controller"
                if (terminal or truncated) and first_terminal is None:
                    first_terminal = {"video_time_s": frame_index / args.fps, "sim_time_s": float(data.time)}
            if terminal or truncated:
                phase = "TERMINATED - frozen" if args.on_terminal == "hold" else "TERMINATED - reset next frame"
            np.testing.assert_allclose(data.ctrl[ids], 0, atol=1e-12)
            np.testing.assert_allclose(data.actuator_force[ids], 0, atol=1e-12)
            samples.append({"frame": frame_index, "episode": episode, "sim_time_s": float(data.time),
                            "phase": phase, "ctrl_R_L": data.ctrl[ids].tolist(),
                            "actuator_force_R_L": data.actuator_force[ids].tolist(),
                            "pelvis_height_m": float(data.joint("pelvis_ty").qpos[0]),
                            "terminated": bool(terminal), "truncated": bool(truncated)})
            if renderer is not None:
                renderer.update_scene(native_data, camera=camera, scene_option=option)
                rgb = renderer.render()
                if rgb.dtype != np.uint8 or rgb.shape != (args.height, args.width, 3):
                    raise RuntimeError(f"Expected RGB uint8 frame, got {rgb.shape}/{rgb.dtype}")
                # Imageio expects RGB (OpenCV would require BGR). No vertical flip is needed.
                image = Image.fromarray(rgb)
                draw = ImageDraw.Draw(image)
                draw.rectangle((0, 0, args.width, round(args.height * 0.15)), fill=(18, 25, 35))
                lines = ["MyoAssist + DephyExoBoot_L1 | devices OFF | no gait controller",
                         f"Episode {episode} | sim {data.time:.3f} s | video {frame_index / args.fps:.2f} s",
                         phase]
                for i, line in enumerate(lines):
                    draw.text((12, 5 + i * round(args.height * 0.044)), line, font=font, fill=(235, 240, 245))
                frame = np.ascontiguousarray(np.asarray(image))
                writer.append_data(frame)
                if frame_index == 0:
                    image.save(args.output.with_suffix(".preview.png"))
            if window is not None:
                window.sync()
                time.sleep(max(0, started + (frame_index + 1) / args.fps - time.monotonic()))
            episode_frames += 1
        if writer is not None:
            writer.close()
            writer = None
            if not partial.exists() or partial.stat().st_size == 0:
                raise RuntimeError("Encoder produced no video")
            # Decode the output to verify actual frames/resolution, not only requested metadata.
            with imageio.get_reader(str(partial), format="FFMPEG") as reader:
                metadata = reader.get_meta_data()
                decoded = sum(1 for _ in reader)
            if (decoded != len(samples) or tuple(metadata["size"]) != (args.width, args.height)
                    or not math.isclose(metadata["fps"], args.fps, abs_tol=0.01)):
                raise RuntimeError("Encoded frame count/resolution/fps mismatch")
            partial.replace(args.output)
        summary = {
            "video_path": str(args.output) if args.output else None,
            "video_duration_s": len(samples) / args.fps, "resolution": [args.width, args.height],
            "frame_count": len(samples), "fps": args.fps, "seed": args.seed,
            "physics_steps": physics_steps, "total_integrated_sim_time_s": physics_steps * env.dt,
            "episodes": episode, "first_terminal": first_terminal, "on_terminal": args.on_terminal,
            "initial_hold_s": args.initial_hold, "muscle_action": -1, "exo_action_R_L": [1, 1],
            "actual_exo_ctrl_R_L": data.ctrl[ids].tolist(),
            "actuator_force_R_L": data.actuator_force[ids].tolist(),
            "camera": {"azimuth": args.azimuth, "elevation": args.elevation,
                       "distance": args.distance, "lookat": camera.lookat.tolist()},
            "python_backend": "dm_control", "gl_backend": os.environ.get("MUJOCO_GL"),
            "walking_claim": False,
        }
        if args.output:
            summary.update(file_bytes=args.output.stat().st_size, encoder=encoder)
            args.output.with_suffix(".json").write_text(json.dumps(summary, indent=2) + "\n")
            args.output.with_suffix(".jsonl").write_text("".join(json.dumps(s) + "\n" for s in samples))
        print(json.dumps(summary, indent=2), flush=True)
        return summary
    finally:
        # Close encoding before GL; close our viewer ourselves to avoid MJRenderer.close()->quit().
        try:
            if writer is not None:
                writer.close()
        finally:
            try:
                if window is not None:
                    window.close()
            finally:
                try:
                    if renderer is not None:
                        renderer.close()
                finally:
                    if env is not None:
                        env.close()


if __name__ == "__main__":
    try:
        run(arguments())
    except KeyboardInterrupt:
        print("Demo interrupted; rendering resources closed.")
        raise SystemExit(130)
    except Exception as exc:
        raise SystemExit(f"Demo failed: {type(exc).__name__}: {exc}") from exc
