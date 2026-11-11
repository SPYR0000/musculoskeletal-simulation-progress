"""Device configuration and direct-joint mappings; standard library only (ROS-safe)."""

import json
import math
from pathlib import Path

DEFAULT_CONFIG = Path(__file__).with_name("dephy_exo.json")


def finite(value, label):
    if type(value) not in (int, float) or not math.isfinite(value):
        raise ValueError(f"{label}: expected a finite number, not {value!r}")
    return value


def action_to_ctrl(channel, action):
    lo, hi = channel["ctrlrange"]
    return (lo + hi) / 2 + max(-1, min(1, finite(action, "action"))) * (hi - lo) / 2


def ctrl_to_action(channel, ctrl):
    lo, hi = channel["ctrlrange"]
    ctrl = finite(ctrl, "ctrl")
    if not lo <= ctrl <= hi:
        raise ValueError(f"{channel['name']}: ctrl {ctrl} outside [{lo}, {hi}]")
    return 2 * (ctrl - lo) / (hi - lo) - 1


def torque_to_action(channel, torque):
    return ctrl_to_action(channel, finite(torque, "joint torque Nm") / (channel["gain"] * channel["gear"]))


def off_actions(config):
    return [c["off_action"] for c in config["channels"]]


def pulse_actions(config, side, torque=None):
    """Joint-coordinate torque; no anatomical sign reinterpretation or extra sign multiplier."""
    return [torque_to_action(c, c["demo_torque_nm"] if torque is None else torque)
            if c["name"] == side else c["off_action"] for c in config["channels"]]


def validate(config):
    if config.get("schema_version") != 1 or config.get("ready") is not True:
        raise ValueError("Device must have schema_version=1 and ready=true; resolve all TBD fields first")
    if config.get("adapter") != "direct_joint_fixed_gain":
        raise ValueError("Only direct_joint_fixed_gain is supported; servo/filter/tendon requires a separate adapter")
    if not config.get("model_xml") and not all(isinstance(config.get(k), str) and config[k] for k in ("msk_key", "device_key")):
        raise ValueError("Supply a composed model_xml or msk_key + device_key")
    if config.get("model_xml") is not None and not isinstance(config["model_xml"], str):
        raise ValueError("model_xml must be a relative/absolute filename or null")
    if config.get("muscle_action") not in (-1, 0) or type(config["muscle_action"]) is bool:
        raise ValueError("muscle_action must be -1 or 0; this does not select a gait controller")
    finite(config.get("target_velocity_m_s"), "target_velocity_m_s")
    channels = config.get("channels")
    if not isinstance(channels, list) or len(channels) != 2 or {c.get("name") for c in channels} != {"right", "left"}:
        raise ValueError("Exactly two named right/left channels required; list order is wire order")
    for key in ("actuator", "joint"):
        names = [c.get(key) for c in channels]
        if any(not isinstance(n, str) or not n for n in names) or len(set(names)) != len(names):
            raise ValueError(f"Distinct nonempty {key} names required")
    for c in channels:
        bounds = c.get("ctrlrange")
        if not isinstance(bounds, list) or len(bounds) != 2:
            raise ValueError("ctrlrange must have two entries")
        lo, hi = [finite(v, "ctrlrange") for v in bounds]
        if hi <= lo or not lo <= 0 <= hi:
            raise ValueError("ctrlrange must be ordered and contain the zero-ctrl off state")
        for k in ("gain", "gear"):
            if finite(c.get(k), k) == 0:
                raise ValueError(f"{k} cannot be zero")
        if finite(c.get("off_ctrl"), "off_ctrl") != 0:
            raise ValueError("Bias-free direct adapter requires off_ctrl=0")
        off = ctrl_to_action(c, c["off_ctrl"])
        if "off_action" in c and not math.isclose(finite(c["off_action"], "off_action"), off, abs_tol=1e-12):
            raise ValueError("off_action contradicts ctrlrange/off_ctrl")
        c["off_action"] = off
        sign = 1 if c["gain"] * c["gear"] > 0 else -1
        if type(c.get("positive_ctrl_joint_sign")) is not int or c["positive_ctrl_joint_sign"] != sign:
            raise ValueError("positive_ctrl_joint_sign must agree with gain*gear; not an additional multiplier")
        if c.get("ctrl_unit") not in ("dimensionless", "Nm", "A") or c.get("output_unit") != "Nm":
            raise ValueError("Declare ctrl_unit (dimensionless/Nm/A) and actuator output_unit=Nm")
        if c.get("torque_coordinate") != "joint" or not isinstance(c.get("direction"), str) or not c["direction"]:
            raise ValueError("Declare joint torque coordinate and direction evidence/unknown status")
        torque_to_action(c, c.get("demo_torque_nm"))
    joints = config.get("state_joints")
    if not isinstance(joints, list) or not joints or any(not isinstance(j, str) or not j for j in joints) or len(set(joints)) != len(joints):
        raise ValueError("state_joints must be unique nonempty names")
    if any(c["joint"] not in joints for c in channels):
        raise ValueError("state_joints must include each device joint")
    camera = config.get("camera", {})
    if not isinstance(camera.get("lookat_body"), str):
        raise ValueError("camera.lookat_body is required")
    for key in ("distance", "azimuth", "elevation", "lookat_z"):
        finite(camera.get(key), "camera." + key)
    if camera["distance"] <= 0:
        raise ValueError("camera distance must be positive")
    return config


def load_config(path=DEFAULT_CONFIG):
    def reject(value):
        raise ValueError(f"Nonfinite JSON constant: {value}")
    def unique(pairs):
        result = {}
        for k, v in pairs:
            if k in result:
                raise ValueError(f"Duplicate config key: {k}")
            result[k] = v
        return result
    path = Path(path).resolve()
    config = validate(json.loads(path.read_text(), parse_constant=reject, object_pairs_hook=unique))
    if config.get("model_xml"):
        model_path = Path(config["model_xml"])
        if not model_path.is_absolute():
            model_path = path.parent / model_path
        if not model_path.is_file():
            raise ValueError(f"Composed model XML missing: {model_path}")
        config["model_xml"] = str(model_path.resolve())
    return config


def model_input(config):
    """Custom delivery can use an already composed, asset-resolvable MJCF without editing packages."""
    if config.get("model_xml"):
        return config["model_xml"]
    from myoassist_utils.compose import compose_env_model
    return compose_env_model(config["msk_key"], config["device_key"], terrain=None)
