"""Real-process TCP integration checks: .my_venv/bin/python -m unittest tools.test_exo_tcp -v."""

import json
import io
import os
from pathlib import Path
import subprocess
import tempfile
import time
import unittest
from unittest.mock import patch

from tools.exo_tcp_client import ExoClient

ROOT = Path(__file__).resolve().parents[1]


class RunningServer:
    def __init__(self, watchdog=0.6, window=False):
        self.temp = tempfile.TemporaryDirectory(prefix="exo_tcp_")
        self.directory = Path(self.temp.name)
        self.stdout = (self.directory / "stdout.txt").open("w+")
        self.log = self.directory / "server.jsonl"
        self.process = subprocess.Popen(
            [os.environ.get("MYOASSIST_SIM_PYTHON", str(ROOT / ".my_venv/bin/python")), "-m", "tools.exo_tcp_server", "--port", "0",
             "--watchdog", str(watchdog), "--log", str(self.log)] + (["--window"] if window else []),
            cwd=ROOT, env={**os.environ, "MUJOCO_GL": "egl"}, stdout=self.stdout, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 25
        while time.monotonic() < deadline:
            self.stdout.seek(0)
            output = self.stdout.read()
            for line in output.splitlines():
                if line.startswith('{"event": "listening"'):
                    self.port = json.loads(line)["port"]
                    return
            if self.process.poll() is not None:
                break
            time.sleep(0.05)
        self.close()
        raise RuntimeError(f"Server failed to start: {output}")

    def close(self):
        if self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait()
        self.stdout.close()
        self.temp.cleanup()


class TcpTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.server = RunningServer()

    @classmethod
    def tearDownClass(cls):
        cls.server.close()

    def setUp(self):
        self.client = ExoClient(self.server.port)
        self.addCleanup(self.client.close)
        response = self.client.request("reset")
        self.assertTrue(response["ok"], response)

    def assertControls(self, response, controls):
        self.assertTrue(response["ok"], response)
        for actual, expected in zip(response["state"]["actual_ctrl"], controls):
            self.assertAlmostEqual(actual, expected)
        for actual, expected in zip(response["state"]["actuator_force"], controls):
            self.assertAlmostEqual(actual, expected * 100)

    def test_ping_reset_state(self):
        response = self.client.request("ping")
        self.assertControls(response, [0, 0])
        state = response["state"]
        self.assertEqual(state["sim_time_s"], 0)
        self.assertEqual(len(state["joints"]), 6)
        self.assertEqual(state["actuator_names"], ["Exo_R", "Exo_L"])
        self.assertTrue(all(set(j) == {"angle_rad", "velocity_rad_s"} for j in state["joints"].values()))

    def test_mapping_independence_clipping_repeatability(self):
        for action in ([1, 1], [0, 1], [1, 0], [-1, 1], [1, -1], [2, -2]):
            states = []
            expected = [(max(-1, min(1, v)) - 1) / 2 for v in action]
            for _ in range(2):
                self.client.request("reset")
                r = self.client.request("step", action=action, steps=1)
                self.assertControls(r, expected)
                s = r["state"]
                self.assertAlmostEqual(s["sim_time_s"], 1 / 30)
                self.assertEqual(s["env_step_action"][-2:], s["clipped_action"])
                self.assertEqual(s["env_step_action"][:22], [0] * 22)
                states.append({k: s[k] for k in ("sim_time_s", "actual_ctrl", "actuator_force", "joints", "muscle_activation")})
            self.assertEqual(states[0], states[1])

    def test_multistep(self):
        r = self.client.request("step", action=[0.9, 1], steps=3)
        self.assertControls(r, [-0.05, 0])
        self.assertEqual(r["steps_completed"], 3)
        self.assertAlmostEqual(r["state"]["sim_time_s"], 0.1)

    def test_validation(self):
        for fields in ({"action": [1]}, {"action": [True, 0]}, {"action": [1e309, 0]},
                       {"action": [1, 1], "steps": 0}, {"action": [1, 1], "steps": True},
                       {"action": [1, 1], "steps": 11}):
            self.client.seq += 1
            req = {"version": 1, "seq": self.client.seq, "cmd": "step", "epoch": self.client.epoch,
                   "steps": 1, **fields}
            self.client.socket.sendall(json.dumps(req).encode() + b"\n")
            response = self.read_raw()
            self.assertFalse(response["ok"])
            self.assertEqual(response["state"]["sim_time_s"], 0)

    def read_raw(self):
        data = b""
        while b"\n" not in data:
            data += self.client.socket.recv(65536)
        return json.loads(data.split(b"\n")[0])

    def test_bad_json_versions_commands_and_sequence(self):
        for raw, code in ((b"{broken", "invalid_json"), (b'{"x":NaN}', "invalid_json"),
                          (b'{"x":Infinity}', "invalid_json"), (b'{"x":1,"x":2}', "invalid_json")):
            self.client.socket.sendall(raw + b"\n")
            self.assertEqual(self.read_raw()["error"]["code"], code)
        self.assertEqual(self.client.request("unknown")["error"]["code"], "unknown_command")
        self.assertEqual(self.client.request("ping", version=99)["error"]["code"], "bad_version")
        self.client.socket.sendall(b'{"version":1,"seq":0,"cmd":"ping"}\n')
        self.assertEqual(self.read_raw()["error"]["code"], "stale_seq")
        self.assertEqual(self.client.request("step", action=[1, 1], steps=1, epoch=-1)["error"]["code"], "stale_epoch")

    def test_framing(self):
        # TCP segmentation and multiple requests in one write.
        self.client.socket.sendall(b'{"version":1,')
        self.client.socket.sendall(b'"seq":10,"cmd":"ping"}\n{"version":1,"seq":11,"cmd":"ping"}\n')
        f = self.client.socket.makefile("rb")
        self.addCleanup(f.close)
        self.assertEqual(json.loads(f.readline())["seq"], 10)
        self.assertEqual(json.loads(f.readline())["seq"], 11)

    def test_oversize_frame(self):
        self.client.socket.sendall(b"x" * 16385 + b"\n")
        self.assertEqual(self.read_raw()["error"]["code"], "frame_too_large")

    def test_watchdog_latches_and_rejects_old_epoch(self):
        self.assertControls(self.client.request("step", action=[0, 0], steps=1), [-0.5, -0.5])
        old_epoch = self.client.epoch
        time.sleep(0.75)
        r = self.client.request("ping")  # ping must not renew the control deadline
        self.assertControls(r, [0, 0])
        self.assertEqual(r["state"]["status"], "watchdog_timeout")
        self.assertEqual(r["state"]["effective_exo_action"], [1, 1])
        self.assertEqual(self.client.request("step", action=[0, 0], steps=1)["error"]["code"], "reset_required")
        self.client.request("reset")
        self.assertEqual(self.client.request("step", action=[0, 0], steps=1, epoch=old_epoch)["error"]["code"], "stale_epoch")

    def test_disconnect_closes_devices(self):
        self.client.request("step", action=[0, 0], steps=1)
        self.client.close()
        with ExoClient(self.server.port) as other:
            r = other.request("ping")
            self.assertControls(r, [0, 0])
            self.assertEqual(r["state"]["status"], "disconnected")

    def test_terminal_blocks_more_steps(self):
        for _ in range(60):
            r = self.client.request("step", action=[1, 1], steps=10)
            if r["state"]["status"] == "terminated":
                break
        self.assertEqual(r["state"]["status"], "terminated")
        self.assertControls(r, [0, 0])
        t = r["state"]["sim_time_s"]
        r = self.client.request("step", action=[0, 0], steps=1)
        self.assertEqual(r["error"]["code"], "reset_required")
        self.assertEqual(r["state"]["sim_time_s"], t)

    def test_z_stop(self):
        self.assertControls(self.client.request("stop"), [0, 0])
        self.server.process.wait(timeout=5)
        self.assertEqual(self.server.process.returncode, 0)


class AdapterTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        os.environ.setdefault("MUJOCO_GL", "egl")
        from tools.exo_tcp_server import Simulation
        cls.sim = Simulation()

    @classmethod
    def tearDownClass(cls):
        cls.sim.close()

    def test_exception_closes_and_latches(self):
        from tools.exo_tcp_server import Server
        server = Server(self.sim, io.StringIO())
        _, last = server.request(b'{"version":1,"seq":1,"cmd":"reset"}', -1)
        request = json.dumps({"version": 1, "seq": 2, "cmd": "step", "action": [0, 0],
                              "steps": 1, "epoch": server.epoch}).encode()
        with patch.object(self.sim.env, "step", side_effect=RuntimeError("injected simulation failure")):
            response, last = server.request(request, last)
        self.assertEqual(response["error"]["code"], "simulation_fault")
        self.assertEqual(response["state"]["status"], "fault")
        self.assertEqual(response["state"]["actual_ctrl"], [0, 0])
        response, _ = server.request(request.replace(b'"seq": 2', b'"seq": 3'), last)
        self.assertEqual(response["error"]["code"], "reset_required")

    def test_configuration_channel_order(self):
        from tools.exo_tcp_server import DEFAULT_CONFIG, Simulation
        config = json.loads(DEFAULT_CONFIG.read_text())
        config["channels"].reverse()
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "reversed.json"
            path.write_text(json.dumps(config))
            sim = Simulation(config_path=path)
            try:
                sim.step([0, 1])
                self.assertEqual(sim.model.actuator(sim.ids[0]).name, "Exo_L")
                self.assertEqual(sim.data.actuator("Exo_L").ctrl[0], -0.5)
                self.assertEqual(sim.data.actuator("Exo_R").ctrl[0], 0)
            finally:
                sim.close()


if __name__ == "__main__":
    unittest.main()
