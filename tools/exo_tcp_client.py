"""Standard-library JSON-lines client. Run with python -m tools.exo_tcp_client."""

import argparse
import json
import math
import socket
from tools.exo_device import off_actions, pulse_actions


class ExoClient:
    def __init__(self, port=8765, timeout=5.0):
        self.socket = socket.create_connection(("127.0.0.1", port), timeout=timeout)
        self.buffer = b""
        self.seq = 0
        self.epoch = None

    def close(self):
        self.socket.close()

    def __enter__(self):
        return self

    def __exit__(self, *args):
        self.close()

    def request(self, cmd, **fields):
        self.seq += 1
        if cmd == "step":
            if self.epoch is None:
                raise ValueError("Call reset on this connection before step")
            fields.setdefault("epoch", self.epoch)
        req = {"version": 1, "seq": self.seq, "cmd": cmd, **fields}
        self.socket.sendall(json.dumps(req, allow_nan=False).encode() + b"\n")
        while b"\n" not in self.buffer:
            chunk = self.socket.recv(4096)
            if not chunk:
                raise ConnectionError("Server closed connection")
            self.buffer += chunk
            if len(self.buffer) > 1024 * 1024:
                raise ConnectionError("Response too large")
        line, self.buffer = self.buffer.split(b"\n", 1)
        response = json.loads(line)
        if response.get("version") != 1 or response.get("seq") != self.seq:
            raise ConnectionError("Unexpected response version or seq")
        if cmd == "reset" and response["ok"]:
            self.epoch = response["state"]["epoch"]
        return response


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("cmd", choices=("ping", "reset", "step", "stop", "demo"))
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--right", type=float, default=None, help="normalized action, default configured OFF")
    parser.add_argument("--left", type=float, default=None, help="normalized action, default configured OFF")
    parser.add_argument("--steps", type=int, default=1)
    args = parser.parse_args()
    if not all(v is None or math.isfinite(v) for v in (args.right, args.left)):
        parser.error("actions must be finite")
    with ExoClient(args.port) as client:
        commands = []
        if args.cmd in ("step", "demo"):
            # Each CLI invocation owns a new connection/episode, intentionally.
            commands.append(("reset", {}))
        if args.cmd not in ("demo", "step"):
            commands.append((args.cmd, {}))
        for cmd, fields in commands:
            response = client.request(cmd, **fields)
            print(json.dumps(response, indent=2, ensure_ascii=False))
            if not response["ok"]:
                raise SystemExit(1)
        if args.cmd in ("demo", "step"):
            config = response["state"]["device_config"]
            if args.cmd == "demo":
                actions = [off_actions(config), pulse_actions(config, "right"),
                           pulse_actions(config, "left"), off_actions(config)]
            else:
                overrides = {"right": args.right, "left": args.left}
                actions = [[c["off_action"] if overrides[c["name"]] is None else overrides[c["name"]]
                            for c in config["channels"]]]
            for action in actions:
                reply = client.request("step", action=action, steps=args.steps)
                print(json.dumps(reply, indent=2, ensure_ascii=False))
                if not reply["ok"] or reply["state"]["status"] != "ready":
                    raise SystemExit(1)


if __name__ == "__main__":
    main()
