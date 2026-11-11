"""ROS 2 Humble (system Python 3.10) -> loopback JSON TCP, no MuJoCo imports."""

import argparse
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, HistoryPolicy, ReliabilityPolicy
from std_msgs.msg import String

from tools.exo_tcp_client import ExoClient
from tools.exo_tcp_server import reject_constant, unique_object


class Bridge(Node):
    def __init__(self, port=8765, prefix="/myoassist", poll_period=0.1):
        super().__init__("myoassist_tcp_bridge")
        self.port = port
        self.client = None
        self.last_seq = -1
        qos = QoSProfile(depth=1, history=HistoryPolicy.KEEP_LAST,
                         reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        self.publisher = self.create_publisher(String, prefix + "/state", 10)
        self.subscription = self.create_subscription(String, prefix + "/command", self.command, qos)
        self.timer = self.create_timer(poll_period, self.poll)
        self.get_logger().info(f"Simulation only: {prefix}/command -> TCP 127.0.0.1:{port} -> {prefix}/state")

    def publish(self, payload):
        msg = String()
        msg.data = json.dumps(payload, allow_nan=False)
        self.publisher.publish(msg)

    def connect(self):
        if self.client is None:
            self.client = ExoClient(self.port, timeout=0.5)

    def disconnect(self):
        if self.client is not None:
            self.client.close()
            self.client = None

    def poll(self):
        try:
            self.connect()
            response = self.client.request("ping")
            response.update(source="poll", command_seq=None)
            self.publish(response)
        except (OSError, ValueError, ConnectionError) as exc:
            self.disconnect()
            self.publish({"version": 1, "ok": False, "source": "poll", "command_seq": None,
                          "error": {"code": "tcp_unavailable", "message": str(exc)}})

    def command(self, msg):
        seq = None
        try:
            req = json.loads(msg.data, parse_constant=reject_constant, object_pairs_hook=unique_object)
            if not isinstance(req, dict):
                raise ValueError("Command must be an object")
            seq = req.get("seq")
            if type(seq) is not int or not 0 <= seq <= 2**53 - 1 or seq <= self.last_seq:
                seq = None
                raise ValueError("seq must strictly increase within this bridge lifetime")
            self.last_seq = seq
            if type(req.get("version")) is not int or req["version"] != 1:
                raise ValueError("version must be integer 1")
            cmd = req.get("cmd")
            if cmd not in ("reset", "step", "ping", "stop"):
                raise ValueError("cmd must be reset, step, ping or stop")
            fields = {k: v for k, v in req.items() if k not in ("seq", "cmd", "version")}
            if cmd == "step" and "epoch" not in fields:
                raise ValueError("step requires epoch returned by reset (prevents replay after timeout)")
            self.connect()
            response = self.client.request(cmd, **fields)
            response.update(source="command", command_seq=seq)
            self.publish(response)
        except (ValueError, TypeError, RecursionError) as exc:
            self.publish({"version": 1, "ok": False, "source": "command", "command_seq": seq,
                          "error": {"code": "invalid_ros_command", "message": str(exc)}})
        except (OSError, ConnectionError) as exc:
            self.disconnect()  # never retry/replay a control command
            self.publish({"version": 1, "ok": False, "source": "command", "command_seq": seq,
                          "error": {"code": "tcp_unavailable", "message": str(exc)}})


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--prefix", default="/myoassist")
    parser.add_argument("--poll-period", type=float, default=0.1)
    args, ros_args = parser.parse_known_args()
    if not 1 <= args.port <= 65535 or not math.isfinite(args.poll_period) or args.poll_period <= 0:
        parser.error("Invalid port or poll-period")
    rclpy.init(args=ros_args)
    node = Bridge(args.port, args.prefix, args.poll_period)
    try:
        rclpy.spin(node)  # single-thread executor: all TCP calls ordered
    except KeyboardInterrupt:
        pass
    finally:
        node.disconnect()  # server detects EOF, applies mapped OFF, latches reset_required
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
