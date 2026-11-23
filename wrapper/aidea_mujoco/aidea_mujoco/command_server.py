"""ZMQ command channel for the person-following bridge.

The ROS bridge connects a PUSH socket here and sends ``aidea.command.Command``
messages.  Commands are written into the same ``robot_command`` dict the MCP
tools use, so the G1 plugin and the person plugin pick them up on their next
step without any additional plumbing.
"""

import threading

import zmq

from aidea_mujoco.msg import command_pb2


class CommandServer:
    def __init__(self, data_proxies, endpoint="tcp://*:5556"):
        self.data_proxies = data_proxies
        self.endpoint = endpoint
        self._ctx = zmq.Context()
        self._sock = self._ctx.socket(zmq.PULL)
        self._sock.setsockopt(zmq.LINGER, 0)
        self._sock.bind(endpoint)
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True,
                                        name="zmq-command")
        self._thread.start()

    def _loop(self):
        while self._running:
            try:
                raw = self._sock.recv()
            except zmq.ZMQError:
                break
            except Exception:
                continue
            command = command_pb2.Command()
            try:
                command.ParseFromString(raw)
            except Exception:
                continue
            try:
                self._apply(command)
            except Exception as exc:  # keep the sim alive on malformed input
                print(f"[command] apply failed: {exc}")

    def _apply(self, command):
        cmds = self.data_proxies.get_robot_command() or {}
        if command.kind == "person":
            cmds["person"] = {"x": float(command.x), "y": float(command.y),
                              "yaw": float(command.yaw)}
        else:
            robot_id = command.robot_id or "g1"
            entry = dict(cmds.get(robot_id, {}))
            entry["vx"] = float(command.vx)
            entry["wz"] = float(command.omega)
            if command.vy:
                entry["vy"] = float(command.vy)
            if command.mode:
                entry["mode"] = command.mode
            cmds[robot_id] = entry
        self.data_proxies.set_robot_command(cmds)

    def close(self):
        self._running = False
        try:
            self._sock.close(0)
        except Exception:
            pass
        try:
            self._ctx.term()
        except Exception:
            pass
