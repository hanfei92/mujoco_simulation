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
        # kind="robot" drives the wheeled base (the observer);
        # kind="person" drives the G1 (a walking velocity for the target).
        if command.kind == "person":
            cmds = self.data_proxies.get_robot_command() or {}
            robot_id = command.robot_id or "g1"
            entry = dict(cmds.get(robot_id, {}))
            entry["vx"] = float(command.vx)
            entry["wz"] = float(command.omega)
            entry["mode"] = command.mode or "walk"
            cmds[robot_id] = entry
            self.data_proxies.set_robot_command(cmds)
            return

        from aidea_mujoco.datamodels.status_command import CommandBaseVelocity

        status = self.data_proxies.get_command()
        status.set_base_velocity(CommandBaseVelocity(
            v_x=float(command.vx), v_y=float(command.vy),
            omega=float(command.omega), trigger=True))
        self.data_proxies.set_command(status)

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
