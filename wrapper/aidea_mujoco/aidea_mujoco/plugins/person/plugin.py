"""Mocap person: the follow target the bridge moves around the scene."""

import math

import mujoco

from ..base import RobotPlugin


class PersonPlugin(RobotPlugin):
    def __init__(self, mujoco_server, config):
        super().__init__(mujoco_server, config)
        self.body_id = -1
        self.mocap_id = -1

    def on_load(self, mjmodel, mjdata):
        self.body_id = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_BODY,
                                         self.robot_id)
        if self.body_id < 0 or mjmodel.body_mocapid[self.body_id] < 0:
            print(f"[person] mocap body '{self.robot_id}' not found; add mjcf/person.xml and include it from scene.xml")
            return
        self.mocap_id = int(mjmodel.body_mocapid[self.body_id])
        spawn = self.config.get("spawn", [2.0, 0.0, 0.0])
        mjdata.mocap_pos[self.mocap_id] = [float(spawn[0]),
                                           float(spawn[1]), 0.0]
        mjdata.mocap_quat[self.mocap_id] = [1.0, 0.0, 0.0, 0.0]
        print(f"[person] mocap '{self.robot_id}' id={self.mocap_id} "
              f"spawn={spawn}")

    def step(self, mjdata):
        if self.mocap_id < 0:
            return
        commands = self.mujoco_server.data_proxies.get_robot_command() or {}
        command = commands.get(self.robot_id)
        if not command or "x" not in command:
            return
        mjdata.mocap_pos[self.mocap_id] = [float(command["x"]),
                                           float(command["y"]), 0.0]
        half = float(command.get("yaw", 0.0)) * 0.5
        mjdata.mocap_quat[self.mocap_id] = [math.cos(half), 0.0, 0.0,
                                            math.sin(half)]
