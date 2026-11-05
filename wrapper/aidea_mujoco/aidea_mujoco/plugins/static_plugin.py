import mujoco
import numpy as np

from .base import RobotPlugin


class StaticPlugin(RobotPlugin):
    """没有策略的机器人：把 actuator 保持在默认角度。"""

    def on_load(self, mjmodel, mjdata):
        self.actuator_ids = []
        self.default_ctrl = []
        prefix = f"{self.robot_id}_"
        for i in range(mjmodel.nu):
            name = mujoco.mj_id2name(mjmodel, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name and name.startswith(prefix):
                self.actuator_ids.append(i)
                self.default_ctrl.append(0.0)
        self.default_ctrl = np.array(self.default_ctrl, dtype=np.float64)
        print(f"[static] {self.robot_id}: matched {len(self.actuator_ids)} actuators")

    def step(self, mjdata):
        self.step_counter += 1
        if self.step_counter == 1:
            print(f"[static] {self.robot_id} first step called")
        if self.step_counter % 500 == 0:
            print(f"[static] {self.robot_id} step called, counter={self.step_counter}")
        for i, aid in enumerate(self.actuator_ids):
            mjdata.ctrl[aid] = self.default_ctrl[i]