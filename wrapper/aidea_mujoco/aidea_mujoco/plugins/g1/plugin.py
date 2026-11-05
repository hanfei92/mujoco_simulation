import mujoco
import numpy as np

from ..base import RobotPlugin


class G1Plugin(RobotPlugin):
    def __init__(self, mujoco_server, config):
        super().__init__(mujoco_server, config)
        self.default_angles = np.array(config["default_angles"], dtype=np.float32)
        self.policy = None

    def on_load(self, mjmodel, mjdata):
        self.mjmodel = mjmodel
        prefix = f"{self.robot_id}_"

        self.qpos_idx = []
        self.dof_idx = []
        self.act_idx = []
        for jn in self.config["leg_joint_names"]:
            full_j = f"{self.robot_id}_{jn}"
            jid = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_JOINT, full_j)
            if jid < 0:
                print(f"[g1] joint not found: {full_j}")
                continue
            self.qpos_idx.append(mjmodel.jnt_qposadr[jid])
            self.dof_idx.append(mjmodel.jnt_dofadr[jid])
            # actuator 名和 joint 名一样，不带 _motor
            aid = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_ACTUATOR, full_j)
            if aid >= 0:
                self.act_idx.append(aid)
            else:
                print(f"[g1] actuator not found: {full_j}")

        # 非腿部 actuator 冻结
        self.freeze_act_ids = []
        for i in range(mjmodel.nu):
            name = mujoco.mj_id2name(mjmodel, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            if name and name.startswith(prefix) and i not in self.act_idx:
                self.freeze_act_ids.append(i)

        # 找 root body 和 IMU sensor（可选）
        self.root_body_id = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_BODY, f"{self.robot_id}_pelvis")
        self.sensor_gyro_id = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_SENSOR, f"{self.robot_id}_imu_gyro")
        self.sensor_acc_id = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_SENSOR, f"{self.robot_id}_imu_acc")
        self.sensor_quat_id = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_SENSOR, f"{self.robot_id}_imu_quat")

        print(f"[g1] matched {len(self.qpos_idx)} legs, {len(self.act_idx)} leg actuators, "
              f"{len(self.freeze_act_ids)} freeze actuators, root_body={self.root_body_id}, "
              f"gyro={self.sensor_gyro_id}, acc={self.sensor_acc_id}, quat={self.sensor_quat_id}")

    def step(self, mjdata):
        # Stage 7: 只冻结非腿部关节
        for aid in self.freeze_act_ids:
            mjdata.ctrl[aid] = 0.0