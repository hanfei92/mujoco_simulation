# import os
# import mujoco
# import numpy as np

# from ..base import RobotPlugin


# class G1Plugin(RobotPlugin):

#     def __init__(self, mujoco_server, config):
#         super().__init__(mujoco_server, config)
#         self.kps = np.array(config["kps"], dtype=np.float32)
#         self.kds = np.array(config["kds"], dtype=np.float32)
#         self.stand_height = config.get("stand_height", 0.793)
#         self.stand_angles = np.array(
#             config.get("stand_angles", config["default_angles"]), dtype=np.float32
#         )

#         # walk 假步态参数
#         self.walk_amplitude = config.get("walk_amplitude", 0.3)
#         self.walk_period = config.get("walk_period", 0.8)

#         # fall 模式：加载 motion.pt
#         self.fall_policy = None
#         self.fall_loaded = False
#         policy_path = config.get("policy_path")
#         if policy_path:
#             try:
#                 import torch
#                 full_path = os.path.join(os.path.dirname(__file__), policy_path)
#                 if os.path.exists(full_path):
#                     self.fall_policy = torch.jit.load(full_path)
#                     self.fall_policy.eval()
#                     self.fall_loaded = True
#                     print(f"[g1] loaded fall policy from {full_path}")
#             except Exception as e:
#                 print(f"[g1] failed to load fall policy: {e}")

#         self.fall_last_action = np.zeros(12, dtype=np.float32)
#         self.fall_cmd = np.array([0.5, 0.0, 0.0], dtype=np.float32)

#         self.mode = "stand"          # "stand" | "walk" | "fall"
#         self.base_x = 0.0
#         self.base_y = 0.0
#         self.base_yaw = 0.0
#         self.vx = 0.0
#         self.wz = 0.0
#         self.teleport = None

#     def step(self, mjdata):
#         self.step_counter += 1

#         # 读 MCP 指令
#         if self.step_counter % 10 == 0:
#             cmds = self.mujoco_server.data_proxies.get_robot_command()
#             c = cmds.get(self.robot_id, {})
#             if c:
#                 self.mode = c.get("mode", self.mode)
#                 self.vx = c.get("vx", 0.0)
#                 self.wz = c.get("wz", 0.0)
#                 if c.get("teleport"):
#                     tp = c["teleport"]
#                     self.base_x = tp["x"]
#                     self.base_y = tp["y"]
#                     self.base_yaw = tp["yaw"]
#                     c["teleport"] = None
#                     cmds[self.robot_id] = c
#                     self.mujoco_server.data_proxies.set_robot_command(cmds)

#         # 冻结非腿部
#         for aid in self.freeze_act_ids:
#             mjdata.ctrl[aid] = 0.0

#         if self.mode == "stand":
#             self._step_stand(mjdata)
#         elif self.mode == "walk":
#             self._step_walk(mjdata)
#         elif self.mode == "fall":
#             self._step_fall(mjdata)

#     def _step_stand(self, mjdata):
#         qpos = mjdata.qpos[self.qpos_idx]
#         qvel = mjdata.qvel[self.dof_idx]
#         tau = self.kps * (self.stand_angles - qpos) - self.kds * qvel
#         for i, aid in enumerate(self.act_idx):
#             mjdata.ctrl[aid] = tau[i]
#         self._lock_pelvis(mjdata, self.base_x, self.base_y, self.stand_height, self.base_yaw)

#     def _step_walk(self, mjdata):
#         t = mjdata.time
#         phase = 2 * np.pi * t / self.walk_period
#         swing = self.walk_amplitude * np.sin(phase)
#         target = self.stand_angles.copy()
#         for i, jn in enumerate(self.leg_joint_names):
#             if "left_hip_pitch" in jn:
#                 target[i] += swing
#             elif "right_hip_pitch" in jn:
#                 target[i] -= swing
#             elif "left_knee" in jn:
#                 target[i] += abs(swing) * 0.5
#             elif "right_knee" in jn:
#                 target[i] += abs(swing) * 0.5
#         qpos = mjdata.qpos[self.qpos_idx]
#         qvel = mjdata.qvel[self.dof_idx]
#         tau = self.kps * (target - qpos) - self.kds * qvel
#         for i, aid in enumerate(self.act_idx):
#             mjdata.ctrl[aid] = tau[i]
#         # 平移
#         if self.step_counter % 10 == 0:
#             dt = self.mjmodel.opt.timestep * 10
#             self.base_x += self.vx * np.cos(self.base_yaw) * dt
#             self.base_y += self.vx * np.sin(self.base_yaw) * dt
#             self.base_yaw += self.wz * dt
#         self._lock_pelvis(mjdata, self.base_x, self.base_y, self.stand_height, self.base_yaw)

#     def _step_fall(self, mjdata):
#         if not self.fall_loaded:
#             # 没加载策略，退化成 stand
#             self._step_stand(mjdata)
#             return
#         # 每 10 步推理一次
#         if self.step_counter % 10 != 0:
#             return
#         obs = self._build_fall_obs(mjdata)
#         with torch.no_grad():
#             action = self.fall_policy(obs)
#         action = action.squeeze(0).numpy()
#         self.fall_last_action = action
#         target = action * 0.25 + self.stand_angles
#         qpos = mjdata.qpos[self.qpos_idx]
#         qvel = mjdata.qvel[self.dof_idx]
#         tau = self.kps * (target - qpos) - self.kds * qvel
#         for i, aid in enumerate(self.act_idx):
#             mjdata.ctrl[aid] = tau[i]
#         # 不固定 pelvis，让重力/力矩自由作用，身体会倒

#     def _lock_pelvis(self, mjdata, x, y, z, yaw):
#         if self.free_joint_qadr is None:
#             return
#         mjdata.qpos[self.free_joint_qadr] = x
#         mjdata.qpos[self.free_joint_qadr+1] = y
#         mjdata.qpos[self.free_joint_qadr+2] = z
#         cy, sy = np.cos(yaw/2), np.sin(yaw/2)
#         mjdata.qpos[self.free_joint_qadr+3:self.free_joint_qadr+7] = [cy, 0, 0, sy]
#         mjdata.qvel[self.free_joint_vadr:self.free_joint_vadr+6] = 0

import os
import mujoco
import numpy as np

from ..base import RobotPlugin


class G1Plugin(RobotPlugin):

    def __init__(self, mujoco_server, config):
        super().__init__(mujoco_server, config)

        self.kps = np.array(config["kps"], dtype=np.float32)
        self.kds = np.array(config["kds"], dtype=np.float32)
        self.stand_height = config.get("stand_height", 0.793)
        self.stand_angles = np.array(
            config.get("stand_angles", config["default_angles"]), dtype=np.float32
        )
        self.leg_joint_names = config["leg_joint_names"]

        # walk 假步态参数
        self.walk_amplitude = config.get("walk_amplitude", 0.3)
        self.walk_period = config.get("walk_period", 0.8)

        # fall 模式
        self.fall_policy = None
        self.fall_loaded = False
        policy_path = config.get("policy_path")
        if policy_path:
            try:
                import torch
                full_path = os.path.join(os.path.dirname(__file__), policy_path)
                if os.path.exists(full_path):
                    self.fall_policy = torch.jit.load(full_path)
                    self.fall_policy.eval()
                    self.fall_loaded = True
                    print(f"[g1] loaded fall policy from {full_path}")
            except Exception as e:
                print(f"[g1] failed to load fall policy: {e}")

        self.fall_last_action = np.zeros(12, dtype=np.float32)
        self.fall_cmd = np.array([0.5, 0.0, 0.0], dtype=np.float32)

        spawn = config.get("spawn_pos", [0.0, 0.0, 0.0])
        self.mode = "stand"
        self.base_x = float(spawn[0])
        self.base_y = float(spawn[1])
        self.base_yaw = float(spawn[2])
        self.vx = 0.0
        self.wz = 0.0
        self.teleport = None

        # ---- step 依赖的所有属性，全部给默认值 ----
        self.mjmodel = None
        self.step_counter = 0
        self.qpos_idx = []
        self.dof_idx = []
        self.act_idx = []
        self.freeze_act_ids = []
        self.free_joint_qadr = None
        self.free_joint_vadr = None

    def on_load(self, mjmodel, mjdata):
        self.mjmodel = mjmodel
        prefix = f"{self.robot_id}_"

        # ---- joint id -> actuator id，不依赖命名 ----
        joint_to_act = {}
        for aid in range(mjmodel.nu):
            if mjmodel.actuator_trntype[aid] == mujoco.mjtTrn.mjTRN_JOINT:
                jid = mjmodel.actuator_trnid[aid, 0]
                joint_to_act[jid] = aid

        self.qpos_idx = []
        self.dof_idx = []
        self.act_idx = []
        for jn in self.leg_joint_names:
            full_j = f"{self.robot_id}_{jn}"
            jid = mujoco.mj_name2id(mjmodel, mujoco.mjtObj.mjOBJ_JOINT, full_j)
            if jid < 0:
                print(f"[g1] joint not found: {full_j}")
                continue
            self.qpos_idx.append(mjmodel.jnt_qposadr[jid])
            self.dof_idx.append(mjmodel.jnt_dofadr[jid])
            aid = joint_to_act.get(jid)
            if aid is not None:
                self.act_idx.append(aid)
            else:
                print(f"[g1] no actuator for joint: {full_j}")

        # 冻结非腿部（属于 g1 的、不在 act_idx 里的 actuator）
        leg_aid_set = set(self.act_idx)
        self.freeze_act_ids = []
        for aid in range(mjmodel.nu):
            if mjmodel.actuator_trntype[aid] != mujoco.mjtTrn.mjTRN_JOINT:
                continue
            jid = mjmodel.actuator_trnid[aid, 0]
            jname = mujoco.mj_id2name(mjmodel, mujoco.mjtObj.mjOBJ_JOINT, jid)
            if jname and jname.startswith(prefix) and aid not in leg_aid_set:
                self.freeze_act_ids.append(aid)

        # 找 pelvis free joint
        self.free_joint_qadr = None
        self.free_joint_vadr = None
        for i in range(mjmodel.njnt):
            name = mujoco.mj_id2name(mjmodel, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name and name.startswith(prefix) and mjmodel.jnt_type[i] == mujoco.mjtJoint.mjJNT_FREE:
                self.free_joint_qadr = mjmodel.jnt_qposadr[i]
                self.free_joint_vadr = mjmodel.jnt_dofadr[i]
                break

        # 腿部 qpos 设到 stand_angles
        for i, qadr in enumerate(self.qpos_idx):
            mjdata.qpos[qadr] = self.stand_angles[i]
        if self.free_joint_qadr is not None:
            mjdata.qpos[self.free_joint_qadr:self.free_joint_qadr+3] = [0, 0, self.stand_height]
            mjdata.qpos[self.free_joint_qadr+3:self.free_joint_qadr+7] = [1, 0, 0, 0]
        mujoco.mj_forward(mjmodel, mjdata)

        print(f"[g1] on_load: {len(self.act_idx)} leg actuators, "
              f"{len(self.freeze_act_ids)} frozen, "
              f"free_joint_qadr={self.free_joint_qadr}")

    def step(self, mjdata):
        # on_load 没跑/没找到 actuator，直接返回，不要抛
        if not self.act_idx:
            return

        self.step_counter += 1

        # 读 MCP 指令
        if self.step_counter % 10 == 0:
            cmds = self.mujoco_server.data_proxies.get_robot_command()
            c = cmds.get(self.robot_id, {})
            if c:
                self.mode = c.get("mode", self.mode)
                self.vx = c.get("vx", self.vx)
                self.wz = c.get("wz", self.wz)
                if c.get("teleport"):
                    tp = c["teleport"]
                    self.base_x = tp["x"]
                    self.base_y = tp["y"]
                    self.base_yaw = tp["yaw"]
                    c["teleport"] = None
                    cmds[self.robot_id] = c
                    self.mujoco_server.data_proxies.set_robot_command(cmds)

        # 冻结非腿部
        for aid in self.freeze_act_ids:
            mjdata.ctrl[aid] = 0.0

        if self.mode == "stand":
            self._step_stand(mjdata)
        elif self.mode == "walk":
            self._step_walk(mjdata)
        elif self.mode == "fall":
            self._step_fall(mjdata)

        # 调试打印：每 500 步一次
        if self.step_counter % 500 == 0:
            print(f"[g1] mode={self.mode} vx={self.vx:.2f} wz={self.wz:.2f} "
                  f"base_x={self.base_x:.2f} qpos0={mjdata.qpos[self.qpos_idx[0]]:.3f}")

    def _step_stand(self, mjdata):
        qpos = mjdata.qpos[self.qpos_idx]
        qvel = mjdata.qvel[self.dof_idx]
        tau = self.kps * (self.stand_angles - qpos) - self.kds * qvel
        for i, aid in enumerate(self.act_idx):
            mjdata.ctrl[aid] = tau[i]
        self._lock_pelvis(mjdata, self.base_x, self.base_y, self.stand_height, self.base_yaw)

    def _step_walk(self, mjdata):
        t = mjdata.time
        phase = 2 * np.pi * t / self.walk_period
        swing = self.walk_amplitude * np.sin(phase)
        target = self.stand_angles.copy()
        for i, jn in enumerate(self.leg_joint_names):
            if "left_hip_pitch" in jn:
                target[i] += swing
            elif "right_hip_pitch" in jn:
                target[i] -= swing
            elif "left_knee" in jn or "right_knee" in jn:
                target[i] += abs(swing) * 0.5
        qpos = mjdata.qpos[self.qpos_idx]
        qvel = mjdata.qvel[self.dof_idx]
        tau = self.kps * (target - qpos) - self.kds * qvel
        for i, aid in enumerate(self.act_idx):
            mjdata.ctrl[aid] = tau[i]

        # 平移：每 10 步积分一次
        if self.step_counter % 10 == 0:
            dt = self.mjmodel.opt.timestep * 10
            self.base_x += self.vx * np.cos(self.base_yaw) * dt
            self.base_y += self.vx * np.sin(self.base_yaw) * dt
            self.base_yaw += self.wz * dt
        self._lock_pelvis(mjdata, self.base_x, self.base_y, self.stand_height, self.base_yaw)

    def _step_fall(self, mjdata):
        if not self.fall_loaded:
            self._step_stand(mjdata)
            return
        if self.step_counter % 10 != 0:
            return
        obs = self._build_fall_obs(mjdata)
        import torch
        with torch.no_grad():
            action = self.fall_policy(obs)
        action = action.squeeze(0).numpy()
        self.fall_last_action = action
        target = action * 0.25 + self.stand_angles
        qpos = mjdata.qpos[self.qpos_idx]
        qvel = mjdata.qvel[self.dof_idx]
        tau = self.kps * (target - qpos) - self.kds * qvel
        for i, aid in enumerate(self.act_idx):
            mjdata.ctrl[aid] = tau[i]

    def _build_fall_obs(self, mjdata):
        import torch
        # 和 Stage 8 的 47 维观测一致，用 free_joint_qadr 动态偏移
        qadr = self.free_joint_qadr
        vadr = self.free_joint_vadr
        ang_vel = mjdata.qvel[vadr+3:vadr+6].astype(np.float32) * 0.25
        quat = mjdata.qpos[qadr+3:qadr+7]
        w, x, y, z = quat
        gravity = np.array([2*(x*z - w*y), 2*(y*z + w*x), 1-2*(x*x+y*y)], dtype=np.float32)
        cmd_scaled = self.fall_cmd * np.array([2.0, 2.0, 0.25], dtype=np.float32)
        qpos = mjdata.qpos[self.qpos_idx].astype(np.float32)
        joint_pos_rel = qpos - self.stand_angles
        qvel = mjdata.qvel[self.dof_idx].astype(np.float32) * 0.05
        t = mjdata.time
        phase = (t % 0.8) / 0.8
        gait = np.array([np.sin(2*np.pi*phase), np.cos(2*np.pi*phase)], dtype=np.float32)
        obs = np.concatenate([ang_vel, gravity, cmd_scaled, joint_pos_rel, qvel,
                              self.fall_last_action, gait])
        return torch.from_numpy(obs).unsqueeze(0)

    def _lock_pelvis(self, mjdata, x, y, z, yaw):
        if self.free_joint_qadr is None:
            return
        mjdata.qpos[self.free_joint_qadr] = x
        mjdata.qpos[self.free_joint_qadr+1] = y
        mjdata.qpos[self.free_joint_qadr+2] = z
        cy, sy = np.cos(yaw/2), np.sin(yaw/2)
        mjdata.qpos[self.free_joint_qadr+3:self.free_joint_qadr+7] = [cy, 0, 0, sy]
        mjdata.qvel[self.free_joint_vadr:self.free_joint_vadr+6] = 0