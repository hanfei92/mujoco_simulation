"""
原生 mujoco.viewer + pynput 键盘控制
适配：四轮差速底盘 + 6 轴机械臂 + 夹爪

重构要点：
- pynput.Listener 只负责把按键塞进 key_buffer
- process_keys 只更新控制器状态，不写 data.ctrl
- 主循环统一 apply(dt) 写 data.ctrl，再 mj_step
- 底盘带加速/减速 ramp，避免停车前后晃
"""

from time import sleep, perf_counter
from pynput import keyboard

import click
import mujoco
import mujoco.viewer

import numpy as np


# =========================================================
# 工具函数
# =========================================================
def actuator_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)


def joint_id(model, name):
    return mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, name)


def print_keyboard_options():
    click.secho("\n       Keyboard Controls:", fg="yellow")
    click.secho("=====================================", fg="yellow")
    print("W / S       : 前进 / 后退")
    print("A / D       : 左转 / 右转 (差速)")
    print("Q / E       : 左转 / 右转 (备用)")
    print()
    print("U / J       : joint2 +/-")
    print("I / K       : joint3 +/-")
    print("O / L       : joint4 +/-")
    print("P / ;       : joint5 +/-")
    print("[ / ]       : joint6 +/-")
    print(", / .       : joint6output +/-")
    print()
    print("N / M       : 夹爪 开 / 合")
    print("Z           : 打印状态")
    print("X           : 停止底盘")
    print("ctrl + )    : 启用键盘控制")
    print("ctrl + (    : 禁用键盘控制")
    print("ctrl + @    : 提高底盘速度")
    print("ctrl + !    : 降低底盘速度")
    click.secho("=====================================", fg="yellow")


# =========================================================
# 底盘控制器（只维护状态 + ramp，不写 data.ctrl）
# =========================================================
class BaseController:
    def __init__(self):
        self.forward_velocity = 40.0     # rad/s，对应 2 m/s
        self.clockwise_velocity = 50.0    # rad/s，对应车体约 1.2 rad/s
        self.is_forward = None           # True 前进, False 后退, None 停
        self.is_clockwise = None         # True 左转, False 右转, None 不转

        # ramp 状态
        self.current_v = 0.0
        self.current_w = 0.0
        self.accel_limit = 40.0          # rad/s²
        self.decel_limit = 80.0          # rad/s²

    def get_wheel_speeds(self, dt):
        v_target = 0.0
        w_target = 0.0
        if self.is_forward is not None:
            v_target = self.forward_velocity if self.is_forward else -self.forward_velocity
        if self.is_clockwise is not None:
            w_target = self.clockwise_velocity if self.is_clockwise else -self.clockwise_velocity

        # 判断加速还是减速
        rate_v = self.accel_limit if abs(v_target) > abs(self.current_v) else self.decel_limit
        rate_w = self.accel_limit if abs(w_target) > abs(self.current_w) else self.decel_limit

        max_delta_v = rate_v * dt
        max_delta_w = rate_w * dt

        self.current_v += max(-max_delta_v, min(max_delta_v, v_target - self.current_v))
        self.current_w += max(-max_delta_w, min(max_delta_w, w_target - self.current_w))

        # 接近 0 且目标为 0 时直接归零
        if v_target == 0.0 and abs(self.current_v) < 0.01:
            self.current_v = 0.0
        if w_target == 0.0 and abs(self.current_w) < 0.01:
            self.current_w = 0.0

        left = self.current_v - self.current_w
        right = self.current_v + self.current_w
        return left, right

    def increase_base_velocity(self):
        self.forward_velocity += 4
        self.clockwise_velocity += 0.5
        print(f"forward_velocity={self.forward_velocity:.2f}, "
              f"clockwise_velocity={self.clockwise_velocity:.2f}")

    def decrease_base_velocity(self):
        self.forward_velocity = max(0.0, self.forward_velocity - 4)
        self.clockwise_velocity = max(0.0, self.clockwise_velocity - 0.5)
        print(f"forward_velocity={self.forward_velocity:.2f}, "
              f"clockwise_velocity={self.clockwise_velocity:.2f}")

    def stop(self):
        self.is_forward = None
        self.is_clockwise = None


# =========================================================
# 机械臂 + 夹爪控制器（只维护状态，不写 data.ctrl）
# =========================================================
class ArmController:
    JOINT_NAMES = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]
    JOINT_RANGES = [
        (-2.9321, 2.9321),
        (-2.4434, 2.4434),
        (-2.6179, 2.6179),
        (-2.6179, 2.6179),
        (-2.7052, 2.7925),
        (-3.14, 3.14159),
    ]
    JOINT_STEP = 0.1

    def __init__(self, model, data):
        self.targets = {}
        for name in self.JOINT_NAMES:
            jid = joint_id(model, name)
            if jid >= 0:
                self.targets[name] = float(data.qpos[model.jnt_qposadr[jid]])
            else:
                self.targets[name] = 0.0

        self.gripper_target = 0.0
        self.gripper_step = 0.05
        self.gripper_range = (-0.74, 0.15)

    @staticmethod
    def _clamp(v, lo, hi):
        return max(lo, min(hi, v))

    def move_joint(self, name, delta):
        idx = self.JOINT_NAMES.index(name)
        lo, hi = self.JOINT_RANGES[idx]
        self.targets[name] = self._clamp(self.targets[name] + delta, lo, hi)

    def move_gripper(self, delta):
        lo, hi = self.gripper_range
        self.gripper_target = self._clamp(self.gripper_target + delta, lo, hi)


# =========================================================
# 键盘控制器：只更新状态，不写 data.ctrl
# =========================================================
class KeyboardController:
    ACT_NAMES = [
        "left_behind_wheel", "left_front_wheel",
        "right_behind_wheel", "right_front_wheel",
        "joint2_to_joint1", "joint3_to_joint2",
        "joint4_to_joint3", "joint5_to_joint4",
        "joint6_to_joint5", "joint6output_to_joint6",
        "gripper_controller",
    ]

    def __init__(self, model, data):
        self.model = model
        self.data = data

        self.act_ids = {n: actuator_id(model, n) for n in self.ACT_NAMES}
        for n, i in self.act_ids.items():
            if i < 0:
                print(f"[WARN] actuator not found: {n}")

        self.base = BaseController()
        self.arm = ArmController(model, data)

        self.key_buffer = []
        self.is_keyboard_control_active = True
        self.ctrl_pressed = False
        self.quit_requested = False

    # ---------- pynput 回调 ----------
    def on_press(self, key):
        if key == keyboard.Key.ctrl:
            self.ctrl_pressed = True
        if key == keyboard.Key.esc:
            self.quit_requested = True
            return
        if key not in self.key_buffer and len(self.key_buffer) <= 5:
            self.key_buffer.append(key)

    def on_release(self, key):
        if key == keyboard.Key.ctrl:
            self.ctrl_pressed = False
        if key in self.key_buffer:
            self.key_buffer.remove(key)
        if isinstance(key, keyboard.KeyCode):
            self.handle_release(key.char)

    # ---------- 按键处理：只改状态 ----------
    def handle_press(self, key: str | None):
        if key is None:
            return

        # ctrl 组合键
        if self.ctrl_pressed:
            if key == ")":
                self.is_keyboard_control_active = True
                print("Keyboard control enabled")
            elif key == "(":
                self.is_keyboard_control_active = False
                self.key_buffer.clear()
                print("Keyboard control disabled")
            elif key == "@":
                self.base.increase_base_velocity()
            elif key == "!":
                self.base.decrease_base_velocity()

        # 底盘
        if key == "w":
            self.base.is_forward = True
        elif key == "s":
            self.base.is_forward = False
        elif key in ("a", "q"):
            self.base.is_clockwise = True
        elif key in ("d", "e"):
            self.base.is_clockwise = False

        # 机械臂
        elif key == "u":
            self.arm.move_joint("joint2_to_joint1", +self.arm.JOINT_STEP)
        elif key == "j":
            self.arm.move_joint("joint2_to_joint1", -self.arm.JOINT_STEP)
        elif key == "i":
            self.arm.move_joint("joint3_to_joint2", +self.arm.JOINT_STEP)
        elif key == "k":
            self.arm.move_joint("joint3_to_joint2", -self.arm.JOINT_STEP)
        elif key == "o":
            self.arm.move_joint("joint4_to_joint3", +self.arm.JOINT_STEP)
        elif key == "l":
            self.arm.move_joint("joint4_to_joint3", -self.arm.JOINT_STEP)
        elif key == "p":
            self.arm.move_joint("joint5_to_joint4", +self.arm.JOINT_STEP)
        elif key == ";":
            self.arm.move_joint("joint5_to_joint4", -self.arm.JOINT_STEP)
        elif key == "[":
            self.arm.move_joint("joint6_to_joint5", +self.arm.JOINT_STEP)
        elif key == "]":
            self.arm.move_joint("joint6_to_joint5", -self.arm.JOINT_STEP)
        elif key == ",":
            self.arm.move_joint("joint6output_to_joint6", +self.arm.JOINT_STEP)
        elif key == ".":
            self.arm.move_joint("joint6output_to_joint6", -self.arm.JOINT_STEP)

        # 夹爪
        elif key == "n":
            self.arm.move_gripper(+self.arm.gripper_step)
        elif key == "m":
            self.arm.move_gripper(-self.arm.gripper_step)

        # 状态 / 停止
        elif key == "z":
            self.print_status()
        elif key == "x":
            self.base.stop()
            print("Base stopping...")

    def handle_release(self, key: str | None):
        if key is None:
            return
        if key in ("w", "s"):
            self.base.is_forward = None
        elif key in ("a", "d", "q", "e"):
            self.base.is_clockwise = None

    # ---------- 主循环调用：消费 key_buffer ----------
    def process_keys(self):
        if not self.is_keyboard_control_active:
            self.key_buffer.clear()
            return
        # 只处理一次，处理后清空 buffer，避免重复触发
        keys_to_process = list(self.key_buffer)
        self.key_buffer.clear()
        for key in keys_to_process:
            if isinstance(key, keyboard.KeyCode):
                self.handle_press(key.char)

    # ---------- 主循环调用：把状态写入 data.ctrl ----------
    def apply(self, dt):
        # 底盘
        left, right = self.base.get_wheel_speeds(dt)
        self.data.ctrl[self.act_ids["left_behind_wheel"]]  = left
        self.data.ctrl[self.act_ids["left_front_wheel"]]   = left
        self.data.ctrl[self.act_ids["right_behind_wheel"]] = right
        self.data.ctrl[self.act_ids["right_front_wheel"]]  = right

        # 机械臂
        for name in self.arm.JOINT_NAMES:
            self.data.ctrl[self.act_ids[name]] = self.arm.targets[name]

        # 夹爪
        self.data.ctrl[self.act_ids["gripper_controller"]] = self.arm.gripper_target

    # ---------- 辅助 ----------
    # def print_status(self):
    #     print("---- Status ----")
    #     print("wheel ctrl:", self.data.ctrl[:4])
    #     print("arm targets:", self.arm.targets)
    #     print("gripper target:", self.arm.gripper_target)
    #     print("base pos:", self.data.qpos[:3])
    #     print("base quat:", self.data.qpos[3:7])
    #     print("base linvel (vx, vy, vz):", self.data.qvel[0:3])
    #     print("base angvel (wx, wy, wz):", self.data.qvel[3:6])
    #     print("|v| =", float(np.linalg.norm(self.data.qvel[0:3])), "m/s")

    #     for name in ["left_behind_wheel_joint", "left_front_wheel_joint",
    #                  "right_behind_wheel_joint", "right_front_wheel_joint"]:
    #         jid = joint_id(self.model, name)
    #         if jid >= 0:
    #             qpos = self.data.qpos[self.model.jnt_qposadr[jid]]
    #             rng = self.model.jnt_range[jid]
    #             print(f"{name}: qpos={qpos:.3f} range=({rng[0]:.3f}, {rng[1]:.3f})")
    #             print(f"{name} qvel = "
    #                   f"{self.data.qvel[self.model.jnt_dofadr[jid]]:.3f} rad/s")
    def print_status(self):
        ...
        print(f"gripper target: {self.arm.gripper_target:.3f}")
        print(f"gripper ctrl: {self.data.ctrl[self.act_ids['gripper_controller']]:.3f}")
        
        for jname in ["gripper_controller", "gripper_base_to_gripper_right3",
                    "gripper_left3_to_gripper_left1", "gripper_right3_to_gripper_right1"]:
            jid = joint_id(self.model, jname)
            if jid >= 0:
                qpos = self.data.qpos[self.model.jnt_qposadr[jid]]
                rng = self.model.jnt_range[jid]
                print(f"{jname}: qpos={qpos:.3f} range=({rng[0]:.3f}, {rng[1]:.3f})")

        import mujoco
        tid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_TENDON, "split")
        if tid >= 0:
            print(f"tendon split: length={self.data.ten_length[tid]:.4f}, "
                  f"velocity={self.data.ten_velocity[tid]:.4f}")

        aid = self.act_ids["gripper_controller"]
        print(f"gripper actuator: ctrl={self.data.ctrl[aid]:.4f}, "
              f"force={self.data.actuator_force[aid]:.4f}, "
              f"length={self.data.actuator_length[aid]:.4f}, "
              f"velocity={self.data.actuator_velocity[aid]:.4f}")

        # ---- 手指关节诊断 ----
        for jname in ["gripper_controller", "gripper_base_to_gripper_right3",
                      "gripper_left3_to_gripper_left1", "gripper_right3_to_gripper_right1",
                      "gripper_base_to_gripper_left2", "gripper_base_to_gripper_right2"]:
            jid = joint_id(self.model, jname)
            if jid >= 0:
                qpos = self.data.qpos[self.model.jnt_qposadr[jid]]
                qvel = self.data.qvel[self.model.jnt_dofadr[jid]]
                rng = self.model.jnt_range[jid]
                print(f"  {jname}: qpos={qpos:.4f} qvel={qvel:.4f} "
                      f"range=({rng[0]:.3f}, {rng[1]:.3f})")   


# =========================================================
# 主程序
# =========================================================
@click.command()
@click.option("--xml", "xml_path", type=str,
              default="robot_description_v2.xml",
              help="Path to the robot XML file")
@click.option("--rate", type=float, default=60.0, help="Sim loop rate (Hz)")
def main(xml_path: str, rate: float):

    model = mujoco.MjModel.from_xml_path(xml_path)
    data = mujoco.MjData(model)

    print("Actuators:")
    for i in range(model.nu):
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i}] {name}")

    print_keyboard_options()

    kbd = KeyboardController(model, data)
    listener = keyboard.Listener(
        on_press=kbd.on_press,
        on_release=kbd.on_release,
    )
    listener.start()

    loop_time = 1.0 / rate
    last_loop = perf_counter()

    # 屏蔽 viewer 默认快捷键，避免和 pynput 冲突
    BLOCKED_KEYCODES = {
        ord('W'), ord('A'), ord('S'), ord('D'),
        ord('Q'), ord('E'),
        ord('U'), ord('J'), ord('I'), ord('K'),
        ord('O'), ord('L'), ord('P'),
        ord('N'), ord('M'),
        ord('Z'), ord('X'),
    }

    def viewer_key_callback(keycode):
        if keycode in BLOCKED_KEYCODES:
            return True
        return False

    with mujoco.viewer.launch_passive(
        model, data,
        key_callback=viewer_key_callback,
    ) as viewer:
        try:
            while viewer.is_running() and not kbd.quit_requested:

                # 限速
                elapsed = perf_counter() - last_loop
                if elapsed < loop_time:
                    sleep(loop_time - elapsed)
                last_loop = perf_counter()

                # 1) 消费键盘事件（只更新控制器状态）
                kbd.process_keys()

                # 2) 把控制器状态写入 ctrl（带 ramp）
                dt = model.opt.timestep
                kbd.apply(dt)

                # 3) 物理步进
                mujoco.mj_step(model, data)

                # 4) 同步 viewer
                viewer.sync()

        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
            print("Simulation stopped.")


if __name__ == "__main__":
    main()