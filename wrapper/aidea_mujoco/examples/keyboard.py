"""
原生 mujoco.viewer + pynput 键盘控制
适配：四轮差速底盘 + 6 轴机械臂 + 夹爪
"""

from time import sleep, perf_counter
from pynput import keyboard
from pprint import pprint

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
# 底盘控制器
# =========================================================
class BaseController:
    def __init__(self):
        self.forward_velocity = 50.0     # rad/s  50*0.05=2.5m/s
        self.clockwise_velocity = 70   # rad/s
        self.is_forward = None          # True 前进, False 后退, None 停
        self.is_clockwise = None        # True 左转, False 右转, None 不转

    def get_wheel_speeds(self):
        v = 0.0
        w = 0.0
        if self.is_forward is not None:
            v = self.forward_velocity if self.is_forward else -self.forward_velocity
        if self.is_clockwise is not None:
            w = self.clockwise_velocity if self.is_clockwise else -self.clockwise_velocity

        # 差速：左轮 = v - w，右轮 = v + w
        left = v - w
        right = v + w
        return left, right

    def increase_base_velocity(self):
        self.forward_velocity += 4
        self.clockwise_velocity += 4
        print(f"forward_velocity={self.forward_velocity:.2f}, "
              f"clockwise_velocity={self.clockwise_velocity:.2f}")

    def decrease_base_velocity(self):
        self.forward_velocity = max(0.0, self.forward_velocity - 4)
        self.clockwise_velocity = max(0.0, self.clockwise_velocity - 4)
        print(f"forward_velocity={self.forward_velocity:.2f}, "
              f"clockwise_velocity={self.clockwise_velocity:.2f}")

    def apply(self, model, data, act_ids):
        left, right = self.get_wheel_speeds()
        data.ctrl[act_ids["left_behind_wheel"]]  = left
        data.ctrl[act_ids["left_front_wheel"]]   = left
        # 右轮 axis 方向一致，不用取负
        data.ctrl[act_ids["right_behind_wheel"]] = right
        data.ctrl[act_ids["right_front_wheel"]]  = right


# =========================================================
# 机械臂 + 夹爪控制器
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

    def __init__(self, model, data, act_ids):
        self.model = model
        self.data = data
        self.act_ids = act_ids

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

    def apply(self):
        for name in self.JOINT_NAMES:
            self.data.ctrl[self.act_ids[name]] = self.targets[name]
        self.data.ctrl[self.act_ids["gripper_controller"]] = self.gripper_target


# =========================================================
# 键盘控制器
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
        self.arm = ArmController(model, data, self.act_ids)

        self.key_buffer = []
        self.is_keyboard_control_active = True
        self.ctrl_pressed = False

        # 退出标志
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
            self.keyboard_control_release(key.char)

    # ---------- 按键处理 ----------
    def keyboard_control(self, key: str | None):
        print(f"[KEY]{key!r}")
        if key is None:
            return

        # ctrl 组合键
        if self.ctrl_pressed:
            if key == ")":
                self.is_keyboard_control_active = True
                print("Keyboard control enabled")
            elif key == "(":
                self.is_keyboard_control_active = False
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
        elif key == "a" or key == "q":
            self.base.is_clockwise = True
        elif key == "d" or key == "e":
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
            self.stop_all()

        # 每帧写 ctrl
        self.base.apply(self.model, self.data, self.act_ids)
        self.arm.apply()

    def keyboard_control_release(self, key: str | None):
        if key is None:
            return
        if key in ("w", "s"):
            self.base.is_forward = None
        elif key in ("a", "d", "q", "e"):
            self.base.is_clockwise = None
        self.base.apply(self.model, self.data, self.act_ids)

    # ---------- 辅助 ----------
    def print_status(self):
        print("---- Status ----")
        print("wheel ctrl:", self.data.ctrl[:4])
        print("arm targets:", self.arm.targets)
        print("gripper target:", self.arm.gripper_target)
        print("base pos:", self.data.qpos[:3])
        print("base quat:", self.data.qpos[3:7])
        print("base linvel (vx, vy, vz):", self.data.qvel[0:3])
        print("base angvel (wx, wy, wz):", self.data.qvel[3:6])
        print("|v| =", float(np.linalg.norm(self.data.qvel[0:3])), "m/s")

        # 轮子 joint 的实际速度
        for name in ["left_behind_wheel_joint", "left_front_wheel_joint",
                    "right_behind_wheel_joint", "right_front_wheel_joint"]:
            jid = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_JOINT, name)
            if jid >= 0:
                print(f"{name} qvel = {self.data.qvel[self.model.jnt_dofadr[jid]]:.3f} rad/s")

    def stop_all(self):
        self.base.is_forward = None
        self.base.is_clockwise = None
        self.base.apply(self.model, self.data, self.act_ids)
        print("Base stopped.")


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

    # 可选：打印 actuator 列表，确认索引
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

    with mujoco.viewer.launch_passive(model, data) as viewer:
        try:
            while viewer.is_running() and not kbd.quit_requested:

                # 限速
                elapsed = perf_counter() - last_loop
                if elapsed < loop_time:
                    sleep(loop_time - elapsed)
                last_loop = perf_counter()

                # 处理键盘
                if kbd.is_keyboard_control_active:
                    for key in list(kbd.key_buffer):
                        if isinstance(key, keyboard.KeyCode):
                            kbd.keyboard_control(key.char)

                # 物理步进
                mujoco.mj_step(model, data)

                # 同步 viewer
                viewer.sync()

        except KeyboardInterrupt:
            pass
        finally:
            listener.stop()
            print("Simulation stopped.")


if __name__ == "__main__":
    main()