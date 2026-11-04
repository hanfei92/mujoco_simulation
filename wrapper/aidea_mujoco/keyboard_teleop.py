"""
Aidea 键盘控制：通过 sim API 发命令，不直接写 data.ctrl。

架构：
  主进程（本模块）                MujocoServer 子进程
  ─────────────                  ──────────────────
  pynput 监听键盘                 physics loop
  kbd.apply(sim)                  _ctrl_callback
    ├── sim._set_base_velocity      push_command
    └── sim._move_to                ↓
        ↓                         data.ctrl
      data_proxies.set_command
"""

from time import sleep

import click
from pynput import keyboard

from aidea_mujoco.enums.actuators import Actuators


# =========================================================
# 底盘控制器（差速）
# =========================================================
class BaseController:
    """
    输出 v (m/s) 和 omega (rad/s)，通过 sim._set_base_velocity 发送。

    单位对齐 AideaMujocoSimulator._set_base_velocity：
      curr_max_vel_xy_m = 20.0 (rad/s on wheel)
      curr_max_vel_w_r  = 10.0
    所以 v / omega 的单位是 m/s 和 rad/s（sim 内部再做差速逆解）。
    """

    def __init__(self):
        self.forward_velocity = 1.0      # m/s
        self.clockwise_velocity = 2.0    # rad/s
        self.is_forward = None           # True 前进, False 后退, None 停
        self.is_clockwise = None         # True 左转, False 右转, None 不转

        # ramp 状态
        self.current_v = .0
        self.current_w = .0
        self.accel_limit = 4.0           # m/s²
        self.decel_limit = 4.0           # m/s²

    def get_base_velocity(self, dt: float) -> tuple[float, float]:
        v_target = 0.0
        w_target = 0.0
        if self.is_forward is not None:
            v_target = self.forward_velocity if self.is_forward else -self.forward_velocity
        if self.is_clockwise is not None:
            w_target = self.clockwise_velocity if self.is_clockwise else -self.clockwise_velocity

        rate_v = self.accel_limit if abs(v_target) > abs(self.current_v) else self.decel_limit
        rate_w = self.accel_limit if abs(w_target) > abs(self.current_w) else self.decel_limit

        max_delta_v = rate_v * dt
        max_delta_w = rate_w * dt

        self.current_v += max(-max_delta_v, min(max_delta_v, v_target - self.current_v))
        self.current_w += max(-max_delta_w, min(max_delta_w, w_target - self.current_w))

        if v_target == 0.0 and abs(self.current_v) < 0.01:
            self.current_v = 0.0
        if w_target == 0.0 and abs(self.current_w) < 0.01:
            self.current_w = 0.0

        return self.current_v, self.current_w

    def increase_base_velocity(self):
        self.forward_velocity += 0.2
        self.clockwise_velocity += 0.2
        print(f"forward_velocity={self.forward_velocity:.2f}, "
              f"clockwise_velocity={self.clockwise_velocity:.2f}")

    def decrease_base_velocity(self):
        self.forward_velocity = max(0.0, self.forward_velocity - 0.2)
        self.clockwise_velocity = max(0.0, self.clockwise_velocity - 0.2)
        print(f"forward_velocity={self.forward_velocity:.2f}, "
              f"clockwise_velocity={self.clockwise_velocity:.2f}")

    def stop(self):
        self.is_forward = None
        self.is_clockwise = None


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

    def __init__(self):
        self.targets = {n: 0.0 for n in self.JOINT_NAMES}
        self.gripper_target = 0.0
        self.gripper_step = 0.05
        self.gripper_range = (-0.74, 0.15)
        # self.gripper_range = (-0.37, 0.075)

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
# 键盘控制器
# =========================================================
class KeyboardController:
    def __init__(self, sim):
        self.sim = sim
        self.base = BaseController()
        self.arm = ArmController()

        self.key_buffer = []
        self.is_keyboard_control_active = True
        self.ctrl_pressed = False
        self.quit_requested = False

        # 记录上次发送的机械臂目标，避免每帧重复发
        self._last_sent_arm: dict[str, float] = {}
        self._last_sent_gripper = None

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

    # ---------- 按键处理 ----------
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

    # ---------- 主循环调用 ----------
    def process_keys(self):
        if not self.is_keyboard_control_active:
            self.key_buffer.clear()
            return
        keys_to_process = list(self.key_buffer)
        self.key_buffer.clear()
        for key in keys_to_process:
            if isinstance(key, keyboard.KeyCode):
                self.handle_press(key.char)

    def apply(self, dt: float):
        """通过 sim API 发命令。"""
        if not self.is_keyboard_control_active:
            return

        # 底盘速度
        v, omega = self.base.get_base_velocity(dt)
        try:
            self.sim._set_base_velocity(v, 0.0, omega)
        except ConnectionError:
            return

        # 机械臂：只在目标变化时发
        for name in self.arm.JOINT_NAMES:
            target = self.arm.targets[name]
            if self._last_sent_arm.get(name) != target:
                try:
                    self.sim._move_to(Actuators[name], target)
                    self._last_sent_arm[name] = target
                except (ConnectionError, NotImplementedError):
                    pass

        # 夹爪
        if self._last_sent_gripper != self.arm.gripper_target:
            try:
                self.sim._move_to(
                    Actuators.gripper_controller, self.arm.gripper_target
                )
                self._last_sent_gripper = self.arm.gripper_target
            except (ConnectionError, NotImplementedError):
                pass
        self.print_status()

    # ---------- 状态打印 ----------
    def print_status(self):
        try:
            status = self.sim.pull_status()
        except ConnectionError:
            print("Simulator not running.")
            return
        print("---- Status ----")
        print(f"base pose: x={status.base.x:.3f} y={status.base.y:.3f} "
              f"theta={status.base.theta:.3f}")
        print(f"base vel: x_vel={status.base.x_vel:.3f} "
              f"theta_vel={status.base.theta_vel:.3f}")
        
        for name in self.arm.JOINT_NAMES:
            pv = getattr(status, name, None)
            if pv is not None:
                print(f"{name}: pos={pv.pos:.3f} vel={pv.vel:.3f}")
        print(f"gripper: pos={status.gripper_controller.pos:.3f}")
        print(f"gripper target: {self.arm.gripper_target:.3f}")
        print(f"gripper actual: {status.gripper_controller.pos:.3f}")
        print(f"gripper ctrl: {self.sim.data_proxies.get_command().move_to.get('gripper_controller')}")


# =========================================================
# 帮助信息
# =========================================================
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
    print("Esc         : 退出")
    click.secho("=====================================", fg="yellow")