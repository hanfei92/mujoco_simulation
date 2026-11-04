from enum import Enum
from functools import cache

from aidea_mujoco.datamodels.status_aidea_joints import StatusAideaJoints


class Actuators(Enum):
    """
    Aidea 机器人的 actuator 枚举。

    与 robot_description_v2.xml 里的 <actuator> 定义一一对应：
        left_behind_wheel / left_front_wheel / right_behind_wheel / right_front_wheel
        joint2_to_joint1 / joint3_to_joint2 / joint4_to_joint3
        joint5_to_joint4 / joint6_to_joint5 / joint6output_to_joint6
        gripper_controller
    """

    # ---- 四轮差速底盘（velocity actuator） ----
    left_behind_wheel = 0
    left_front_wheel = 1
    right_behind_wheel = 2
    right_front_wheel = 3

    # ---- 6 轴机械臂（position actuator） ----
    joint2_to_joint1 = 4
    joint3_to_joint2 = 5
    joint4_to_joint3 = 6
    joint5_to_joint4 = 7
    joint6_to_joint5 = 8
    joint6output_to_joint6 = 9

    # ---- 夹爪（tendon position actuator） ----
    gripper_controller = 10

    # ================================================================ #
    #  分组属性
    # ================================================================ #
    @property
    def is_wheel(self) -> bool:
        """是否是底盘轮子的 velocity actuator。"""
        return self in (
            Actuators.left_behind_wheel,
            Actuators.left_front_wheel,
            Actuators.right_behind_wheel,
            Actuators.right_front_wheel,
        )

    @property
    def is_arm_joint(self) -> bool:
        """是否是机械臂的 position actuator。"""
        return self in (
            Actuators.joint2_to_joint1,
            Actuators.joint3_to_joint2,
            Actuators.joint4_to_joint3,
            Actuators.joint5_to_joint4,
            Actuators.joint6_to_joint5,
            Actuators.joint6output_to_joint6,
        )

    @property
    def is_gripper(self) -> bool:
        return self == Actuators.gripper_controller

    # ================================================================ #
    #  分组快捷方法
    # ================================================================ #
    @staticmethod
    def all_wheels() -> list["Actuators"]:
        return [
            Actuators.left_behind_wheel,
            Actuators.left_front_wheel,
            Actuators.right_behind_wheel,
            Actuators.right_front_wheel,
        ]

    @staticmethod
    def all_arm_joints() -> list["Actuators"]:
        return [
            Actuators.joint2_to_joint1,
            Actuators.joint3_to_joint2,
            Actuators.joint4_to_joint3,
            Actuators.joint5_to_joint4,
            Actuators.joint6_to_joint5,
            Actuators.joint6output_to_joint6,
        ]

    @staticmethod
    def all_gripper() -> list["Actuators"]:
        return [Actuators.gripper_controller]

    @staticmethod
    def all_aidea() -> list["Actuators"]:
        return (
            Actuators.all_wheels()
            + Actuators.all_arm_joints()
            + Actuators.all_gripper()
        )

    @staticmethod
    def get_actuator_by_joint_names_in_mjcf(joint_name: str) -> "Actuators":
        """
        按 MJCF 里的 joint 名找对应的 actuator。

        在你的 MJCF 里 actuator 名 == joint 名（gripper_controller 是 tendon），
        所以直接按名字查即可。
        """
        return Actuators[joint_name]

    # ================================================================ #
    #  从 StatusAideaJoints 读位置 / 速度
    # ================================================================ #
    def _get_status_attribute(self, is_position: bool, status: StatusAideaJoints) -> float:
        """
        从 StatusAideaJoints 读该 actuator 对应字段的 pos / vel。

        在你的 StatusAideaJoints 里，字段名 == actuator 名
        （6 轴 + gripper_controller 都是）。
        轮子没有 pos/vel 语义，会抛异常。
        """
        if self.is_wheel:
            raise NotImplementedError(
                f"{self.name} 是 velocity actuator，没有位置/速度语义。"
            )

        attribute_name = "pos" if is_position else "vel"
        pv = getattr(status, self.name, None)
        if pv is None:
            raise NotImplementedError(
                f"StatusAideaJoints 里没有字段 {self.name}。"
            )
        return float(getattr(pv, attribute_name))

    def _get_base_status_attribute(
        self, is_position: bool, status: StatusAideaJoints
    ) -> tuple[float, float, float]:
        """
        底盘状态。Aidea 是四轮差速，没有 base_translate / base_rotate 这类虚拟关节。

        这个函数目前不会被调用（因为 is_wheel 会先抛异常），
        保留是为了兼容上层可能有的调用。
        """
        if not self.is_wheel:
            raise NotImplementedError(
                f"{self.name} 不是底盘 actuator，没有相对位置语义。"
            )

        x = "x" if is_position else "x_vel"
        theta = "theta" if is_position else "theta_vel"
        # BaseStatus 里没有 y_vel 字段，用 0.0 占位
        return (
            float(getattr(status.base, x)),
            0.0,
            float(getattr(status.base, theta)),
        )

    def get_position(self, status: StatusAideaJoints) -> float:
        if self.is_wheel:
            raise Exception(f"Please use `get_position_relative()` for {self.name}")
        return self._get_status_attribute(True, status)

    def get_position_relative(self, status: StatusAideaJoints) -> tuple[float, float, float]:
        if not self.is_wheel:
            raise Exception(f"Please use `get_position()` for {self.name}")
        return self._get_base_status_attribute(True, status)

    def get_velocity(self, status: StatusAideaJoints) -> float:
        if self.is_wheel:
            raise Exception(f"Please use `get_velocity_relative()` for {self.name}")
        return self._get_status_attribute(False, status)

    def get_velocity_relative(self, status: StatusAideaJoints) -> tuple[float, float, float]:
        if not self.is_wheel:
            raise Exception(f"Please use `get_velocity()` for {self.name}")
        return self._get_base_status_attribute(False, status)