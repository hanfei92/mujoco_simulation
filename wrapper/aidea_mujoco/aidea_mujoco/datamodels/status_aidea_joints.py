import copy
from dataclasses import asdict, dataclass
from aidea_mujoco.utils import dataclass_from_dict

@dataclass
class PositionVelocity:
    pos: float
    vel: float
    effort: float = 0.0

    @staticmethod
    def default():
        return PositionVelocity(0, 0, 0.0)

@dataclass
class BaseStatus:
    x:float
    y:float
    theta:float
    x_vel:float
    theta_vel:float

    @staticmethod
    def default():
        return BaseStatus(0, 0, 0,0,0)

@dataclass
class StatusAideaJoints:
    time: float
    fps:float
    sim_to_real_time_ratio_msg: str
    base:BaseStatus

    joint2_to_joint1: PositionVelocity
    joint3_to_joint2: PositionVelocity
    joint4_to_joint3: PositionVelocity
    joint5_to_joint4: PositionVelocity
    joint6_to_joint5: PositionVelocity
    joint6output_to_joint6: PositionVelocity

    gripper_controller: PositionVelocity
    gripper_left_finger: PositionVelocity
    gripper_right_finger: PositionVelocity

    is_self_colliding: bool = False

    def __getitem__(self, name:str):
        """For backward compatibility: allows access with the square brackets []"""
        return getattr(self, name)

    def to_dict(self):
        return asdict(self)

    def copy(self):
        return StatusAideaJoints.from_dict(copy.copy(self.to_dict()))

    @staticmethod
    def from_dict(dict_data:dict)-> "StatusAideaJoints":
        return dataclass_from_dict(StatusAideaJoints, dict_data) #type: ignore


    @staticmethod
    def default():
        """
        Returns an empty instance with None or zeros for properties.
        """
        return StatusAideaJoints(
            0,
            0,
            "",
            BaseStatus.default(),
            joint2_to_joint1=PositionVelocity.default(),
            joint3_to_joint2=PositionVelocity.default(),
            joint4_to_joint3=PositionVelocity.default(),
            joint5_to_joint4=PositionVelocity.default(),
            joint6_to_joint5=PositionVelocity.default(),
            joint6output_to_joint6=PositionVelocity.default(),
            gripper_controller=PositionVelocity.default(),
            gripper_left_finger=PositionVelocity.default(),
            gripper_right_finger=PositionVelocity.default(),
            is_self_colliding=False,
        )
