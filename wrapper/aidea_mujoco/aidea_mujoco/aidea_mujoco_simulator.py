import tempfile
import time

import click
import numpy as np
from mujoco._structs import MjModel


import aidea_mujoco.utils as utils
from aidea_mujoco.datamodels.status_command import CommandBaseVelocity
from aidea_mujoco.enums.actuators import Actuators
from aidea_mujoco.enums.aidea_cameras import AideaCameras
from aidea_mujoco.pointcloud_utils import get_pointcloud_from_camera_status
from aidea_mujoco.mujoco_simulator import MujocoSimulator
from aidea_mujoco.utils import block_until_check_succeeds, require_connection, get_pointcloud_from_airy_camera_status

from aidea_mujoco.utils import URDFmodel


class AideaMujocoSimulator(MujocoSimulator):
    """
    Aidea Mujoco Simulator class for interfacing with the Mujoco Server.

    Calling `start()` will spawn a new process that runs `MujocoServer` and the simulator.

    You can specify `start(headless=True)` to run the simulation without a GUI.

    Data from the MujocoServer is sent to MujocoSimulator using proxies.

    Use `pull_status()` and `pull_camera_data()` to access simulation data.
    """

    def __init__(
        self,
        scene_xml_path: str | None = None,
        model: MjModel | None = None,
        camera_hz: float = 30,
        cameras_to_use: list[AideaCameras] = [],
        start_translation: list | None = None,
        start_rotation_quat: list | None = None,
    ) -> None:
        # Dynamically generate the MJCF so it's ready for any scene that includes it
        if model is None and scene_xml_path is None:
            # if no scene or model is provided, use the default
            scene_xml_path = self.get_scene_xml_path()

        super().__init__(
            scene_xml_path=scene_xml_path,
            model=model,
            camera_hz=camera_hz,
            cameras_to_use=cameras_to_use,
            start_translation=start_translation,
            start_rotation_quat=start_rotation_quat,
        )
        self._cameras_to_use = cameras_to_use
        print(f"[AideaMujocoSimulator] cameras_to_use = {[c.name for c in cameras_to_use]}")
        print(f"[AideaMujocoSimulator] self._cameras_to_use = {[c.name for c in self._cameras_to_use]}")
        self.T_base_airy = self._make_T_base_airy_from_mjcf()

    @staticmethod
    def _make_T_base_airy_from_mjcf() -> np.ndarray:
        """
        """
        from scipy.spatial.transform import Rotation as R

        # 从你的 MJCF 里抄下来（MuJoCo quat 顺序是 w, x, y, z）
        pos = np.array([0.03155, -0.0013198, 0.25402])
        quat_wxyz = np.array([0.0124759, 0.706997, 0.0124811, 0.706997])

        # scipy 要 (x, y, z, w)
        rot = R.from_quat([quat_wxyz[1], quat_wxyz[2], quat_wxyz[3], quat_wxyz[0]])

        T = np.eye(4)
        T[:3, :3] = rot.as_matrix()
        T[:3, 3] = pos
        return T

    @staticmethod
    def get_scene_xml_path() -> str:
        """
        Returns the default scene XML path for the Mujoco Simulator.
        """
        # return str("/home/amigo/Documents/ament_ws/src/aidea_mujoco/mjcf/scene.xml")
        return str(utils.models_path / "scene.xml")

    @staticmethod
    def get_all_cameras() -> list[AideaCameras]:
        return AideaCameras.all_aidea()

    @staticmethod
    def get_rgb_cameras() -> list[AideaCameras]:
        return AideaCameras.rgb_aidea()

    # def is_reached_set_position(self, actuator: str | Actuators, position_tolerance: float = 0.05):
    #     """
    #     Checks if the joint has reached a previously commanded location.

    #     Only listens to the `move_to` command.
    #     """
    #     if isinstance(actuator, str):
    #         actuator = Actuators[actuator]

    #     if actuator in [
    #         Actuators.base_rotate,
    #         Actuators.base_translate,
    #         Actuators.left_wheel_vel,
    #         Actuators.right_wheel_vel,
    #         Actuators.back_wheel_vel,
    #     ]:
    #         raise NotImplementedError(f"Check joint reached is not supported for {actuator}.")

    #     return super().is_reached_set_position(
    #         actuator=actuator, position_tolerance=position_tolerance
    #     )

    def is_reached_set_position(self, actuator: str | Actuators, position_tolerance: float = 0.05):
        if isinstance(actuator, str):
            actuator = Actuators[actuator]
        if actuator.is_wheel:
            raise NotImplementedError(f"Check joint reached is not supported for {actuator}.")
        return super().is_reached_set_position(
            actuator=actuator, position_tolerance=position_tolerance
        )

    def wait_while_is_moving(
        self,
        actuator,
        timeout: float | None = 5.0,
        check_interval: float = 0.1,
        position_tolerance: float = 0.0005,
    ):
        if isinstance(actuator, str):
            actuator = Actuators[actuator]

        def check_if_moved():
            time.sleep(check_interval)

            if actuator.is_wheel:
                # 差速底盘是 velocity 控制，没有"到位"概念
                return False

            current_position = actuator.get_position(self.pull_status())

            if actuator not in self._last_movement_positions:
                self._last_movement_positions[actuator] = current_position
                return True

            last_position = self._last_movement_positions[actuator]
            is_moved = not np.isclose(
                current_position, last_position, atol=position_tolerance
            )
            self._last_movement_positions[actuator] = current_position
            return is_moved

        if not block_until_check_succeeds(
            wait_timeout=timeout,
            check=lambda: check_if_moved() == False,
            is_alive=self.is_running,
        ):
            if timeout is not None:
                click.secho(
                    f"Timeout: Joint {actuator.name} is still moving after {timeout}.",
                    fg="red",
                )
            return False
        return True

    @require_connection
    def _move_to(self, actuator, pos: float) -> None:
        if isinstance(actuator, str):
            actuator = Actuators[actuator]
        if actuator.is_wheel:
            raise Exception(
                f"Cannot set an absolute position for a velocity actuator {actuator.name}"
            )
        return super()._move_to(actuator=actuator, pos=pos)

    @require_connection
    def _move_by(self, actuator, pos: float):
        if isinstance(actuator, str):
            actuator = Actuators[actuator]
        if actuator.is_wheel:
            click.secho(
                f"Cannot set a position for a velocity joint {actuator.name}",
                fg="red",
            )
            raise Exception(
                f"Cannot set an absolute position for a velocity actuator {actuator.name}"
            )
        return super()._move_by(actuator=actuator, pos=pos)

    @require_connection
    def _set_base_velocity(self, v_x: float, v_y: float, omega: float) -> None:
        """
        Set the base velocity of the robot
        Args:
            v_x: float, linear velocity
            v_y: float, linear velocity
            omega: float, angular velocity
        """

        with self._command_lock:
            command = self.data_proxies.get_command()
            command.set_base_velocity(
                CommandBaseVelocity(v_x=v_x, v_y=v_y, omega=omega, trigger=True)
            )


            self.data_proxies.set_command(command)

    def pull_airy_lidar_points(self, in_world_frame: bool = True) -> np.ndarray:
        """
        从 Airy 的 equirectangular depth image 反投影出 (N, 3) 点云。
        """
        # base frame -> world frame
        base_xyt = self.get_base_pose()
        T_world_base = np.eye(4)
        T_world_base[:3, :3] = utils.Rz(base_xyt[2])
        T_world_base[:2, 3] = base_xyt[:2]

        return get_pointcloud_from_airy_camera_status(
            self.pull_camera_data(),
            in_world_frame=in_world_frame,
            T_base_airy=self.T_base_airy,
            T_world_base=T_world_base,
        )

    def pull_airy_lidar_points_raytrace(self, in_world_frame=True):
        """从 data_proxies 读 ray tracing 点云。

        in_world_frame=True  → 返回 odom 系的点
        in_world_frame=False → 返回 base_link 系的点
        """
        world_points = self.data_proxies.get_lidar_points()

        if world_points is None or len(world_points) == 0:
            return np.zeros((0, 3), dtype=np.float32)

        if in_world_frame:
            return world_points

        # world -> base_link
        base_xyt = self.get_base_pose()
        c, s = np.cos(base_xyt[2]), np.sin(base_xyt[2])
        R = np.array([[c, s, 0], [-s, c, 0], [0, 0, 1]])   # world -> base_link
        t = np.array([base_xyt[0], base_xyt[1], 0.0])
        return (world_points - t) @ R.T