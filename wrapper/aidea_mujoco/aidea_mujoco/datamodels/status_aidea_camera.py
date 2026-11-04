import copy
from dataclasses import asdict, dataclass
import cv2
import numpy as np

# from aidea_mujoco.aidea_mujoco.enums.aidea_cameras import AideaCameras
from aidea_mujoco.enums.aidea_cameras import AideaCameras
from aidea_mujoco.utils import dataclass_from_dict, get_depth_color_map


@dataclass
class StatusAideaCameras:
    """
    A dataclass and helper methods to pack camera data.
    """
    time: float
    fps: float
    
    cam_d435i_rgb:np.ndarray|None = None
    cam_d435i_depth:np.ndarray|None = None
    cam_d435i_K: np.ndarray|None = None

    cam_nav_rgb: np.ndarray|None = None

    cam_hemilidar_airy: np.ndarray|None = None


    def get_all(self, *, auto_rotate: bool = True, auto_correct_rgb=True, use_depth_color_map=False)-> dict[AideaCameras, np.ndarray]:
        """Returns the camera `{Cameras: pixels}` that are available (not None).

        `auto_rotate` will correct the rotation of the cam_d435i_rgb and cam_d435i_depth from their innately rotated optical frame. default: True
        `auto_correct_rgb` will call `cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)` before returning the pixels. default: True
        `use_depth_color_map` default: False

        Note: This get the values inside this dataclass; it does not poll from the simulator.

        Note: Alternatively, use `get_camera_data()` to get a specific camera's data.
        """
        data: dict[AideaCameras, np.ndarray] = {}
        for camera in [cam for cam in AideaCameras]:
            try:
                data[camera] = self.get_camera_data(camera=camera, auto_rotate=auto_rotate, auto_correct_rgb=auto_correct_rgb,use_depth_color_map=use_depth_color_map)
            except ValueError: ... # get_camera_data throws a ValueError when the value is None or doesn't exist.

        return data

    def get_camera_data(self, camera:AideaCameras, *, auto_rotate: bool = True, auto_correct_rgb=True, use_depth_color_map=False) -> np.ndarray:
        """
        Use this to get the camera data (pixels) using a Cameras instance.

        Throws a ValueError if the data is None.

        `auto_rotate` will correct the rotation of the cam_d435i_rgb and cam_d435i_depth from their innately rotated optical frame. default: True
        `auto_correct_rgb` will call `cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)` before returning the pixels. default: True
        `use_depth_color_map` default: False

        Note: This get the values inside this dataclass; it does not poll from the simulator.
        """
        data:np.ndarray|None = None
        if camera == AideaCameras.cam_hemilidar_airy and self.cam_hemilidar_airy is not None:
            data = self.cam_hemilidar_airy
            data = get_depth_color_map(data) if use_depth_color_map else data
        elif camera == AideaCameras.cam_d435i_rgb and self.cam_d435i_rgb is not None:
            data = self.cam_d435i_rgb
            rotate_num = camera.initial_camera_settings.rotate_number_of_times
            data = np.rot90(data, rotate_num) if auto_rotate and rotate_num != 0 else data
            data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if auto_correct_rgb else data
        elif camera == AideaCameras.cam_d435i_depth and self.cam_d435i_depth is not None:
            data = self.cam_d435i_depth
            rotate_num = camera.initial_camera_settings.rotate_number_of_times
            data = np.rot90(data, rotate_num) if auto_rotate and rotate_num != 0 else data
            data = get_depth_color_map(data) if use_depth_color_map else data
        elif camera == AideaCameras.cam_nav_rgb and self.cam_nav_rgb is not None:
            data = self.cam_nav_rgb
            rotate_num = camera.initial_camera_settings.rotate_number_of_times
            data = np.rot90(data, rotate_num) if auto_rotate and rotate_num != 0 else data
            data = cv2.cvtColor(data, cv2.COLOR_RGB2BGR) if auto_correct_rgb else data

        if data is None:
            raise ValueError(f"Tried to get {camera} data, but it is empty or not implemented.")

        return data

    def set_camera_data(self, camera:AideaCameras, data:np.ndarray):
        """
        Use this to match a Cameras enum instance with its property in StatusAideaCameras dataclass, to set the camera data property within this dataclass.

        Note: This sets the values inside this dataclass; it does not send data to the simulator.
        """
        if camera == AideaCameras.cam_d435i_rgb:
            self.cam_d435i_rgb = data
            return
        if camera == AideaCameras.cam_d435i_depth:
            self.cam_d435i_depth = data
            return
        if camera == AideaCameras.cam_nav_rgb :
            self.cam_nav_rgb = data
            return
        if camera == AideaCameras.cam_hemilidar_airy:
            self.cam_hemilidar_airy = data
            return

        raise NotImplementedError(f"Camera {camera} is not implemented.")

    @staticmethod
    def default():
        """
        Returns an empty instance with None or zeros for properties.
        """
        return StatusAideaCameras(time=0, fps=0)

    def to_dict(self):
        return asdict(self)

    def copy(self):
        return StatusAideaCameras.from_dict(copy.copy(self.to_dict()))

    @staticmethod
    def from_dict(dict_data:dict)-> "StatusAideaCameras":
        return dataclass_from_dict(StatusAideaCameras, dict_data) #type: ignore
