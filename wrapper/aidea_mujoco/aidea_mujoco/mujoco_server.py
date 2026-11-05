import contextlib
from dataclasses import dataclass
from multiprocessing.managers import DictProxy, SyncManager
import os
import signal
import threading
import time
from typing import Callable

import click
import mujoco
import mujoco._functions
import mujoco._enums
import numpy as np

from aidea_mujoco.zmq_publisher import ZmqPublisher
from aidea_mujoco.msg import lidar_pb2

from mujoco._structs import MjData, MjModel
import mujoco._enums

from aidea_mujoco.datamodels.status_aidea_camera import StatusAideaCameras
from aidea_mujoco.datamodels.status_aidea_joints import StatusAideaJoints
from aidea_mujoco.datamodels.status_aidea_sensors import StatusAideaSensors
from aidea_mujoco.enums.actuators import Actuators
from aidea_mujoco.enums.aidea_cameras import AideaCameras
import aidea_mujoco.config as config
from aidea_mujoco.enums.aidea_sensors import AideaSensors
from aidea_mujoco.mujoco_server_camera_manager import (
    MujocoServerCameraManagerThreaded,
    MujocoServerCameraManagerSync,
)
from aidea_mujoco.datamodels.status_command import CommandBaseVelocity, CommandMove, StatusCommand
from aidea_mujoco.mujoco_server_sensor_manager import MujocoServerSensorManagerThreaded
import aidea_mujoco.utils as utils
from aidea_mujoco.utils import FpsCounter, change_start_pose, H0_from_driving_dir, inverse_3x3_matrix, rotation_3x3_matrix
from aidea_mujoco.trapezoidal_profile import TrapezoidalProfile


from mujoco_lidar import MjLidarWrapper, scan_gen

from aidea_mujoco.plugins.registry import PLUGIN_REGISTRY
import yaml

@dataclass
class MujocoServerProxies:
    _command: "DictProxy[str, StatusCommand]"
    _status: "DictProxy[str, StatusAideaJoints]"
    _cameras: "DictProxy[str, StatusAideaCameras]"
    _sensors: "DictProxy[str, StatusAideaSensors]"
    _joint_limits: "DictProxy[str, dict[Actuators, tuple[float, float]]]"
    _lidar_points: "DictProxy[str, np.ndarray]"   # ← 新增
    _robot_command: "DictProxy[str, dict]"

    def __setattr__(self, name: str, value) -> None:
        try:
            super().__setattr__(name, value)
        except BrokenPipeError:
            ...

    def get_status(self) -> StatusAideaJoints:
        return self._status["val"]

    def set_status(self, value: StatusAideaJoints):
        self._status["val"] = value

    def get_command(self) -> StatusCommand:
        return self._command["val"]

    def set_command(self, value: StatusCommand):
        self._command["val"] = value

    def get_cameras(self) -> StatusAideaCameras:
        return self._cameras["val"]

    def set_cameras(self, value: StatusAideaCameras):
            self._cameras["val"] = value

    def get_sensors(self) -> StatusAideaSensors:
        return self._sensors["val"]

    def set_sensors(self, value: StatusAideaSensors):
        self._sensors["val"] = value

    def get_joint_limits(self) -> dict[Actuators, tuple[float, float]]:
        return self._joint_limits["val"]

    def set_joint_limit(self, actuator: Actuators, min_max: tuple[float, float]):
        limits = self._joint_limits["val"]
        limits[actuator] = min_max

        self._joint_limits["val"] = limits

    def get_lidar_points(self) -> np.ndarray: #eric# ← 新增
        return self._lidar_points["val"]

    def set_lidar_points(self, value: np.ndarray): #eric# ← 新增
        self._lidar_points["val"] = value 

    def get_robot_command(self) -> dict:
        return self._robot_command["val"]

    def set_robot_command(self, value: dict):
        self._robot_command["val"] = value

    @staticmethod
    def default(manager: SyncManager) -> "MujocoServerProxies":
        return MujocoServerProxies(
            _command=manager.dict({"val": StatusCommand.default()}),
            _status=manager.dict({"val": StatusAideaJoints.default()}),
            _cameras=manager.dict({"val": StatusAideaCameras.default()}),
            _sensors=manager.dict({"val": StatusAideaSensors.default()}),
            _joint_limits=manager.dict({"val": {}}),
            _lidar_points=manager.dict({"val": np.zeros((0, 3), dtype=np.float32)}), #eric
            _robot_command=manager.dict({"val": {}}),
        )


class BaseController:
 
    WHEEL_RADIUS_M = 0.05        # 轮子 size="0.05 0.0125" -> 半径 0.05
    WHEEL_BASE_M = 0.122         # 左右轮 y 坐标差 ≈ 0.060998 * 2

    def __init__(self, mujoco_server: "MujocoServer") -> None:
        self.mujoco_server = mujoco_server
        self.last_command: CommandMove | CommandBaseVelocity | None = None
        self.start_pose = np.array([0, 0, 0])

        # Trapezoidal profiles for omni wheels
        # Values can be tuned. max_vel and max_accel should match or slightly exceed robot capabilities.
        # Stepper motors have a max velocity around 50.0 rad/s
        # max_vel_multiplier = 10
        # self.left_wheel_profile = TrapezoidalProfile(max_vel=50.0*max_vel_multiplier, max_accel=15.0*max_vel_multiplier)
        # self.right_wheel_profile = TrapezoidalProfile(max_vel=50.0*max_vel_multiplier, max_accel=15.0*max_vel_multiplier)
        # self.back_wheel_profile = TrapezoidalProfile(max_vel=50.0*max_vel_multiplier, max_accel=15.0*max_vel_multiplier)
        # self.profiles_initialized = False

        # omnibase params from config
        # self.params = config.robot_settings_se4['omnibase']

        # Set max dynamics
        # motion_max = self.params['motion']['max']
        # self.curr_max_accel_xy_m = motion_max.get('accel_xy_m', 0.5) * max_vel_multiplier
        # self.curr_max_accel_w_r = motion_max.get('accel_w_r', 4.0) * max_vel_multiplier
        # self.curr_max_vel_xy_m = motion_max.get('vel_xy_m', 0.6) * max_vel_multiplier
        # self.curr_max_vel_w_r = motion_max.get('vel_w_r', 4.0) * max_vel_multiplier

        # Omnibase kinematics
        # self.H0 = H0_from_driving_dir(self.params['wheel_diameter_m'], self.params['base_radius_m'], self.params['forward_dir'])
        # self.H0_inv = inverse_3x3_matrix(self.H0)

        # 速度 / 加速度限制（wheel actuator 是 velocity，ctrlrange="-40 40"）
        self.curr_max_vel_xy_m = 100.0     # rad/s，作用在轮上
        self.curr_max_vel_w_r = 100.0      # rad/s，角速度
        self.curr_max_accel_xy_m = 40.0
        self.curr_max_accel_w_r = 40.0

        # Status
        self.status = {'x': 0, 'y': 0, 'theta': 0, 'x_vel': 0, 'y_vel': 0, 'theta_vel': 0,
                       'pose_time_s': 0, 'wheel_left_vel': 0, 'wheel_right_vel': 0}

    def push_command(self, command: CommandMove | CommandBaseVelocity):
        """Push a command to the base. Call `update()` to set the next trajectory."""
        self.last_command = command
        self.start_pose = self.get_base_pose()

    def _clear_command(self, is_stop_motion: bool):
        self.last_command = None

        if is_stop_motion:
            self._set_base_velocity(0.0, 0.0, 0.0)

    def _set_base_velocity(self, v_linear: float, v_angular: float, omega: float | None = None) -> None:
        """
        差速逆运动学：
            w_left  = (v - omega * L/2) / r
            w_right = (v + omega * L/2) / r

        兼容两种调用：
            _set_base_velocity(v, omega)              # 两个参数
            _set_base_velocity(vx, vy, omega)         # 三个参数，vy 被忽略
        """
        if omega is None:
            omega = v_angular

        v_linear = float(np.clip(v_linear, -self.curr_max_vel_xy_m, self.curr_max_vel_xy_m))
        omega = float(np.clip(omega, -self.curr_max_vel_w_r, self.curr_max_vel_w_r))

        w_left = (v_linear - omega * self.WHEEL_BASE_M / 2.0) / self.WHEEL_RADIUS_M
        w_right = (v_linear + omega * self.WHEEL_BASE_M / 2.0) / self.WHEEL_RADIUS_M

        data = self.mujoco_server.mjdata
        data.actuator(Actuators.left_behind_wheel.name).ctrl = w_left
        data.actuator(Actuators.left_front_wheel.name).ctrl = w_left
        data.actuator(Actuators.right_behind_wheel.name).ctrl = w_right
        data.actuator(Actuators.right_front_wheel.name).ctrl = w_right

    def _update_odom(self, dt):
        """
        Calculate SE2 position of the base in odom frame from wheel odometry
        Important:
         - Assumes pull_status() was just called
         - Assumes the user calls this method at a regular frequency (>30hz)

        dt: 1/frequency at which this method is called
        """
        data = self.mujoco_server.mjdata
        w_left = data.actuator(Actuators.left_behind_wheel.name).velocit[0] # ← 新增y[0]
        w_right = data.actuator(Actuators.right_behind_wheel.name).velocity[0]

        v = (w_left + w_right) * self.WHEEL_RADIUS_M / 2.0
        omega = (w_right - w_left) * self.WHEEL_RADIUS_M / self.WHEEL_BASE_M

        self.status["x_vel"] = v
        self.status["theta_vel"] = omega
        self.status["x"] += v * np.cos(self.status["theta"]) * dt
        self.status["y"] += v * np.sin(self.status["theta"]) * dt
        self.status["theta"] += omega * dt
        self.status["pose_time_s"] = time.time()

    def update(self):
        """每步 mj_step 前调用，把 last_command 转成 ctrl。"""
        if self.last_command is None:
            return
        if isinstance(self.last_command, CommandMove):
            return self.handle_move_by(self.last_command)
        if isinstance(self.last_command, CommandBaseVelocity):
            return self._set_base_velocity(
                self.last_command.v_x,
                self.last_command.v_y,
                self.last_command.omega,
            )

    def get_base_pose(self) -> np.ndarray:
        """Get the se(2) base pose: x, y, and theta"""
        xyz = self.mujoco_server.mjdata.body("base_link").xpos
        rotation = self.mujoco_server.mjdata.body("base_link").xmat.reshape(3, 3)
        theta = np.arctan2(rotation[1, 0], rotation[0, 0])
        return np.array([xyz[0], xyz[1], theta])

    def handle_move_by(self, command: CommandMove):
        raise NotImplementedError(f"不支持.")

class LidarController:

    def __init__(self, mujoco_server: "MujocoServer") -> None:
        self.mujoco_server = mujoco_server

        self._pub = mujoco_server.zmq_publisher

        # ---- 射线：Airy 96 线，水平 360°、垂直上半球 ----
        self.rays_theta, self.rays_phi = scan_gen.generate_airy96()
        self.rays_theta = np.ascontiguousarray(self.rays_theta).astype(np.float32)
        self.rays_phi = np.ascontiguousarray(self.rays_phi).astype(np.float32)

        # ---- 可选：降采样射线，减少 CPU ----
        # self.rays_theta = self.rays_theta[::2]
        # self.rays_phi = self.rays_phi[::2]

        # ---- geomgroup：排除 group 3（碰撞组）----
        geomgroup = np.ones((6,), dtype=np.ubyte)
        geomgroup[3:] = 0

        # ---- MjLidarWrapper ----
        self.lidar = MjLidarWrapper(
            self.mujoco_server.mjmodel,
            site_name="lidar_site",
            # backend="cpu",
            backend="warp",
            args={
                "bodyexclude": self.mujoco_server.mjmodel.body("airy").id,
                "geomgroup": geomgroup,
            },
        )

        # ---- 线程锁：和物理循环共享 mjdata，避免竞态 ----
        # self._mjdata_lock = threading.Lock()
        self._lidar_mjdata = mujoco.MjData(self.mujoco_server.mjmodel)

        # ---- 线程 ----
        self._thread = None
        self._running = False
        self._rate_hz = 10.0

    def start(self):
        """启动 LiDAR 扫描线程。"""
        if self._thread is not None:
            return
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def stop(self):
        """停止 LiDAR 扫描线程。"""
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None

    def _scan_once(self):
        with self.mujoco_server.camera_manager.camera_lock:
            mujoco.mj_copyData(self._lidar_mjdata, self.mujoco_server.mjmodel, self.mujoco_server.mjdata)
            base_pos = self._lidar_mjdata.body("base_link").xpos.copy()
            base_mat = self._lidar_mjdata.body("base_link").xmat.reshape(3, 3).copy()

        self.lidar.trace_rays(self._lidar_mjdata, self.rays_theta, self.rays_phi)   # ← 用副本
        points = self.lidar.get_hit_points()
        world_points = points @ self.lidar.sensor_rotation.T + self.lidar.sensor_position
        # base_pts = (world_points - base_pos) @ base_mat         
        self.mujoco_server.data_proxies.set_lidar_points(
            world_points.astype(np.float32)
        )        

    def _world_to_base_link(self, world_points: np.ndarray) -> np.ndarray:
        """world 系 -> base_link 系。"""
        data = self.mujoco_server.mjdata
        base_pos = data.body("base_link").xpos          # (3,)
        base_mat = data.body("base_link").xmat.reshape(3, 3)  # (3,3)

        # p_base = R_base^T @ (p_world - base_pos)
        return (world_points - base_pos) @ base_mat

    def _publish_lidar(self, pts: np.ndarray):
        """构造 PointCloud2 并通过 ZMQ 发布，topic='lidar'。"""
        now = time.time()
        msg = lidar_pb2.PointCloud2()
        msg.header.stamp_sec = int(now)
        msg.header.stamp_nsec = int((now - int(now)) * 1e9)
        msg.header.frame_id = "world"
        msg.height = 1
        msg.width = len(pts)
        msg.point_step = 12
        msg.row_step = len(pts) * 12
        msg.is_dense = True
        msg.data = pts.astype(np.float32).tobytes()
        self._pub.publish("lidar", msg.SerializeToString())

    def _loop(self):
        print("[lidar] thread started")
        time.sleep(0.5)
        while self._running and not self.mujoco_server._is_requested_to_stop():
            try:
                self._scan_once()
                pts = self.mujoco_server.data_proxies.get_lidar_points()
                print(f"[lidar] debug: pts={None if pts is None else pts.shape}, pub={self._pub is not None}")
                if pts is not None and len(pts) > 0 and self._pub is not None:
                    self._publish_lidar(pts)
                    print(f"[lidar] published {len(pts)} points")
            except Exception as e:
                print(f"[lidar] trace failed: {e}")
            time.sleep(1.0 / self._rate_hz)

class MujocoServer:
    """
    Use `MucocoServer.launch_server()` to start the headless simulator.

    This uses the mujoco simulator in headless mode.
    """

    @classmethod
    def launch_server(
        cls,
        scene_xml_path: str | None,
        model: MjModel | None,
        camera_hz: float,
        show_viewer_ui: bool,
        stop_mujoco_process_event: threading.Event,
        data_proxies: MujocoServerProxies,
        cameras_to_use: list[AideaCameras],
        start_translation: list|None,
        start_rotation_quat: list|None
    ):
        server = cls(scene_xml_path, model, stop_mujoco_process_event, data_proxies, start_translation, start_rotation_quat)
        server.run(
            show_viewer_ui=show_viewer_ui,
            camera_hz=camera_hz,
            cameras_to_use=cameras_to_use,
        )

    def __init__(
        self,
        scene_xml_path: str | None,
        model: MjModel | None,
        stop_mujoco_process_event: threading.Event,
        data_proxies: MujocoServerProxies,
        start_translation: list|None,
        start_rotation_quat: list|None
    ):
        """
        Initialize the Simulator handle with a scene
        Args:
            scene_xml_path: str, path to the scene xml file
            model: MjModel, Mujoco model object
        """
        if model is not None and scene_xml_path is not None:
            raise ValueError("You should not provide both a model and a scene_xml_path. Please provide only one.")

        if scene_xml_path is None:
            # Import here to avoid circular dependency
            from aidea_mujoco import MujocoSimulator
            scene_xml_path = MujocoSimulator.get_scene_xml_path()
            print(scene_xml_path)

        # if model is None:
        #     print(f"debug: {scene_xml_path}")

        #     model = MjModel.from_xml_path(scene_xml_path)
            # model = MjModel.from_xml_path(str("/home/amigo/Documents/ament_ws/src/aidea_mujoco/mjcf/scene.xml"))

        if model is None:
            # 方案 B: MjSpec 加载 scene + 各机器人 MJCF
            scene_spec = mujoco.MjSpec.from_file(scene_xml_path)
            for robot_xml in os.environ.get("ROBOT_MJCF_INCLUDES", "").split(","):
                robot_xml = robot_xml.strip()
                if not robot_xml:
                    continue
                robot_spec = mujoco.MjSpec.from_file(robot_xml)
                robot_id = os.path.basename(robot_xml).split("_")[0]  # "g1"
                # for obj in [mujoco.mjtObj.mjOBJ_BODY, mujoco.mjtObj.mjOBJ_JOINT,
                #             mujoco.mjtObj.mjOBJ_ACTUATOR, mujoco.mjtObj.mjOBJ_SITE,
                #             mujoco.mjtObj.mjOBJ_SENSOR]:
                #     robot_spec.add_prefix(f"{robot_id}_", obj)
                # scene_spec.worldbody.attach(robot_spec)

                # 挂到某个 site 上
                # mount_site = scene_spec.worldbody.add_site(name="g1_mount", pos=[2, 0, 0])
                # scene_spec.attach(robot_spec, site=mount_site, prefix="g1_")

                # 直接 attach，用 prefix 参数加前缀
                frame = scene_spec.worldbody.add_frame(pos=[0, 0, 0.793])
                scene_spec.attach(robot_spec, frame=frame, prefix=f"{robot_id}_")
            model = scene_spec.compile()

        change_start_pose(model, name="base_link", translation=start_translation, rotation_quat=start_rotation_quat)

        self.mjmodel = model

        self.mjdata = MjData(self.mjmodel)

        self.use_diff_drive = True
        # for b_name in Actuators.back_wheel_vel.get_joint_names_in_mjcf():
        #     if mujoco.mj_name2id(self.mjmodel, mujoco.mjtObj.mjOBJ_JOINT, b_name) != -1:
        #         self.use_diff_drive = False
        #         break

        # self._base_in_pos_motion = False

        self._stop_mujoco_process_event = stop_mujoco_process_event

        self.data_proxies = data_proxies


        # ---- Stage 2: 唯一的 ZMQ 发布器，传给各 Controller ----
        self.zmq_publisher = ZmqPublisher(
            endpoint=os.environ.get("ZMQ_PUB_ENDPOINT", "tcp://*:5555")
        )
        print(f"[mujoco_server] ZMQ PUB bound to {self.zmq_publisher._endpoint}")    


        # ---- 加载机器人插件 ----
        self.plugins = []
        plugin_cfg_paths = os.environ.get("ROBOT_PLUGIN_CONFIGS", "").split(",")
        plugin_cfg_paths = [p.strip() for p in plugin_cfg_paths if p.strip()]
        for cfg_path in plugin_cfg_paths:
            try:
                with open(cfg_path) as f:
                    cfg = yaml.safe_load(f)
                cls = PLUGIN_REGISTRY[cfg["type"]]
                plugin = cls(self, cfg)
                plugin.on_load(self.mjmodel, self.mjdata)
                self.plugins.append(plugin)
                print(f"[plugins] loaded {cfg['robot_id']} ({cfg['type']}) from {cfg_path}")
            except Exception as e:
                print(f"[plugins] failed to load {cfg_path}: {e}")


        self.base_controller = BaseController(self)

        self.lidar_controller = LidarController(self)

        self.physics_fps_counter = FpsCounter()

        self._last_wall_time = time.perf_counter()
        self._last_sim_time = 0.0

        self.sensor_manager = MujocoServerSensorManagerThreaded(
            sensor_hz=15,
            sensors_to_use=AideaSensors.from_mjmodel(self.mjmodel),
            mujoco_server=self,
        )

        self.update_joint_limits()

        signal.signal(signal.SIGTERM, lambda num, h: self.request_to_stop())
        signal.signal(signal.SIGINT, lambda num, h: self.request_to_stop())

    @property
    def robot_settings(self):
        """Get the robot settings from config.py"""
        return config.robot_settings


    def update_joint_limits(self):
        limits: dict[Actuators, tuple[float, float]] = {}
        for i in range(self.mjmodel.njnt):
            name = mujoco.mj_id2name(self.mjmodel, mujoco.mjtObj.mjOBJ_JOINT, i)
            if name is None:
                continue
            jr = self.mjmodel.jnt_range[i]
            try:
                actuator = Actuators[name]
            except KeyError:
                continue
            limits[actuator] = (float(jr[0]), float(jr[1]))

        for actuator, (lo, hi) in limits.items():
            self.data_proxies.set_joint_limit(actuator, (lo, hi))

    def set_camera_manager(
        self,
        camera_hz: float,
        cameras_to_use: list[AideaCameras],
        *,
        use_camera_thread: bool,
        use_threadpool_executor: bool,
    ):
        """
        This should be called before trying to render offscreen cameras.

        If `use_camera_thread` is false, `self.camera_manager.pull_camera_data_at_camera_rate()` should be called on a UI thread.
        This is the recommended usage.

        If `use_camera_thread` is true, a thread will be spawned to call Renderer.render().
        This may not work on all platforms since rendering should happen on the main thread.
        This mode is mainly used with the Mujoco Managed Viewer, to avoid rendering on the physics thread.
        """
        if use_camera_thread or use_threadpool_executor:
            self.camera_manager = MujocoServerCameraManagerThreaded(
                use_camera_thread=use_camera_thread,
                use_threadpool_executor=use_threadpool_executor,
                camera_hz=camera_hz,
                cameras_to_use=cameras_to_use,
                mujoco_server=self,
            )
        else:
            self.camera_manager = MujocoServerCameraManagerSync(
                camera_hz=camera_hz, cameras_to_use=cameras_to_use, mujoco_server=self
            )

    def run(
        self,
        show_viewer_ui: bool,
        camera_hz: float,
        cameras_to_use: list[AideaCameras],
    ):
        # self.__run_headless_simulation(camera_hz=camera_hz, cameras_to_use=cameras_to_use)
        self.__run_headless_simulation_with_physics_thread(
            camera_hz=camera_hz, cameras_to_use=cameras_to_use
        )

    def _is_requested_to_stop(self):
        try:
            return self._stop_mujoco_process_event.is_set()
        except (EOFError, BrokenPipeError):
            # We likely lost connection to the main process if we've hit this.
            return True

    def request_to_stop(self):
        try:
            self._stop_mujoco_process_event.set()
        except (EOFError, BrokenPipeError):
            # We likely lost connection to the main process if we've hit this.
            ...

    def close(self):
        """
        Clean up C++ resources
        """
        self.request_to_stop()
        
        if hasattr(self, "lidar_controller"):
            self.lidar_controller.stop()


        if isinstance(self.camera_manager, MujocoServerCameraManagerThreaded):
            self.camera_manager.cameras_thread.join()

        if isinstance(self.sensor_manager, MujocoServerSensorManagerThreaded):
            self.sensor_manager.sensors_thread.join()

        self.camera_manager.close()

        for plugin in self.plugins:
            if hasattr(plugin, "on_close"):
                try:
                    plugin.on_close()
                except Exception:
                    pass

        if hasattr(self, "zmq_publisher") and self.zmq_publisher is not None:
            self.zmq_publisher.close()
            self.zmq_publisher = None
            print("[mujoco_server] ZMQ PUB closed")        

    def _run_ui_simulation(self, show_viewer_ui: bool) -> None:
        """
        Run the simulation with the viewer
        """
        raise NotImplementedError(
            "This is headless mode. Use MujocoServerPassive or MujocoServerManaged to run the UI simulator."
        )

    def _physics_step(self, lock: contextlib.AbstractContextManager):
        """
        Calls mj_step and _ctrl_callback, and sleeps until the next timestep.
        """
        start_time = time.perf_counter()

        with lock:
            mujoco._functions.mj_step(self.mjmodel, self.mjdata)
            self._ctrl_callback(self.mjmodel, self.mjdata)

        time_until_next_step = self.mjmodel.opt.timestep - (time.perf_counter() - start_time)
        if time_until_next_step > 0:
            # Sleep to match the timestep.
            time.sleep(time_until_next_step)

    def _physics_loop(
        self, lock: contextlib.AbstractContextManager, termination_check: Callable[[], bool]
    ):
        """
        A loop to use when starting physics in a thread.
        """
        while termination_check():
            self._physics_step(lock=lock)

        click.secho("Physics Loop has terminated.", fg="red")

    def __run_headless_simulation(
        self, camera_hz: float, cameras_to_use: list[AideaCameras]
    ) -> None:
        """
        Run the simulation without the viewer headless.

        Headless mode manages its own `set_camera_manager()` call.
        """
        print("Running headless simulation...")

        self.set_camera_manager(
            use_camera_thread=False,
            use_threadpool_executor=False,
            camera_hz=camera_hz,
            cameras_to_use=cameras_to_use,
        )

        while not self._is_requested_to_stop():
            self._physics_step(contextlib.nullcontext())
            self.camera_manager.pull_camera_data_at_camera_rate(is_sleep_until_ready=False)

        self.close()

    def __run_headless_simulation_with_physics_thread(
        self, camera_hz: float, cameras_to_use: list[AideaCameras]
    ) -> None:
        """
        Run the simulation without the viewer headless.

        Headless mode manages its own `set_camera_manager()` call.
        """
        print("Running headless simulation...")

        self.set_camera_manager(
            use_camera_thread=False,
            use_threadpool_executor=False,
            camera_hz=camera_hz,
            cameras_to_use=cameras_to_use,
        )

        self.lidar_controller.start()

        physics_thread = threading.Thread(
            target=self._physics_loop,
            args=(self.camera_manager.camera_lock, lambda: not self._is_requested_to_stop()),
            daemon=True,
        )
        physics_thread.start()

        while not self._is_requested_to_stop():
            self.camera_manager.pull_camera_data_at_camera_rate(is_sleep_until_ready=True)

        physics_thread.join()
        self.close()

    def _ctrl_callback(self, model: MjModel, data: MjData) -> None:
        """
        Callback function that gets executed with mj_step
        """
        self.mjdata = data
        self.mjmodel = model

        if not self.mjdata or not self.mjdata.time:
            print("WARNING: no mujoco data to report")
            return

        self.physics_fps_counter.tick(sim_time=data.time)
        self.pull_status()
        self.push_command(self.data_proxies.get_command())

        # ---- 机器人插件 step ----
        for plugin in self.plugins:
            try:
                plugin.step(data)
            except Exception as e:
                print(f"[plugins] {plugin.robot_id} step failed: {e}")

        # ---- LiDAR 扫描 ----
        # self._lidar_counter += 1
        # if self._lidar_counter % self._lidar_interval == 0:
        #     try:
        #         self.lidar.trace_rays(self.mjdata, self.rays_theta, self.rays_phi)
        #         points = self.lidar.get_hit_points()
        #         world_points = (
        #             points @ self.lidar.sensor_rotation.T + self.lidar.sensor_position
        #         )
        #         self.data_proxies.set_lidar_points(world_points.astype(np.float32))
        #     except Exception as e:
        #         print(f"[MujocoServer] LiDAR trace failed: {e}", flush=True)


    def pull_status(self):
        data = self.mjdata
        new_status = StatusAideaJoints.default()
        new_status.fps = self.physics_fps_counter.fps
        new_status.time = data.time
        new_status.sim_to_real_time_ratio_msg = self.physics_fps_counter.sim_to_real_time_ratio_msg

        # ---- 6 轴机械臂 ----
        for act_name in [
            "joint2_to_joint1",
            "joint3_to_joint2",
            "joint4_to_joint3",
            "joint5_to_joint4",
            "joint6_to_joint5",
            "joint6output_to_joint6",
        ]:
            a = data.actuator(act_name)
            pv = getattr(new_status, act_name)
            pv.pos = float(a.length[0])
            pv.vel = float(a.velocity[0])
            pv.effort = float(a.force[0])

        # ---- 夹爪 ----
        a = data.actuator("gripper_controller")
        new_status.gripper_controller.pos = float(a.length[0])
        new_status.gripper_controller.vel = float(a.velocity[0])
        new_status.gripper_controller.effort = float(a.force[0])

        # 左右指（从动关节，从 qpos / qvel 读）
        for field, jname in [
            ("gripper_left_finger", "gripper_left3_to_gripper_left1"),
            ("gripper_right_finger", "gripper_right3_to_gripper_right1"),
        ]:
            jid = mujoco.mj_name2id(self.mjmodel, mujoco.mjtObj.mjOBJ_JOINT, jname)
            if jid < 0:
                continue
            qadr = self.mjmodel.jnt_qposadr[jid]
            vadr = self.mjmodel.jnt_dofadr[jid]
            pv = getattr(new_status, field)
            pv.pos = float(data.qpos[qadr])
            pv.vel = float(data.qvel[vadr])
            pv.effort = 0.0

        # ---- base ----
        w_left = data.actuator(Actuators.left_behind_wheel.name).velocity[0]
        w_right = data.actuator(Actuators.right_behind_wheel.name).velocity[0]

        v = (w_left + w_right) * BaseController.WHEEL_RADIUS_M / 2.0
        omega = (w_right - w_left) * BaseController.WHEEL_RADIUS_M / BaseController.WHEEL_BASE_M

        pose = self.base_controller.get_base_pose()
        new_status.base.x = float(pose[0])
        new_status.base.y = float(pose[1])
        new_status.base.theta = float(pose[2])
        new_status.base.x_vel = float(v)
        new_status.base.theta_vel = float(omega)

        new_status.is_self_colliding = False

        self.data_proxies.set_status(new_status)
        self._last_wall_time = time.perf_counter()
        self._last_sim_time = data.time

 

    def _to_real_gripper_range(self, pos: float) -> float:
        """
        Map the gripper position to real gripper range
        """
        return utils.map_between_ranges(
            pos,
            self.robot_settings["sim_gripper_min_max"],
            self.robot_settings["gripper_min_max"],
        )

    def push_command(self, command_status: StatusCommand):
        """
        Handles setting mujoco ctrl properties to move joints.

        Aidea 版本：
          - 轮子（is_wheel）是 velocity actuator，不能用 move_by / move_to
          - 6 轴 + gripper_controller 是 position actuator，直接写 ctrl
          - gripper_controller 是 tendon actuator，直接给关节角度
          - 没有 base_translate / base_rotate 这类虚拟关节
        """
        data = self.mjdata

        # ---------------- move_by ----------------
        for _, command in command_status.move_by.items():
            if not command.trigger:
                continue
            command.trigger = False

            actuator_name = command.actuator_name
            pos = command.pos

             # G1/Go2 插件的 actuator，直接写 ctrl，不走 Actuators 枚举
            if actuator_name.startswith("g1_") or actuator_name.startswith("go2_"):
                data.actuator(actuator_name).ctrl = pos
                continue

            # 找到对应的 Actuators 枚举
            try:
                actuator = Actuators[actuator_name]
            except KeyError:
                raise NotImplementedError(
                    f"Unknown actuator: {actuator_name}"
                )

            # 轮子是 velocity actuator，不能做相对位置
            if actuator.is_wheel:
                raise NotImplementedError(
                    f"Cannot set move_by for velocity actuator {actuator_name}."
                )

            # 其余（6 轴 + 夹爪）都是 position actuator，直接叠加
            current_value = data.actuator(actuator_name).length[0]
            data.actuator(actuator_name).ctrl = current_value + pos

        # ---------------- move_to ----------------
        for _, command in command_status.move_to.items():
            if not command.trigger:
                continue
            command.trigger = False

            actuator_name = command.actuator_name
            pos = command.pos

            try:
                actuator = Actuators[actuator_name]
            except KeyError:
                raise NotImplementedError(
                    f"Unknown actuator: {actuator_name}"
                )

            if actuator.is_wheel:
                raise NotImplementedError(
                    f"Cannot set move_to for velocity actuator {actuator_name}."
                )

            # 6 轴 + 夹爪都是 position actuator，直接写目标
            data.actuator(actuator_name).ctrl = pos

        # ---------------- set_base_velocity ----------------
        if (
            command_status.base_velocity is not None
            and command_status.base_velocity.trigger
        ):
            command_status.base_velocity.trigger = False
            self.base_controller.push_command(command_status.base_velocity)

        # ---------------- keyframe ----------------
        if (
            command_status.keyframe is not None
            and command_status.keyframe.trigger
        ):
            command_status.keyframe.trigger = False
            self.mjdata.ctrl = self.mjmodel.keyframe(command_status.keyframe.name).ctrl

        # ---------------- 底盘更新 ----------------
        self.base_controller.update()

        self.data_proxies.set_command(command_status)

    # def push_command(self, command_status:StatusCommand):
    #     """
    #     Handles setting mujoco ctrl properties to move joints.
    #     """
    #     # move_by
    #     for _, command in command_status.move_by.items():
    #         if command.trigger:
    #             command.trigger = False
    #             actuator_name = command.actuator_name
    #             pos = command.pos
    #             if actuator_name in (Actuators.base_translate.name, Actuators.base_translate_y.name, Actuators.base_rotate.name):
    #                 self.base_controller.push_command(command)
    #             elif actuator_name == Actuators.gripper.name:
    #                 if self.use_diff_drive:
    #                     current_value = self._to_real_gripper_range(
    #                         self.mjdata.actuator(actuator_name).length[0]
    #                     )
    #                     self.mjdata.actuator(actuator_name).ctrl = self._to_sim_gripper_range(
    #                         current_value + pos
    #                     )
    #                 else:
    #                     current_value_left = self.mjdata.actuator(Actuators.gripper_left_finger.name).length[0]
    #                     current_value_right = self.mjdata.actuator(Actuators.gripper_right_finger.name).length[0]
    #                     finger_cmd = self.aperture_angle_radians_to_urdf_angle_radians(pos)
    #                     self.mjdata.actuator(Actuators.gripper_left_finger.name).ctrl = current_value_left + finger_cmd
    #                     self.mjdata.actuator(Actuators.gripper_right_finger.name).ctrl = current_value_right + finger_cmd
    #             else:
    #                 current_value = self.mjdata.actuator(actuator_name).length[0]
    #                 self.mjdata.actuator(actuator_name).ctrl = current_value + pos

    #     # move_to
    #     for _, command in command_status.move_to.items():
    #         if command.trigger:
    #             command.trigger = False
    #             actuator_name = command.actuator_name

    #             pos = command.pos
    #             if actuator_name in (Actuators.base_translate.name, Actuators.base_translate_y.name, Actuators.base_rotate.name):
    #                 raise NotImplementedError(
    #                     f"Cannot set move_to for {actuator_name}, which is a relative joint."
    #                 )
    #             elif actuator_name == Actuators.gripper.name:
    #                 if self.use_diff_drive:
    #                     self.mjdata.actuator(actuator_name).ctrl = self._to_sim_gripper_range(pos)
    #                 else:
    #                     finger_pos = self.aperture_angle_radians_to_urdf_angle_radians(pos)
    #                     self.mjdata.actuator(Actuators.gripper_left_finger.name).ctrl = finger_pos
    #                     self.mjdata.actuator(Actuators.gripper_right_finger.name).ctrl = finger_pos
    #             else:
    #                 self.mjdata.actuator(actuator_name).ctrl = pos

    #     # set_base_velocity
    #     if command_status.base_velocity is not None and command_status.base_velocity.trigger:
    #         command_status.base_velocity.trigger = False
    #         self.base_controller.push_command(command_status.base_velocity)

    #     # keyframe
    #     if command_status.keyframe is not None and command_status.keyframe.trigger:
    #         command_status.keyframe.trigger = False
    #         self.mjdata.ctrl = self.mjmodel.keyframe(command_status.keyframe.name).ctrl

    #     self.base_controller.update()

    #     self.data_proxies.set_command(command_status)

    def _to_sim_gripper_range(self, pos: float) -> float:
        """
        Map the gripper position to sim gripper range
        """
        return utils.map_between_ranges(
            pos,
            self.robot_settings["gripper_min_max"],
            self.robot_settings["sim_gripper_min_max"],
        )

    # NOTE: This is copied from gripper_conversions.py to avoid a aidea_body dependency, except the urdf offset hsa been removed
    def aperture_angle_radians_to_urdf_angle_radians(self, aperture_angle_radians):

        def get_angle_from_chord_length_and_radius(radius_m, chord_m):
            return 2 * np.arcsin(chord_m / (2 * radius_m))   # radians

        aperture_open_rad = get_angle_from_chord_length_and_radius(
                                self.robot_settings['gripper_conversion']['finger_length_m'],
                                self.robot_settings['gripper_conversion']['aperture_open_m'],
                                )

        aperture_close_rad = get_angle_from_chord_length_and_radius(
                                self.robot_settings['gripper_conversion']['finger_length_m'],
                                self.robot_settings['gripper_conversion']['aperture_closed_m'],
                                )

        return utils.map_between_ranges(
            aperture_angle_radians,
            (aperture_close_rad, aperture_open_rad),
            (self.robot_settings['gripper_conversion']['urdf_closed_rad'], self.robot_settings['gripper_conversion']['urdf_open_rad'])
            )

    def urdf_angle_radians_to_aperture_angle_radians(self, urdf_angle_radians):
        def get_angle_from_chord_length_and_radius(radius_m, chord_m):
            return 2 * np.arcsin(chord_m / (2 * radius_m))   # radians

        aperture_open_rad = get_angle_from_chord_length_and_radius(
                                self.robot_settings['gripper_conversion']['finger_length_m'],
                                self.robot_settings['gripper_conversion']['aperture_open_m'],
                                )

        aperture_close_rad = get_angle_from_chord_length_and_radius(
                                self.robot_settings['gripper_conversion']['finger_length_m'],
                                self.robot_settings['gripper_conversion']['aperture_closed_m'],
                                )

        return utils.map_between_ranges(
            urdf_angle_radians,
            (self.robot_settings['gripper_conversion']['urdf_closed_rad'], self.robot_settings['gripper_conversion']['urdf_open_rad']),
            (aperture_close_rad, aperture_open_rad)
            )
