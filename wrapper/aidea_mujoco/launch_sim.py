from examples.rerun_utils import RerunLogger
import aidea_mujoco
import click
import cv2

from pynput import keyboard
from keyboard_teleop import (
    KeyboardController,
    print_keyboard_options,
)

from aidea_mujoco.enums.aidea_cameras import AideaCameras


@click.command()
@click.option("--scene-xml-path", help="Path to a scene xml file")
@click.option("--headless", is_flag=True, help="Run the simulation headless")
@click.option("--imagery", is_flag=True, help="Show the cameras' imagery")
@click.option("--lidar3d", is_flag=True, help="Show the point cloud in Rerun")
@click.option("--keyboard", "use_keyboard", is_flag=True, help="Enable keyboard control")
def main(
    scene_xml_path: str,
    headless: bool,
    imagery: bool,
    lidar3d: bool,
    use_keyboard:bool
):

    rerun_logger = RerunLogger()
    # rerun_logger = None

    cameras_to_use =(AideaCameras.all_aidea()) if imagery else []


    if lidar3d:
        rerun_logger.init_pointcloud_viz()

    sim = aidea_mujoco.AideaMujocoSimulator(scene_xml_path,cameras_to_use=cameras_to_use)

    sim.start(headless=headless)

    kbd = None
    listener = None
    if use_keyboard:
        print_keyboard_options()
        kbd = KeyboardController(sim)          # sim 已连接，能读初始关节位置
        listener = keyboard.Listener(
            on_press=kbd.on_press,
            on_release=kbd.on_release,
        )
        listener.start()

    # MuJoCo 默认 timestep
    dt = 0.002

    try:
        while sim.is_running() and (kbd is None or not kbd.quit_requested):

            # keyboard control
            if kbd is not None:
                kbd.process_keys()
                kbd.apply(dt)

            try:
                camera_data = sim.pull_camera_data()
            except ConnectionError:
                break

            for camera in cameras_to_use:
                cv2.namedWindow(camera.name, cv2.WINDOW_NORMAL)
                cv2.imshow(camera.name, camera_data.get_camera_data(camera))
                if camera != AideaCameras.cam_hemilidar_airy:
                    if camera.is_depth:
                        raw = camera_data.get_camera_data(camera)
                        rerun_logger.update_camera_viz(camera.name, raw, is_depth=True)
                    else:
                        raw = camera_data.get_camera_data(camera)   # BGR uint8 (H, W, 3)
                        rerun_logger.update_camera_viz(camera.name, raw, is_depth=False)

            if lidar3d:
                try:
                    # world frame
                    # pts = sim.pull_airy_lidar_points(in_world_frame=True)
                    # rerun_logger.update_pointcloud_viz(pts, "world/lidar_points")

                    # base_frame
                    pts_base = sim.pull_airy_lidar_points(in_world_frame=False)
                    rerun_logger.update_pointcloud_viz(pts_base, "base_link/lidar_points")
                except Exception as e:
                    print(f"[warn] lidar: {e}")

            cv2.waitKey(10)

    except KeyboardInterrupt:
        pass
    finally:
        if listener is not None:
            listener.stop()
        rerun_logger.stop()
        sim.stop()
        cv2.destroyAllWindows()



if __name__ == "__main__":
    main()
