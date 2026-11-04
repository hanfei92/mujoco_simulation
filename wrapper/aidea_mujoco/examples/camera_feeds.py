import random
import threading
import time
import click
import cv2
from aidea_mujoco.enums.actuators import Actuators
from aidea_mujoco.enums.aidea_cameras import AideaCameras
from aidea_mujoco.aidea_mujoco_simulator import AideaMujocoSimulator
from aidea_mujoco.aidea_mujoco_simulator import MujocoSimulator


def show_camera_feeds_sync(sim: MujocoSimulator, print_fps: bool):
    """
    Pull camera data from the simulator and display it using OpenCV.

    Call this after calling MujocoSimulator::start().
    """

    camera_data = sim.pull_camera_data()

    if print_fps:
        print(
            f"Physics fps: {sim.pull_status().fps}. Camera FPS: {camera_data.fps}. {sim.pull_status().sim_to_real_time_ratio_msg}"
        )

    for camera_name, pixels in camera_data.get_all(use_depth_color_map=True).items():
        cv2.namedWindow(camera_name.name, cv2.WINDOW_NORMAL)
        cv2.imshow(camera_name.name, pixels)

    cv2.waitKey(1)


def my_control_loop(sim: MujocoSimulator,):
    while sim.is_running():
        sim.lift.move_to(random.random())
        time.sleep(3)


@click.command()

def main():
    cameras_to_use = (AideaCameras.rgb_aidea())

    sim = (AideaMujocoSimulator(cameras_to_use=cameras_to_use))

    sim.start(headless=False)

    threading.Thread(target=my_control_loop, daemon=True, args=[sim]).start()

    try:
        while sim.is_running():
            show_camera_feeds_sync(sim, True)

    except KeyboardInterrupt:
        sim.stop()


if __name__ == "__main__":
    main()
