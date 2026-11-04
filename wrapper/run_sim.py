# wrapper/run_sim.py
"""Stage 2 长驻入口：启动 headless 仿真，持续运行，ZMQ 发布 LiDAR。"""
import time

import aidea_mujoco


def main():
    sim = aidea_mujoco.AideaMujocoSimulator(cameras_to_use=[])
    sim.start(headless=True)
    print("[run_sim] simulation running, Ctrl+C to stop")
    try:
        while sim.is_running():
            time.sleep(1)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
        print("[run_sim] stopped")


if __name__ == "__main__":
    main()