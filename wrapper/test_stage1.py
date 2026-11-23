# # wrapper/test_stage1_container.py
# """
# Stage 1 container test: headless simulation + LiDAR point cloud.

# No GUI, no cameras, no Rerun, no keyboard.
# Only verifies:
#   - simulator starts headless
#   - physics loop advances (status.time > 0)
#   - LiDAR ray tracing produces non-empty point cloud
# """
# import time
# import numpy as np

# import aidea_mujoco
# from aidea_mujoco.enums.aidea_cameras import AideaCameras


# def main():
#     print("[stage1] creating simulator (no cameras)...")
#     sim = aidea_mujoco.AideaMujocoSimulator(
#         scene_xml_path=None,          # 用默认 utils.models_path / "scene.xml"
#         cameras_to_use=[],            # headless 不需要相机
#     )

#     print("[stage1] starting headless...")
#     sim.start(headless=True)

#     # 等物理循环和 LiDAR 线程跑几轮
#     print("[stage1] warming up (5s)...")
#     time.sleep(5)

#     # ---- 状态检查 ----
#     status = sim.pull_status()
#     print(f"[stage1] sim time: {status.time:.3f}")
#     print(f"[stage1] base pose: x={status.base.x:.3f}, y={status.base.y:.3f}, theta={status.base.theta:.3f}")

#     if status.time <= 0:
#         print("[stage1] FAIL: sim time is 0, physics loop not running")
#         sim.stop()
#         return

#     # ---- LiDAR ray tracing 点云（不依赖相机） ----
#     print("[stage1] pulling ray-traced lidar points (world frame)...")
#     pts_world = sim.pull_airy_lidar_points_raytrace(in_world_frame=True)
#     print(f"[stage1] world points shape: {pts_world.shape}, dtype: {pts_world.dtype}")

#     if pts_world.shape[0] == 0:
#         print("[stage1] FAIL: empty world point cloud")
#         sim.stop()
#         return

#     print(f"[stage1] world x: [{pts_world[:,0].min():.3f}, {pts_world[:,0].max():.3f}]")
#     print(f"[stage1] world y: [{pts_world[:,1].min():.3f}, {pts_world[:,1].max():.3f}]")
#     print(f"[stage1] world z: [{pts_world[:,2].min():.3f}, {pts_world[:,2].max():.3f}]")

#     print("[stage1] pulling ray-traced lidar points (base_link frame)...")
#     pts_base = sim.pull_airy_lidar_points_raytrace(in_world_frame=False)
#     print(f"[stage1] base_link points shape: {pts_base.shape}")

#     if pts_base.shape[0] == 0:
#         print("[stage1] FAIL: empty base_link point cloud")
#         sim.stop()
#         return

#     print("[stage1] PASS: headless sim + non-empty lidar points")
#     sim.stop()
#     print("[stage1] done.")


# if __name__ == "__main__":
#     main()

# wrapper/test_stage_gui.py
"""宿主机调试用：默认开 GUI，可 --headless 关掉。"""
import argparse
import time

import aidea_mujoco


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--headless", action="store_true", help="run without GUI")
    args = parser.parse_args()

    print(f"[gui-test] creating simulator (headless={args.headless})...")
    sim = aidea_mujoco.AideaMujocoSimulator(
        scene_xml_path=None,
        cameras_to_use=[],
    )

    print("[gui-test] starting...")
    sim.start(headless=args.headless)

    print("[gui-test] running 30s, close window or Ctrl+C to stop...")
    try:
        while sim.is_running():
            time.sleep(0.1)
    except KeyboardInterrupt:
        pass
    finally:
        sim.stop()
        print("[gui-test] done.")


if __name__ == "__main__":
    main()