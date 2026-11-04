import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("mjcf/scene.xml")
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

cid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_CAMERA, "realsense_rgb")
print("cam_pos (world):", d.cam_xpos[cid])
print("cam_mat (world):\n", d.cam_xmat[cid].reshape(3, 3))
print("视线方向 (-Z_cam):", -d.cam_xmat[cid].reshape(3, 3)[:, 2])