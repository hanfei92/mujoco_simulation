import mujoco
import numpy as np

m = mujoco.MjModel.from_xml_path("mujoco_description_formatted_test.xml")
d = mujoco.MjData(m)
mujoco.mj_forward(m, d)

# 整体重心（world frame）
total_mass = 0.0
com = np.zeros(3)
for i in range(m.nbody):
    mass = m.body_mass[i]
    if mass > 0:
        total_mass += mass
        com += mass * d.xipos[i]   # xipos 是 body 惯性中心的世界坐标
com /= total_mass
print(f"总质量: {total_mass:.2f} kg")
print(f"整体重心 (world): {com}")