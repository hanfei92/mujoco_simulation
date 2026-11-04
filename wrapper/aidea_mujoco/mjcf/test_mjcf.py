import mujoco
import mujoco.viewer
import time

# 加载你生成的 XML 文件
model = mujoco.MjModel.from_xml_path("/home/amigo/Documents/ament_ws/src/aidea_mujoco/mjcf/room-732.xml")
data = mujoco.MjData(model)

# 启动被动查看器
with mujoco.viewer.launch_passive(model, data) as viewer:
    # 让窗口保持打开，直到你手动关闭
    while viewer.is_running():
        mujoco.mj_step(model, data)  # 如果模型有物理属性，会开始仿真
        viewer.sync()
        time.sleep(1.0 / 60.0)  # 约 60 FPS