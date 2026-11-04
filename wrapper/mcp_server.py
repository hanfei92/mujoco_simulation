# wrapper/mcp_server.py
"""Stage 3 最小 MCP Server：启动仿真子进程，暴露 LiDAR 快照和基础控制工具。"""
import time
import multiprocessing
from multiprocessing import Manager, Process

from mcp.server import MCPServer

from aidea_mujoco.mujoco_server import MujocoServer, MujocoServerProxies
from aidea_mujoco.datamodels.status_command import (
    CommandBaseVelocity,
    CommandMove,
)

mcp = MCPServer("aidea-mujoco")

_data_proxies = None
_manager = None
_stop_event = None
_server_process = None


def _start_sim():
    global _data_proxies, _manager, _stop_event, _server_process

    _manager = Manager()
    _stop_event = _manager.Event()
    _data_proxies = MujocoServerProxies.default(_manager)

    multiprocessing.set_start_method("spawn", force=True)
    _server_process = Process(
        target=MujocoServer.launch_server,
        args=(
            None,          # scene_xml_path → 默认
            None,          # model
            30.0,          # camera_hz
            False,         # show_viewer_ui
            _stop_event,
            _data_proxies,
            [],            # cameras_to_use
            None, None,    # start_translation, start_rotation_quat
        ),
        daemon=False,
    )
    _server_process.start()

    print("[mcp] waiting for simulation...")
    for _ in range(60):
        try:
            if _data_proxies.get_status().time > 0:
                break
        except Exception:
            pass
        time.sleep(0.5)
    print("[mcp] simulation ready")


@mcp.tool()
def get_lidar_snapshot() -> dict:
    """获取一次 LiDAR 点云快照（world 系）。"""
    pts = _data_proxies.get_lidar_points()
    if pts is None or len(pts) == 0:
        return {"num_points": 0, "frame_id": "world"}
    return {
        "num_points": int(len(pts)),
        "frame_id": "world",
        "x_range": [float(pts[:, 0].min()), float(pts[:, 0].max())],
        "y_range": [float(pts[:, 1].min()), float(pts[:, 1].max())],
        "z_range": [float(pts[:, 2].min()), float(pts[:, 2].max())],
    }


@mcp.tool()
def set_base_velocity(vx: float, omega: float) -> dict:
    """设置底盘线速度 (m/s) 和角速度 (rad/s)。"""
    cmd = _data_proxies.get_command()
    cmd.set_base_velocity(
        CommandBaseVelocity(v_x=vx, v_y=0.0, omega=omega, trigger=True)
    )
    _data_proxies.set_command(cmd)
    return {"status": "ok", "vx": vx, "omega": omega}


@mcp.tool()
def move_joint(actuator_name: str, pos: float) -> dict:
    """移动指定关节到绝对位置 (rad)。"""
    cmd = _data_proxies.get_command()
    cmd.set_move_to(CommandMove(actuator_name=actuator_name, pos=pos, trigger=True))
    _data_proxies.set_command(cmd)
    return {"status": "ok", "actuator": actuator_name, "pos": pos}


def main():
    _start_sim()
    try:
        mcp.run(transport="streamable-http", host="0.0.0.0", port=8080)
    finally:
        if _stop_event is not None:
            _stop_event.set()
        if _server_process is not None:
            _server_process.join(timeout=10)


if __name__ == "__main__":
    main()