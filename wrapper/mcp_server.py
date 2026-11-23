# wrapper/mcp_server.py
"""MCP Server：LiDAR 快照 + 底盘控制 + G1 模式/位移控制。"""
import time
import multiprocessing
from multiprocessing import Manager, Process

from mcp.server import MCPServer

from aidea_mujoco.mujoco_server import MujocoServer, MujocoServerProxies
from aidea_mujoco.enums.aidea_cameras import AideaCameras
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
            None, None, 30.0, True,
            _stop_event, _data_proxies, [AideaCameras.cam_nav_rgb],
            None, None,
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


# ---------------- LiDAR ----------------

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


# ---------------- 现有机器人底盘 ----------------

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


# ---------------- 通用机器人指令（g1/go2/...） ----------------

@mcp.tool()
def set_robot_velocity(robot_id: str, vx: float, vy: float, wz: float) -> dict:
    """设置机器人速度指令。不会覆盖 mode/teleport。"""
    cmds = _data_proxies.get_robot_command()
    entry = cmds.get(robot_id, {})
    entry.update({"vx": vx, "vy": vy, "wz": wz})
    cmds[robot_id] = entry
    _data_proxies.set_robot_command(cmds)
    return {"status": "ok", "robot_id": robot_id, "cmd": entry}


@mcp.tool()
def move_robot_joint(robot_id: str, joint_name: str, pos: float) -> dict:
    """移动指定机器人的关节。joint_name 不带 robot_id 前缀。"""
    full_name = f"{robot_id}_{joint_name}"
    cmd = _data_proxies.get_command()
    cmd.set_move_to(CommandMove(actuator_name=full_name, pos=pos, trigger=True))
    _data_proxies.set_command(cmd)
    return {"status": "ok", "actuator": full_name, "pos": pos}


# ---------------- G1 专用：模式 / 速度 / 瞬移 / 重置 ----------------

@mcp.tool()
def set_g1_mode(mode: str) -> dict:
    """设置 G1 模式。mode: 'stand' | 'walk' | 'fall'。"""
    if mode not in ("stand", "walk", "fall"):
        return {"error": f"invalid mode: {mode}, must be stand/walk/fall"}
    cmds = _data_proxies.get_robot_command()
    entry = cmds.get("g1", {})
    entry["mode"] = mode
    cmds["g1"] = entry
    _data_proxies.set_robot_command(cmds)
    return {"status": "ok", "mode": mode}


@mcp.tool()
def set_g1_velocity(vx: float, wz: float) -> dict:
    """设置 G1 平移速度 (m/s) 和转向速度 (rad/s)。"""
    cmds = _data_proxies.get_robot_command()
    entry = cmds.get("g1", {})
    entry.update({"vx": vx, "wz": wz})
    cmds["g1"] = entry
    _data_proxies.set_robot_command(cmds)
    return {"status": "ok", "vx": vx, "wz": wz}


@mcp.tool()
def move_g1_to(x: float, y: float, yaw: float) -> dict:
    """瞬移 G1 到指定位置和朝向（弧度）。"""
    cmds = _data_proxies.get_robot_command()
    entry = cmds.get("g1", {})
    entry["teleport"] = {"x": x, "y": y, "yaw": yaw}
    cmds["g1"] = entry
    _data_proxies.set_robot_command(cmds)
    return {"status": "ok", "x": x, "y": y, "yaw": yaw}


@mcp.tool()
def reset_g1() -> dict:
    """重置 G1：站立、原点、零速度。"""
    cmds = _data_proxies.get_robot_command()
    cmds["g1"] = {
        "mode": "stand",
        "vx": 0.0,
        "vy": 0.0,
        "wz": 0.0,
        "teleport": {"x": 0.0, "y": 0.0, "yaw": 0.0},
    }
    _data_proxies.set_robot_command(cmds)
    return {"status": "ok"}


@mcp.tool()
def get_g1_state() -> dict:
    """获取 G1 当前指令状态（mode、vx、wz）。"""
    cmds = _data_proxies.get_robot_command()
    entry = cmds.get("g1", {})
    return {
        "mode": entry.get("mode", "stand"),
        "vx": entry.get("vx", 0.0),
        "wz": entry.get("wz", 0.0),
    }


@mcp.tool()
def set_person_pose(x: float, y: float, yaw: float = 0.0) -> dict:
    """把 mocap 人偶移动到 world 系 (x, y, yaw)，用于手工调试。"""
    cmds = _data_proxies.get_robot_command()
    cmds["person"] = {"x": x, "y": y, "yaw": yaw}
    _data_proxies.set_robot_command(cmds)
    return {"status": "ok", "x": x, "y": y, "yaw": yaw}


# ---------------- 启动 ----------------

def main():
    _start_sim()
    try:
        mcp.run(transport="streamable-http", host="0.0.0.0", port=8081)
    finally:
        if _stop_event is not None:
            _stop_event.set()
        if _server_process is not None:
            _server_process.join(timeout=10)


if __name__ == "__main__":
    main()