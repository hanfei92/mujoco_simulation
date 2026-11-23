# 行人跟随 × MuJoCo 仿真集成

把 `person_following_ws` 里的**观测点管理器**接到 MuJoCo 仿真上：G1 作为被控机器人，
mocap 人偶作为跟随目标，LiDAR 提供障碍。

## 数据流

```text
mujoco-wrapper ──ZMQ PUB :5555 (lidar / pose)──► pf-sim ──ROS──► manager
      ▲                                              │
      └──── ZMQ PULL :5556 (robot / person 命令) ◄────┘
```

| 方向 | 通道 | 内容 |
| --- | --- | --- |
| wrapper → pf-sim | ZMQ `pose` | `g1_pelvis` 世界位姿，30 Hz |
| wrapper → pf-sim | ZMQ `lidar` | 世界系点云（原有） |
| pf-sim → wrapper | ZMQ `Command` | `kind=robot`：`vx/omega/mode`；`kind=person`：`x/y/yaw` |
| pf-sim → ROS | `/tf`、`/odom` | `world→odom`（静态）、`odom→base_footprint`（G1 位姿） |
| pf-sim → ROS | `/map`、两个 costmap、`/map_follow/localization_ready` | 空白房间 + LiDAR 局部障碍 |
| pf-sim → ROS | `/monocular_people_tracking/tracks`、`/monocular_person_following/target` | 人偶真值 |
| manager → pf-sim | `/cmd_vel` | 管理器输出的底盘速度 |

## 运行

前提：仓库旁边有 `person_following_ws`（compose 会以只读方式挂到 `/opt/person_following_ws`），
且宿主机能跑 MuJoCo（GPU / EGL）。

```bash
# 启动 MuJoCo + 桥接 + 管理器
scripts/run_pf_sim.sh

# 只启动桥接，不跑管理器（例如想自己起管理器）
PF_MANAGER=0 docker compose -f docker-compose.yml -f docker-compose.pf-sim.yml \
    --profile pf up -d --build mujoco-wrapper pf-sim

# 看管理器日志与决策
docker exec pf-sim bash -lc 'tail -f /tmp/person_follow_observe/observe-latest.jsonl'
```

人偶路径由 `person_walker.py` 参数控制：`~path`（`circle`/`line`/`still`）、`~speed`、
`~radius`、`~center`。手工驱动人偶也可以（MCP）：

```bash
curl -s -X POST http://localhost:8081/mcp -H 'Content-Type: application/json' \
  -d '{"method":"tools/call","params":{"name":"set_person_pose","arguments":{"x":3,"y":1,"yaw":0}}}'
```

## 改动内容（feature/person-following-sim）

**MuJoCo wrapper**
- `proto/command.proto` + `command_server.py`：新增 ZMQ PULL 命令通道（5556），写入与 MCP 相同的 `robot_command`。
- `mujoco_server._publish_pose()`：30 Hz 发布 `g1_pelvis` 位姿（`topic=pose`）。
- `mjcf/person.xml` + `scene.xml` include：mocap 人偶（躯干/头/腿三个 geom）。
- `plugins/person/`：读 `robot_command['person']` 写 `mocap_pos/quat`；注册进 `PLUGIN_REGISTRY`。
- `mcp_server.set_person_pose()`：手工摆位人偶。
- `Dockerfile`：生成 `command_pb2`。

**ROS 桥接**
- `ros-bridge/pf_sim/pose_tf.py`、`cmd_vel_bridge.py`、`sim_world.py`、`person_walker.py`。
- `ros-bridge/Dockerfile.pf-sim`：ROS Noetic + 消息替身（`pf_sim/msgs/`，从
  `person_following_ws/tools/ros_smoke/msgs` 复制）。
- `docker-compose.pf-sim.yml` + `scripts/run_pf_sim.sh`。

## 限制（务必先读）

- **未实跑验证**：本机没有可用的 GPU/GUI 环境，只做了 Python 语法、shell 语法与 compose 合并校验。
  首次运行请先 `PF_MANAGER=0` 只起桥接，确认 `/tf`、`/lidar/points`、`/map` 正常后再开管理器。
- **全局地图是 20×20 的空白房间**（带边界墙），不含 office 的家具；家具只通过 LiDAR 进入
  **局部**代价地图（5×5 m）。全局可达性因此偏乐观。
- **mocap 人偶没有物理碰撞**，G1 会穿过它；避让只依赖 LiDAR 与代价地图。
- **G1 是运动学锁定 + 假步态**：只有 `mode=walk` 才会响应 `vx/omega`（`cmd_vel_bridge` 会自动置 walk）。
- **管理器以 `world` 为地图坐标系**（不是 `map`），TF 由 entrypoint 里的静态发布器搭好。
- `person_following_ws` 是只读挂载，管理器日志写到容器内 `/tmp/person_follow_observe`；
  想留在宿主机就自己加一个 volume。

## 排查

| 现象 | 先看 |
| --- | --- |
| 没有 `/tf` | `docker logs mujoco-wrapper \| grep pose`；wrapper 是否打印 `command PULL bound` |
| G1 不动 | MCP `get_g1_state` 看 mode 是否 walk；`rostopic hz /cmd_vel` |
| 管理器一直 RECOVERING | `rostopic echo /map_follow/localization_ready`、`rosrun tf tf_echo world base_link` |
| 人偶不动 | `rostopic hz /monocular_people_tracking/tracks`；`docker logs pf-sim` 里 person_walker 是否报错 |
| 规划总失败 | 局部代价地图把家具点标成了障碍，检查 `sim_world.py` 的 lidar 回调 |
