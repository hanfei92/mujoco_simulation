# 仿真验证流程（启动顺序）

**默认行走模式是 `route`**：G1 按房间地图走一条带安全间距的覆盖路线。

## 一条命令（推荐）

```bash
cd /home/amigo/project/RND/mujoco_simulation
scripts/start_pf_sim.sh
```

脚本按下面的顺序做完全部步骤，并在每步打印就绪判据：

```
1/6 docker network          sim-net
2/6 mujoco wrapper          起容器 → wait_ready.sh --wrapper（MCP 能应答）
3/6 occupancy map           生成 tmp/pf_map.json（路线模式必需）
4/6 pf-sim                  route + 真实跟踪 + 管理器 + 相机视图 + 真值轨迹
5/6 wait                    wait_ready.sh --bridge（跟踪话题就绪）
6/6 ready                   打印相机地址、实时表、热力图、录包命令
```

可选开关（都是环境变量）：

| 变量 | 默认 | 说明 |
| --- | --- | --- |
| `MUJOCO_VIEWER` | 1 | 0 = 无头（无桌面时用） |
| `PF_PERSON_MODE` | route | `route` / `circle` / `still` |
| `MAP_REGEN` | 0 | 1 = 重新扫描房间地图 |
| `PF_TRACKER` | real | fake = 不编译跟踪节点，用真值轨迹 |
| `PF_CAMERA_VIEW` | 1 | 相机网页视图 |
| `PF_PERSON_TRACE` | 1 | 记录真值轨迹（热力图要用） |

```bash
MUJOCO_VIEWER=0 scripts/start_pf_sim.sh        # 无头
MAP_REGEN=1 scripts/start_pf_sim.sh            # 房间改了，重扫地图
PF_PERSON_MODE=circle scripts/start_pf_sim.sh  # 快速看闭环跟随
```

## 手动分步（与脚本等价）

```bash
# 0) 镜像（代码改动后重做）
docker build -f wrapper/Dockerfile -t mujoco_simulation-mujoco-wrapper:latest wrapper
docker build -f ros-bridge/Dockerfile.pf-sim -t mujoco_simulation-pf-sim:latest ros-bridge

# 1) 网络
docker network create sim-net 2>/dev/null || true

# 2) 仿真（带窗口）
xhost +local:root
docker run -d --name mujoco-wrapper --network sim-net --device nvidia.com/gpu=all \
  -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl -e MUJOCO_VIEWER=1 \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw \
  -e ROBOT_MJCF_INCLUDES=/app/aidea_mujoco/mjcf/robots/g1/g1_12dof.xml \
  -e ROBOT_PLUGIN_CONFIGS=/app/aidea_mujoco/aidea_mujoco/plugins/g1/config.yaml \
  mujoco_simulation-mujoco-wrapper:latest
scripts/wait_ready.sh --wrapper

# 3) 房间地图（route 模式必需；房间不变只需做一次）
docker exec mujoco-wrapper mkdir -p /app/tools
docker cp tools/make_map_from_mjcf.py mujoco-wrapper:/app/tools/
docker exec mujoco-wrapper bash -lc \
  'cd /app && uv run python /app/tools/make_map_from_mjcf.py --out /tmp/pf_map.json'
docker cp mujoco-wrapper:/tmp/pf_map.json tmp/pf_map.json

# 4) 桥接 + 跟踪 + 管理器 + 路线
mkdir -p tmp/sim_logs
docker run -d --name pf-sim --network sim-net \
  -e ZMQ_LIDAR_ENDPOINT=tcp://mujoco-wrapper:5555 \
  -e ZMQ_CMD_ENDPOINT=tcp://mujoco-wrapper:5556 \
  -e PF_MANAGER=1 -e PF_TRACKER=real -e PF_PERSON_MODE=route \
  -e PF_PERSON_TRACE=1 -e PF_CAMERA_VIEW=1 \
  -e PF_MAP_FILE=/tmp/pf_map.json -e PF_LOG_DIR=/tmp/person_follow_observe \
  -p 8090:8090 \
  -v /home/amigo/project/RND/person_following_ws:/opt/person_following_ws:ro \
  -v $PWD/tmp/sim_logs:/tmp/person_follow_observe \
  -v $PWD/tmp/pf_map.json:/tmp/pf_map.json:ro \
  mujoco_simulation-pf-sim:latest
scripts/wait_ready.sh --bridge
```

## 验收标准

| 级别 | 判据 |
| --- | --- |
| 仿真起来 | `wait_ready.sh --wrapper` 通过，`get_g1_state '{}'` 有返回 |
| 有图像 | `/pf_cam/image_raw` 30 Hz（开窗口约 15 Hz），`http://127.0.0.1:8090/` 能看 |
| 有 TF | `/tf` 里有 `base_footprint`、`pf_cam_optical` |
| 有检测 | `/pose_estimator/pose` 15 Hz，每帧 3 个部位（颈+双踝） |
| 有轨迹 | `/monocular_people_tracking/tracks` 非空 |
| 有路线 | `/person_route/path` 33 个航点 |
| G1 在走 | `/person_pose_stamped` 位置持续变化 |
| 有决策 | `/observe_manager/state` 在 OBSERVING / MICRO_ADJUST / SEARCHING 间变化 |
| 底盘响应 | `/cmd_vel` 非 0 且 odom 变化 |

## 观察与取证

```bash
# 相机（含 GT 颈/踝点、tracks 数、管理器状态）
http://127.0.0.1:8090/

# 实时表
docker exec pf-sim bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; \
  python3 /app/pf_sim/dev_watch_loop.py 60 2'

# 状态×动作计数、/cmd_vel 分布
... dev_probe_decision.py 6

# 网页时间轴
python3 /home/amigo/project/RND/person_following_ws/tools/ros_smoke/web_view.py \
    --dir tmp/sim_logs --port 8781

# 文本摘要
python3 scripts/summarize_pf_session.py tmp/sim_logs
```

## 覆盖度热力图

```bash
python3 scripts/coverage_heatmap.py tmp/sim_logs \
    --map tmp/pf_map.json --trace tmp/sim_logs/person_trace.jsonl --out tmp/coverage.png
```

黑色 = 家具，深灰 = 没走到，红→绿 = 可见比例 0→100%，蓝点 = 小车。
一次约 2 分钟的路线实测：可见比例 33%，40 个采样格里 23 格是盲区。

## 录包与回灌

```bash
DURATION=60 TAG=run1 scripts/record_pf_session.sh
scripts/replay_pf_session.sh tmp/sim_sessions/run1
python3 scripts/summarize_pf_session.py \
    tmp/sim_sessions/run1 tmp/sim_sessions/run1/replay/replay-*
```

## 坑位对照表

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `ERROR: no SID` | MCP 未就绪 | `scripts/wait_ready.sh --wrapper` |
| `grep "simulation ready"` 卡住 | print 块缓冲 | 已加 `PYTHONUNBUFFERED=1` |
| 没有窗口 | `run()` 写死 headless | `MUJOCO_VIEWER=1` |
| G1 不动 | 地图缺失或 `PF_PERSON_MODE=still` | 看 `docker logs pf-sim | grep 'map file'` |
| G1 原地打转 | 缺 `/person_pose_stamped` 的 yaw | 用新镜像 |
| 小车不动 | 观测点已够好，或人不在视场 | 属设计行为，看 `/cmd_vel` |
| 有图像没 tracks | 缺 `camera_info` remap | entrypoint 已修 |
| 有 pose 没 tracks | 脚踝被画面裁掉 | 相机加俯角或拉远 |
| 回灌 master 连不上 | `--network none` 下无地址 | `ROS_IP=127.0.0.1` |
| 回灌全 RECOVERING | 坐标系不同 | `EXTRA_ARGS="_map_frame:=world"` |

## 一句话顺序

**镜像 → 网络 → wrapper → 等 MCP → 地图 → pf-sim(route) → 等 tracks → 看相机/实时表 → 热力图 → 录包 → 回灌对比。**
