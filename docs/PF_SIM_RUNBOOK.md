# 仿真验证流程（启动顺序）

从零到「能看到闭环 + 能录包回灌」的完整顺序。每一步都有明确的**就绪判据**，不满足就别往下走。

## 0. 构建镜像（代码改动后只需重做这步）

```bash
cd /home/amigo/project/RND/mujoco_simulation
docker build -f wrapper/Dockerfile -t mujoco_simulation-mujoco-wrapper:latest wrapper
docker build -f ros-bridge/Dockerfile.pf-sim -t mujoco_simulation-pf-sim:latest ros-bridge
```

## 1. 建网络（只需一次）

```bash
docker network create sim-net 2>/dev/null || true
```

## 2. 起 MuJoCo（带窗口）

```bash
xhost +local:root
docker rm -f mujoco-wrapper 2>/dev/null
docker run -d --name mujoco-wrapper --network sim-net --device nvidia.com/gpu=all \
  -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl -e MUJOCO_VIEWER=1 \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw \
  -e ROBOT_MJCF_INCLUDES=/app/aidea_mujoco/mjcf/robots/g1/g1_12dof.xml \
  -e ROBOT_PLUGIN_CONFIGS=/app/aidea_mujoco/aidea_mujoco/plugins/g1/config.yaml \
  mujoco_simulation-mujoco-wrapper:latest
```

**判据**

```bash
scripts/wait_ready.sh --wrapper          # MCP 能应答
docker logs mujoco-wrapper | grep "simulation ready"
docker exec mujoco-wrapper bash /app/mcp_call.sh get_g1_state '{}'
```

无桌面时把 `-e MUJOCO_VIEWER=1` 换成 `=0`（无头，相机仍照常发布）。

## 3. 起桥接 + 跟踪 + 管理器

```bash
mkdir -p tmp/sim_logs tmp/sim_sessions
docker rm -f pf-sim 2>/dev/null
docker run -d --name pf-sim --network sim-net \
  -e ZMQ_LIDAR_ENDPOINT=tcp://mujoco-wrapper:5555 \
  -e ZMQ_CMD_ENDPOINT=tcp://mujoco-wrapper:5556 \
  -e PF_MANAGER=1 \
  -e PF_TRACKER=real \
  -e PF_PERSON_MODE=still \
  -e PF_CAMERA_VIEW=1 \
  -e PF_LOG_DIR=/tmp/person_follow_observe \
  -p 8090:8090 \
  -v /home/amigo/project/RND/person_following_ws:/opt/person_following_ws:ro \
  -v $PWD/tmp/sim_logs:/tmp/person_follow_observe \
  mujoco_simulation-pf-sim:latest
```

**判据**（首次会编译跟踪节点，约 20 s）

```bash
scripts/wait_ready.sh --bridge
docker exec pf-sim bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; \
  rostopic hz /pf_cam/image_raw; rostopic hz /pose_estimator/pose'
```

## 4. 摆人 / 让人动

```bash
# 站着（看观察点是否稳定）
docker exec mujoco-wrapper bash /app/mcp_call.sh move_g1_to '{"x":1.9,"y":-0.64,"yaw":0}'

# 绕圈走（看管理器是否跟随/搜索）
docker rm -f pf-sim && docker run ... -e PF_PERSON_MODE=circle PF_PERSON_VX=0.4 PF_PERSON_WZ=0.25 ...

# 覆盖路线（前面要先做第 6 步的地图）
docker run ... -e PF_PERSON_MODE=route -e PF_MAP_FILE=/tmp/pf_map.json \
  -v $PWD/tmp/pf_map.json:/tmp/pf_map.json:ro ...
```

## 5. 观察与取证

```bash
# MuJoCo 窗口（车 + G1）
# 相机画面（含 GT 关键点）
http://127.0.0.1:8090/

# 实时表
docker exec pf-sim bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; \
  python3 /app/pf_sim/dev_watch_loop.py 60 2'

# 状态×动作计数、/cmd_vel 分布
docker exec pf-sim bash -lc '... python3 /app/pf_sim/dev_probe_decision.py 6'

# 网页时间轴
python3 /home/amigo/project/RND/person_following_ws/tools/ros_smoke/web_view.py \
    --dir tmp/sim_logs --port 8781

# 文本摘要
python3 scripts/summarize_pf_session.py tmp/sim_logs
```

## 6. 地图（只在用路线模式时需要）

```bash
docker exec mujoco-wrapper mkdir -p /app/tools
docker cp tools/make_map_from_mjcf.py mujoco-wrapper:/app/tools/
docker exec mujoco-wrapper bash -lc 'cd /app && uv run python /app/tools/make_map_from_mjcf.py --out /tmp/pf_map.json'
docker cp mujoco-wrapper:/tmp/pf_map.json tmp/pf_map.json
```

## 6.5 覆盖度热力图

G1 走路线时，统计「哪些位置相机看得到、哪些位置看不到」，输出成一张图。

```bash
# 起 pf-sim 时打开真值轨迹记录（热力图要靠它还原丢失时的位置）
-e PF_PERSON_MODE=route -e PF_PERSON_TRACE=1

# 走一两分钟后生成
python3 scripts/coverage_heatmap.py tmp/sim_logs \
    --map tmp/pf_map.json --trace tmp/sim_logs/person_trace.jsonl --out tmp/coverage.png
```

图例：**黑色 = 家具**，**深灰 = 没走到**，**红→绿 = 可见比例 0→100%**，
**蓝点 = 小车位置**，底部是色阶条。

判读要点：红块是视场死角/被家具挡住；绿块是能稳定 `OBSERVING` 的位置。
实测一次路线（约 2 分钟）：可见比例 33%，40 个采样格里 23 格是盲区——
因为小车停在原点，而路线把人带到了 5 m 以外和家具背后。

> 丢失时管理器没有可信的目标位置（`target` 会过期或外推），所以热力图必须用
> `--trace` 的真值轨迹按时间戳配对，否则丢样会被过滤掉、结果虚高。

## 7. 录包与回灌

```bash
DURATION=60 TAG=doorway scripts/record_pf_session.sh      # 录
scripts/replay_pf_session.sh tmp/sim_sessions/doorway      # 回灌（open loop）
python3 scripts/summarize_pf_session.py \
    tmp/sim_sessions/doorway tmp/sim_sessions/doorway/replay/replay-*   # 对比
```

## 每级的验收标准

| 级别 | 判据 | 不满足时看什么 |
| --- | --- | --- |
| 仿真起来 | `wait_ready.sh --wrapper` 通过 | 窗口/X11、GPU、日志里的 Traceback |
| 有图像 | `/pf_cam/image_raw` 30 Hz（开窗口约 15 Hz） | `camera_bridge.py` 的 ZMQ 地址、`MUJOCO_VIEWER` |
| 有 TF | `/tf` 里 `base_footprint`、`pf_cam_optical` | `pose_tf.py` 是否收到 `pose` |
| 有检测 | `/pose_estimator/pose` 15 Hz，每帧 3 个部位 | 人是否在视场、脚踝是否被裁掉 |
| 有轨迹 | `/monocular_people_tracking/tracks` 非空 | 跟踪节点是否编译成功、camera_info 是否 remap |
| 有决策 | `/observe_manager/state` 变化 | 管理器是否订阅到 tracks/target |
| 底盘响应 | `/cmd_vel` 非 0，odom 变化 | 轮子摩擦、`publish_guarded` 是否拦截 |

## 坑位对照表（都踩过）

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `ERROR: no SID` | MCP 还没就绪 | `scripts/wait_ready.sh --wrapper` |
| `grep "simulation ready"` 卡死 | print 块缓冲 | 已加 `PYTHONUNBUFFERED=1` |
| 没有窗口 | `run()` 写死 headless | `MUJOCO_VIEWER=1` |
| 窗口连不上 X | 容器缺 `DISPLAY`/X11 挂载 | 见第 2 步 |
| G1 不动 | `PF_PERSON_MODE=still` | 换 `circle` / `route` |
| 小车不动 | 观测点已够好（HOLD），或人不在视场 | 看状态与 `/cmd_vel`；用 `circle` 触发 |
| 有图像没 tracks | 跟踪节点缺 `camera_info` remap | entrypoint 已修 |
| 有 pose 没 tracks | 脚踝被画面裁掉（建轨需要脚踝） | 相机加俯角或拉远 |
| 回灌 `Unable to communicate with master` | `--network none` 下 ROS 无可用地址 | `ROS_IP=127.0.0.1` |
| 回灌决策全是 RECOVERING | 地图坐标系不同 | `EXTRA_ARGS="_map_frame:=world"` |
| 三个 static TF 互相杀 | 同名节点 | 已移除，改由 `pose_tf` 广播 |
| 跟踪节点编译失败 | `cppoptlib` 版本 | 已 pin 到 `2a0f98e` |

## 一句话顺序

**镜像 → 网络 → wrapper → 等 MCP → pf-sim → 等 tracks → 摆人/开路线 → 看窗口+相机+实时表 → 录包 → 回灌对比。**
