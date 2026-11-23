# 行人跟随 × MuJoCo 仿真集成

把 `person_following_ws` 的**感知与跟随算法**接到 MuJoCo 仿真上。

## 角色分工

| 仿真实体 | 角色 | 依据 |
| --- | --- | --- |
| 轮式小车（`base_link`） | **跟随机器人** | `fisheye` 相机和 LiDAR 都挂在它身上（`airy` 机体） |
| G1（`g1_pelvis`） | **被跟随的人** | 人形、由 G1 插件按 `vx/wz` 行走 |

> 上一版把 G1 当成了被控机器人，是反的；已按实际挂载改正。

## 数据流

```text
MuJoCo wrapper ──ZMQ PUB :5555 (lidar / pose / person_pose / image)──► pf-sim
       ▲                                                                   │
       └────────── ZMQ PULL :5556 (kind=robot 底盘 / kind=person G1 行走) ◄─┘

pf-sim 内部：
  image ─► camera_bridge ─► /pf_cam/image_raw + /pf_cam/camera_info + TF world→pf_cam_optical
  person_pose ─► gt_pose ─► /pose_estimator/pose (tfpose_ros/Persons，真实像素投影)
                ─► monocular_people_tracking ─► /monocular_people_tracking/tracks
                ─► target_bridge ─► /monocular_person_following/target
                ─► observation_manager ─► /cmd_vel ─► ZMQ ─► 底盘
```

## 追踪算法怎么进来的

- `gt_pose.py` 是**真值位姿适配器**：用仿真相机内参 + `world→pf_cam_optical` 把 G1 的
  颈/踝投影成 `tfpose_ros/Persons`，代替 YOLO。这样**真正的跟踪算法**
  （`monocular_people_tracking`：UKF + 最近邻关联 + 马氏距离门限）跑的是真实像素观测，
  而不是直接喂 3D 坐标。
- `PF_TRACKER=real` 时，entrypoint 会从挂载的 `/opt/person_following_ws` 里
  **首次启动时编译** `monocular_people_tracking`（catkin_make，约 1 分钟），然后运行它。
- `target_bridge.py` 把跟踪结果转换成管理器的 `Target`：选距真值最近的 track id
  （相当于 ReID 阶段的工作，先用位置代替外观）。
- 想换成真 YOLO：只替换 `gt_pose.py` 即可——它和其他节点唯一的接口就是
  `/pose_estimator/pose`。相机图像已经按 `sensor_msgs/Image` + `CameraInfo` 发出。

## 启动顺序与命令

顺序有依赖：wrapper 要先出 `simulation ready`，MCP 才能调；pf-sim 要先编译完跟踪节点，
才有 `/monocular_people_tracking/tracks`；最后才摆人。

```bash
cd /home/amigo/project/RND/mujoco_simulation

# 1) 仿真（带窗口）
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

# 2) 等 MCP 就绪（不等就调 mcp_call.sh 会报 "no SID"）
docker logs -f mujoco-wrapper | grep -m1 "simulation ready"

# 3) 桥接 + 跟踪 + 管理器（日志写进 host，方便可视化）
mkdir -p tmp/sim_logs tmp/sim_sessions
docker rm -f pf-sim 2>/dev/null
docker run -d --name pf-sim --network sim-net \
  -e ZMQ_LIDAR_ENDPOINT=tcp://mujoco-wrapper:5555 \
  -e ZMQ_CMD_ENDPOINT=tcp://mujoco-wrapper:5556 \
  -e PF_MANAGER=1 -e PF_TRACKER=real -e PF_PERSON_MODE=still \
  -e PF_LOG_DIR=/tmp/person_follow_observe \
  -v /home/amigo/project/RND/person_following_ws:/opt/person_following_ws:ro \
  -v $PWD/tmp/sim_logs:/tmp/person_follow_observe \
  mujoco_simulation-pf-sim:latest

# 4) 等跟踪节点起来
docker logs -f pf-sim | grep -m1 "monocular_people_tracking started"

# 5) 按车头朝向把人放到前方 2 m
docker exec mujoco-wrapper bash /app/mcp_call.sh move_g1_to '{"x":1.9,"y":-0.64,"yaw":0}'
```

## 测试指标与可视化

运行数据有三层，从粗到细：

| 方式 | 命令 | 看什么 |
| --- | --- | --- |
| MuJoCo 窗口 | `MUJOCO_VIEWER=1` | 车、G1、实时光照/几何是否正常 |
| 实时表 | `python3 /app/pf_sim/dev_watch_loop.py 60 2` | t / x / y / yaw / 状态 / cmd / 轨迹数 |
| 决策分布 | `python3 /app/pf_sim/dev_probe_decision.py 6` | 状态×动作计数、`/cmd_vel` 取值分布 |
| 网页时间轴 | `python3 tools/ros_smoke/web_view.py --dir tmp/sim_logs --port 8781` | 机器人路径、FOV、候选点、状态时间线 |
| 文本摘要 | `python3 scripts/summarize_pf_session.py tmp/sim_logs` | 状态占比、动作、拦截原因、评分项均值 |
| 两次运行对比 | `python3 scripts/summarize_pf_session.py run_a run_b` | 参数改动的回归差异 |

网页时间轴用的是 `person_following_ws` 里现成的 `tools/ros_smoke/web_view.py`（纯标准库），
管理器每 0.5 s 写的 JSONL 直接在浏览器里回放，不需要 RViz。

## 录制与回灌

```bash
# 录一段（默认不含图像；图像 800x600 每秒约 20 MB）
DURATION=60 TAG=doorway scripts/record_pf_session.sh
RECORD_IMAGE=1 DURATION=30 scripts/record_pf_session.sh   # 需要图像时

# 回灌：bag 里的 TF/tracks/target/map/costmap 喂给管理器，仿真不参与
scripts/replay_pf_session.sh tmp/sim_sessions/doorway

# 对比实时运行与回灌结果
python3 scripts/summarize_pf_session.py \
    tmp/sim_sessions/doorway \
    tmp/sim_sessions/doorway/replay/replay-20261123-084316
```

回灌容器用 `--network none` + `ROS_IP=127.0.0.1`（否则 rosmaster 起不来），
`/use_sim_time=true`，`_map_frame:=world`——仿真的地图坐标系和真机的 `map` 不同。

实测：8 s 的录像回放后是 16 条决策、8.95 s、visible 0.81、OBSERVING 89%，与实时运行逐项一致。

## 打开仿真窗口

`MujocoServer.run()` 只走 headless 分支（`show_viewer_ui` 参数被忽略），
带窗口的实现是 `MujocoServerPassive`（内部用 `mujoco.viewer.launch_passive`）。
所以默认情况下**不会弹窗**，需要 `MUJOCO_VIEWER=1`：

```bash
MUJOCO_VIEWER=1 scripts/run_pf_sim.sh
```

容器里必须有 X11：

```bash
xhost +local:root
docker run -d --name mujoco-wrapper --network sim-net --device nvidia.com/gpu=all \
  -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl \
  -e MUJOCO_VIEWER=1 \
  -e DISPLAY=$DISPLAY -e QT_X11_NO_MITSHM=1 \
  -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
  -v ${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw \
  mujoco_simulation-mujoco-wrapper:latest
```

验证窗口真的开了：

```bash
DISPLAY=:1 xwininfo -root -tree | grep -i mujoco
# 0x... "MuJoCo : aidea scene": ("MuJoCo" "MuJoCo")  2504x1376+0+37
```

`MUJOCO_VIEWER=0` 回到无头模式（远程/无桌面时用）。
窗口模式下相机仍然照常发布 `image`/`pose`，桥接不受影响。

## 运行

```bash
# 默认：真实追踪 + 管理器
scripts/run_pf_sim.sh

# 只跑仿真与桥接（自己起管理器）
PF_MANAGER=0 scripts/run_pf_sim.sh

# 追踪器还没编译好时，用真值轨迹先验证链路
PF_TRACKER=fake docker compose -f docker-compose.yml -f docker-compose.pf-sim.yml \
    --profile pf up -d --build mujoco-wrapper pf-sim
```

## 话题

| 话题 | 类型 | 来源 |
| --- | --- | --- |
| `/pf_cam/image_raw` / `/pf_cam/camera_info` | `sensor_msgs` | `camera_bridge.py` |
| `/pose_estimator/pose` | `tfpose_ros/Persons` | `gt_pose.py`（可换成 YOLO） |
| `/monocular_people_tracking/tracks` | `TrackArray` | 真实跟踪节点 |
| `/monocular_person_following/target` | `Target` | `target_bridge.py` |
| `/odom`、`/tf` | — | `pose_tf.py`（`world→base_footprint`） |
| `/person_pose` | `geometry_msgs/PointStamped` | `pose_tf.py`（G1 真值） |
| `/map`、两个 costmap、`/map_follow/localization_ready` | — | `sim_world.py` |
| `/cmd_vel` | `geometry_msgs/Twist` | 管理器 → `cmd_vel_bridge.py` → 底盘 |

## 验证记录

### 真实 MuJoCo 运行（GPU / EGL）

在带 RTX 4060 的容器里跑通了真实 wrapper + 桥接 + 跟踪 + 管理器：

| 环节 | 结果 |
| --- | --- |
| ZMQ（wrapper → pf-sim） | `image` 29.8 Hz、`pose` 29.0 Hz、`person_pose` 28.8 Hz、`lidar` 9.5 Hz |
| `/pf_cam/image_raw` | 30.0 Hz（800×600） |
| `/pose_estimator/pose` | 每帧 3 个部位（颈 + 双踝），颈归一化坐标 (0.50, 0.48) |
| `/monocular_people_tracking/tracks` | `id=0 pos=(2.67, 0.00)`，真值 (2.5, 0) |
| 管理器 | `/observe_manager/state` = `OBSERVING`，决策 `HOLD / good_enough`，`/cmd_vel` 20 Hz |

复现要点（容器名要对上）：

```bash
docker network create sim-net
docker run -d --name mujoco-wrapper --network sim-net --device nvidia.com/gpu=all \
  -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl \
  -e ROBOT_MJCF_INCLUDES=/app/aidea_mujoco/mjcf/robots/g1/g1_12dof.xml \
  -e ROBOT_PLUGIN_CONFIGS=/app/aidea_mujoco/aidea_mujoco/plugins/g1/config.yaml \
  mujoco_simulation-mujoco-wrapper:latest

docker run -d --name pf-sim --network sim-net \
  -e ZMQ_LIDAR_ENDPOINT=tcp://mujoco-wrapper:5555 \
  -e ZMQ_CMD_ENDPOINT=tcp://mujoco-wrapper:5556 \
  -e PF_MANAGER=1 -e PF_TRACKER=real -e PF_PERSON_MODE=still \
  -v /home/amigo/project/RND/person_following_ws:/opt/person_following_ws:ro \
  mujoco_simulation-pf-sim:latest

# 把人放到相机正前方
docker exec mujoco-wrapper bash /app/mcp_call.sh move_g1_to '{"x":2.5,"y":0.0,"yaw":3.14159}'
```

检查时要 **source 两个工作区**，否则 `rostopic` 加载不了 `tfpose_ros` 消息（这一步踩过）：

```bash
docker exec pf-sim bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; rostopic hz /monocular_people_tracking/tracks'
```

### 无 GPU 时的合成数据验证

`dev_check_bridge.sh` / `dev_check_tracks.sh` 用合成 ZMQ 数据，不需要 MuJoCo：

| 检查 | 结果 |
| --- | --- |
| `dev_check_bridge.sh`：图像、CameraInfo、`/tf`、`/odom`、`/person_pose`、地图与代价地图、GT 位姿 | **17/17 通过** |
| `dev_check_tracks.sh`：编译并运行 `monocular_people_tracking` | **通过**，输出 `track id=0 pos=(2.17, -0.62)`（真值 2.0, ±0.6） |

复现：

```bash
# 桥接半段（不需要编译 C++）
docker run --rm --entrypoint bash \
  -v $PWD/ros-bridge/pf_sim:/app/pf_sim:ro \
  -v $PWD/ros-bridge/proto:/app/proto:ro \
  pf-sim-tracker:latest -lc 'bash /app/pf_sim/dev_check_bridge.sh'

# 追踪半段（首次会编译跟踪节点）
docker run --rm --entrypoint bash \
  -v $PWD/ros-bridge/pf_sim:/app/pf_sim:ro \
  -v /home/amigo/project/RND/person_following_ws:/opt/person_following_ws:ro \
  pf-sim-tracker:built -lc 'bash /app/pf_sim/dev_check_tracks.sh'
```

### 过程中踩到的坑（已修）

1. **`cppoptlib` 版本**：`3rdparty/CppNumericalSolvers` 在本地是空目录，上游 master 已删掉
   `cppoptlib/problem.h`。构建脚本现在克隆仓库并 checkout 到 `2a0f98e`（v1 API 的最后一个提交）。
2. **tf1 看不到 `/tf_static`**：跟踪节点用 `tf::TransformListener`，读不到 tf2 的静态发布器。
   `pose_tf.py` 现在每帧把 `world→odom`、`base_footprint→base_link`、`base_link→g_base`
   重新广播到 `/tf`。
3. **跟踪器只接受带脚踝的观测来新建轨迹**（`people_tracker.cpp`）：合成数据里脚踝若被画面
   裁掉，就永远没有 tracks。真实场景同理——**相机必须能拍到人的脚**，否则不会有新轨迹。
   真机的 `fisheye` 相机是水平 1.2 m，2 m 外人脚接近画面下边缘，建议在 mjcf 里给它加
   ~10° 俯角。
4. **真实 wrapper 之前根本没开相机**：`mcp_server._start_sim` 传的 `cameras_to_use=[]`，
   现在改为 `[AideaCameras.cam_nav_rgb]`。

## 闭环运动：定位到的根因与修复

**现象**：管理器日志 `search dir=1.0 angular=0.45 published=True`，但 `/cmd_vel` 采样
113 帧全是 0，底盘不动。

**定位过程**（每一步都做了对照实验）：

1. 停掉管理器，用 MCP `set_base_velocity(0.5, 0.5)` → 底盘 3 秒走 1 m；
2. 直接给四个轮子 `ctrl=20` → 3000 步位移 3.72 m；
3. `rostopic pub /cmd_vel` 1.0 rad/s → 3 秒转 0.606 rad（桥接→ZMQ→底盘全通）；
4. 打开 `publish_guarded` 的分支日志，看到 `[GUARD] accept w=0.10/0.20` —— 实际发出去的是
   **slew 限速后的爬升值**，不是 0.45。

**根因**：SEARCH 每 tick 都走斜率限制，而目标在视场边缘时管理器的状态会在
`SEARCHING` 和 `OBSERVING` 之间抖动；每次 `OBSERVING/HOLD` 都会调用 `publish_stop()`，
把 `current_w` 清零，于是 0.45 的爬升永远从头开始，平均只有 ~0.02 rad/s，
`/cmd_vel` 大部分帧是 0。

**修复**：扫视（SEARCH）是定速巡视，不需要轨迹平滑，改成 `publish_guarded(..., slew=False)`
直接给满速。修复后 `/cmd_vel` 中 0.45 占 ~54%，其余是目标短暂回到视场时的 HOLD。

**修复后验证**：人走出视场 → 底盘转向搜索 → 人回到视场 → 状态自动回到 `OBSERVING`
（GT 位姿 15 Hz、tracks 15 Hz）。

> 注意：仿真里四轮打滑，指令 0.45 rad/s 实际只有 ~0.11–0.2 rad/s 的转动（同样的轮速在
> 隔离模型里 1 秒只转 0.111 rad）。这是模型摩擦特性，不是控制器问题。

涉及 `person_following_ws` 的改动（该目录不是 git 仓库）：

- `observation_manager_supervisor.py`：SEARCH 改用 `slew=False`；增加 2 s 节流的
  `search dir/angular/acc/published` 诊断日志。
- `map_person_follow_supervisor_legacy.py`：`publish_guarded` 的分支判定、
  `trajectory_is_safe` 的拒绝原因、`publish_raw` 的实际下发值，全部挂在
  `~observe_debug_verbose` 开关后面（默认关闭）。

## 限制

- **未实跑验证**：本机没有 GPU/GUI，只做了语法与 compose 校验。首次务必
  `PF_MANAGER=0 PF_TRACKER=fake` 起，确认 `/pf_cam/image_raw`、`/tf`、`/lidar/points` 正常。
- **GT 位姿没有遮挡与漏检**：人走到墙后仍然会被投影出来（只要在视锥内）。真实检测器
  才会有漏检，这也正是"丢失/搜索"逻辑的关键输入；换成 YOLO 后才能验证这部分。
- 全局地图仍是 20×20 空白房间，家具只进局部代价地图。
- G1 是运动学锁定 + 假步态，`mode=walk` 才响应速度；它不会被人偶/家具挡住。
- `monocular_people_tracking` 首次启动要编译，镜像里已装 OpenCV/Eigen/cv_bridge 等依赖。
