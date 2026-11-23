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

## 闭环运动的已知问题（未完成）

管理器能正确决策并调用发布，但**底盘没动**，本轮没跑通闭环：

- 管理器日志：`search dir=1.0 angular=0.45 published=True`（`publish_guarded` 返回成功）
- 同时 `/cmd_vel` 采样 113 帧全部 `angular.z = 0`
- 对照实验：停掉管理器（`PF_MANAGER=0`）后，用 MCP `set_base_velocity(0.5, 0.5)`，
  底盘 3 秒走了约 1 m —— **仿真底盘和 ZMQ 命令通路都是好的**
- 模型对照：直接给四个轮子 `ctrl=20` 跑 3000 步，位移 3.72 m —— 模型动力学也正常

所以问题在管理器→`/cmd_vel` 这一段：要么走了 `publish_guarded` 的 fallback 分支
（发布 0 但返回 True），要么 `publish_raw` 之后被别的路径覆盖。下一步该在
`publish_guarded` 里区分「candidate 被拒」和「fallback 成功」，以及确认
`/cmd_vel` 上没有第二个发布者。

## 限制

- **未实跑验证**：本机没有 GPU/GUI，只做了语法与 compose 校验。首次务必
  `PF_MANAGER=0 PF_TRACKER=fake` 起，确认 `/pf_cam/image_raw`、`/tf`、`/lidar/points` 正常。
- **GT 位姿没有遮挡与漏检**：人走到墙后仍然会被投影出来（只要在视锥内）。真实检测器
  才会有漏检，这也正是"丢失/搜索"逻辑的关键输入；换成 YOLO 后才能验证这部分。
- 全局地图仍是 20×20 空白房间，家具只进局部代价地图。
- G1 是运动学锁定 + 假步态，`mode=walk` 才响应速度；它不会被人偶/家具挡住。
- `monocular_people_tracking` 首次启动要编译，镜像里已装 OpenCV/Eigen/cv_bridge 等依赖。
