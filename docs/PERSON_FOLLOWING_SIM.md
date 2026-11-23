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

## 限制

- **未实跑验证**：本机没有 GPU/GUI，只做了语法与 compose 校验。首次务必
  `PF_MANAGER=0 PF_TRACKER=fake` 起，确认 `/pf_cam/image_raw`、`/tf`、`/lidar/points` 正常。
- **GT 位姿没有遮挡与漏检**：人走到墙后仍然会被投影出来（只要在视锥内）。真实检测器
  才会有漏检，这也正是"丢失/搜索"逻辑的关键输入；换成 YOLO 后才能验证这部分。
- 全局地图仍是 20×20 空白房间，家具只进局部代价地图。
- G1 是运动学锁定 + 假步态，`mode=walk` 才响应速度；它不会被人偶/家具挡住。
- `monocular_people_tracking` 首次启动要编译，镜像里已装 OpenCV/Eigen/cv_bridge 等依赖。
