# 行人跟随 × MuJoCo 仿真：设计说明与测试报告

> 目标：把 `person_following_ws` 的**感知与跟随算法**接进 MuJoCo 仿真，在不开真机的前提下
> 验证观测点管理器的行为，并能录包、回灌、量化场景覆盖度。

---

## 1. 系统模块设计

整体是「仿真进程 + 桥接进程 + 算法进程」三段式，用 ZMQ（数据）和 ROS（算法接口）连接。

```
┌─────────────────────────────┐        ZMQ PUB :5555            ┌──────────────────────────────┐
│  MuJoCo wrapper (GPU/EGL)   │  lidar / pose / person_pose /   │  pf-sim 容器                 │
│  · 官方 mujoco 3.17 Python  │  image / overview               │  · pose_tf     → /odom /tf   │
│  · 轮式底盘 + G1 插件        │ ──────────────────────────────► │  · lidar_bridge → /lidar     │
│  · 3 路相机（nav/overview）  │                                 │  · camera_bridge → /pf_cam   │
│  · 半球雷达（mujoco-lidar）  │        ZMQ PULL :5556           │  · sim_world    → /map/costmap│
│  · MCP 工具（move_g1_to…）   │ ◄────────────────────────────── │  · cmd_vel_bridge → 底盘     │
└─────────────────────────────┘   robot / person / view 命令     │  · gt_pose → tfpose_ros      │
                                                                 │  · monocular_people_tracking │
                                                                 │  · target_bridge → Target    │
                                                                 │  · observation manager       │
                                                                 │  · dashboard(8090)/timeline  │
                                                                 └──────────────────────────────┘
```

### 模块职责

| 模块 | 文件 | 职责 |
| --- | --- | --- |
| 仿真服务 | `wrapper/aidea_mujoco/...` | 物理、渲染、底盘/G1 控制、相机与雷达数据发布 |
| 命令通道 | `command_server.py` | ZMQ PULL → 机器人/行人/视角命令（`kind=robot/person/view`） |
| 位姿桥 | `pose_tf.py` | 底盘位姿 → `/odom` `/tf`；G1 位姿 → `/person_pose(_stamped)` |
| 雷达桥 | `lidar_bridge.py` | ZMQ 点云 → `/lidar/points`（`sim_world` 据此生成局部代价地图） |
| 图像桥 | `camera_bridge.py` | ZMQ 图像 → `/pf_cam/image_raw`、`/pf_cam/overview` |
| 世界模型 | `sim_world.py` | `/map`、局部/全局代价地图、`localization_ready` |
| 真值检测 | `gt_pose.py` | 把 G1 颈/踝投影成 `tfpose_ros/Persons`（代替 YOLO，接口可替换） |
| 跟踪 | `monocular_people_tracking` | UKF + 最近邻关联 + 马氏门限，输出 `TrackArray` |
| 目标桥 | `target_bridge.py` | tracks → `monocular_person_following/Target` |
| 观测点管理器 | `observation_manager*.py` | 8 项加权评分 + 7 状态机，输出 `/cmd_vel` |
| 行人驱动 | `person_walker.py` / `person_route.py` | `still`/`circle`/`route` 三种行走模式 |
| 可视化 | `dashboard.py`、`web_view.py` | 网页面板 + 决策时间轴 |

## 2. 功能设计

### 2.1 观测点管理器（核心）

评分 `Score(p) = w1·visibility + w2·distance + w3·view + w4·robustness − w5·motion − w6·occlusion − w7·collision − w8·churn`，
7 状态：`OBSERVING / MICRO_ADJUST / REPOSITION / SEARCHING / AVOIDING / RECOVERING / IDLE`。

行为取向是**稳定、少动**：

| 场景 | 行为 |
| --- | --- |
| 人在好观测区 | `OBSERVING + HOLD`，`/cmd_vel` 全 0 |
| 人可见但位置不佳 | `MICRO_ADJUST`（小步走位/转向） |
| 人丢失 | `SEARCHING`：**先朝最后已知位置开，到位再原地扫** |
| 扫视 12 s 未找到 | `REPOSITION`：无候选点时继续靠近最后已知位置 |
| 脚下不安全 | `AVOIDING`（STOP） |
| 定位/TF 异常 | `RECOVERING`（STOP） |

### 2.2 行人行走模式

| 模式 | 用途 |
| --- | --- |
| `still` | 调参、跑用例（位置可控） |
| `circle` | 快速看跟随/搜索闭环 |
| `route` | 覆盖度评估：按房间地图走带 0.55 m 安全间距的往返扫描路线 |

`route` 路线来自对 MuJoCo 场景的 `mj_ray` 扫描（200×200@0.1 m，家具 517 格），
航线间距 1.2 m，航点间做视线连通。

## 3. 可视化设计

| 视图 | 位置 | 内容 |
| --- | --- | --- |
| 总览 | `:8090` 左侧大画面 | `pf_overview` 自由相机，1280×720，**拖拽旋转/滚轮缩放** |
| 机器人相机 | 右上 | 前视相机 + GT 颈/踝点 + tracks 数 |
| 雷达 | 右下 | 3D 点云（按高度着色），可拖拽 |
| 状态卡 | 右中 | 状态、轨迹数、`/cmd_vel`、G1 位置 |
| 时间轴 | `:8781`（面板内嵌） | 决策回放：机器人路径、FOV、候选点、状态序列 |
| 验证卡 | 右侧 | 会话列表 → 摘要文本 + 覆盖热力图 + 一键回灌 |
| 覆盖热力图 | `tmp/coverage.png` | 黑=家具，红→绿=可见比例，蓝点=小车 |

设计原则：**算法需要看的（候选点、FOV、评分）用时间轴；系统健康度（频率、状态、指令）用面板；
空间覆盖度用热力图**。三者都不依赖 RViz。

## 4. 测试设计思路

四层，从便宜到贵，下层失败就不必跑上层：

| 层 | 用例 | 判据 | 成本 |
| --- | --- | --- | --- |
| L1 单元 | `tests/test_observation_manager.py`（54 项） | 评分、门限、7 状态迁移、预算、门洞、不可达 | 0.5 s，无 ROS |
| L2 在线冒烟 | `sim_smoke_test.py`（10 项） | 每层链路有数据、频率达标、代价地图非空、API 可用 | ~40 s |
| L3 闭环行为 | 人可见 → 藏走 → 观察位移/状态 | 丢失后应「靠近 + 扫视」，不能停死 | ~1 min |
| L4 回灌回归 | `record_pf_session.sh` + `replay_pf_session.sh` | 回灌结果与实时逐项一致 | ~1 min |
| 附加 | 覆盖度热力图 | 可见比例、盲区格数 | 2 min 路线 |

**为什么这样分层**：管理器是纯 Python 决策器，绝大多数 bug（评分权重、门限、状态机、预算）
在 L1 就能抓住；L2 保证接进仿真后「数据真的在流动」；L3 验证行为而非数值；L4 提供
可重复的回归基线，改参数后跑一次回灌即可对比。

## 5. 测试结论

### 5.1 结果汇总

| 层 | 结果 |
| --- | --- |
| L1 单元 | **54 / 54 通过**（0.53 s） |
| L2 在线冒烟 | **9 通过 / 0 失败 / 1 跳过**（跳过项为 route 用例，测试时路线被停用） |
| L3 闭环行为 | 通过（见 5.2） |
| L4 回灌 | 通过（见 5.3） |
| 覆盖度 | 33% 可见，23/40 格为盲区（见 5.4） |

### 5.2 在线链路实测

| 指标 | 数值 |
| --- | --- |
| 相机 `/pf_cam/image_raw` | 11.8 Hz（开面板+时间轴时；无头约 30 Hz） |
| 总览 `/pf_cam/overview` | 11.7 Hz（1280×720） |
| 雷达 `/lidar/points` | 9.2 Hz，86400 点/帧 |
| 真值检测 `/pose_estimator/pose` | 15.0 Hz，**3 个关键点**（颈+双踝） |
| 跟踪 `/monocular_people_tracking/tracks` | 15.0 Hz |
| 局部代价地图 | 317 格占据（修复前 0 格） |
| `/cmd_vel` | 18–30 Hz |

**闭环行为**（人在 2.5 m 处被发现后藏走）：

```
t= 3s moved=0.40 m  SEARCHING
t= 6s moved=0.84 m  SEARCHING
t= 9s moved=1.31 m  SEARCHING
t=12s moved=1.47 m  REPOSITION   ← 到达 1.2 m 停止距离
t=27s moved=1.47 m  SEARCHING    ← 原地扫视
```

修复前同一场景只有 **0.02 m** 的原地转动。

### 5.3 回灌一致性

8 秒录像回放：

| 指标 | 实时 | 回灌 |
| --- | --- | --- |
| 决策数 | 16 | 16 |
| 时长 | 8.95 s | 8.95 s |
| 可见比例 | 0.81 | 0.81 |
| OBSERVING 占比 | 89% | 89% |

逐项一致，说明回灌可作为参数回归的基线。

### 5.4 覆盖度

G1 按 33 航点 / 60.6 m 的路线走约 2 分钟，与家具最小距离 **0.77 m**（设计安全间距 0.55 m）。
热力图统计：40 个采样格中 **23 格为盲区**、仅 16 格稳定可见，整体可见比例 **33%**。

原因明确：小车停在原点，路线把人带到 5 m 外和家具背后，相机 FOV（60°）跟不上。
这也给出了下一步方向——让路线围绕小车当前位置组织，或让管理器更主动地跟随。

### 5.5 过程中发现并修复的缺陷

| 缺陷 | 影响 | 修复 |
| --- | --- | --- |
| 雷达未进 ROS | 代价地图全空，避障失效，车开进家具卡死 | 新增 `lidar_bridge.py` |
| 机身点被当障碍 | 一直 `AVOIDING`，无法起步 | 过滤机器人 0.45 m 内的点 |
| 丢失目标只扫视 | 车不换位置（0.02 m） | SEARCH/REPOSITION 先靠近最后已知位置 |
| 扫描用 slew 限速 | 0.45 rad/s 实际只有 0.02 | 扫视/接近绕过斜率限制 |
| 相机未启用 | wrapper 没有图像输出 | `cameras_to_use` 加入 nav + overview |
| 跟踪节点缺 `camera_info` remap | 永远等不到相机信息 | entrypoint 补 remap |
| tf1 看不到 `/tf_static` | 跟踪节点启动即崩 | 固定坐标系改走 `/tf` |
| `cppoptlib` 版本 | 跟踪节点编译失败 | 锁定 v1 提交 `2a0f98e` |

### 5.6 已知限制

- 相机在开满可视化时掉到 ~12 Hz（viewer/面板/总览抢渲染线程）；无头模式可回到 30 Hz。
- G1 是运动学模型，不参与物理碰撞；「不碰撞」靠路线安全间距保证，不是动力学结果。
- GT 位姿没有遮挡与漏检；换 YOLO 后才能验证真正的「丢失→搜索」。
- 四轮底盘原地转向打滑，实际转速约为指令的 25%。

---

## 6. 复现

```bash
cd /home/amigo/project/RND/mujoco_simulation
scripts/start_pf_sim.sh        # 起仿真 + 桥接 + 面板
scripts/run_sim_tests.sh       # L1 + L2
```

详见 [PF_SIM_RUNBOOK.md](PF_SIM_RUNBOOK.md)。
