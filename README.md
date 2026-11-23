# mujoco_simulation


```
pip install pre-commit

chmod +x scripts/sync-proto.sh

pre-commit install
# echo pre-commit installed at .git/hooks/pre-commit
```
Note:如果脚本修改了文件（比如复制产生了新文件），这些修改是在你 git add 之后发生的，不会被自动包含在这次提交里。你需要再次 git add ros-bridge/proto/，然后重新 git commit

## 通信
```text
┌─────────────────────────────────────────────────────────────┐
│                    Docker 网络 (sim-net)                     │
│                                                             │
│  ┌─────────────────────┐          ┌─────────────────────┐  │
│  │  mujoco-wrapper     │          │  ros1-bridge        │  │
│  │                     │  ZMQ     │                     │  │
│  │  MuJoCo 物理循环     │  PUB     │  ZMQ SUB            │  │
│  │  LiDAR 扫描         ├─────────►│  Protobuf 反序列化   │  │
│  │  MCP Server (HTTP)  │  (proto) │  构造 sensor_msgs    │  │
│  │  ZMQ PUB            │          │  ROS1/ROS2 发布      │  │
│  │                     │          │                     │  │
│  └─────────┬───────────┘          └─────────────────────┘  │
│            │                                                │
│            │ MCP (HTTP)                                     │
│            ▼                                                │
│  ┌─────────────────────┐                                   │
│  │  agent              │                                   │
│  │                     │                                   │
│  │  MCP Client         │                                   │
│  │  (控指令，不碰 ZMQ)   │                                   │
│  │                     │                                   │
│  └─────────────────────┘                                   │
└─────────────────────────────────────────────────────────────┘
```

| 服务 | 端口 | 说明 |
|---|---|---|
| llama.cpp（LLM） | 8080 | 你本地已有，不动 |
| MCP Server（仿真控制） | 8081 | 改 `mcp_server.py` 的 port 和 Dockerfile 的映射 |
| ZMQ PUB（LiDAR） | 5555 | 不变 |
| Agent 容器 | 无对外端口 | 主动连 8080/8081/5555 |
## 结构

your-repo/
├── .git/
├── .gitignore
├── README.md
├── docker-compose.yml
│
├── wrapper/                      # MuJoCo Wrapper 服务
│   ├── .dockerignore
│   ├── Dockerfile
│   ├── pyproject.toml
│   ├── uv.lock
│   ├── proto/                    # proto 唯一定义源
│   │   └── lidar.proto
│   ├── mcp_server.py
│   ├── zmq_publisher.py
│   ├── test_stage1.py
│   └── aidea_mujoco/
│
├── ros-bridge/                   # ROS1/ROS2 bridge 服务
│   ├── .dockerignore
│   ├── Dockerfile
│   ├── proto/                    # 从 wrapper/proto 复制
│   │   └── lidar.proto
│   ├── zmq_sub_node.py
│   └── entrypoint.sh
│
└── agent/                        # Agent 服务
    ├── .dockerignore
    ├── Dockerfile
    ├── requirements.txt
    └── agent_main.py

ros-bridge/proto/lidar.proto 与 wrapper/proto/lidar.proto 是同一份文件。
同步方法：在仓库根放一个 scripts/sync-proto.sh，在 ros-bridge 构建前执行复制：

## git 工作流

### STAGE 1： Wrapper

#### git

```
git checkout -b stage1-wrapper-base
# wrapper/ 下：pyproject.toml, uv.lock, aidea_mujoco/, test_stage1.py
# wrapper/Dockerfile 只含基础环境
# docker-compose.yml 只含 mujoco-wrapper
git add . && git commit -m "stage1: wrapper base env"
git tag stage1
```

#### docker build
```
cd /home/amigo/project/RND/mujoco_simulation
# 构建
docker build --build-arg CUDA_VERSION=12.4.1 \
  -f wrapper/Dockerfile \
  -t mujoco-wrapper:stage1 ./wrapper
```

#### git diff
```
cd /home/amigo/project/RND/mujoco_simulation

# 确认 Dockerfile 改动（EGL 库、build-essential、libglib2.0-0 等）已保存
git diff wrapper/Dockerfile

# 提交 Dockerfile 改动
git add wrapper/Dockerfile
git commit -m "stage1: container build passes, EGL + opencv system libs added"

# 更新 tag
git tag -d stage1
git tag stage1
```

#### test

```
# 1. GPU 透传
docker run --gpus all --rm mujoco-wrapper:stage1 nvidia-smi

# 2. import 路径
docker run --gpus all --rm mujoco-wrapper:stage1 \
  python -c "import aidea_mujoco; print(aidea_mujoco.__file__)"

# 3. Stage 1 测试
docker run --gpus all --rm mujoco-wrapper:stage1 \
  python3 test_stage1.py
```  



### STAGE 2: ZMQ + proto

#### design
ZmqPublisher 作为一个通用的、按 topic 分发的发布器，payload 就是序列化后的 protobuf 字节。每个传感器（LiDAR、你的相机、你的 IMU、G1 的 IMU、G1 的相机……）各自负责把数据打包成对应的 protobuf message，然后调用同一个 publish(topic, payload_bytes)。


#### proto
```
uv run python -m grpc_tools.protoc -I proto --python_out=. proto/lidar.proto

cd /home/amigo/project/RND/mujoco_simulation/wrapper

# proto
uv run python -m grpc_tools.protoc -I proto \
  --python_out=aidea_mujoco/aidea_mujoco/msg \
  proto/common.proto proto/lidar.proto proto/image.proto \
  proto/imu.proto proto/joint_state.proto proto/pose.proto

```


#### test

```
docker build --build-arg CUDA_VERSION=12.4.1   -f wrapper/Dockerfile   -t mujoco-wrapper:stage2 ./wrapper
docker run --gpus all --rm -d --name wrapper-stage2 -p 5555:5555 mujoco-wrapper:stage2
docker logs -f wrapper-stage2
#单独仿真 覆盖CMD
docker run ... uv run python run_sim.py

```

```
# 宿主机 SUB（test_stage2_sub.py）
cd /home/amigo/project/RND/mujoco_simulation/wrapper
env -u PYTHONPATH uv run python test_stage2_sub.py
```

```
git checkout -b stage2-zmq
# wrapper/proto/lidar.proto
# wrapper/zmq_publisher.py
# wrapper/Dockerfile 追加 uv add pyzmq protobuf
git add . && git commit -m "stage2: ZMQ PUB with proto"
git tag stage2
```

### STAGE 3: MCP Server

#### docker

```
# dockerfile

CMD ["uv", "run", "python", "mcp_server.py"]

cd /home/amigo/project/RND/mujoco_simulation

# 如果改了 pyproject.toml/uv.lock，需要重新 build
docker build --build-arg CUDA_VERSION=12.4.1 -f wrapper/Dockerfile -t mujoco-wrapper:stage3 ./wrapper

# 停旧容器，启动新的
docker rm -f wrapper-stage3 2>/dev/null
docker run --gpus all --rm -d --name wrapper-stage3 -p 5555:5555 -p 8080:8080 mujoco-wrapper:stage3

# 看日志，确认 MCP 启动和 LiDAR 发布
docker logs -f wrapper-stage3

```

#### test

##### initialize 拿 session id

```
SID=$(curl -sD- -o/dev/null -X POST http://127.0.0.1:8080/mcp \
  -H 'Content-Type: application/json' \
  -H 'Accept: application/json, text/event-stream' \
  -d '{"jsonrpc":"2.0","id":1,"method":"initialize","params":{"protocolVersion":"2025-06-18","capabilities":{},"clientInfo":{"name":"curl","version":"1"}}}' \
  | tr -d '\r' | grep -i '^mcp-session-id' | awk '{print $2}')

echo "SID=$SID"
```

##### 发送 notification/initialized

```
curl -s -X POST http://127.0.0.1:8080/mcp \
  -H "Content-Type: application/json" \
  -H "Accept: application/json, text/event-stream" \
  -H "Mcp-Session-Id: $SID" \
  -d '{"jsonrpc":"2.0","method":"notifications/initialized"}'
```

##### tool/list 或 tools/call

```
# 列出工具
curl -s -X POST http://127.0.0.1:8080/mcp \
  -H "Content-Type: application/json" \
  -H "Accept: application/json, text/event-stream" \
  -H "Mcp-Session-Id: $SID" \
  -d '{"jsonrpc":"2.0","id":2,"method":"tools/list","params":{}}'

# 调用 get_lidar_snapshot
curl -s -X POST http://127.0.0.1:8080/mcp \
  -H "Content-Type: application/json" \
  -H "Accept: application/json, text/event-stream" \
  -H "Mcp-Session-Id: $SID" \
  -d '{"jsonrpc":"2.0","id":3,"method":"tools/call","params":{"name":"get_lidar_snapshot","arguments":{}}}'
```

```
git checkout -b stage3-mcp
# wrapper/mcp_server.py
# wrapper/Dockerfile 追加 uv add mcp uvicorn
git add . && git commit -m "stage3: MCP server"
git tag stage3
```

```
cd /home/amigo/project/RND/mujoco_simulation

# 1. 确认暂存区
git add wrapper/mcp_server.py wrapper/Dockerfile \
        wrapper/pyproject.toml wrapper/uv.lock wrapper/.gitignore

# 2. 确认没有不该进的文件
git status --short | grep -E "mujoco_description|mcp_server\.py.*bak|^..zmq_publisher" || echo "clean"

# 3. 提交
SKIP=sync-proto git commit -m "stage3: minimal MCP server for lidar snapshot + base/joint control"
git tag -d stage3 2>/dev/null
git tag stage3

# 4. 确认
git log --oneline -1
git show --stat HEAD | tail -10
```


### STAGE 4: ROS-bridge container

```
git checkout -b stage4-ros-bridge
# scripts/sync-proto.sh
# ros-bridge/ 下：Dockerfile, zmq_sub_node.py, entrypoint.sh
# 先执行 sync-proto.sh 复制 proto
# docker-compose.yml 加 ros-bridge 服务
git add . && git commit -m "stage4: ros-bridge with ZMQ SUB -> ROS"
git tag stage4
```
##### ros1-bridge

rospy.loginfo 默认输出到 ~/.ros/log/rosout.log，不是 stdout。docker compose logs 只能看到 stdout/stderr，看不到 ROS 的日志文件。

```

cd /home/amigo/project/RND/mujoco_simulation


# 1. build
docker compose --profile ros build ros-bridge

# 2. 启动
docker compose --profile ros up ros-bridge

# 3. 另开终端看日志
docker compose --profile ros logs -f ros-bridge

```
###### test
ros-bridge 启动后，另开终端进容器：
```
docker compose exec ros-bridge bash
source /opt/ros/noetic/setup.bash
rostopic list
rostopic hz /lidar/points
rostopic echo -n 1 /lidar/points/width
```

##### ros2-bridge

```
cd /home/amigo/project/RND/mujoco_simulation

# 1. 同步 proto
bash scripts/sync-proto.sh
ls ros2-bridge/proto/

# 2. 启动 mujoco-wrapper（后台）
docker compose up -d mujoco-wrapper

# 3. 构建并启动 ros2-bridge
docker compose --profile ros build ros2-bridge
docker compose --profile ros up ros2-bridge

# 4. 另开终端，进容器验证
docker compose --profile ros exec ros2-bridge bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /lidar/points
ros2 topic echo /lidar/points --once | head -10

# 5. 录包
ros2 bag record /lidar/points -o /tmp/lidar_bag
# Ctrl+C 停止后
ros2 bag info /tmp/lidar_bag
```



### STAGE 5: Agent container

```
git checkout -b stage5-agent
# agent/ 下：Dockerfile, requirements.txt, agent_main.py
# docker-compose.yml 加 agent 服务
git add . && git commit -m "stage5: agent with MCP client"
git tag stage5
```

```
cd /home/amigo/project/RND/mujoco_simulation
docker compose build agent
docker compose up mujoco-wrapper agent
```

```
docker run --gpus all -v /home/amigo/project/RND/agent_models/models:/models -p 8080:8080 ghcr.io/ggml-org/llama.cpp:server-cuda -m /models/Qwen3.5-4B_Q4_K_M.gguf --port 8080 --host 0.0.0.0 -n 512
```

```
cd /home/amigo/project/RND/mujoco_simulation

# 1. 确认暂存区
git add agent/ docker-compose.yml wrapper/mcp_server.py wrapper/Dockerfile wrapper/pyproject.toml wrapper/uv.lock wrapper/.gitignore

# 2. 确认没有不该进的文件
git status --short | grep -E "mujoco_description|_pb2\.py|\.venv" || echo "clean"

# 3. 提交
SKIP=sync-proto git commit -m "stage4: agent container with MCP client + ZMQ + llama.cpp LLM, end-to-end lidar snapshot"
git tag -d stage4 2>/dev/null
git tag stage4

# 4. 确认
git log --oneline -1
git show --stat HEAD | tail -10
```

##### 分别查看log

```
docker compose up --build -d mujoco-wrapper agent
docker compose logs -f agent
```




### Roll back

```
git checkout stage3
docker compose build mujoco-wrapper
# 回到只有 MCP、没有 ros-bridge 和 agent 的状态
```

### 添加人型机器人

#### 测试
```
cd /home/amigo/project/RND/mujoco_simulation/wrapper

ROBOT_MJCF_INCLUDES=$PWD/aidea_mujoco/mjcf/robots/g1/g1_12dof.xml \
ROBOT_PLUGIN_CONFIGS=$PWD/aidea_mujoco/aidea_mujoco/plugins/g1/config.yaml \
env -u PYTHONPATH uv run python test_stage1.py
```

#### 通过mcp测试模式
```
cd wrapper

# 1. stand（看 G1 站着，腿在 -0.13 附近，身体不动）
./mcp_call.sh set_g1_mode '{"mode":"stand"}'

# 2. walk + 速度（看 G1 腿正弦摆动，身体前移）
./mcp_call.sh set_g1_mode '{"mode":"walk"}'
./mcp_call.sh set_g1_velocity '{"vx":0.3,"wz":0}'

# 3. fall（看 G1 腿乱蹬，身体倒地）
./mcp_call.sh set_g1_mode '{"mode":"fall"}'

# 4. reset（看 G1 回原点站起）
./mcp_call.sh reset_g1 '{}'
```