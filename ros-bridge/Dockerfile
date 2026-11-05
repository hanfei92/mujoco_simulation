# ============================================================
# ros-bridge/Dockerfile
# ============================================================
FROM ubuntu:20.04

ENV DEBIAN_FRONTEND=noninteractive

# ---------- ROS1 Noetic ----------
RUN apt-get update && apt-get install -y curl gnupg2 lsb-release \
    && curl -s https://raw.githubusercontent.com/ros/rosdistro/master/ros.asc | apt-key add - \
    && echo "deb http://packages.ros.org/ros/ubuntu focal main" > /etc/apt/sources.list.d/ros1.list \
    && apt-get update && apt-get install -y ros-noetic-ros-base \
    && rm -rf /var/lib/apt/lists/*

# ---------- ROS2 Humble + ros1_bridge ----------
# RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
#     && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu focal main" > /etc/apt/sources.list.d/ros2.list \
#     && apt-get update && apt-get install -y ros-humble-ros-base ros-humble-ros1-bridge \
#     && rm -rf /var/lib/apt/lists/*

# ---------- Python 依赖（ZMQ + Protobuf） ----------
RUN apt-get update && apt-get install -y python3-pip \
    && pip3 install pyzmq protobuf \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /app

# proto 文件（从 wrapper/proto 复制而来，需在构建前执行 sync-proto.sh）
COPY proto/ ./proto/
RUN python3 -m grpc_tools.protoc -I proto --python_out=. proto/*.proto || \
    (pip3 install grpcio-tools && python3 -m grpc_tools.protoc -I proto --python_out=. proto/*.proto)

# ZMQ SUB 节点 + ROS 发布逻辑
COPY zmq_sub_node.py .

# 启动脚本：source 两边环境，启动 roscore + dynamic_bridge + ZMQ SUB 节点
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]