# ============================================================
# ros2-bridge/Dockerfile
# ============================================================
FROM ros:humble-ros-base

ENV DEBIAN_FRONTEND=noninteractive

# Python 依赖 + rosbag2
RUN apt-get update && apt-get install -y \
    python3-pip \
    ros-humble-rosbag2 \
    && rm -rf /var/lib/apt/lists/*

RUN python3 -m pip install --upgrade pip && \
    python3 -m pip install pyzmq protobuf grpcio-tools

WORKDIR /app

# proto 源文件（sync-proto.sh 从 wrapper/proto 复制）
COPY proto/ ./proto/

# 生成 Python proto 代码
RUN ls -la proto/ && \
    python3 -m grpc_tools.protoc -I proto --python_out=. \
        proto/common.proto proto/lidar.proto proto/image.proto \
        proto/imu.proto proto/joint_state.proto proto/pose.proto && \
    ls -la *_pb2.py

COPY zmq_to_ros2.py .
COPY entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]