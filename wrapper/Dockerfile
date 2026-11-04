ARG CUDA_VERSION=12.4.1
FROM nvidia/cuda:${CUDA_VERSION}-runtime-ubuntu22.04

ENV DEBIAN_FRONTEND=noninteractive
ENV MUJOCO_GL=egl
ENV PYOPENGL_PLATFORM=egl

RUN apt-get update && apt-get install -y --no-install-recommends \
    python3.11 python3.11-venv python3.11-dev \
    libgl1-mesa-glx libosmesa6-dev libglew-dev libglfw3 \
    libgl1 libegl-dev libgles2 libegl-mesa0\
    libglib2.0-0 libsm6 libxext6 libxrender1 libgomp1 \
    build-essential \
    curl \
    && rm -rf /var/lib/apt/lists/*

RUN update-alternatives --install /usr/bin/python3 python3 /usr/bin/python3.11 1

COPY --from=ghcr.io/astral-sh/uv:latest /uv /bin/uv
WORKDIR /app

COPY pyproject.toml uv.lock ./
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked --no-install-project

COPY . .
RUN --mount=type=cache,target=/root/.cache/uv \
    uv sync --locked

# Stage 2: 生成 proto
COPY proto/ ./proto/
RUN uv run python -m grpc_tools.protoc -I proto \
    --python_out=aidea_mujoco/aidea_mujoco/msg \
    proto/common.proto proto/lidar.proto proto/image.proto \
    proto/imu.proto proto/joint_state.proto proto/pose.proto

# 确保 msg 是包
RUN touch aidea_mujoco/aidea_mujoco/msg/__init__.py

# ZMQ + Protobuf + MCP
RUN --mount=type=cache,target=/root/.cache/uv \
    uv add pyzmq protobuf "mcp[cli]" uvicorn

CMD ["uv", "run", "python", "mcp_server.py"]