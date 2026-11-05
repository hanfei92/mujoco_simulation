#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash

# 启动 ROS1 master（bridge 容器内自带 roscore）
roscore &
sleep 2

# 启动 ZMQ SUB → ROS 发布节点
exec python3 zmq_sub_node.py "$@"