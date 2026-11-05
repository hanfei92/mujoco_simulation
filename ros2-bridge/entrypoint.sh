#!/bin/bash
set -e

source /opt/ros/humble/setup.bash

exec python3 /app/zmq_to_ros2.py "$@"