#!/usr/bin/env python3
"""订阅 mujoco-wrapper 的 ZMQ，反序列化 protobuf，发布 ROS2 消息。"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import zmq

import common_pb2
import lidar_pb2


class ZmqToRos2(Node):
    def __init__(self):
        super().__init__("zmq_to_ros2")
        self.declare_parameter("zmq_endpoint", "tcp://mujoco-wrapper:5555")
        endpoint = self.get_parameter("zmq_endpoint").value

        self.pub_lidar = self.create_publisher(PointCloud2, "/lidar/points", 10)

        self.ctx = zmq.Context()
        self.sub = self.ctx.socket(zmq.SUB)
        self.sub.connect(endpoint)
        self.sub.setsockopt_string(zmq.SUBSCRIBE, "")
        self.sub.setsockopt(zmq.RCVTIMEO, 5000)

        self.get_logger().info(f"zmq_to_ros2: connected to {endpoint}")

    def spin_once(self):
        try:
            raw = self.sub.recv()
        except zmq.Again:
            self.get_logger().warn("zmq recv timeout", throttle_duration_sec=5.0)
            return

        env = common_pb2.ZmqEnvelope()
        env.ParseFromString(raw)

        if env.topic == "lidar":
            pb = lidar_pb2.PointCloud2()
            pb.ParseFromString(env.payload)
            msg = self._pb_to_ros_pointcloud2(pb)
            self.pub_lidar.publish(msg)
            self.get_logger().info(
                f"published lidar: {msg.width} points, frame={msg.header.frame_id}",
                throttle_duration_sec=5.0,
            )

    def _pb_to_ros_pointcloud2(self, pb) -> PointCloud2:
        msg = PointCloud2()
        msg.header = Header()
        msg.header.stamp.sec = pb.header.stamp_sec
        msg.header.stamp.nanosec = pb.header.stamp_nsec
        msg.header.frame_id = pb.header.frame_id
        msg.height = pb.height
        msg.width = pb.width
        msg.is_bigendian = False
        msg.point_step = pb.point_step
        msg.row_step = pb.row_step
        msg.is_dense = pb.is_dense
        msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        ]
        msg.data = pb.data
        return msg

    def destroy_node(self):
        self.sub.close()
        self.ctx.term()
        super().destroy_node()


def main():
    rclpy.init()
    node = ZmqToRos2()
    try:
        while rclpy.ok():
            node.spin_once()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()