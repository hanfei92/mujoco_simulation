#!/usr/bin/env python3
"""订阅 mujoco-wrapper 的 ZMQ lidar topic，反序列化 protobuf，发布 sensor_msgs/PointCloud2 到 ROS1。"""
import struct
import time

import numpy as np
import rospy
import zmq
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import common_pb2
import lidar_pb2


def pb_pointcloud2_to_ros(msg: lidar_pb2.PointCloud2) -> PointCloud2:
    ros_msg = PointCloud2()
    ros_msg.header = Header(
        stamp=rospy.Time(msg.header.stamp_sec, msg.header.stamp_nsec),
        frame_id=msg.header.frame_id,
    )
    ros_msg.height = msg.height
    ros_msg.width = msg.width
    ros_msg.is_bigendian = False
    ros_msg.point_step = msg.point_step
    ros_msg.row_step = msg.row_step
    ros_msg.is_dense = msg.is_dense

    ros_msg.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    ros_msg.data = msg.data
    return ros_msg


def main():
    rospy.init_node("zmq_lidar_bridge")

    endpoint = rospy.get_param("~zmq_endpoint", "tcp://mujoco-wrapper:5555")
    topic = rospy.get_param("~ros_topic", "/lidar/points")

    pub = rospy.Publisher(topic, PointCloud2, queue_size=10)

    ctx = zmq.Context()
    sub = ctx.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt_string(zmq.SUBSCRIBE, "")
    sub.setsockopt(zmq.RCVTIMEO, 5000)

    rospy.loginfo(f"zmq_lidar_bridge: connected to {endpoint}, publishing to {topic}")

    while not rospy.is_shutdown():
        try:
            raw = sub.recv()
        except zmq.Again:
            rospy.logwarn("zmq recv timeout")
            continue

        env = common_pb2.ZmqEnvelope()
        env.ParseFromString(raw)

        if env.topic == "lidar":
            pb = lidar_pb2.PointCloud2()
            pb.ParseFromString(env.payload)
            ros_msg = pb_pointcloud2_to_ros(pb)
            pub.publish(ros_msg)
            rospy.loginfo_throttle(5, f"published lidar: {ros_msg.width} points, frame={ros_msg.header.frame_id}")
        else:
            rospy.loginfo_throttle(10, f"skip topic: {env.topic}")

    sub.close()
    ctx.term()


if __name__ == "__main__":
    main()