#!/usr/bin/env python3
"""Simulator lidar ZMQ stream -> /lidar/points (sensor_msgs/PointCloud2).

Without this the local costmap stays empty and the observation manager's
collision check happily drives the base into the furniture.
"""

import numpy as np
import rospy
import zmq
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header

import common_pb2
import lidar_pb2

FIELDS = [
    PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
    PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
    PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
]


def main():
    rospy.init_node('pf_sim_lidar')
    endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5555')
    topic = rospy.get_param('~topic', '/lidar/points')
    frame = rospy.get_param('~frame', 'world')
    every = max(1, int(rospy.get_param('~decimate', 1)))

    publisher = rospy.Publisher(topic, PointCloud2, queue_size=1)
    context = zmq.Context()
    sub = context.socket(zmq.SUB)
    sub.connect(endpoint)
    sub.setsockopt(zmq.SUBSCRIBE, b'')
    sub.setsockopt(zmq.RCVTIMEO, 500)

    while not rospy.is_shutdown():
        try:
            raw = sub.recv()
        except zmq.Again:
            continue
        except zmq.ZMQError:
            break
        envelope = common_pb2.ZmqEnvelope()
        try:
            envelope.ParseFromString(raw)
        except Exception:
            continue
        if envelope.topic != 'lidar':
            continue
        cloud = lidar_pb2.PointCloud2()
        try:
            cloud.ParseFromString(envelope.payload)
        except Exception:
            continue
        points = np.frombuffer(cloud.data, dtype=np.float32)
        if points.size % 3:
            continue
        points = points.reshape(-1, 3)[::every]

        message = PointCloud2()
        message.header = Header(stamp=rospy.Time.now(), frame_id=frame)
        message.height = 1
        message.width = int(points.shape[0])
        message.fields = FIELDS
        message.is_bigendian = False
        message.point_step = 12
        message.row_step = message.point_step * message.width
        message.is_dense = True
        message.data = np.ascontiguousarray(
            points, dtype=np.float32).tobytes()
        publisher.publish(message)


if __name__ == '__main__':
    main()
