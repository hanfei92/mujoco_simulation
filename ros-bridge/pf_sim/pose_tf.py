#!/usr/bin/env python3
"""MuJoCo G1 pose (ZMQ) -> ROS TF and /odom."""

import math

import rospy
import tf2_ros
import zmq
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry

import common_pb2
import pose_pb2


def main():
    rospy.init_node('pf_sim_pose_tf')
    endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5555')
    parent = rospy.get_param('~parent_frame', 'world')
    child = rospy.get_param('~child_frame', 'base_footprint')

    broadcaster = tf2_ros.TransformBroadcaster()
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)

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
        if envelope.topic != 'pose':
            continue
        pose = pose_pb2.PoseStamped()
        try:
            pose.ParseFromString(envelope.payload)
        except Exception:
            continue

        stamp = rospy.Time.now()
        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = child
        transform.transform.translation.x = pose.position_x
        transform.transform.translation.y = pose.position_y
        transform.transform.translation.z = 0.0
        if pose.orientation_w == 0.0 and pose.orientation_z == 0.0:
            transform.transform.rotation.w = 1.0
        else:
            transform.transform.rotation.w = pose.orientation_w
            transform.transform.rotation.z = pose.orientation_z
        broadcaster.sendTransform(transform)

        odom = Odometry()
        odom.header = transform.header
        odom.child_frame_id = child
        odom.pose.pose.position.x = pose.position_x
        odom.pose.pose.position.y = pose.position_y
        odom.pose.pose.orientation = transform.transform.rotation
        odom_pub.publish(odom)

    sub.close()
    context.term()


if __name__ == '__main__':
    main()
