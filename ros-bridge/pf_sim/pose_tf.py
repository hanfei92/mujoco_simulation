#!/usr/bin/env python3
"""MuJoCo G1 pose (ZMQ) -> ROS TF and /odom."""

import math

import rospy
import tf2_ros
import zmq
from geometry_msgs.msg import TransformStamped
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Header

import common_pb2
import pose_pb2


def main():
    rospy.init_node('pf_sim_pose_tf')
    endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5555')
    parent = rospy.get_param('~parent_frame', 'world')
    child = rospy.get_param('~child_frame', 'base_footprint')

    broadcaster = tf2_ros.TransformBroadcaster()
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=5)
    person_pub = rospy.Publisher('/person_pose', PointStamped, queue_size=1)
    person_full_pub = rospy.Publisher('/person_pose_stamped', PoseStamped,
                                      queue_size=1)

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
        if envelope.topic not in ('pose', 'person_pose'):
            continue
        pose = pose_pb2.PoseStamped()
        try:
            pose.ParseFromString(envelope.payload)
        except Exception:
            continue
        if envelope.topic == 'person_pose':
            stamp = rospy.Time.now()
            person_pub.publish(PointStamped(
                header=Header(frame_id=parent, stamp=stamp),
                point=Point(x=pose.position_x, y=pose.position_y,
                            z=pose.position_z)))
            # The route driver needs the heading; PointStamped does not carry it.
            full = PoseStamped()
            full.header = Header(frame_id=parent, stamp=stamp)
            full.pose.position.x = pose.position_x
            full.pose.position.y = pose.position_y
            full.pose.position.z = pose.position_z
            full.pose.orientation.x = pose.orientation_x
            full.pose.orientation.y = pose.orientation_y
            full.pose.orientation.z = pose.orientation_z
            full.pose.orientation.w = pose.orientation_w
            person_full_pub.publish(full)
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

        # The tracking node uses tf1, which does not consume tf2's /tf_static,
        # so the fixed frames are re-broadcast on /tf every cycle.
        for parent_frame, child_frame, xyz in (
                ('world', 'odom', (0.0, 0.0, 0.0)),
                ('base_footprint', 'base_link', (0.0, 0.0, 0.0)),
                ('base_link', 'g_base', (-0.028185, -0.0016001, 0.4765))):
            fixed = TransformStamped()
            fixed.header.stamp = stamp
            fixed.header.frame_id = parent_frame
            fixed.child_frame_id = child_frame
            fixed.transform.translation.x = xyz[0]
            fixed.transform.translation.y = xyz[1]
            fixed.transform.translation.z = xyz[2]
            if child_frame == 'g_base':
                fixed.transform.rotation.z = 0.7071067811865476
                fixed.transform.rotation.w = 0.7071067811865476
            else:
                fixed.transform.rotation.w = 1.0
            broadcaster.sendTransform(fixed)

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
