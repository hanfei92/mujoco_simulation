#!/usr/bin/env python3
"""Simulator camera frames -> /pf_cam/image_raw + CameraInfo + TF."""

import rospy
import tf2_ros
import zmq
from geometry_msgs.msg import TransformStamped
from sensor_msgs.msg import CameraInfo, Image

import common_pb2
import image_pb2


def main():
    rospy.init_node('pf_sim_camera')
    endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5555')
    image_topic = rospy.get_param('~image_topic', '/pf_cam/image_raw')
    overview_topic = rospy.get_param('~overview_topic', '/pf_cam/overview')
    info_topic = rospy.get_param('~info_topic', '/pf_cam/camera_info')
    frame = rospy.get_param('~frame', 'pf_cam_optical')
    parent = rospy.get_param('~parent_frame', 'world')

    image_pub = rospy.Publisher(image_topic, Image, queue_size=1)
    overview_pub = rospy.Publisher(overview_topic, Image, queue_size=1)
    info_pub = rospy.Publisher(info_topic, CameraInfo, queue_size=1, latch=True)
    broadcaster = tf2_ros.TransformBroadcaster()

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
        if envelope.topic not in ('image', 'overview'):
            continue
        pb = image_pb2.Image()
        try:
            pb.ParseFromString(envelope.payload)
        except Exception:
            continue

        stamp = rospy.Time.now()
        image = Image()
        image.header.stamp = stamp
        image.header.frame_id = frame
        image.height = pb.height
        image.width = pb.width
        image.encoding = pb.encoding or 'rgb8'
        image.step = pb.step or (pb.width * 3)
        image.is_bigendian = pb.is_bigendian
        image.data = pb.data
        if envelope.topic == 'overview':
            overview_pub.publish(image)
            continue
        image_pub.publish(image)

        info = CameraInfo()
        info.header = image.header
        info.height = pb.height
        info.width = pb.width
        info.distortion_model = 'plumb_bob'
        info.D = [0.0, 0.0, 0.0, 0.0, 0.0]
        info.K = [pb.fx, 0.0, pb.cx, 0.0, pb.fy, pb.cy, 0.0, 0.0, 1.0]
        info.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        info.P = [pb.fx, 0.0, pb.cx, 0.0,
                  0.0, pb.fy, pb.cy, 0.0,
                  0.0, 0.0, 1.0, 0.0]
        info_pub.publish(info)

        transform = TransformStamped()
        transform.header.stamp = stamp
        transform.header.frame_id = parent
        transform.child_frame_id = frame
        transform.transform.translation.x = pb.cam_x
        transform.transform.translation.y = pb.cam_y
        transform.transform.translation.z = pb.cam_z
        transform.transform.rotation.x = pb.cam_qx
        transform.transform.rotation.y = pb.cam_qy
        transform.transform.rotation.z = pb.cam_qz
        transform.transform.rotation.w = pb.cam_qw or 1.0
        broadcaster.sendTransform(transform)

    sub.close()
    context.term()


if __name__ == '__main__':
    main()
