#!/usr/bin/env python3
"""Publish synthetic pose/person_pose/image frames for bridge testing.

This stands in for the MuJoCo wrapper so the ROS bridge can be verified
without a GPU: a robot at the origin looking along +x, a person drifting in
front of it, and a synthetic 640x480 image with the matching pinhole intrinsics.
"""

import math
import time

import numpy as np
import zmq

import common_pb2
import image_pb2
import pose_pb2

WIDTH, HEIGHT = 640, 480
FOVY = 60.0
FOCAL = 0.5 * HEIGHT / math.tan(math.radians(FOVY) / 2.0)


def quaternion_from_matrix(matrix):
    trace = matrix[0][0] + matrix[1][1] + matrix[2][2]
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        return (0.25 * s, (matrix[2][1] - matrix[1][2]) / s,
                (matrix[0][2] - matrix[2][0]) / s,
                (matrix[1][0] - matrix[0][1]) / s)
    if matrix[0][0] > matrix[1][1] and matrix[0][0] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[0][0] - matrix[1][1] - matrix[2][2]) * 2.0
        return ((matrix[2][1] - matrix[1][2]) / s, 0.25 * s,
                (matrix[0][1] + matrix[1][0]) / s,
                (matrix[0][2] + matrix[2][0]) / s)
    if matrix[1][1] > matrix[2][2]:
        s = math.sqrt(1.0 + matrix[1][1] - matrix[0][0] - matrix[2][2]) * 2.0
        return ((matrix[0][2] - matrix[2][0]) / s,
                (matrix[0][1] + matrix[1][0]) / s, 0.25 * s,
                (matrix[1][2] + matrix[2][1]) / s)
    s = math.sqrt(1.0 + matrix[2][2] - matrix[0][0] - matrix[1][1]) * 2.0
    return ((matrix[1][0] - matrix[0][1]) / s,
            (matrix[0][2] + matrix[2][0]) / s,
            (matrix[1][2] + matrix[2][1]) / s, 0.25 * s)


def main():
    context = zmq.Context()
    publisher = context.socket(zmq.PUB)
    publisher.bind('tcp://*:5555')

    # Optical frame: x right, y down, z forward, with the robot looking +x.
    # A small downward pitch keeps the person's ankles inside the image; the
    # tracker refuses to spawn a new track from neck-only observations.
    pitch = -0.10
    base = np.array([[0.0, 0.0, 1.0],
                     [-1.0, 0.0, 0.0],
                     [0.0, -1.0, 0.0]])
    rotation = base @ np.array([[1.0, 0.0, 0.0],
                                [0.0, math.cos(pitch), -math.sin(pitch)],
                                [0.0, math.sin(pitch), math.cos(pitch)]])
    qw, qx, qy, qz = quaternion_from_matrix(rotation)

    start = time.time()
    frame = np.zeros((HEIGHT, WIDTH, 3), dtype=np.uint8)
    frame[:, :] = (30, 30, 30)
    frame[100:380, 240:400] = (200, 140, 40)

    while True:
        elapsed = time.time() - start

        pose = pose_pb2.PoseStamped()
        pose.header.frame_id = 'world'
        pose.position_x = 0.0
        pose.position_y = 0.0
        pose.position_z = 0.0
        pose.orientation_w = 1.0

        person = pose_pb2.PoseStamped()
        person.header.frame_id = 'world'
        person.position_x = 2.0
        person.position_y = 0.6 * math.sin(elapsed * 0.5)
        # G1 pelvis height: keeps the ankles inside the image, which the
        # tracker requires before it will create a new track.
        person.position_z = 0.79
        person.orientation_w = 1.0

        image = image_pb2.Image()
        image.header.frame_id = 'pf_cam_optical'
        image.height = HEIGHT
        image.width = WIDTH
        image.encoding = 'rgb8'
        image.step = WIDTH * 3
        image.data = frame.tobytes()
        image.fx = FOCAL
        image.fy = FOCAL
        image.cx = WIDTH / 2.0
        image.cy = HEIGHT / 2.0
        image.cam_x = 0.0
        image.cam_y = 0.0
        image.cam_z = 1.0
        image.cam_qx = qx
        image.cam_qy = qy
        image.cam_qz = qz
        image.cam_qw = qw

        for topic, message in (('pose', pose), ('person_pose', person),
                               ('image', image)):
            envelope = common_pb2.ZmqEnvelope()
            envelope.topic = topic
            envelope.payload = message.SerializeToString()
            publisher.send(envelope.SerializeToString())

        time.sleep(1.0 / 15.0)


if __name__ == '__main__':
    main()
