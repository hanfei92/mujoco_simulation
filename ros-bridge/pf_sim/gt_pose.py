#!/usr/bin/env python3
"""Project the simulator person into the camera and publish tfpose_ros/Persons.

This is a ground-truth pose source: it replaces the YOLO detector so the real
tracking algorithm can run against the simulator.  Swap this node for a real
detector later; the tracker only consumes ``/pose_estimator/pose``.
"""

import math

import rospy
import tf
from geometry_msgs.msg import PointStamped
from sensor_msgs.msg import CameraInfo
from std_msgs.msg import Header
from tfpose_ros.msg import BodyPartElm, Person, Persons

# Body offsets relative to the G1 pelvis, in metres.
NECK_Z = 0.55
ANKLE_Z = -0.75


class GroundTruthPose(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'pf_cam_optical')
        self.rate = rospy.get_param('~rate', 15.0)
        self.listener = tf.TransformListener()
        self.info = None
        self.person = None
        rospy.Subscriber('/pf_cam/camera_info', CameraInfo, self.info_cb,
                         queue_size=1)
        rospy.Subscriber('/person_pose', PointStamped, self.person_cb,
                         queue_size=1)
        self.publisher = rospy.Publisher('/pose_estimator/pose', Persons,
                                         queue_size=1)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)

    def info_cb(self, message):
        self.info = message

    def person_cb(self, message):
        self.person = message

    def _project(self, point_world, fx, fy, cx, cy, width, height):
        try:
            transform, _ = self.listener.lookupTransform(
                self.frame, 'world', rospy.Time(0))
        except (tf.LookupException, tf.ConnectivityException,
                tf.ExtrapolationException):
            return None
        px, py, pz = transform[0]
        (qx, qy, qz, qw) = transform[1]
        import numpy as np
        rotation = np.array(tf.transformations.quaternion_matrix(
            [qx, qy, qz, qw]))[:3, :3]
        point = np.array([point_world.x, point_world.y, point_world.z])
        camera = rotation @ point + np.array([px, py, pz])
        if camera[2] <= 0.1:
            return None
        u = fx * camera[0] / camera[2] + cx
        v = fy * camera[1] / camera[2] + cy
        if not (0.0 <= u < width and 0.0 <= v < height):
            return None
        return u / width, v / height

    def tick(self, _event):
        if self.info is None or self.person is None:
            return
        fx = self.info.K[0]
        fy = self.info.K[4]
        cx = self.info.K[2]
        cy = self.info.K[5]
        width = float(self.info.width or 1)
        height = float(self.info.height or 1)
        x = self.person.point.x
        y = self.person.point.y
        z = self.person.point.z

        def at(dz):
            return PointStamped(header=self.person.header,
                                point=type(self.person.point)(
                                    x=x, y=y, z=z + dz))

        neck = self._project(at(NECK_Z), fx, fy, cx, cy, width, height)
        if neck is None:
            return
        parts = [BodyPartElm(part_id=1, x=neck[0], y=neck[1], confidence=0.95)]
        for part_id, dz in ((13, ANKLE_Z), (10, ANKLE_Z)):
            ankle = self._project(at(dz), fx, fy, cx, cy, width, height)
            if ankle is not None:
                parts.append(BodyPartElm(part_id=part_id, x=ankle[0],
                                         y=ankle[1], confidence=0.9))

        person = Person(body_part=parts)
        persons = Persons(header=Header(stamp=rospy.Time.now(),
                                        frame_id=self.frame),
                          image_w=int(width), image_h=int(height),
                          persons=[person])
        self.publisher.publish(persons)


if __name__ == '__main__':
    rospy.init_node('pf_sim_gt_pose')
    GroundTruthPose()
    rospy.spin()
