#!/usr/bin/env python3
"""Scripted mocap person: drives the sim target and publishes its track."""

import math

import rospy
import zmq
from tfpose_ros.msg import Person
from monocular_people_tracking.msg import Track, TrackArray
from monocular_person_following.msg import Target

import command_pb2


class PersonWalker(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'world')
        self.mode = rospy.get_param('~path', 'circle')
        self.speed = rospy.get_param('~speed', 0.4)
        self.radius = rospy.get_param('~radius', 2.0)
        self.center = rospy.get_param('~center', [1.5, 0.0])
        self.period = 1.0 / max(1.0, rospy.get_param('~rate', 20.0))

        endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5556')
        self.context = zmq.Context()
        self.push = self.context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(endpoint)

        self.tracks_pub = rospy.Publisher(
            '/monocular_people_tracking/tracks', TrackArray, queue_size=1)
        self.target_pub = rospy.Publisher(
            '/monocular_person_following/target', Target, queue_size=1)

        self.started = rospy.Time.now()
        self.state = self._pose_at(0.0)
        rospy.Timer(rospy.Duration(self.period), self.tick)

    def _pose_at(self, elapsed):
        if self.mode == 'still':
            x, y = self.center[0], self.center[1]
            return x, y, 0.0, (0.0, 0.0)
        if self.mode == 'line':
            span = max(0.1, self.radius)
            phase = (self.speed * elapsed) % (4.0 * span)
            if phase < 2.0 * span:
                x, vx = -span + phase, 1.0
            else:
                x, vx = span - (phase - 2.0 * span), -1.0
            return (x + self.center[0], self.center[1],
                    math.atan2(0.0, vx), (vx * self.speed, 0.0))
        omega = self.speed / max(0.1, self.radius)
        angle = omega * elapsed
        x = self.center[0] + self.radius * math.cos(angle)
        y = self.center[1] + self.radius * math.sin(angle)
        vx = -self.radius * omega * math.sin(angle)
        vy = self.radius * omega * math.cos(angle)
        return x, y, math.atan2(vy, vx), (vx, vy)

    def tick(self, _event):
        elapsed = (rospy.Time.now() - self.started).to_sec()
        x, y, yaw, velocity = self._pose_at(elapsed)

        command = command_pb2.Command()
        command.kind = 'person'
        command.x = x
        command.y = y
        command.yaw = yaw
        try:
            self.push.send(command.SerializeToString(), zmq.NOBLOCK)
        except zmq.ZMQError:
            pass

        stamp = rospy.Time.now()
        track = Track()
        track.id = 1
        track.age = elapsed
        track.pos.x = x
        track.pos.y = y
        track.pos.z = 0.0
        track.height = 1.6
        track.vel.x = velocity[0]
        track.vel.y = velocity[1]
        track.vel.z = 0.0
        track.cov = [0.01 if index in (0, 6, 12, 18, 24) else 0.0
                     for index in range(25)]
        track.expected_measurement_mean = [0.0] * 4
        track.expected_measurement_cov = [0.0] * 16
        # The supervisor only advances the strict target for associated tracks.
        track.associated = [Person()]
        track.associated_neck_ankle = []

        array = TrackArray()
        array.header.stamp = stamp
        array.header.frame_id = self.frame
        array.tracks = [track]
        self.tracks_pub.publish(array)

        target = Target()
        target.header.stamp = stamp
        target.state.data = 'tracking'
        target.target_id = 1
        target.track_ids = [1]
        target.confidences = [0.9]
        target.classifier_names = []
        target.classifier_confidences = []
        self.target_pub.publish(target)


if __name__ == '__main__':
    rospy.init_node('pf_sim_person')
    walker = PersonWalker()
    rospy.spin()
    walker.push.close()
    walker.context.term()
