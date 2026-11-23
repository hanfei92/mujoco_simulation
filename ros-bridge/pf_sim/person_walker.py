#!/usr/bin/env python3
"""Drive the simulated person (G1) along a path.

The G1 plugin integrates its own base from a velocity command, so the walker
only needs to keep sending a walk velocity.  Tracking output no longer comes
from here: it comes from the real ``monocular_people_tracking`` node fed by
``gt_pose.py``.  Set ``~publish_tracks:=true`` to fall back to ground-truth
tracks (useful before the tracker is built).
"""

import math

import rospy
import zmq
from tfpose_ros.msg import Person
from monocular_people_tracking.msg import Track, TrackArray
from monocular_person_following.msg import Target

import command_pb2


class PersonWalker(object):
    def __init__(self):
        self.mode = rospy.get_param('~mode', 'circle')
        self.vx = rospy.get_param('~vx', 0.4)
        self.wz = rospy.get_param('~wz', 0.25)
        self.publish_tracks = rospy.get_param('~publish_tracks', False)
        self.rate = rospy.get_param('~rate', 20.0)
        self.start = rospy.Time.now()

        endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5556')
        self.context = zmq.Context()
        self.push = self.context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(endpoint)

        self.tracks_pub = rospy.Publisher(
            '/monocular_people_tracking/tracks', TrackArray, queue_size=1)
        self.target_pub = rospy.Publisher(
            '/monocular_person_following/target', Target, queue_size=1)

        rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)

    def tick(self, _event):
        command = command_pb2.Command()
        command.kind = 'person'
        command.robot_id = 'g1'
        command.mode = 'walk'
        if self.mode == 'still':
            command.vx = 0.0
            command.omega = 0.0
        else:
            command.vx = self.vx
            command.omega = self.wz
        try:
            self.push.send(command.SerializeToString(), zmq.NOBLOCK)
        except zmq.ZMQError:
            pass

        if not self.publish_tracks:
            return
        stamp = rospy.Time.now()
        track = Track()
        track.id = 1
        track.age = (stamp - self.start).to_sec()
        track.pos.x = 0.0
        track.pos.y = 0.0
        track.pos.z = 0.0
        track.height = 1.6
        track.cov = [0.01 if index in (0, 6, 12, 18, 24) else 0.0
                     for index in range(25)]
        track.expected_measurement_mean = [0.0] * 4
        track.expected_measurement_cov = [0.0] * 16
        track.associated = [Person()]
        track.associated_neck_ankle = []
        array = TrackArray()
        array.header.stamp = stamp
        array.header.frame_id = 'world'
        array.tracks = [track]
        self.tracks_pub.publish(array)

        target = Target()
        target.header.stamp = stamp
        target.state.data = 'tracking'
        target.target_id = 1
        self.target_pub.publish(target)


if __name__ == '__main__':
    rospy.init_node('pf_sim_person')
    walker = PersonWalker()
    rospy.spin()
    walker.push.close()
    walker.context.term()
