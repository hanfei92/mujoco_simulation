#!/usr/bin/env python3
"""Tracking output -> Target message for the observation manager."""

import math

import rospy
from geometry_msgs.msg import PointStamped
from std_msgs.msg import Header, String
from monocular_people_tracking.msg import TrackArray
from monocular_person_following.msg import Target


class TargetBridge(object):
    def __init__(self):
        self.tracks = None
        self.person = None
        self.last_id = -1
        rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray,
                         self.tracks_cb, queue_size=1)
        rospy.Subscriber('/person_pose', PointStamped, self.person_cb,
                         queue_size=1)
        self.publisher = rospy.Publisher('/monocular_person_following/target',
                                         Target, queue_size=1)
        rospy.Timer(rospy.Duration(0.1), self.tick)

    def tracks_cb(self, message):
        self.tracks = message

    def person_cb(self, message):
        self.person = message

    def tick(self, _event):
        target = Target()
        target.header = Header(stamp=rospy.Time.now(), frame_id='world')
        track_id = -1
        # The tracker publishes odom/world positions; pick the track closest to
        # the simulated person, which is what a re-identification stage would
        # do from appearance.
        if self.tracks is not None and self.person is not None:
            best = None
            best_distance = float('inf')
            for track in self.tracks.tracks:
                distance = math.hypot(track.pos.x - self.person.point.x,
                                      track.pos.y - self.person.point.y)
                if distance < best_distance:
                    best_distance = distance
                    best = track.id
            if best is not None and best_distance < 2.0:
                track_id = best
        if track_id >= 0:
            target.state = String(data='tracking')
            target.target_id = track_id
            target.track_ids = [track_id]
            target.confidences = [1.0]
        else:
            target.state = String(data='initial')
            target.target_id = -1
        self.publisher.publish(target)


if __name__ == '__main__':
    rospy.init_node('pf_sim_target')
    TargetBridge()
    rospy.spin()
