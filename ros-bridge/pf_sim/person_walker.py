#!/usr/bin/env python3
"""Drive the simulated person (G1) along a path.

The G1 plugin integrates its own base from a velocity command, so the walker
only needs to keep sending a walk velocity.  Tracking output no longer comes
from here: it comes from the real ``monocular_people_tracking`` node fed by
``gt_pose.py``.  Set ``~publish_tracks:=true`` to fall back to ground-truth
tracks (useful before the tracker is built).
"""

import json
import math

import rospy
import zmq
from tfpose_ros.msg import Person
from monocular_people_tracking.msg import Track, TrackArray
from monocular_person_following.msg import Target
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped

import command_pb2


class PersonWalker(object):
    def __init__(self):
        self.mode = rospy.get_param('~mode', 'circle')
        self.vx = rospy.get_param('~vx', 0.4)
        self.wz = rospy.get_param('~wz', 0.25)
        self.publish_tracks = rospy.get_param('~publish_tracks', False)
        self.rate = rospy.get_param('~rate', 20.0)
        # stay still until the tracker has locked on
        self.start_delay = rospy.get_param('~start_delay', 25.0)
        self.started_at = None
        self.start = rospy.Time.now()
        # orbit mode: walk a circle centred on the robot, avoiding furniture
        self.orbit_radius = rospy.get_param('~orbit_radius', 3.0)
        self.orbit_speed = rospy.get_param('~orbit_speed', 0.35)
        self.orbit_clearance = rospy.get_param('~orbit_clearance', 0.6)
        self.orbit = 0.0
        self.robot = None
        self._person = None
        self._person_yaw = 0.0
        self.map_data = None
        map_file = rospy.get_param('~map_file', '')
        if map_file:
            try:
                with open(map_file) as handle:
                    self.map_data = json.load(handle)
            except OSError:
                rospy.logwarn('person_walker: map file %s not found', map_file)
        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/person_pose_stamped', PoseStamped,
                         self.person_cb, queue_size=1)

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

    def odom_cb(self, message):
        self.robot = (message.pose.pose.position.x,
                      message.pose.pose.position.y)

    def person_cb(self, message):
        quaternion = message.pose.orientation
        self._person = (message.pose.position.x, message.pose.position.y)
        self._person_yaw = math.atan2(2.0 * (quaternion.w * quaternion.z),
                                      1.0 - 2.0 * quaternion.z * quaternion.z)

    def _blocked(self, x, y):
        if not self.map_data:
            return False
        resolution = float(self.map_data['resolution'])
        origin_x = float(self.map_data['origin_x'])
        origin_y = float(self.map_data['origin_y'])
        width = int(self.map_data['width'])
        height = int(self.map_data['height'])
        data = self.map_data['data']
        radius = int(round(self.orbit_clearance / resolution))
        ix = int((x - origin_x) / resolution)
        iy = int((y - origin_y) / resolution)
        for dy in range(-radius, radius + 1):
            for dx in range(-radius, radius + 1):
                nx, ny = ix + dx, iy + dy
                if not (0 <= nx < width and 0 <= ny < height):
                    return True
                if data[ny * width + nx] >= 50:
                    return True
        return False

    def orbit_step(self):
        """Drive the person around the robot, counter-clockwise."""
        if self.robot is None:
            return 0.0, 0.0
        step = self.orbit_speed / max(0.5, self.orbit_radius)
        radius = self.orbit_radius
        target = None
        for _ in range(40):
            self.orbit += step
            candidate = (self.robot[0] + radius * math.cos(self.orbit),
                         self.robot[1] + radius * math.sin(self.orbit))
            if not self._blocked(candidate[0], candidate[1]):
                target = candidate
                break
        person = getattr(self, '_person', None)
        if person is None or target is None:
            # the whole ring is blocked: wait instead of walking into furniture
            rospy.logwarn_throttle(5.0, 'person_walker: orbit ring blocked, '
                                        'waiting at the current point')
            return 0.0, 0.0
        dx = target[0] - person[0]
        dy = target[1] - person[1]
        distance = math.hypot(dx, dy)
        if distance < 0.35:
            return self.orbit_speed, 0.0
        error = math.atan2(dy, dx) - self._person_yaw
        error = math.atan2(math.sin(error), math.cos(error))
        wz = max(-0.9, min(0.9, 1.6 * error))
        vx = self.orbit_speed * max(0.0, math.cos(error))
        return vx, wz

    def tick(self, _event):
        if self.started_at is None:
            self.started_at = rospy.Time.now()
        command = command_pb2.Command()
        command.kind = 'person'
        command.robot_id = 'g1'
        command.mode = 'walk'
        if (self.mode == 'still' or
                (rospy.Time.now() - self.started_at).to_sec() <
                self.start_delay):
            command.vx = 0.0
            command.omega = 0.0
        elif self.mode == 'orbit':
            vx, wz = self.orbit_step()
            command.vx = vx
            command.omega = wz
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
