#!/usr/bin/env python3
"""Sample odom / manager state / cmd_vel / tracks to inspect the closed loop."""

import math
import sys

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from monocular_people_tracking.msg import TrackArray


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 60.0
    period = float(sys.argv[2]) if len(sys.argv) > 2 else 2.0

    rospy.init_node('pf_watch_loop', anonymous=True)
    box = {'odom': None, 'state': '', 'cmd': None, 'tracks': None}

    rospy.Subscriber('/odom', Odometry, lambda m: box.update(odom=m))
    rospy.Subscriber('/observe_manager/state', String,
                     lambda m: box.update(state=m.data))
    rospy.Subscriber('/cmd_vel', Twist, lambda m: box.update(cmd=m))
    rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray,
                     lambda m: box.update(tracks=m))

    rospy.sleep(1.0)
    start = rospy.Time.now()
    print('{:>6} {:>7} {:>7} {:>7} {:>14} {:>12} {:>6} {:>6}'.format(
        't', 'x', 'y', 'yaw', 'state', 'cmd(v,w)', 'tracks', 'valid'))
    while not rospy.is_shutdown():
        elapsed = (rospy.Time.now() - start).to_sec()
        if elapsed > duration:
            break
        odom = box['odom']
        cmd = box['cmd']
        tracks = box['tracks']
        x = odom.pose.pose.position.x if odom else float('nan')
        y = odom.pose.pose.position.y if odom else float('nan')
        yaw = 0.0
        if odom:
            q = odom.pose.pose.orientation
            yaw = math.atan2(2 * (q.w * q.z), 1 - 2 * q.z * q.z)
        count = len(tracks.tracks) if tracks else 0
        valid = len(tracks.tracks) if tracks else 0
        print('{:6.1f} {:7.2f} {:7.2f} {:7.2f} {:>14} {:>12} {:6d} {:6d}'.format(
            elapsed, x, y, yaw, box['state'] or '-',
            '{:.2f},{:.2f}'.format(cmd.linear.x, cmd.angular.z) if cmd else '-',
            count, valid))
        sys.stdout.flush()
        rospy.sleep(period)


if __name__ == '__main__':
    main()
