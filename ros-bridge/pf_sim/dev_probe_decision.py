#!/usr/bin/env python3
"""Summarise manager decisions and cmd_vel values for N seconds."""
import collections
import json
import sys

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import String


def main():
    duration = float(sys.argv[1]) if len(sys.argv) > 1 else 6.0
    rospy.init_node('pf_probe_decision', anonymous=True)
    decisions = collections.Counter()
    commands = collections.Counter()

    def on_decision(message):
        data = json.loads(message.data)
        decisions[(data['state'], data['action'])] += 1

    def on_command(message):
        commands[round(message.angular.z, 2)] += 1

    rospy.Subscriber('/observe_manager/decision', String, on_decision)
    rospy.Subscriber('/cmd_vel', Twist, on_command)
    rospy.sleep(duration)
    print('decisions', dict(decisions.most_common(5)))
    print('cmd wz', dict(commands.most_common(5)))


if __name__ == '__main__':
    main()
