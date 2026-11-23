#!/usr/bin/env python3
"""Record the person's ground-truth trace as JSONL for coverage analysis."""

import json
import sys

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node('pf_sim_person_trace')
    # private params must be read after init_node, otherwise the remaps are lost
    out = rospy.get_param('~out', '/tmp/person_trace.jsonl')
    rate = rospy.get_param('~rate', 10.0)
    handle = open(out, 'w')
    last = {'t': 0.0}

    def callback(message):
        now = rospy.Time.now().to_sec()
        if now - last['t'] < 1.0 / rate:
            return
        last['t'] = now
        handle.write(json.dumps({
            't': now,
            'x': message.pose.position.x,
            'y': message.pose.position.y,
        }) + '\n')
        handle.flush()

    rospy.Subscriber('/person_pose_stamped', PoseStamped, callback, queue_size=1)
    rospy.loginfo('person trace -> %s', out)
    rospy.spin()
    handle.close()


if __name__ == '__main__':
    main()
