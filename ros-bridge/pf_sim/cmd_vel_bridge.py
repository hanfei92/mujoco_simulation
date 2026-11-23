#!/usr/bin/env python3
"""ROS /cmd_vel -> MuJoCo ZMQ command channel."""

import rospy
import zmq
from geometry_msgs.msg import Twist

import command_pb2


class CmdVelBridge(object):
    def __init__(self):
        endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5556')
        self.robot_id = rospy.get_param('~robot_id', 'g1')
        self.mode = rospy.get_param('~mode', 'walk')
        self.context = zmq.Context()
        self.push = self.context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(endpoint)
        rospy.Subscriber('/cmd_vel', Twist, self.callback, queue_size=1)
        rospy.loginfo('cmd_vel -> %s (robot_id=%s, mode=%s)',
                      endpoint, self.robot_id, self.mode)

    def callback(self, message):
        command = command_pb2.Command()
        command.kind = 'robot'
        command.robot_id = self.robot_id
        command.vx = message.linear.x
        command.omega = message.angular.z
        command.mode = self.mode
        try:
            self.push.send(command.SerializeToString(), zmq.NOBLOCK)
        except zmq.ZMQError:
            pass


if __name__ == '__main__':
    rospy.init_node('pf_sim_cmd_vel')
    bridge = CmdVelBridge()
    rospy.spin()
    bridge.push.close()
    bridge.context.term()
