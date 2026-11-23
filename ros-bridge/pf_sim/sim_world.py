#!/usr/bin/env python3
"""Static map, costmaps and localization flag for the simulator."""

import itertools
import math

import numpy as np
import rospy
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from std_msgs.msg import Bool


class SimWorld(object):
    def __init__(self):
        self.frame = rospy.get_param('~frame', 'world')
        self.resolution = rospy.get_param('~resolution', 0.1)
        self.width = rospy.get_param('~width', 20.0)
        self.height = rospy.get_param('~height', 20.0)
        self.local_size = rospy.get_param('~local_size', 5.0)
        self.origin_x = -self.width / 2.0
        self.origin_y = -self.height / 2.0
        self.robot = (0.0, 0.0)
        self.lidar_points = np.zeros((0, 3))

        self.map_pub = rospy.Publisher('/map', OccupancyGrid, queue_size=1,
                                       latch=True)
        self.global_pub = rospy.Publisher(
            '/move_base/global_costmap/costmap', OccupancyGrid, queue_size=1,
            latch=True)
        self.local_pub = rospy.Publisher(
            '/move_base/local_costmap/costmap', OccupancyGrid, queue_size=1)
        self.ready_pub = rospy.Publisher('/map_follow/localization_ready',
                                         Bool, queue_size=1, latch=True)

        rospy.Subscriber('/odom', Odometry, self.odom_cb, queue_size=1)
        rospy.Subscriber('/lidar/points', PointCloud2, self.lidar_cb,
                         queue_size=1)

        self.static = self._static_grid()
        self.map_pub.publish(self._grid_message(self.static, self.origin_x,
                                                self.origin_y,
                                                self.static.shape[1],
                                                self.static.shape[0]))
        self.global_pub.publish(self._grid_message(
            self.static, self.origin_x, self.origin_y,
            self.static.shape[1], self.static.shape[0]))
        rospy.Timer(rospy.Duration(1.0), self.publish_ready)
        rospy.Timer(rospy.Duration(0.2), self.publish_local)

    def _static_grid(self):
        width = int(round(self.width / self.resolution))
        height = int(round(self.height / self.resolution))
        grid = np.zeros((height, width), dtype=np.int8)
        border = max(1, int(round(0.2 / self.resolution)))
        grid[:border, :] = 100
        grid[-border:, :] = 100
        grid[:, :border] = 100
        grid[:, -border:] = 100
        return grid

    def _grid_message(self, data, origin_x, origin_y, width, height):
        message = OccupancyGrid()
        message.header.frame_id = self.frame
        message.header.stamp = rospy.Time.now()
        message.info.resolution = self.resolution
        message.info.width = int(width)
        message.info.height = int(height)
        message.info.origin.position.x = float(origin_x)
        message.info.origin.position.y = float(origin_y)
        message.info.origin.orientation.w = 1.0
        message.data = data.astype(np.int8).flatten().tolist()
        return message

    def odom_cb(self, message):
        self.robot = (message.pose.pose.position.x,
                      message.pose.pose.position.y)

    def lidar_cb(self, message):
        try:
            points = list(itertools.islice(
                pc2.read_points(message, field_names=('x', 'y', 'z'),
                                skip_nans=True), 40000))
        except Exception:
            return
        self.lidar_points = np.asarray(points, dtype=np.float32).reshape(-1, 3)

    def publish_ready(self, _event=None):
        self.ready_pub.publish(Bool(data=True))

    def publish_local(self, _event=None):
        size = int(round(self.local_size / self.resolution))
        origin_x = self.robot[0] - self.local_size / 2.0
        origin_y = self.robot[1] - self.local_size / 2.0
        grid = np.zeros((size, size), dtype=np.int8)
        if len(self.lidar_points):
            pts = self.lidar_points
            # only points near the floor matter for a 2D costmap, and points
            # right next to the robot are its own body (the lidar only excludes
            # the airy body), so they must not be treated as obstacles
            mask = (np.abs(pts[:, 2]) < 1.5)
            radius = np.hypot(pts[:, 0] - self.robot[0],
                              pts[:, 1] - self.robot[1])
            mask &= radius > 0.45
            pts = pts[mask]
            if len(pts):
                ix = ((pts[:, 0] - origin_x) / self.resolution).astype(np.int32)
                iy = ((pts[:, 1] - origin_y) / self.resolution).astype(np.int32)
                keep = (ix >= 0) & (ix < size) & (iy >= 0) & (iy < size)
                grid[iy[keep], ix[keep]] = 100
        self.local_pub.publish(self._grid_message(grid, origin_x, origin_y,
                                                  size, size))


if __name__ == '__main__':
    rospy.init_node('pf_sim_world')
    SimWorld()
    rospy.spin()
