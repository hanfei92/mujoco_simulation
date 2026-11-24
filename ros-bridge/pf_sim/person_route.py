#!/usr/bin/env python3
"""Drive the simulated person (G1) along a coverage route over the room map.

The route is a boustrophedon (lawnmower) sweep restricted to the free space of
an occupancy map, with a clearance margin so the person never walks through
furniture.  Segments are stitched with shortest paths on the free grid.

    rosrun/ros python3 person_route.py _map_file:=/tmp/pf_map.json
"""

import collections
import json
import math
import os

import numpy as np
import rospy
import zmq
from geometry_msgs.msg import PointStamped, PoseStamped
from nav_msgs.msg import Path

import command_pb2


class RoutePlanner(object):
    def __init__(self):
        self.resolution = 0.1
        self.grid = None
        self.free = None
        self.lane_spacing = rospy.get_param('~lane_spacing', 1.2)
        self.clearance = rospy.get_param('~clearance', 0.55)
        self.margin = rospy.get_param('~area_margin', 1.5)
        self.max_vx = rospy.get_param('~max_vx', 0.45)
        self.waypoint_tolerance = rospy.get_param('~waypoint_tolerance', 0.25)
        self.loop = bool(rospy.get_param('~loop', True))
        self.rate = rospy.get_param('~rate', 10.0)

        map_file = rospy.get_param('~map_file', '/tmp/pf_map.json')
        if not os.path.isfile(map_file):
            rospy.logerr(
                'person_route: map file %s not found. Generate it with '
                'tools/make_map_from_mjcf.py and mount it into the container '
                '(-v $PWD/tmp/pf_map.json:/tmp/pf_map.json:ro). The person '
                'will stay still.', map_file)
            self.route = []
            self.finished = True
            self.grid = None
            self.free = None
            self.origin_x = self.origin_y = 0.0
            self.resolution = 0.1
            self.index = 0
            self.path_pub = rospy.Publisher('/person_route/path', Path,
                                            queue_size=1, latch=True)
            endpoint = rospy.get_param('~zmq_endpoint',
                                       'tcp://mujoco-wrapper:5556')
            self.context = zmq.Context()
            self.push = self.context.socket(zmq.PUSH)
            self.push.setsockopt(zmq.LINGER, 0)
            self.push.connect(endpoint)
            rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
            return
        self._load_map(map_file)
        self.route = self._build_route()
        self.index = 0
        self.finished = False

        self.path_pub = rospy.Publisher('/person_route/path', Path,
                                        queue_size=1, latch=True)
        self.goal_pub = rospy.Publisher('/person_route/current', PointStamped,
                                        queue_size=1)
        self._publish_path()

        endpoint = rospy.get_param('~zmq_endpoint', 'tcp://mujoco-wrapper:5556')
        self.context = zmq.Context()
        self.push = self.context.socket(zmq.PUSH)
        self.push.setsockopt(zmq.LINGER, 0)
        self.push.connect(endpoint)
        rospy.Timer(rospy.Duration(1.0 / self.rate), self.tick)
        rospy.loginfo('person route: %d waypoints over %.1f m',
                      len(self.route), self._route_length())

    # ---------------- map ----------------
    def _load_map(self, path):
        with open(path) as handle:
            data = json.load(handle)
        self.resolution = float(data['resolution'])
        self.origin_x = float(data['origin_x'])
        self.origin_y = float(data['origin_y'])
        self.grid = np.array(data['data'], dtype=np.int8).reshape(
            int(data['height']), int(data['width']))
        self.free = self._clearance_mask(self.grid, self.clearance)

    def _clearance_mask(self, grid, clearance):
        """Free cells at least `clearance` metres away from any obstacle."""
        blocked = grid >= 50
        if not blocked.any():
            return ~blocked
        distance = np.full(grid.shape, 1e9)
        distance[blocked] = 0.0
        # Multi-source BFS (4-connected) for a Chebyshev-free distance field.
        queue = collections.deque(zip(*np.nonzero(blocked)))
        limit = clearance / self.resolution
        while queue:
            iy, ix = queue.popleft()
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                ny, nx = iy + dy, ix + dx
                if 0 <= ny < grid.shape[0] and 0 <= nx < grid.shape[1]:
                    if distance[ny, nx] > distance[iy, ix] + 1.0:
                        distance[ny, nx] = distance[iy, ix] + 1.0
                        queue.append((ny, nx))
        return distance > limit

    def _world_to_cell(self, x, y):
        return (int((y - self.origin_y) / self.resolution),
                int((x - self.origin_x) / self.resolution))

    def _cell_to_world(self, iy, ix):
        return (self.origin_x + (ix + 0.5) * self.resolution,
                self.origin_y + (iy + 0.5) * self.resolution)

    # ---------------- route ----------------
    def _region(self):
        blocked = self.grid >= 50
        if not blocked.any():
            return 0, self.grid.shape[0], 0, self.grid.shape[1]
        rows = np.nonzero(blocked.any(axis=1))[0]
        cols = np.nonzero(blocked.any(axis=0))[0]
        pad = int(round(self.margin / self.resolution))
        return (max(0, rows[0] - pad), min(self.grid.shape[0], rows[-1] + pad),
                max(0, cols[0] - pad), min(self.grid.shape[1], cols[-1] + pad))

    def _build_route(self):
        y0, y1, x0, x1 = self._region()
        free = np.zeros_like(self.free)
        free[y0:y1, x0:x1] = self.free[y0:y1, x0:x1]
        lane = max(2, int(round(self.lane_spacing / self.resolution)))

        segments = []
        lane_index = 0
        for iy in range(y0 + lane // 2, y1, lane):
            runs, start = [], None
            for ix in range(x0, x1):
                if free[iy, ix] and start is None:
                    start = ix
                elif not free[iy, ix] and start is not None:
                    runs.append((start, ix - 1))
                    start = None
            if start is not None:
                runs.append((start, x1 - 1))
            for run_start, run_end in runs:
                if run_end - run_start < 3:
                    continue
                if lane_index % 2 == 0:
                    ordered = [(iy, run_start), (iy, run_end)]
                else:
                    ordered = [(iy, run_end), (iy, run_start)]
                segments.append(ordered)
            lane_index += 1

        if not segments:
            rospy.logwarn('person route: no free lane found, staying still')
            return []

        route = []
        current = segments[0][0]
        for start, end in segments:
            connector = self._bfs(current, start)
            if connector:
                route.extend(connector[1:])
            route.append(start)
            route.append(end)
            current = end
        simplified = self._simplify([self._cell_to_world(*cell)
                                     for cell in route])
        return simplified

    def _bfs(self, start, goal):
        if start == goal:
            return [start]
        queue = collections.deque([start])
        previous = {start: None}
        while queue:
            iy, ix = queue.popleft()
            if (iy, ix) == goal:
                break
            for dy, dx in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nxt = (iy + dy, ix + dx)
                if nxt in previous:
                    continue
                if not (0 <= nxt[0] < self.free.shape[0] and
                        0 <= nxt[1] < self.free.shape[1]):
                    continue
                if not self.free[nxt]:
                    continue
                previous[nxt] = (iy, ix)
                queue.append(nxt)
        if goal not in previous:
            return []
        path, node = [], goal
        while node is not None:
            path.append(node)
            node = previous[node]
        return list(reversed(path))

    def _simplify(self, points):
        """Keep only the farthest visible point of each straight stretch."""
        if len(points) <= 2:
            return points
        result = [points[0]]
        previous = points[0]
        for point in points[1:]:
            if self._line_of_sight(result[-1], point):
                previous = point
                continue
            result.append(previous)
            previous = point
        result.append(previous)
        return result

    def _line_of_sight(self, a, b):
        steps = max(2, int(math.hypot(b[0] - a[0], b[1] - a[1])
                           / self.resolution))
        for step in range(steps + 1):
            ratio = step / float(steps)
            x = a[0] + (b[0] - a[0]) * ratio
            y = a[1] + (b[1] - a[1]) * ratio
            iy, ix = self._world_to_cell(x, y)
            if not (0 <= iy < self.free.shape[0] and
                    0 <= ix < self.free.shape[1]):
                return False
            if not self.free[iy, ix]:
                return False
        return True

    def _route_length(self):
        return sum(math.hypot(b[0] - a[0], b[1] - a[1])
                   for a, b in zip(self.route, self.route[1:]))

    def _publish_path(self):
        path = Path()
        path.header.frame_id = 'world'
        path.header.stamp = rospy.Time.now()
        from geometry_msgs.msg import PoseStamped
        for x, y in self.route:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = x
            pose.pose.position.y = y
            pose.pose.orientation.w = 1.0
            path.poses.append(pose)
        self.path_pub.publish(path)

    # ---------------- driving ----------------
    def _send(self, vx, wz):
        command = command_pb2.Command()
        command.kind = 'person'
        command.robot_id = 'g1'
        command.mode = 'walk'
        command.vx = float(vx)
        command.omega = float(wz)
        try:
            self.push.send(command.SerializeToString(), zmq.NOBLOCK)
        except zmq.ZMQError:
            pass

    def tick(self, _event):
        if not self.route:
            self._send(0.0, 0.0)
            return
        if self.finished:
            self._send(0.0, 0.0)
            return
        target = self.route[self.index]
        pose = self._person_pose()
        if pose is None:
            self._send(0.0, 0.0)
            return
        x, y, yaw = pose
        dx, dy = target[0] - x, target[1] - y
        distance = math.hypot(dx, dy)
        if distance < self.waypoint_tolerance:
            self.index += 1
            if self.index >= len(self.route):
                if self.loop:
                    self.index = 0
                else:
                    self.finished = True
            return

        goal = PointStamped()
        goal.header.frame_id = 'world'
        goal.header.stamp = rospy.Time.now()
        goal.point.x = target[0]
        goal.point.y = target[1]
        self.goal_pub.publish(goal)

        error = math.atan2(dy, dx) - yaw
        error = math.atan2(math.sin(error), math.cos(error))
        wz = max(-0.9, min(0.9, 1.5 * error))
        vx = self.max_vx * max(0.0, math.cos(error))
        self._send(vx, wz)

    def _person_pose(self):
        return getattr(self, '_latest_person', None)

    def observe_person(self, x, y):
        """Track the person's heading: /person_pose carries no orientation."""
        last = getattr(self, '_latest_person', None)
        yaw = 0.0 if last is None else last[2]
        if last is not None:
            dx, dy = x - last[0], y - last[1]
            if math.hypot(dx, dy) > 0.02:
                yaw = math.atan2(dy, dx)
        self._latest_person = (x, y, yaw)


def main():
    rospy.init_node('pf_sim_person_route')
    planner = RoutePlanner()

    def on_person(message):
        planner.observe_person(message.point.x, message.point.y)

    def on_person_pose(message):
        quaternion = message.pose.orientation
        yaw = math.atan2(2.0 * (quaternion.w * quaternion.z),
                         1.0 - 2.0 * quaternion.z * quaternion.z)
        planner._latest_person = (message.pose.position.x,
                                  message.pose.position.y, yaw)

    rospy.Subscriber('/person_pose', PointStamped, on_person, queue_size=1)
    rospy.Subscriber('/person_pose_stamped', PoseStamped, on_person_pose,
                     queue_size=1)

    def on_route(message):
        """Route drawn in the dashboard replaces the generated coverage path."""
        points = [(pose.pose.position.x, pose.pose.position.y)
                  for pose in message.poses]
        if len(points) < 2:
            rospy.logwarn('person route: ignoring route with %d points',
                          len(points))
            return
        planner.route = points
        planner.index = 0
        planner.finished = False
        planner._publish_path()
        rospy.loginfo('person route: external route with %d points (%.1f m)',
                      len(points), planner._route_length())

    rospy.Subscriber('/person_route/set', Path, on_route, queue_size=1)
    rospy.spin()


if __name__ == '__main__':
    main()
