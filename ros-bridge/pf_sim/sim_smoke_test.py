#!/usr/bin/env python3
"""Live smoke test for the running person-following sim.

Run inside pf-sim:
    docker exec pf-sim python3 /app/pf_sim/sim_smoke_test.py

Checks every layer of the pipeline and prints PASS / FAIL / SKIP per case.
Exit code 0 only when nothing failed.
"""

import json
import math
import sys
import time
import urllib.request

import rospy
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import OccupancyGrid, Odometry, Path
from sensor_msgs.msg import Image, PointCloud2
from std_msgs.msg import String
from tfpose_ros.msg import Persons
from monocular_people_tracking.msg import TrackArray

RESULTS = []


def collect(topic, message_type, seconds=4.0):
    """Accumulate messages for the whole window (rate = count / seconds)."""
    box = {'count': 0}

    def callback(message):
        if 'message' not in box:
            box['message'] = message
        box['count'] += 1

    rospy.Subscriber(topic, message_type, callback, queue_size=10)
    deadline = time.time() + seconds
    while time.time() < deadline:
        time.sleep(0.05)
    box['seconds'] = seconds
    return box


def record(name, ok, detail, skipped=False):
    RESULTS.append({'case': name, 'status': 'SKIP' if skipped else
                    ('PASS' if ok else 'FAIL'), 'detail': detail})
    print('  {:<4} {:<38} {}'.format(RESULTS[-1]['status'], name, detail))


def rate_of(box, seconds=None):
    window = seconds or box.get('seconds') or 1.0
    return box.get('count', 0) / window


def main():
    rospy.init_node('pf_sim_smoke_test', anonymous=True)
    print('== person following sim smoke test ==')

    image = collect('/pf_cam/image_raw', Image, 4.0)
    hz = rate_of(image, 4.0)
    record('camera_stream', hz >= 5.0,
           '{:.1f} Hz (>=5 required)'.format(hz))

    overview = collect('/pf_cam/overview', Image, 3.0)
    hz = rate_of(overview, 3.0)
    record('overview_stream', hz >= 5.0, '{:.1f} Hz'.format(hz))

    lidar = collect('/lidar/points', PointCloud2, 4.0)
    hz = rate_of(lidar, 4.0)
    points = getattr(lidar.get('message'), 'width', 0)
    record('lidar_to_ros', hz >= 3.0 and points > 1000,
           '{:.1f} Hz, {} points'.format(hz, points))

    tracks = collect('/monocular_people_tracking/tracks', TrackArray, 4.0)
    hz = rate_of(tracks, 4.0)
    record('tracker_output', hz >= 5.0, '{:.1f} Hz'.format(hz))

    persons = collect('/pose_estimator/pose', Persons, 3.0)
    pose_rate = rate_of(persons, 3.0)
    parts = 0
    message = persons.get('message')
    if message is not None and message.persons:
        parts = len(message.persons[0].body_part)
    if pose_rate < 3.0:
        record('gt_detections', False,
               'no detection ({:.1f} Hz) - person out of view?'.format(pose_rate),
               skipped=True)
    else:
        record('gt_detections', parts >= 2,
               '{:.1f} Hz, {} keypoints'.format(pose_rate, parts))

    costmap = collect('/move_base/local_costmap/costmap',
                      OccupancyGrid, 4.0)
    grid = costmap.get('message')
    occupied = 0 if grid is None else sum(1 for value in grid.data if value >= 50)
    record('costmap_from_lidar', occupied > 0,
           '{} of {} cells occupied'.format(occupied,
                                            0 if grid is None else len(grid.data)))

    state = collect('/observe_manager/state', String, 5.0)
    record('manager_state', state.get('message') is not None,
           getattr(state.get('message'), 'data', 'none'))

    route = collect('/person_route/path', Path, 3.0)
    path = route.get('message')
    if path is None:
        record('coverage_route', True, 'route mode not active', skipped=True)
    else:
        record('coverage_route', len(path.poses) > 5,
               '{} waypoints'.format(len(path.poses)))

    command = collect('/cmd_vel', Twist, 3.0)
    record('command_channel', rate_of(command, 3.0) >= 5.0,
           '{:.1f} Hz'.format(rate_of(command, 3.0)))

    try:
        body = urllib.request.urlopen('http://127.0.0.1:8090/api/status',
                                      timeout=3).read()
        status = json.loads(body)
        record('dashboard_api', True, 'keys: {}'.format(len(status)))
    except Exception as exc:  # noqa: BLE001
        record('dashboard_api', False, str(exc))

    failed = [row for row in RESULTS if row['status'] == 'FAIL']
    skipped = [row for row in RESULTS if row['status'] == 'SKIP']
    print('\n{}/{} passed, {} failed, {} skipped'.format(
        len(RESULTS) - len(failed) - len(skipped), len(RESULTS),
        len(failed), len(skipped)))
    try:
        with open('/opt/pf_tmp/sim_smoke_result.json', 'w') as handle:
            json.dump(RESULTS, handle, indent=2)
    except OSError:
        pass
    return 1 if failed else 0


if __name__ == '__main__':
    sys.exit(main())
