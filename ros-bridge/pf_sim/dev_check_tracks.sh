#!/bin/bash
# Build and run the real monocular_people_tracking node against synthetic
# detections, then verify it publishes tracks near the simulated person.
#
# Requires the source mounted at /opt/person_following_ws (see docker run in
# docs/PERSON_FOLLOWING_SIM.md).
set +u
source /opt/ros/noetic/setup.bash
source /opt/pf_ws/devel/setup.bash 2>/dev/null || true
export PYTHONPATH=/app:${PYTHONPATH:-}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
ZMQ=tcp://127.0.0.1:5555
TRACKER_BIN=/opt/pf_ws/devel/lib/monocular_people_tracking/monocular_people_tracking_node

pass=0; fail=0
ok() { echo "  PASS  $1"; pass=$((pass+1)); }
no() { echo "  FAIL  $1"; fail=$((fail+1)); }
ok()   { echo "  PASS  $1"; pass=$((pass+1)); }
check_hz() { timeout 6 rostopic hz "$1" 2>&1 | grep -q "average rate" && ok "rate $1" || no "rate $1"; }

if [ ! -x "$TRACKER_BIN" ]; then
    echo "[build] compiling monocular_people_tracking (first run only)"
    cp -r /opt/person_following_ws/src/monocular_people_tracking /opt/pf_ws/src/ || exit 1
CPPOPT_DIR=/opt/pf_ws/src/monocular_people_tracking/3rdparty/CppNumericalSolvers
if [ ! -f "$CPPOPT_DIR/include/cppoptlib/problem.h" ]; then
    echo "[build] fetching cppoptlib headers"
    rm -rf "$CPPOPT_DIR"
    # master (v2) dropped cppoptlib/problem.h; the tracker uses the v1 API.
    git clone "${CPPOPT_URL:-https://github.com/PatWie/CppNumericalSolvers.git}" \
        "$CPPOPT_DIR" >/tmp/cppopt.log 2>&1 \
        && git -C "$CPPOPT_DIR" checkout -q "${CPPOPT_SHA:-2a0f98e7c54c35325641e05c035e43cafd570808}" >>/tmp/cppopt.log 2>&1 \
        || { echo "  FAIL  cppoptlib clone" >&2; tail -5 /tmp/cppopt.log >&2; exit 1; }
fi

    ( cd /opt/pf_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 ) >/tmp/tracker_build.log 2>&1
    if [ ! -x "$TRACKER_BIN" ]; then
        echo "  FAIL  tracker build" >&2
        tail -n 25 /tmp/tracker_build.log >&2
        exit 1
    fi
fi
echo "  PASS  tracker binary present"

roscore >/tmp/roscore.log 2>&1 &
sleep 4
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom >/dev/null 2>&1 &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link >/dev/null 2>&1 &

python3 /app/pf_sim/dev_fake_zmq.py >/tmp/feed.log 2>&1 &
sleep 2
python3 /app/pf_sim/pose_tf.py _zmq_endpoint:=$ZMQ >/tmp/pose.log 2>&1 &
python3 /app/pf_sim/camera_bridge.py _zmq_endpoint:=$ZMQ >/tmp/camera.log 2>&1 &
sleep 2
python3 /app/pf_sim/gt_pose.py >/tmp/gt.log 2>&1 &
sleep 3

rosrun monocular_people_tracking monocular_people_tracking_node \
    camera_info:=/pf_cam/camera_info \
    _detection_confidence_thresh:=0.1 \
    _association_maha_sq_thresh:=9.0 \
    _tracking_remove_trace_thresh:=5.0 \
    _validation_correction_cound:=2 \
    >/tmp/tracker.log 2>&1 &
sleep 8

echo "[tracks]"
check_hz /monocular_people_tracking/tracks
timeout 12 python3 - <<'PY' >/tmp/track_pos.txt 2>&1
import time

import rospy
from monocular_people_tracking.msg import TrackArray

rospy.init_node('pf_track_check', anonymous=True)
received = {}
rospy.Subscriber('/monocular_people_tracking/tracks', TrackArray,
                 lambda message: received.setdefault('message', message))
deadline = time.time() + 8.0
while time.time() < deadline and 'message' not in received:
    time.sleep(0.05)
if 'message' not in received:
    print('no message on /monocular_people_tracking/tracks')
    raise SystemExit(1)
tracks = received['message'].tracks
if not tracks:
    print('message received but no tracks')
    raise SystemExit(1)
track = tracks[0]
print('track id={} pos=({:.2f}, {:.2f})'.format(track.id, track.pos.x,
                                                track.pos.y))
# The synthetic person walks at x=2.0, |y| <= 0.6.
raise SystemExit(0 if 1.2 <= track.pos.x <= 2.8 and abs(track.pos.y) <= 0.8
                 else 1)
PY
if [ $? -eq 0 ]; then ok "track near the simulated person"; else no "track near the simulated person"; fi
sed 's/^/        /' /tmp/track_pos.txt 2>/dev/null

echo
echo "$pass passed, $fail failed"
if [ "$fail" -ne 0 ]; then
    echo "--- logs ---"
    for log in tracker gt camera feed; do echo "## $log"; tail -n 8 "/tmp/$log.log" 2>/dev/null; done
fi

pkill -f dev_fake_zmq.py; pkill -f pose_tf.py; pkill -f camera_bridge.py
pkill -f gt_pose.py; pkill -f monocular_people_tracking_node
pkill -f static_transform_publisher; pkill -f rosmaster; pkill -f rosout
[ "$fail" -eq 0 ]
