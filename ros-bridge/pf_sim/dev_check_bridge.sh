#!/bin/bash
# Verify the ROS half of the person-following bridge against synthetic ZMQ data.
# Usage (inside the pf-sim image):
#   docker run --rm --entrypoint bash \
#     -v $PWD/ros-bridge/pf_sim:/app/pf_sim:ro \
#     -v $PWD/ros-bridge/proto:/app/proto:ro \
#     mujoco_simulation-pf-sim:latest -lc 'bash /app/pf_sim/dev_check_bridge.sh'
set +u
source /opt/ros/noetic/setup.bash
source /opt/pf_ws/devel/setup.bash 2>/dev/null || true

mkdir -p /tmp/pb2
python3 -m grpc_tools.protoc -I /app/proto --python_out=/tmp/pb2 /app/proto/*.proto 2>/dev/null \
    && echo "info: regenerated proto modules from /app/proto" \
    || echo "warn: proto regeneration failed, using the image copy"
export PYTHONPATH=/tmp/pb2:/app:${PYTHONPATH:-}
export ROS_MASTER_URI=${ROS_MASTER_URI:-http://localhost:11311}
ZMQ=tcp://127.0.0.1:5555

pass=0; fail=0
ok()   { echo "  PASS  $1"; pass=$((pass+1)); }
no()   { echo "  FAIL  $1"; fail=$((fail+1)); }
has_topic() { rostopic list 2>/dev/null | grep -qx "$1"; }
check_topic() { has_topic "$1" && ok "topic $1" || no "topic $1"; }
check_msg()   { timeout 5 rostopic echo -n1 "$1" >/dev/null 2>&1 && ok "message $1" || no "message $1"; }
check_hz()    { timeout 6 rostopic hz "$1" 2>&1 | grep -q "average rate" && ok "rate $1" || no "rate $1"; }
# tf2_echo is not shipped with ros-base, so inspect /tf directly.
check_tf()    { timeout 6 rostopic echo /tf 2>/dev/null | grep -qm1 "child_frame_id: \"$1\"" \
                    && ok "tf child $1" || no "tf child $1"; }

roscore >/tmp/roscore.log 2>&1 &
sleep 4
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom >/dev/null 2>&1 &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link >/dev/null 2>&1 &

python3 /app/pf_sim/dev_fake_zmq.py >/tmp/feed.log 2>&1 &
sleep 2
python3 /app/pf_sim/pose_tf.py _zmq_endpoint:=$ZMQ >/tmp/pose.log 2>&1 &
python3 /app/pf_sim/camera_bridge.py _zmq_endpoint:=$ZMQ >/tmp/camera.log 2>&1 &
python3 /app/pf_sim/sim_world.py >/tmp/world.log 2>&1 &
sleep 2
python3 /app/pf_sim/gt_pose.py >/tmp/gt.log 2>&1 &
python3 /app/pf_sim/target_bridge.py >/tmp/target.log 2>&1 &
sleep 5

echo "[topics]"
for topic in /pf_cam/image_raw /pf_cam/camera_info /odom /person_pose \
             /tf /map /move_base/local_costmap/costmap \
             /map_follow/localization_ready /pose_estimator/pose; do
    check_topic "$topic"
done

echo "[rates and payloads]"
check_hz /pf_cam/image_raw
check_hz /pose_estimator/pose
check_msg /pf_cam/camera_info
check_msg /person_pose
check_msg /map_follow/localization_ready

echo "[tf tree]"
check_tf base_footprint
check_tf pf_cam_optical

echo "[gt pose content]"
if timeout 5 rostopic echo -n1 /pose_estimator/pose 2>/dev/null | grep -q "part_id: 1"; then
    ok "neck body part present"
else
    no "neck body part present"
fi

echo
echo "$pass passed, $fail failed"
if [ "$fail" -ne 0 ]; then
    echo "--- node logs ---"
    for log in feed pose camera world gt target; do
        echo "## $log"; tail -n 5 "/tmp/$log.log" 2>/dev/null
    done
fi

pkill -f dev_fake_zmq.py; pkill -f pose_tf.py; pkill -f camera_bridge.py
pkill -f sim_world.py; pkill -f gt_pose.py; pkill -f target_bridge.py
pkill -f static_transform_publisher; pkill -f rosmaster; pkill -f rosout
[ "$fail" -eq 0 ]
