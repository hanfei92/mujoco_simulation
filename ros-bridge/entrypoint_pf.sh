#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/pf_ws/devel/setup.bash
export PYTHONPATH=/app:${PYTHONPATH}

roscore &
sleep 3

# world -> odom -> base_footprint -> base_link -> g_base
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 world odom &
rosrun tf2_ros static_transform_publisher 0 0 0 0 0 0 base_footprint base_link &
rosrun tf2_ros static_transform_publisher -0.028185 -0.0016001 0.4765 1.57079632679 0 0 base_link g_base &

python3 /app/pf_sim/pose_tf.py &
python3 /app/pf_sim/camera_bridge.py &
python3 /app/pf_sim/sim_world.py &
python3 /app/pf_sim/cmd_vel_bridge.py &
python3 /app/pf_sim/person_walker.py &
sleep 2
python3 /app/pf_sim/gt_pose.py &
python3 /app/pf_sim/target_bridge.py &

# Optional: build and run the real tracking algorithm from the mounted source.
if [ "${PF_TRACKER:-fake}" = "real" ]; then
    if [ ! -x /opt/pf_ws/devel/lib/monocular_people_tracking/monocular_people_tracking_node ]; then
        echo "[pf-sim] building monocular_people_tracking"
        cp -r /opt/person_following_ws/src/monocular_people_tracking /opt/pf_ws/src/
        ( cd /opt/pf_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 ) \
            || echo "[pf-sim] tracker build failed"
    fi
    if [ -x /opt/pf_ws/devel/lib/monocular_people_tracking/monocular_people_tracking_node ]; then
        source /opt/pf_ws/devel/setup.bash
        rosrun monocular_people_tracking monocular_people_tracking_node \
            _detection_confidence_thresh:=0.1 \
            _association_maha_sq_thresh:=9.0 \
            _tracking_remove_trace_thresh:=5.0 &
        echo "[pf-sim] monocular_people_tracking started"
    fi
fi

if [ "${PF_MANAGER:-0}" = "1" ]; then
    export ROS_PACKAGE_PATH=/opt/person_following_ws/src:${ROS_PACKAGE_PATH}
    python3 /opt/person_following_ws/src/monocular_person_following/scripts/observation_manager_supervisor.py \
        _map_frame:=world _odom_frame:=odom _base_frame:=base_link _g_base_frame:=g_base \
        _enable_motion:=true _drive_mode:=differential \
        _costmap_topic:=/move_base/local_costmap/costmap \
        _observe_global_costmap_topic:=/move_base/global_costmap/costmap \
        _observe_static_map_topic:=/map \
        _observe_debug_log_dir:=/tmp/person_follow_observe \
        _observe_debug_markers:=false &
fi

wait
