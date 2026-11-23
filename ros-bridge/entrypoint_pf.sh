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
python3 /app/pf_sim/sim_world.py &
python3 /app/pf_sim/person_walker.py &
python3 /app/pf_sim/cmd_vel_bridge.py &

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
