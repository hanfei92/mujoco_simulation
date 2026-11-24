#!/bin/bash
set -e

source /opt/ros/noetic/setup.bash
source /opt/pf_ws/devel/setup.bash
export PYTHONPATH=/app:${PYTHONPATH}

roscore &
sleep 3

# The fixed frames (world->odom, base_footprint->base_link, base_link->g_base)
# are re-broadcast on /tf by pose_tf.py: the tracker uses tf1, which does not
# consume tf2's /tf_static, and three static publishers would collide on the
# same generated node name anyway.

python3 /app/pf_sim/pose_tf.py &
python3 /app/pf_sim/lidar_bridge.py &
python3 /app/pf_sim/camera_bridge.py &
python3 /app/pf_sim/sim_world.py &
python3 /app/pf_sim/cmd_vel_bridge.py \
    _angular_gain:=${PF_ANGULAR_GAIN:-2.5} \
    _linear_gain:=${PF_LINEAR_GAIN:-1.0} &
if [ "${PF_PERSON_MODE:-route}" = "route" ]; then
    python3 /app/pf_sim/person_route.py \
        _map_file:=${PF_MAP_FILE:-/tmp/pf_map.json} \
        _lane_spacing:=${PF_ROUTE_LANE:-1.2} \
        _start_delay:=${PF_ROUTE_DELAY:-25.0} &
else
    python3 /app/pf_sim/person_walker.py _mode:=${PF_PERSON_MODE:-route} \
        _vx:=${PF_PERSON_VX:-0.4} _wz:=${PF_PERSON_WZ:-0.25} \
        _start_delay:=${PF_WALK_DELAY:-25.0} \
        _map_file:=${PF_MAP_FILE:-/tmp/pf_map.json} \
        _orbit_radius:=${PF_ORBIT_RADIUS:-3.0} \
        _orbit_speed:=${PF_ORBIT_SPEED:-0.35} &
fi
sleep 2
python3 /app/pf_sim/gt_pose.py &
python3 /app/pf_sim/target_bridge.py &

if [ "${PF_PERSON_TRACE:-0}" = "1" ]; then
    # ground-truth person trace, used by scripts/coverage_heatmap.py
    python3 /app/pf_sim/dev_person_trace.py \
        _out:=${PF_LOG_DIR:-/tmp/person_follow_observe}/person_trace.jsonl &
fi

if [ "${PF_DASHBOARD:-1}" = "1" ]; then
    python3 /app/pf_sim/dashboard.py \
        _port:=${PF_CAMERA_PORT:-8090} \
        _sessions_dir:=${PF_SESSIONS_DIR:-/opt/pf_tmp/sim_sessions} \
        _scripts_dir:=${PF_SCRIPTS_DIR:-/opt/pf_scripts} &
    if [ "${PF_TIMELINE:-1}" = "1" ]; then
        python3 /opt/person_following_ws/tools/ros_smoke/web_view.py \
            --dir "${PF_LOG_DIR:-/tmp/person_follow_observe}" \
            --host 0.0.0.0 --port "${PF_TIMELINE_PORT:-8781}" >/tmp/timeline.log 2>&1 &
    fi
elif [ "${PF_CAMERA_VIEW:-0}" = "1" ]; then
    python3 /app/pf_sim/camera_view.py _port:=${PF_CAMERA_PORT:-8090} &
fi

# Optional: build and run the real tracking algorithm from the mounted source.
if [ "${PF_TRACKER:-fake}" = "real" ]; then
    if [ ! -x /opt/pf_ws/devel/lib/monocular_people_tracking/monocular_people_tracking_node ]; then
        echo "[pf-sim] building monocular_people_tracking"
        cp -r /opt/person_following_ws/src/monocular_people_tracking /opt/pf_ws/src/
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

        ( cd /opt/pf_ws && catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python3 ) \
            || echo "[pf-sim] tracker build failed"
    fi
    if [ -x /opt/pf_ws/devel/lib/monocular_people_tracking/monocular_people_tracking_node ]; then
        source /opt/pf_ws/devel/setup.bash
        rosrun monocular_people_tracking monocular_people_tracking_node \
            camera_info:=/pf_cam/camera_info \
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
        _observe_debug_log_dir:=${PF_LOG_DIR:-/tmp/person_follow_observe} \
        _observe_debug_markers:=false \
        _observe_turn_max_yaw:=${PF_TURN_MAX_YAW:-0.8} \
        _observe_min_move_interval:=${PF_MIN_MOVE_INTERVAL:-1.5} \
        _observe_reposition_confirm_cycles:=${PF_REPOSITION_CONFIRM:-1} \
        _observe_max_repositions_per_episode:=${PF_MAX_REPOSITIONS:-6} &
fi

wait
