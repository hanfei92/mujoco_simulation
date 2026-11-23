#!/usr/bin/env bash
# Bring the whole person-following simulation up in the right order.
#
#   scripts/start_pf_sim.sh                 # viewer + route (default)
#   MUJOCO_VIEWER=0 scripts/start_pf_sim.sh # headless
#   MAP_REGEN=1 scripts/start_pf_sim.sh     # regenerate the occupancy map
#   PF_PERSON_MODE=circle scripts/start_pf_sim.sh
set -euo pipefail
cd "$(dirname "$0")/.."

VIEWER="${MUJOCO_VIEWER:-1}"
PERSON_MODE="${PF_PERSON_MODE:-route}"
MAP_FILE="tmp/pf_map.json"
SIM_LOGS="tmp/sim_logs"
PFWS="${PFWS:-/home/amigo/project/RND/person_following_ws}"

say() { printf '\n== %s ==\n' "$1"; }

say "1/6 docker network"
docker network create sim-net >/dev/null 2>&1 || true

say "2/6 mujoco wrapper (viewer=$VIEWER)"
if docker ps --format '{{.Names}}' | grep -qx mujoco-wrapper; then
    echo "already running"
else
    xhost +local:root >/dev/null 2>&1 || true
    docker rm -f mujoco-wrapper >/dev/null 2>&1 || true
    docker run -d --name mujoco-wrapper --network sim-net \
        --device nvidia.com/gpu=all \
        -e MUJOCO_GL=egl -e PYOPENGL_PLATFORM=egl \
        -e MUJOCO_VIEWER="$VIEWER" \
        -e DISPLAY="${DISPLAY:-:0}" -e QT_X11_NO_MITSHM=1 \
        -v /tmp/.X11-unix:/tmp/.X11-unix:rw \
        -v "${XAUTHORITY:-$HOME/.Xauthority}:/root/.Xauthority:rw" \
        -e ROBOT_MJCF_INCLUDES=/app/aidea_mujoco/mjcf/robots/g1/g1_12dof.xml \
        -e ROBOT_PLUGIN_CONFIGS=/app/aidea_mujoco/aidea_mujoco/plugins/g1/config.yaml \
        mujoco_simulation-mujoco-wrapper:latest >/dev/null
fi
scripts/wait_ready.sh --wrapper

say "3/6 occupancy map"
if [ "$PERSON_MODE" = "route" ]; then
    if [ "${MAP_REGEN:-0}" = "1" ] || [ ! -f "$MAP_FILE" ]; then
        docker exec mujoco-wrapper mkdir -p /app/tools
        docker cp tools/make_map_from_mjcf.py \
            mujoco-wrapper:/app/tools/make_map_from_mjcf.py >/dev/null
        docker exec mujoco-wrapper bash -lc \
            'cd /app && uv run python /app/tools/make_map_from_mjcf.py --out /tmp/pf_map.json' \
            | tail -1
        docker cp mujoco-wrapper:/tmp/pf_map.json "$MAP_FILE" >/dev/null
    fi
    echo "map: $MAP_FILE ($(du -h "$MAP_FILE" | cut -f1))"
else
    echo "skipped (person mode $PERSON_MODE does not need it)"
fi

say "4/6 pf-sim ($PERSON_MODE)"
mkdir -p "$SIM_LOGS"
docker rm -f pf-sim >/dev/null 2>&1 || true
docker run -d --name pf-sim --network sim-net \
    -e ZMQ_LIDAR_ENDPOINT=tcp://mujoco-wrapper:5555 \
    -e ZMQ_CMD_ENDPOINT=tcp://mujoco-wrapper:5556 \
    -e PF_MANAGER=1 \
    -e PF_TRACKER="${PF_TRACKER:-real}" \
    -e PF_PERSON_MODE="$PERSON_MODE" \
    -e PF_PERSON_TRACE="${PF_PERSON_TRACE:-1}" \
    -e PF_CAMERA_VIEW="${PF_CAMERA_VIEW:-1}" \
    -e PF_MAP_FILE=/tmp/pf_map.json \
    -e PF_LOG_DIR=/tmp/person_follow_observe \
    -p 8090:8090 \
    -v "$PFWS:/opt/person_following_ws:ro" \
    -v "$PWD/$SIM_LOGS:/tmp/person_follow_observe" \
    -v "$PWD/$MAP_FILE:/tmp/pf_map.json:ro" \
    -v "$PWD/tmp:/opt/pf_tmp" \
    -v "$PWD/scripts:/opt/pf_scripts:ro" \
    -v /var/run/docker.sock:/var/run/docker.sock \
    mujoco_simulation-pf-sim:latest >/dev/null

say "5/6 waiting for the tracker"
scripts/wait_ready.sh --bridge

say "6/6 ready"
cat <<EOF
camera view    http://127.0.0.1:8090/
mujoco window  $([ "$VIEWER" = "1" ] && echo "on DISPLAY=${DISPLAY:-:0}" || echo "disabled")
person mode    $PERSON_MODE
live table     docker exec pf-sim bash -lc 'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; python3 /app/pf_sim/dev_watch_loop.py 60 2'
heatmap        python3 scripts/coverage_heatmap.py $SIM_LOGS --map $MAP_FILE --trace $SIM_LOGS/person_trace.jsonl --out tmp/coverage.png
record         DURATION=60 TAG=run1 scripts/record_pf_session.sh
EOF
