#!/usr/bin/env bash
# Record one person-following sim session: a rosbag of every input the manager
# consumes, plus the manager's own JSONL decision log.
#
# Usage:
#   scripts/record_pf_session.sh                      # 30 s, current containers
#   DURATION=60 RECORD_IMAGE=1 scripts/record_pf_session.sh
#   TAG=doorway CONTAINER=pf-sim scripts/record_pf_session.sh
set -euo pipefail
cd "$(dirname "$0")/.."

TAG="${TAG:-sim-$(date +%Y%m%d-%H%M%S)}"
DURATION="${DURATION:-30}"
RECORD_IMAGE="${RECORD_IMAGE:-0}"
CONTAINER="${CONTAINER:-pf-sim}"
OUT="${OUT:-$PWD/tmp/sim_sessions/$TAG}"

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
    echo "container '$CONTAINER' is not running" >&2
    exit 2
fi

TOPICS=(/tf /odom /map
        /move_base/local_costmap/costmap
        /move_base/global_costmap/costmap
        /map_follow/localization_ready
        /pose_estimator/pose
        /monocular_people_tracking/tracks
        /monocular_person_following/target
        /cmd_vel)
if [ "$RECORD_IMAGE" = "1" ]; then
    TOPICS+=(/pf_cam/image_raw /pf_cam/camera_info)
fi

mkdir -p "$OUT"
echo "recording ${DURATION}s -> $OUT"

docker exec "$CONTAINER" bash -lc "source /opt/ros/noetic/setup.bash; \
    source /opt/pf_ws/devel/setup.bash; mkdir -p /tmp/sim_sessions; \
    rosbag record -O /tmp/sim_sessions/${TAG}.bag ${TOPICS[*]}" \
    >/tmp/pf_record.log 2>&1 &
RECORDER=$!
sleep 2
sleep "$DURATION"
# rosbag needs SIGINT; SIGTERM leaves a truncated bag.
docker exec "$CONTAINER" bash -lc "pkill -INT -f 'rosbag record' || true" >/dev/null 2>&1 || true
wait "$RECORDER" || true
sleep 2

docker cp "$CONTAINER:/tmp/sim_sessions/${TAG}.bag" "$OUT/session.bag"
LATEST=$(docker exec "$CONTAINER" bash -lc \
    'ls -t /tmp/person_follow_observe/observe-*.jsonl 2>/dev/null | head -1' || true)
if [ -n "$LATEST" ]; then
    docker cp "$CONTAINER:$LATEST" "$OUT/observe.jsonl"
fi

echo "bag:     $OUT/session.bag ($(du -h "$OUT/session.bag" | cut -f1))"
[ -f "$OUT/observe.jsonl" ] && echo "log:     $OUT/observe.jsonl"
echo
echo "next:"
echo "  python3 scripts/summarize_pf_session.py $OUT"
echo "  scripts/replay_pf_session.sh $OUT"
