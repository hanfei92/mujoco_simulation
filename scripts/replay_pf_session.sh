#!/usr/bin/env bash
# Replay a recorded session through the observation manager ("数据回灌").
#
# The manager runs against the bag's TF/tracks/target/map/costmaps with a
# simulated clock, so two parameter sets can be compared on identical input.
# The MuJoCo sim is not involved: this is an open-loop regression run.
#
# Usage:
#   scripts/replay_pf_session.sh tmp/sim_sessions/sim-20261123-090000
#   RATE=2 TAG=w_view_2 EXTRA_ARGS="_observe_w_view:=2.0" scripts/replay_pf_session.sh <dir>
set -euo pipefail
cd "$(dirname "$0")/.."

SESSION="${1:-}"
RATE="${RATE:-1.0}"
TAG="${TAG:-replay-$(date +%Y%m%d-%H%M%S)}"
EXTRA_ARGS="${EXTRA_ARGS:-}"
PFWS="${PFWS:-/home/amigo/project/RND/person_following_ws}"
IMAGE="${IMAGE:-mujoco_simulation-pf-sim:latest}"

if [ -z "$SESSION" ] || [ ! -f "$SESSION/session.bag" ]; then
    echo "usage: $0 <session dir containing session.bag>" >&2
    exit 2
fi
SESSION=$(cd "$SESSION" && pwd)
mkdir -p "$SESSION/replay/$TAG"

echo "replay $SESSION/session.bag (rate=$RATE) -> $SESSION/replay/$TAG"
docker run --rm --network none --entrypoint bash \
    -v "$PFWS:/opt/person_following_ws:ro" \
    -v "$SESSION:/opt/session:rw" \
    -e PF_WS=/opt/person_following_ws \
    -e PF_MSGS_WS=/opt/pf_ws \
    -e SMOKE_DIR=/opt/session/replay \
    -e TAG="$TAG" -e RATE="$RATE" \
    -e EXTRA_ARGS="_map_frame:=world $EXTRA_ARGS" \
    -e ROS_MASTER_URI=http://127.0.0.1:11311 \
    -e ROS_HOSTNAME=localhost \
    -e ROS_IP=127.0.0.1 \
    "$IMAGE" -lc 'bash /opt/person_following_ws/tools/ros_smoke/replay_session.sh /opt/session/session.bag'

echo
echo "result: $SESSION/replay/$TAG"
echo "  python3 scripts/summarize_pf_session.py $SESSION/replay/$TAG"
