#!/usr/bin/env bash
# Run the person-following test suite against the running sim.
#
#   scripts/run_sim_tests.sh          # unit + live smoke (places the person first)
#   SKIP_UNIT=1 scripts/run_sim_tests.sh
set -uo pipefail
cd "$(dirname "$0")/.."

PFWS="${PFWS:-/home/amigo/project/RND/person_following_ws}"
PASS=0
FAIL=0

say() { printf '\n== %s ==\n' "$1"; }

say "offline unit tests (observation manager)"
if [ "${SKIP_UNIT:-0}" != "1" ]; then
    if ( cd "$PFWS/src/monocular_person_following" && \
         python3 -m pytest tests/test_observation_manager.py -q 2>&1 | tail -2 ); then
        PASS=$((PASS+1))
    else
        FAIL=$((FAIL+1))
    fi
fi

say "freeze the person and place it in front"
# route mode keeps moving the person, which would race the placement
docker exec pf-sim bash -lc 'pkill -f person_route.py || true' >/dev/null 2>&1
sleep 1
FRONT=$(docker exec -i pf-sim bash -c 'source /opt/ros/noetic/setup.bash; python3 - <<PY
import math, rospy
from nav_msgs.msg import Odometry
rospy.init_node("test_place", anonymous=True)
m = rospy.wait_for_message("/odom", Odometry, 5)
q = m.pose.pose.orientation
th = math.atan2(2*(q.w*q.z), 1-2*q.z*q.z)
print(round(m.pose.pose.position.x + 2.0*math.cos(th), 2),
      round(m.pose.pose.position.y + 2.0*math.sin(th), 2))
PY' 2>/dev/null)
FX=$(echo "$FRONT" | cut -d' ' -f1)
FY=$(echo "$FRONT" | cut -d' ' -f2)
if [ -n "${FX:-}" ]; then
    docker exec mujoco-wrapper bash /app/mcp_call.sh \
        move_g1_to "{\"x\":$FX,\"y\":$FY,\"yaw\":0}" >/dev/null 2>&1
    echo "person -> ($FX, $FY)"
    sleep 5
fi

say "live smoke test"
if docker exec pf-sim bash -lc \
        'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; \
         python3 /app/pf_sim/sim_smoke_test.py' 2>&1 | tail -16; then
    PASS=$((PASS+1))
else
    FAIL=$((FAIL+1))
fi

say "summary"
echo "suites passed: $PASS, failed: $FAIL"
echo "details: tmp/sim_smoke_result.json (inside the container: /opt/pf_tmp/sim_smoke_result.json)"
[ "$FAIL" -eq 0 ]
