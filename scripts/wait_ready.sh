#!/usr/bin/env bash
# Wait until the sim and the bridge are actually usable.
#
#   scripts/wait_ready.sh              # wrapper MCP + pf-sim tracker
#   scripts/wait_ready.sh --wrapper    # only the wrapper
#
# A successful MCP tool call proves the simulation finished starting; the UDP
# log line is not reliable because prints are buffered.
set -euo pipefail
cd "$(dirname "$0")/.."

WAIT="${WAIT:-180}"
TARGET="${1:-both}"

wait_wrapper() {
    echo -n "waiting for mujoco-wrapper MCP "
    for _ in $(seq 1 "$WAIT"); do
        if docker exec mujoco-wrapper bash /app/mcp_call.sh get_g1_state '{}' \
                >/dev/null 2>&1; then
            echo " ok"
            return 0
        fi
        echo -n "."
        sleep 1
    done
    echo " TIMEOUT"
    return 1
}

wait_bridge() {
    echo -n "waiting for pf-sim tracker "
    for _ in $(seq 1 "$WAIT"); do
        if docker exec pf-sim bash -lc \
                'source /opt/ros/noetic/setup.bash; source /opt/pf_ws/devel/setup.bash; \
                 timeout 2 rostopic list' 2>/dev/null \
                | grep -qx /monocular_people_tracking/tracks; then
            echo " ok"
            return 0
        fi
        echo -n "."
        sleep 1
    done
    echo " TIMEOUT"
    return 1
}

case "$TARGET" in
    --wrapper) wait_wrapper ;;
    --bridge)  wait_bridge ;;
    *)         wait_wrapper && wait_bridge ;;
esac
