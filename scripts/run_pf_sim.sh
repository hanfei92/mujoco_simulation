#!/usr/bin/env bash
# Start the MuJoCo wrapper plus the person-following bridge.
set -euo pipefail
cd "$(dirname "$0")/.."
docker compose -f docker-compose.yml -f docker-compose.pf-sim.yml \
    --profile pf up -d --build mujoco-wrapper pf-sim
docker compose -f docker-compose.yml -f docker-compose.pf-sim.yml \
    --profile pf logs -f pf-sim
