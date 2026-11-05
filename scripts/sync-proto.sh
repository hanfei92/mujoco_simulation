#!/bin/bash
# scripts/sync-proto.sh
set -e
cp wrapper/proto/*.proto ros-bridge/proto/
echo "proto synced from wrapper to ros-bridge"