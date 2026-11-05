# #!/bin/bash
# # scripts/sync-proto.sh
# set -e
# cp wrapper/proto/*.proto ros-bridge/proto/
# echo "proto synced from wrapper to ros-bridge"

set -e
unalias cp 2>/dev/null || true

SRC_DIR="wrapper/proto"
for DST_DIR in "ros-bridge/proto" "ros2-bridge/proto"; do
    mkdir -p "$DST_DIR"
    for f in "$SRC_DIR"/*.proto; do
        name=$(basename "$f")
        if [ -d "$DST_DIR/$name" ]; then
            rm -rf "$DST_DIR/$name"
        fi
        /bin/cp -f "$f" "$DST_DIR/$name"
    done
done
echo "proto synced to ros-bridge/proto and ros2-bridge/proto"