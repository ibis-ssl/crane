#!/bin/bash
# consai_ros2/robocup_ssl_msgs/proto/ から必要な proto ファイルを ball-calibration/proto/ にコピー
# Dockerfileのビルド前に実行すること

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
BALL_CAL_DIR="$SCRIPT_DIR/.."
CRANE_ROOT="$BALL_CAL_DIR/../../.."
PROTO_SRC="$CRANE_ROOT/consai_ros2/robocup_ssl_msgs/proto"
PROTO_DST="$BALL_CAL_DIR/proto"

if [ ! -d "$PROTO_SRC" ]; then
    echo "Error: proto source not found: $PROTO_SRC" >&2
    exit 1
fi

mkdir -p "$PROTO_DST"

PROTOS=(
    ssl_vision_wrapper_tracked.proto
    ssl_vision_detection_tracked.proto
    ssl_gc_common.proto
    ssl_gc_geometry.proto
    ssl_vision_wrapper.proto
    ssl_vision_detection.proto
    ssl_vision_geometry.proto
)

for f in "${PROTOS[@]}"; do
    cp "$PROTO_SRC/$f" "$PROTO_DST/"
    echo "Copied: $f"
done

echo "Proto files synced to $PROTO_DST"
