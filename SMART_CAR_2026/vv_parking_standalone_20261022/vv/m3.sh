#!/bin/sh
set -e
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
CFG="$ROOT/image_params.txt"
sed -i 's/^new_track_lane_method=.*/new_track_lane_method=3/' "$CFG"
echo '[OK] M3 双边界模式'
