#!/bin/sh
set -e
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
CFG="$ROOT/image_params.txt"
sed -i 's/^new_track_lane_method=.*/new_track_lane_method=4/' "$CFG"
echo '[OK] 最新7.25自动排期 FindLine/FixLine 模式'
