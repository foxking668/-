#!/bin/sh
set -e
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
CFG="$ROOT/image_params.txt"
if [ ! -f "$CFG" ]; then
    echo "[ERROR] 找不到 $CFG" >&2
    exit 1
fi
sed -i 's/^new_track_zebra_enable=.*/new_track_zebra_enable=0/' "$CFG"
echo '[OK] new_track_zebra_enable=0（纯巡线测试）'
