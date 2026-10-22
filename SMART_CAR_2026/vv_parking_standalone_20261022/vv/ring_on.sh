#!/bin/sh
set -e
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
CFG="$ROOT/image_params.txt"
[ -f "$CFG" ] || { echo "找不到 $CFG" >&2; exit 1; }
if grep -q "^new_track_work_mode=" "$CFG"; then
    sed -i 's/^new_track_work_mode=.*/new_track_work_mode=1/' "$CFG"
else
    printf '\nnew_track_work_mode=1\n' >> "$CFG"
fi
if grep -q "^new_track_ring_enable=" "$CFG"; then
    sed -i 's/^new_track_ring_enable=.*/new_track_ring_enable=1/' "$CFG"
else
    printf '\nnew_track_ring_enable=1\n' >> "$CFG"
fi
echo '[OK] ring_on.sh: new_track_work_mode=1 new_track_ring_enable=1'
