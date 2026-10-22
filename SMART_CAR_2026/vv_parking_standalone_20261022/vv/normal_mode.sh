#!/bin/sh
set -e
ROOT="$(CDPATH= cd -- "$(dirname -- "$0")" && pwd)"
CFG="$ROOT/image_params.txt"
[ -f "$CFG" ] || { echo "找不到 $CFG" >&2; exit 1; }
if grep -q "^new_track_work_mode=" "$CFG"; then
    sed -i 's/^new_track_work_mode=.*/new_track_work_mode=0/' "$CFG"
else
    printf '\nnew_track_work_mode=0\n' >> "$CFG"
fi
echo '[OK] normal_mode.sh: new_track_work_mode=0'
