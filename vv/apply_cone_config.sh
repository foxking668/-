#!/bin/sh
# One-time parameter migration; never starts the vehicle or replaces field tuning.
set -eu
if [ "$#" -gt 1 ]; then echo 'Usage: sh apply_cone_config.sh [image_params.txt]' >&2; exit 2; fi
parameter_file="${1:-./image_params.txt}"
if [ -e "$parameter_file" ] && [ ! -f "$parameter_file" ]; then
    echo "Parameter path is not a regular file: $parameter_file" >&2; exit 1
fi
if [ ! -f "$parameter_file" ]; then
    reference="$(dirname "$0")/image_params.reference.txt"
    if [ ! -f "$reference" ]; then echo "Missing parameters and reference: $parameter_file" >&2; exit 1; fi
    cp "$reference" "$parameter_file"
fi
backup="$(mktemp "${parameter_file}.before_cone_window.XXXXXX")"
cp -p "$parameter_file" "$backup"
temporary="$(mktemp "${parameter_file}.tmp.XXXXXX")"
trap 'rm -f "$temporary"' EXIT HUP INT TERM
awk '
BEGIN {
    n=split("new_track_cone_sequence_enable new_track_blue_cone_enable new_track_ring_side_enable new_track_ring_entry_cue_enable",keys," ")
    split("1 0 0 0",numbers," ")
    for(i=1;i<=n;i++) values[keys[i]]=numbers[i]
}
{
    if($0 ~ /^# VV_(FIXED|WINDOW)_CONE_[^:]*:/) next
    key=$0;sub(/#.*/,"",key);sub(/=.*/,"",key)
    gsub(/^[ \t\r]+|[ \t\r]+$/,"",key)
    if(key ~ /^new_track_cone_(enable|detection_mode|gray_.*)$/) next
    if(!(key in values)) print $0
}
END {
    print "# VV_WINDOW_CONE_20261015_4: window entry + gray visual pass L/R/L/R; two ring parking branches off"
    for(i=1;i<=n;i++) print keys[i] "=" values[keys[i]]
}' "$parameter_file" > "$temporary"
mv "$temporary" "$parameter_file"
trap - EXIT HUP INT TERM
printf 'CONFIG_OK WINDOW_GRAY_PASS ring_side_stop=0 ring_entry_stop=0\nBackup: %s\n' "$backup"
