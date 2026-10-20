#!/bin/sh
set -eu
cd "$(dirname "$0")"
[ -x ./vv_zebra_test ] || { echo 'Missing vv_zebra_test. Compile with bash build_zebra.sh first.' >&2; exit 2; }
[ -f ./config/zebra_test.ini ] && [ -f ./image_params.txt ] || { echo 'Missing tuning file or image_params.txt' >&2; exit 2; }
for library_dir in ./lib /home/root/opencv-4.11-loongarch/install/lib /home/root/qy/opencv411/lib /home/qy/opencv-4.11.0/loongson/install/lib /home/han/opencv-4.11.0/loongson/install/lib; do
  if [ -d "$library_dir" ]; then export LD_LIBRARY_PATH="$library_dir${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"; fi
done
if [ -f ./ZEBRA_BUILD_INFO.txt ]; then printf 'Package: '; cat ./ZEBRA_BUILD_INFO.txt; fi
echo 'VV17 zebra validation: lane -> RIGHT reverse brake -> stationary 3s -> lane. Logs: ./logs/'
echo 'Hardware failure rejects startup. After hardware success: automatic start in about 3s, no Enter required.'
echo 'Keep image_params.txt calibrated to this car. Ctrl+C stops motors before closing recordings.'
exec ./vv_zebra_test
