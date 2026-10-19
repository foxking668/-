#!/bin/bash
set -euo pipefail
cd "$(dirname "$0")"
args=()
build_dir=build-zebra-cross
if [[ "${ZEBRA_NATIVE:-0}" == 1 ]]; then args+=(-DZEBRA_NATIVE=ON); build_dir=build-zebra-native; fi
if [[ -n "${TOOLCHAIN_DIR:-}" ]]; then args+=("-DTOOLCHAIN_DIR=$TOOLCHAIN_DIR"); fi
if [[ -n "${OPENCV_DIR:-}" ]]; then args+=("-DOpenCV_DIR=$OPENCV_DIR"); fi
if [[ -z "${OPENCV_DIR:-}" ]]; then
  for candidate in /home/superteen/opencv-4.11.0/loongson/install/lib/cmake/opencv4 /home/han/opencv-4.11.0/loongson/install/lib/cmake/opencv4 /home/qy/opencv-4.11.0/loongson/install/lib/cmake/opencv4 "/home/gy/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4"; do
    if [[ -f "$candidate/OpenCVConfig.cmake" ]]; then args+=("-DOpenCV_DIR=$candidate"); break; fi
  done
fi
cmake -S . -B "$build_dir" "${args[@]}"
cmake --build "$build_dir" --target vv_zebra_test zebra_unit_tests -j"${JOBS:-2}"
cp "$build_dir/vv_zebra_test" ./vv_zebra_test
chmod +x ./vv_zebra_test ./run_zebra_test.sh
printf '\nBuilt vv_zebra_test. Run bash run_zebra_test.sh on the car.\n'
