#!/bin/bash
# Compile/package helper only. The original main.cpp, build.sh and vvv are unchanged.
set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
OPENCV_DIR="${OPENCV_DIR:-/home/gy/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4}"
if [[ ! -f "$OPENCV_DIR/OpenCVConfig.cmake" ]]; then
    printf 'OpenCVConfig.cmake not found: %s\n' "$OPENCV_DIR" >&2; exit 1
fi
for required_tool in cmake zip sha256sum; do
    command -v "$required_tool" >/dev/null || { printf 'Missing command: %s\n' "$required_tool" >&2; exit 1; }
done
cmake -S "$ROOT" -B "$ROOT/build" -DOpenCV_DIR="$OPENCV_DIR"
cmake --build "$ROOT/build" --parallel "$(nproc)"
cp "$ROOT/build/vv" "$ROOT/vv"
chmod +x "$ROOT/vv" "$ROOT/vvv"
VERSION="$(sed -n 's/.*revision=\(VV_WINDOW_CONE_[A-Za-z0-9_]*\).*/\1/p' "$ROOT/src/NewTrack/NewTrack.cpp")"
[[ "$VERSION" == VV_WINDOW_CONE_20261015_4 ]] || { echo 'Unexpected or missing source revision' >&2; exit 1; }
PACKAGE_DIR="$(mktemp -d "$ROOT/build/package-window-cone.XXXXXX")"
cp "$ROOT/vv" "$ROOT/vvv" "$ROOT/cone_avoidance.ini" "$ROOT/apply_cone_config.sh" "$PACKAGE_DIR/"
cp "$ROOT/image_params.txt" "$PACKAGE_DIR/image_params.reference.txt"
cp "$ROOT"/ring_*.png "$PACKAGE_DIR/"
printf '%s\n' "$VERSION" > "$PACKAGE_DIR/VV_VERSION"
(
    cd "$PACKAGE_DIR"
    sha256sum vv vvv cone_avoidance.ini apply_cone_config.sh image_params.reference.txt ring_*.png VV_VERSION > VV_SHA256SUMS
    zip -q vv_window_cone_20261015_4.zip vv vvv cone_avoidance.ini apply_cone_config.sh image_params.reference.txt ring_*.png VV_VERSION VV_SHA256SUMS
)
mv "$PACKAGE_DIR/vv_window_cone_20261015_4.zip" "$ROOT/vv_window_cone_20261015_4.zip"
printf '\nBUILD_OK %s\nPackage: %s/vv_window_cone_20261015_4.zip\n' "$VERSION" "$ROOT"
