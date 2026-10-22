#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
ARGS=(-DBUILD_TESTING=OFF)
BUILD="$ROOT/build-loongarch"
if [[ "${1:-}" == --native ]]; then
    BUILD="$ROOT/build-native"
    shift
else
    TOOLCHAIN_DIR="${TOOLCHAIN_DIR:-/opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6}"
    CC_PATH="$TOOLCHAIN_DIR/bin/loongarch64-linux-gnu-gcc"
    CXX_PATH="$TOOLCHAIN_DIR/bin/loongarch64-linux-gnu-g++"
    [[ -x "$CC_PATH" && -x "$CXX_PATH" ]] || { echo "找不到龙芯交叉编译器；设置 TOOLCHAIN_DIR，或在龙芯板上使用 --native。" >&2; exit 1; }
    ARGS+=(-DCMAKE_SYSTEM_NAME=Linux -DCMAKE_SYSTEM_PROCESSOR=loongarch64 "-DCMAKE_C_COMPILER=$CC_PATH" "-DCMAKE_CXX_COMPILER=$CXX_PATH")
fi
[[ $# == 0 ]] || { echo "Usage: OpenCV_DIR=/actual/path bash build_test.sh [--native]" >&2; exit 2; }
if [[ -n "${OpenCV_DIR:-}" ]]; then
    [[ -f "$OpenCV_DIR/OpenCVConfig.cmake" ]] || { echo "OpenCV_DIR 下没有 OpenCVConfig.cmake" >&2; exit 1; }
    ARGS+=("-DOpenCV_DIR=$OpenCV_DIR")
fi
cmake -S "$ROOT" -B "$BUILD" "${ARGS[@]}"
cmake --build "$BUILD" --parallel "${JOBS:-2}"
chmod +x "$ROOT/vv_parking_test"
echo "编译成功：$ROOT/vv_parking_test；运行 bash ./run_parking_test.sh"
