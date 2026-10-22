#!/bin/bash
set -e
cd "$(dirname "$0")"
rm -rf build
mkdir build
cd build

OPENCV_DIR="/home/superteen/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
if [ ! -f "$OPENCV_DIR/OpenCVConfig.cmake" ]; then
    OPENCV_DIR="/home/han/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
fi
if [ ! -f "$OPENCV_DIR/OpenCVConfig.cmake" ]; then
    echo "找不到 OpenCV 4.11 的 OpenCVConfig.cmake"
    echo "请检查 /home/superteen/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
    exit 1
fi

cmake .. -DOpenCV_DIR="$OPENCV_DIR"
make -j"$(nproc)"
ROOT_DIR="$(cd .. && pwd)"
mv -f vv "$ROOT_DIR/vv"
cd "$ROOT_DIR"
rm -rf build
chmod +x "$ROOT_DIR/vv" "$ROOT_DIR/vvv"
printf '\n编译完成：%s/vv；./vvv 开启处理视频录像（共用一个二进制）。\n' "$ROOT_DIR"
