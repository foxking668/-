#!/bin/bash
# ============================================================
# VV15自动入库编译脚本（OpenCV自动探测、本地编译，避免HGFS并发问题）
#
#   ★ 本脚本在 Linux 本地目录 ~/builds/vv15_auto_parking 里编译，
#     而不是直接在 /mnt/hgfs/share/vv 里编译 ——
#     因为 VMware 共享目录（HGFS）扛不住 make -j 并发建目录，
#     会出现 "opening dependency file ... No such file or directory" 的错误。
#   编译完成后把 vv_auto_parking 拷回源码目录，并生成板端压缩包。
#
#   用法:  cd /mnt/hgfs/share/vv15_auto_parking && bash build.sh
# ============================================================
set -e

SRC_DIR="$(cd "$(dirname "$0")" && pwd)"
BUILD_DIR="$HOME/builds/vv15_auto_parking"

echo "[源目录] $SRC_DIR"

# ---------- ① 找 OpenCV 4.11 ----------
OPENCV_DIR=""
for d in \
    "${OpenCV_DIR:-}" \
    "$HOME/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4" \
    "$HOME/opencv-4.11.0/loongson/install/lib/cmake/opencv4" \
    "/home/tingxiao/opencv-4.11.0/loongson/install/lib/cmake/opencv4" \
    /home/*/opencv-4.11.0/loongson/install/lib/cmake/opencv4
do
    if [ -f "$d/OpenCVConfig.cmake" ]; then
        OPENCV_DIR="$d"
        break
    fi
done

if [ -z "$OPENCV_DIR" ]; then
    echo "找不到 OpenCV 4.11 的 OpenCVConfig.cmake"
    echo "已尝试:"
    echo "  OpenCV_DIR environment override"
    echo "  \$HOME/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
    echo "  \$HOME/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
    echo "  /home/tingxiao/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
    echo "  /home/*/opencv-4.11.0/loongson/install/lib/cmake/opencv4"
    echo "请修改 build.sh 里的路径。"
    exit 1
fi
echo "[OpenCV] $OPENCV_DIR"

# ---------- ② 复制源码到本地 ----------
echo "[复制] $SRC_DIR -> $BUILD_DIR"
case "$SRC_DIR/" in "$BUILD_DIR/"*) echo 'Source must be outside the build directory'; exit 1;; esac
rm -rf "$BUILD_DIR"
mkdir -p "$BUILD_DIR"
tar --exclude=./build --exclude=./build-validation --exclude=./captures --exclude=./experiment_logs \
    --exclude=./vv --exclude=./vvv --exclude=./vv_auto_parking --exclude=./.git --exclude='*.zip' \
    -cf - -C "$SRC_DIR" . | tar -xf - -C "$BUILD_DIR"

# ---------- ③ 本地编译 ----------
cd "$BUILD_DIR"
mkdir -p build
cd build
cmake .. -DOpenCV_DIR="$OPENCV_DIR"
make -j"$(nproc)"

# ---------- ④ 拷回源码目录 ----------
cd "$BUILD_DIR"
if [ ! -f build/vv_auto_parking ]; then
    echo "编译失败：没有生成 build/vv_auto_parking"
    exit 1
fi
cp -f build/vv_auto_parking "$SRC_DIR/vv_auto_parking"
chmod +x "$SRC_DIR/vv_auto_parking"

echo "===== 编译完成: $SRC_DIR/vv_auto_parking ====="
cd "$SRC_DIR"
python3 tools/package_board.py
