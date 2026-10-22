#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "$0")" && pwd)"
cd "$ROOT"
[[ -f image_params.txt ]] || { echo "缺少 image_params.txt；不会自动覆盖为默认模板。" >&2; exit 1; }
[[ -x ./vv_parking_test ]] || { echo "请先编译本独立入口 vv_parking_test；原比赛程序 vv 不能代替。" >&2; exit 1; }
# Keep the lock across exec; use one instance of this entry at a time.
command -v flock >/dev/null || { echo "需要 flock（util-linux）。" >&2; exit 1; }
exec 9>.parking_test.lock
flock -n 9 || { echo "本目录已有独立测试程序运行。" >&2; exit 1; }
exec ./vv_parking_test "$@"
