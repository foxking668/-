#!/bin/sh
set -eu
cd "$(dirname "$0")"
if [ ! -f ./vv_auto_parking ]; then
    printf '%s\n' 'Missing vv_auto_parking: compile with bash build.sh on your Linux build machine.' >&2
    exit 1
fi
chmod +x ./vv_auto_parking
printf '%s\n' 'VV15自动入库：start_mode=manual等待一次Enter直接巡线计时入库；race保持第二次斑马线停车后计时入库。'
printf '%s\n' '入库参数config/parking_tuning.vv.ini；巡线参数image_params.txt。沿用原舵机开关操作，结束先关舵机再Ctrl+C。'
exec ./vv_auto_parking "$@"
