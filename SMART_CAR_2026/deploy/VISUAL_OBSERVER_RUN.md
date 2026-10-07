# 手推视觉观察模式：编译及第一次运行

2026-10-07。新版manual_capture版本为2026-10-07.1。本模式记录建议，**不写舵机、电机或输出GPIO，不初始化IMU**；默认采集模式和既有显式固定舵机模式仍保留。观察参数与--steer-command互斥，包括命令0，不能在观察时另开设备控制程序。

它将最初连续稳定的线图像作为参考，记录之后的图像横向差和远近位置差。该参考不是已测得的库轴、车身角度或后轴位置。suggested_command是诊断性舵机命令建议，不是实际前轮角度；不要照着数值手动执行转向。当前还没有自动调舵选项。

## 1. 在虚拟机编译

共享源码仍在/mnt/hgfs/share/SMART_CAR_2026，沿用已有成功的LoongArch构建缓存：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
cmake -S . -B /home/gy/builds/SMART_CAR_2026-loongarch -DBUILD_VEHICLE=ON
cmake --build /home/gy/builds/SMART_CAR_2026-loongarch --target manual_capture --parallel 2
```

只有构建成功后复制，保留旧采集程序：

```sh
cp /home/gy/builds/SMART_CAR_2026-loongarch/manual_capture /mnt/hgfs/share/SMART_CAR_2026/deploy/manual_capture_observer_20261007
```

将该新文件上传到板上/home/root/gy/deploy/。现有manual_capture.ini、config/calibration_hardware.ini和config/calibration_vehicle.ini继续使用，不需修改轮程、舵机比例、PID或运动标定标志。本次新增opencv_imgproc链接，项目原本已要求这一OpenCV组件；若编译/链接失败，保留完整错误，不切换到虚拟机宿主架构的库。

当前电脑已完成Windows运行测试和Linux x86_64源码编译检查，**尚未构建或运行这个新的LoongArch板上文件**。旧manual_capture_steer_20261006不会自动具有观察选项。

## 2. 板上先核对版本与配置

```sh
cd /home/root/gy/deploy
chmod +x manual_capture_observer_20261007
export LD_LIBRARY_PATH="/home/root/opencv-4.11-loongarch/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./manual_capture_observer_20261007 --help
./manual_capture_observer_20261007 --config manual_capture.ini --hardware-config config/calibration_hardware.ini --vehicle-config config/calibration_vehicle.ini --observe-steering forward --duration 15 --allow-partial --check-config
```

帮助应显示version=2026-10-07.1和--observe-steering forward|reverse。--check-config不访问硬件；灰度通道未配置时会列出UNCONFIGURED，--allow-partial沿用原有部分采集方式，返回2不等同于崩溃。

## 3. 第一段只做前推观察

前轮保持原来的回正状态，让相机看到同一条清楚的黑线。车体先大致平行参考线，周围留有短距离移动空间。启动后先保持画面稳定约1秒，等REFERENCE_READY，再缓慢向前推一小段；不用原地扭车、横向强拉，也不用重做五段固定舵机采集。

```sh
./manual_capture_observer_20261007 --config manual_capture.ini --hardware-config config/calibration_hardware.ini --vehicle-config config/calibration_vehicle.ini --observe-steering forward --duration 15 --allow-partial --output captures/visual_observer_forward
```

显示OBSERVE ONLY和NOT applied表示仅观察。观察工具不改变当前前轮方向，也不会在结束时写回0，因为整个模式没有舵机输出。Q回车或Ctrl+C结束；原相机、编码器和超声波由同一锁与采集进程记录。

这次目的首先是验证板上实时帧时序、识别状态和新增日志，不要求推满1米，不测角，不要求本段完成入库。等待参考始终失败或出现LOW_QUALITY/STALE_FRAME时，可以停推保留日志，不需要横向扶正来强迫程序输出正常。先保留这一段结果，再决定是否安排后推观察；不要重复采集已完成的固定档位录像。

## 输出及判读

每个新会话在原camera.avi、frames.csv、sensors.csv、markers.csv、session.txt之外增加visual_observer.csv，并保存本次vehicle_config.ini。

| 字段/状态 | 含义 |
|---|---|
| reference_id | 本会话建立的图像参考编号；丢线恢复不自动改目标 |
| lateral_error_image | 相对起始参考的归一化图像横向差，不是厘米 |
| heading_error_image | 相对起始参考的远近位置差变化，不是实测车身角度 |
| suggestion_valid | 1才有建议；0时错误值和建议字段留空 |
| suggested_command | 未执行的诊断性命令建议，限制±5及3命令单位/秒 |
| motor_writes / servo_writes | 观察模式均为0 |
| WAIT_REFERENCE | 起始图像尚未连续稳定，先不要推 |
| TRACKING | 本帧通过观察模式的质量条件，可记录建议 |
| RECONFIRM_REFERENCE | 丢失后的连续确认，保留原目标但暂不给建议 |
| LOW_QUALITY / PARTIAL_REFERENCE | 评分不足或近中远行缺失；原因不能只靠该状态确定 |
| AMBIGUOUS / DISCONTINUOUS | 候选歧义或碎片混接，撤销建议 |
| STALE_FRAME / FRAME_GAP | 处理延迟或新帧间隔超限，撤销建议 |

实时年龄从相机读取开始算到识别完成，包含读取、录像和处理耗时；仍不是已知的硬件曝光时间。缓存旧画面若没有硬件时间戳，无法仅靠这些时间完全识别。编码器与超声波本阶段只记录，不用未标定轮差计算车身角度，也不依赖无目标的超声波读数补足视觉。

完整源码检查与615帧离线结果见visual_observer_20261007/REPORT.md。
