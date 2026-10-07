# 手推视觉观察模式：编译及运行

最新状态：`.2` 龙芯板后拉录像已收到，73 帧中后 49 帧连续跟踪。本次源码已更新为 `.3`，新增显式舵机响应试验，下一步按 `MANUAL_REVERSE_STEERING_PROBE.md` 编译和操作，不重复下面的观察采集。下面 `.2` 的编译/运行步骤保留作历史记录；`.3` 的 `--observe-steering` 仍保持无执行器写入，自动调舵仍未启用。

2026-10-07。新版manual_capture版本为2026-10-07.2。本模式记录建议，**不写舵机、电机或输出GPIO，不初始化IMU**；默认采集模式和既有显式固定舵机模式仍保留。观察参数与--steer-command互斥，包括命令0，不能在观察时另开设备控制程序。

它只将最初连续稳定、通过局部直线检查的线图像作为参考，记录之后的拟合图像横向差和远近位置差。拟合检查覆盖图像高度40%～90%，弯线或交点折弯超限时输出UNSUITABLE_REFERENCE，不建立目标、不输出建议。已建立目标遇到此状态会保留身份，恢复后重新连续确认，不能自动吸收新的偏移。该参考不是已测得的库轴、车身角度或后轴位置。suggested_command是诊断性舵机命令建议，不是实际前轮角度；不要照着数值手动执行转向。当前还没有自动调舵选项。

2026-10-07.1已在板上完成前推和后拉观察。后拉暴露了起始弯线参考与交点折弯问题，.2已用既有760帧回放验证并完成新增73帧板上后拉记录。已有录像不用重拍。不要为了取得建议而在弯线段不断重新启动；只有局部直线可靠时才建立诊断参考。这是直线观察限制，不是完整的分支身份识别或倒车入库控制。

## 1. 在虚拟机编译

共享源码仍在/mnt/hgfs/share/SMART_CAR_2026，沿用已有成功的LoongArch构建缓存：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
cmake -S . -B /home/gy/builds/SMART_CAR_2026-loongarch -DBUILD_VEHICLE=ON
cmake --build /home/gy/builds/SMART_CAR_2026-loongarch --target manual_capture --parallel 2
```

只有构建成功后复制，保留旧采集程序：

```sh
cp /home/gy/builds/SMART_CAR_2026-loongarch/manual_capture /mnt/hgfs/share/SMART_CAR_2026/deploy/manual_capture_observer_straight_20261007
```

将该新文件上传到板上/home/root/gy/deploy/。现有manual_capture.ini、config/calibration_hardware.ini和config/calibration_vehicle.ini继续使用，不需修改轮程、舵机比例、PID或运动标定标志。构建继续使用已有opencv_imgproc组件；若编译/链接失败，保留完整错误，不切换到虚拟机宿主架构的库。

当前电脑已完成Windows运行测试和Linux x86_64源码编译检查，**尚未构建或运行这个.2的LoongArch板上文件**。旧manual_capture_observer_20261007保留.1行为；旧manual_capture_steer_20261006没有观察选项。

## 2. 板上先核对版本与配置

```sh
cd /home/root/gy/deploy
chmod +x manual_capture_observer_straight_20261007
export LD_LIBRARY_PATH="/home/root/opencv-4.11-loongarch/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./manual_capture_observer_straight_20261007 --help
./manual_capture_observer_straight_20261007 --config manual_capture.ini --hardware-config config/calibration_hardware.ini --vehicle-config config/calibration_vehicle.ini --observe-steering reverse --duration 15 --allow-partial --check-config
```

帮助应显示version=2026-10-07.2和--observe-steering forward|reverse。--check-config不访问硬件；灰度通道未配置时会列出UNCONFIGURED，--allow-partial沿用原有部分采集方式，返回2不等同于崩溃。

## 3. 新版本只需一段局部直线后拉观察

前轮保持名义回正，车体大致平行清楚的纵向直线。让前相机近、中、远区域看到同一局部直线段，避开交点合并区和弯线；保留短距离后退空间。启动后保持不动，等REFERENCE_READY再缓慢向后拉一小段，不用原地扭车、横向强拉，不要求走满1米或完成入库。该短段用于核对.2的实时行为，无需重做此前七段录像。

```sh
./manual_capture_observer_straight_20261007 --config manual_capture.ini --hardware-config config/calibration_hardware.ini --vehicle-config config/calibration_vehicle.ini --observe-steering reverse --duration 15 --allow-partial --output captures/visual_observer_reverse_straight
```

显示OBSERVE ONLY和NOT applied表示仅观察。观察工具不改变当前前轮方向，也不会在结束时写回0，因为整个模式没有舵机输出。Q回车或Ctrl+C结束；原相机、编码器和超声波由同一锁与采集进程记录。

这次核对板上实时直线参考、识别状态和日志，不测角，不要求完成入库。出现UNSUITABLE_REFERENCE表示拟合支持不足、点异常或线形不适用于直线观察；出现LOW_QUALITY/STALE_FRAME等状态时同样撤销建议。停止推动并保留日志即可，不通过横向扶正强迫程序输出正常，也不要按建议自行打舵。

## 输出及判读

每个新会话在原camera.avi、frames.csv、sensors.csv、markers.csv、session.txt之外增加visual_observer.csv，并保存本次vehicle_config.ini。

| 字段/状态 | 含义 |
|---|---|
| reference_id | 本会话建立的图像参考编号；丢线恢复不自动改目标 |
| lateral_error_image | 相对起始直线参考的拟合归一化横向差，不是厘米 |
| heading_error_image | 拟合直线在高度45%与84%的位置差变化，不是实测车身角度 |
| suggestion_valid | 1才有建议；0时错误值和建议字段留空 |
| suggested_command | 未执行的诊断性命令建议，限制±5及3命令单位/秒 |
| motor_writes / servo_writes | 观察模式均为0 |
| WAIT_REFERENCE | 起始图像尚未连续稳定，先不要推 |
| TRACKING | 本帧通过观察模式的质量条件，可记录建议 |
| RECONFIRM_REFERENCE | 丢失后的连续确认，保留原目标但暂不给建议 |
| LOW_QUALITY / PARTIAL_REFERENCE | 评分不足或近中远行缺失；原因不能只靠该状态确定 |
| AMBIGUOUS / DISCONTINUOUS | 候选歧义或碎片混接，撤销建议 |
| UNSUITABLE_REFERENCE | 弯曲/折弯超出直线拟合残差限制，或拟合点异常/支持不足，撤销建议 |
| STALE_FRAME / FRAME_GAP | 处理延迟或新帧间隔超限，撤销建议 |

实时年龄从相机读取开始算到识别完成，包含读取、录像和处理耗时；仍不是已知的硬件曝光时间。缓存旧画面若没有硬件时间戳，无法仅靠这些时间完全识别。编码器与超声波本阶段只记录，不用未标定轮差计算车身角度，也不依赖无目标的超声波读数补足视觉。

完整源码检查、五项代码检查和760帧离线结果见visual_observer_straight_20261007/REPORT.md。
