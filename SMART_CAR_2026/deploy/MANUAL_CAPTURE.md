# 手推倒车同步采集

采集工具 `manual_capture` 独立于比赛控制程序。它不创建 `FactoryBoard`、不初始化 IMU、不写电机/PWM/舵机/语音或 GPIO 输出，不会自动让小车行驶。它不会关闭其他进程已经打开的电机输出，因此运行前应自行停止车辆控制程序，并让驱动轮能够正常滚动。

它使用现有车辆互斥锁，不能与比赛程序或 `hardware_servo_test` 同时运行。编码器增量读取可能清零驱动计数，所以不能让其他程序同时读取编码器。未使用同一锁的旧程序也需要先停掉。

## 编译和传输

若继续使用原先文档中的虚拟机共享目录和已经配置好的龙芯交叉构建目录，在虚拟机执行：

```bash
cd /mnt/hgfs/share/SMART_CAR_2026
cmake -S . -B /home/gy/builds/SMART_CAR_2026-loongarch -DBUILD_VEHICLE=ON
cmake --build /home/gy/builds/SMART_CAR_2026-loongarch --target manual_capture --parallel 2
mkdir -p deploy/config
cp /home/gy/builds/SMART_CAR_2026-loongarch/manual_capture deploy/
cp config/hardware.ini deploy/config/hardware.ini
```

这是沿用已经配置好的构建目录，目标 OpenCV、交叉编译器和 sysroot 继续使用它的缓存配置。不要改用宿主机 x86 OpenCV。GNU 8 的 filesystem 库已在 CMake 中单独链接。不应在 x86 虚拟机运行这个龙芯二进制。

通过共享文件夹和 MobaXterm SFTP，将以下文件上传到小车的 `/home/root/gy/deploy/`（如果实际目录不同，替换为实际路径）：

- `manual_capture`
- `manual_capture.ini`
- `collect_hardware_info.sh`
- `config/hardware.ini`，保留 config 子目录

## 接口核实：目前尚缺灰度和超声波

已知接口来自工程硬件配置：摄像头 `/dev/video0`，编码器 `/dev/zf_encoder_1`、`/dev/zf_encoder_2`。它们的实际读返回和有效性仍要上车验证，不能把“路径存在”当作“数据正确”。

照片不能确定四路灰度及前后超声波的引脚、总线协议和驱动节点。配置中这六路暂时留空。先在小车执行：

```bash
cd /home/root/gy/deploy
sh collect_hardware_info.sh
```

该脚本只列设备信息、查询摄像头格式及依赖，不扫描 I2C 地址、不读编码器、不写硬件输出。它把结果放在 `captures/hardware_info_*.txt`。将结果及传感器接线/例程提供后，才能确定驱动。摄像头使用 C++ OpenCV，采集工具不依赖 Python。

当前读取适配器仅支持已存在的传感器输入节点：`u8` 原始单字节、`i16le` 有符号双字节、`text` 单个数字文本。它不实现未知 UART、I2C、HC-SR04 触发回响协议。不能把总线设备直接填入数字文本配置。

灰度编号应按车体左到右确定，并用黑/白表面实测原始值。超声波 `_unit` 必须填写驱动真实输出单位；工具不会猜测是毫米、厘米或回波时间。

## 启动前先做 5 秒采集检查

暂时只验证视频和编码器时，显式使用 `--allow-partial`。这不是八路传感器全部就绪的采集：

```bash
cd /home/root/gy/deploy
chmod +x manual_capture
./manual_capture --allow-partial --duration 5
```

不加 `--allow-partial`，缺少接口时会在打开硬件前报错。若默认摄像头格式不被设备支持，按设备信息报告更改分辨率/FPS。

核实摄像头有视频，左右编码器在轻轻推车时读数变化；同时查看 `sensors.csv` 的 `valid/status/read_return`。某些已有驱动出现过返回值为零的问题：即使缓冲区发生改变，工具仍标记为无效并保留原始字节，不会自动放宽驱动协议。

## 正式手推

仅当灰度和超声波接口已接入且检查有效后，启动完整采集：

```bash
cd /home/root/gy/deploy
./manual_capture --duration 180
```

每次运行建立独立目录 `deploy/captures/manual_<UTC时间>_<进程号>_<唯一标记>/`，不覆盖上次记录。

先保持车辆静止几秒，再按正常车轮滚动方式推车。可由另一人在 MobaXterm 输入字母后回车添加节点：

- A：起倒点
- B：开始回正
- C：回正结束
- D：车头灰度到达库口
- E：最终停位

输入 Q 回车或 Ctrl+C 结束。录像中断或输出存储不足会停止采集并保留文件。硬件驱动若无视非阻塞读取而卡住，退出可能延迟；目标板行为仍需实际验证。

## 输出与同步

- `camera.avi`：MJPG 摄像头视频，保存实际读到的分辨率。
- `frames.csv`：从零开始的帧号、摄像头读取开始/结束时刻、相对开始时间。
- `sensors.csv`：每个传感器的读取开始/结束时刻、原始值、声明单位、有效性、错误、系统调用返回值和原始字节。
- `markers.csv`：A～E 人工标记与时间。
- `session.txt`：时间原点、配置来源、是否缺少通道、读数有效/无效数量、录制异常。
- 两份 `.ini` 快照：保留本次采集使用的配置。

所有时间使用同一进程的单调时钟。视频帧时间是主机读取区间，不是已知曝光时间；传感器也按各自读取区间记录，不能声称严格同时采样。分析应按 CSV 时间匹配，不应只拿“帧号/FPS”当真实经过时间。

编码器先保存原始值，不依赖未标定的轮径、PPR 或方向符号。delta 模式第一次读取包含启动前未知时间的计数，换算里程/速度时应忽略第一次。这里没有真实舵机角度传感器，也没有 IMU，所以日志不会伪造转向角或航向。

退出码：0 表示录像完成且全部通道所有样本有效；2 表示文件已保存但通道不齐或有无效读数；1 表示配置、录像或文件错误。不要把退出码 2 的记录当作完整八路同步数据。
