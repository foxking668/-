# 出厂硬件接口适配说明

本次修改对象：`C:/Users/巩钰/Desktop/2026龙芯/SMART_CAR_2026`。协议依据为用户提供的微信附件 `3.出厂源码/example/example/visual_line_follow` 中的 motor.h/motor.cpp、zf_driver_file.h、zf_driver_pwm.h、zf_driver_gpio.cpp、zf_driver_encoder.cpp 和 zf_device_imu_core.cpp。附件中的说明仅作为接口资料；未执行附件程序或脚本。

## 结构与接口

上层视觉/比赛任务 → Drive（双轮速度 PID）→ FactoryBoard / HardwareImu → LinuxDeviceIo → 出厂设备节点。所有设备路径和物理方向集中在 `config/hardware.ini`，任务/速度/标定参数仍在 `config/competition.ini`。

| 功能 | 默认接口 | 协议及处理 |
|---|---|---|
| 左右电机 PWM | /dev/zf_device_pwm_motor_1、motor_2 | 读取 24 字节 PWM 信息，写入 uint16 占空比 |
| 左右方向 | /dev/zf_driver_gpio_motor_1、motor_2 | 写 ASCII 0/1，默认高电平正转，换向前清零 PWM |
| 舵机 | /dev/zf_device_pwm_servo | 从设备频率和 duty_max 换算固定微秒脉宽 |
| 左右编码器 | /dev/zf_encoder_1、zf_encoder_2 | int16 计数，真实 dt 转 RPS；计数直接转里程 |
| 摄像头 | /dev/video0 | OpenCV 的 Linux V4L2 后端，设备路径可配置 |
| 板载按键 | /dev/zf_driver_gpio_key_0..3 | 接受二进制或 ASCII 0/1，可指定停车按键 |
| 拨码 | /dev/zf_driver_gpio_switch_0..1 | 同上，可指定停车拨码 |
| 蜂鸣器 | /dev/zf_driver_gpio_beep | ASCII 0/1，故障时鸣叫，退出时关闭 |
| IIO IMU | /sys/bus/iio/devices/iio:device1 | 支持附件列出的三个型号，静止校准后积分选定轴角速度 |
| 串口 IMU | /dev/ttyS2 | 可选 serial 后端，继续支持 WIT11/ASCII 协议 |
| 语音 | /dev/i2c-2，地址 52（0x34） | 附件没有语音协议，保留原车 WonderEcho 0x6e / FF 11 协议，路径和地址可配置 |

出厂包还包含其他未被本比赛程序调用的库（例如屏幕、ADC、SPI 和独立 TCP 图传示例）。本次没有把这些示例添加成比赛程序的新功能；所列接口覆盖当前车控使用的硬件及新增板载输入/蜂鸣器。MyLoong 图传工具不是本程序的车控驱动，本次没有实现其图传服务。

## 单位和标定变化

1. `motor_command_range=50000` 保留原控制量尺度。设备 duty_max 为 10000 时，控制量 12000 对应 duty=2400，即 24%；PID 参数没有直接扩大五倍，输出继续受 `pwm_limit` 限制。
2. 舵机脉宽来自出厂例程 300Hz、duty_max=10000 下的中位 4470、左限 5170、右限 3370，换算为 1490us / 1723.333us / 1123.333us。频率由设备读取，本程序不修改设备树。`servo_reference_deg=22` 是初始换算参考，真实机械角度需测量。旧 `steer_offset_deg=-15` 改为 0，避免旧标定叠加到新中位。
3. 编码器默认 `delta`，依据是附件例程直接把每次读取计数用于速度计算；附件未提供内核驱动，不能据此证明板上一定读后清零。上板重复读静止/转动计数；若实际是累积计数则改 `cumulative16`，程序会处理 16 位回绕。`encoder_ppr` 必须是每车轮整圈的实际计数，包含齿比和驱动计数倍频。
4. 左编码器硬件方向默认 -1，右默认 +1；`competition.ini` 的 encoder_*_sign 作为原有额外标定修正，默认均 +1，实际符号为两者相乘。电机方向单独设置，不能用编码器符号掩盖电机反转。
5. IIO 的自动 scale 按 Linux ABI 的 rad/s 转为 deg/s，参考 [Linux IIO ABI](https://www.kernel.org/doc/html/v6.12/admin-guide/abi-testing.html)。若设备不提供 scale，必须明确填写 `imu_gyro_scale_deg_s`（deg/s 每原始单位），不猜测型号量程。静止校准抵消固定零偏/offset，但不能消除长期漂移。
6. 默认读取 Z 轴；安装方向不同可设 x/y 并配置 `imu_yaw_sign`。IIO 航向是原始陀螺仪积分的相对角度，不是磁罗盘绝对朝向，也不是完整姿态融合。车体倾斜明显时单轴积分误差需要额外验证。
7. IIO 和编码器超过 150ms 的采样间隔、计数饱和/速度越界、错误设备数据和失败 I/O 会锁定故障。sysfs 原始值没有附带采样时间戳，当前检查只能证明按时读取，无法证明内核传感器每次都产生新数据；这是实车验证项。

## 构建与检查

在目标龙芯 Linux 上进入工程目录：

```bash
bash build_linux.sh
sudo ./build-linux/SMART_CAR_2026 --hardware-check
./build-linux/SMART_CAR_2026 --preview
sudo ./build-linux/hardware_servo_test
```

`--hardware-check` 不写电机/舵机/GPIO，读取 PWM 元数据、编码器、方向、按键、拨码、蜂鸣器和 IIO 信息。delta 驱动的编码器读取会消费该次计数，所以该检查与车控/舵机工具使用同一互斥锁。摄像头使用 `--preview` 实际取帧；设备检查只列出语音设置，不发送播报。

舵机工具直接复用脉宽转换，只访问舵机输出，不初始化电机或编码器。输入角度、c 回中、q 退出；SIGINT/SIGTERM 请求结束并尝试回中。SIGKILL/断电无法执行用户态清理。

使用其他路径：

```bash
sudo ./build-linux/SMART_CAR_2026 --hardware-check --hardware-config config/hardware.ini
sudo ./build-linux/hardware_servo_test --hardware-config config/hardware.ini --offset 0 --sign 1 --max 22
```

当前 `motion_calibrated=0`。完成方向、真实计数、舵机限位、IMU 比例系数/零偏/采样、PID 和停车标定后再设置为 1，使用原 `--drive` 命令。IIO 起步等待约校准样本数×20ms+3s；如果校准期间移动底盘/轮子或传感器缺少比例系数，程序会报错停车。选择串口时设置 `imu_backend=serial`，保留 `competition.ini` 波特率/协议；非空旧 `imu_device` 优先于新配置路径。

板载输入默认读取，但 `stop_key_index=-1`、`stop_switch_index=-1`，未擅自把某个按钮设为急停。确认实际电平后可设置 0..3 / 0..1；触发会锁定故障。板载输入或蜂鸣器不存在时，需要明确关闭相应 enabled 设置，不能把读取失败当作正常电平。语音属于可选功能，无法打开只报提示并保留控制台文本。

## 修改后五项检查

- 重复代码：左右电机共用同一换向/占空比函数；舵机工具和车控共用脉宽函数；底层读写共用完整传输检查。旧驱动仅保留作参考，不链接到新车控。
- 命名：FactoryBoard、HardwareConfig、LinuxDeviceIo、HardwareImu、IioYaw 对应明确职责；脉宽用 _us，速度用 RPS，航向用度，设备路径独立命名。
- 结构：视觉/任务不读写设备。硬件层负责协议与单位，Drive 负责双轮 PID、看门狗和故障锁定；代码测试通过模拟 DeviceIo 注入数据和故障。
- 隐藏问题：初始化前先独立清零双轮，元数据异常也尝试停车；换向先零输出；短读/短写不被当作成功；单侧失败不阻止另一侧停车；真实 dt、累计计数回绕、静止速度、NaN、IMU 校准/断流均有验证。停止写入失败会反馈，不能据此声称机械系统一定已停。
- 优化建议：先验证设备树/内核驱动实际语义及实车方向标定；随后用传感器时间戳或 IIO 缓冲采样提高航向可靠性，依据安装姿态补融合与漂移校正，再考虑提高采样效率和速度。

本机验证结果见 `VALIDATION.md`。模拟设备通过和 Linux 编译通过不等于目标板链接通过或实车验收。
