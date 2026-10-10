# 新车入库直接调参

使用 `run_new_car_parking.sh`，调参文件为 **config/parking_tuning.new_car.ini**。
这是原入库调参程序的新车接口版本 **2026-10-10.1**，没有另建电机测试程序。
旧可执行文件不能读写新车接口；首次须编译并替换，之后直接修改INI并运行。

## 首次编译和启动

新车板上运行目录固定使用 **/home/root/smartcar/gy/**。
把部署包上传到该目录，解压后程序和启动脚本直接放在gy下，不再套一层deploy。
配置放在gy/config，录像和阶段记录放在gy/captures。启动脚本按自身所在目录定位，
不会读取旧的/home/root/gy/deploy配置。

```text
/home/root/smartcar/gy/
  parking_rehearsal_20261007
  run_new_car_parking.sh
  manual_capture.new_car.ini
  config/
    parking_tuning.new_car.ini
    new_car_hardware.ini
    new_car_vehicle.ini
  captures/parking_tuning/
```

原来的Linux编译虚拟机，在共享目录执行：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
sh deploy/build_new_car_parking.sh
```

使用现有 `/home/gy/builds/SMART_CAR_2026-loongarch` 构建配置及目标OpenCV；
不得拿x86可执行文件替代LoongArch版本。编译脚本会检查真正ELF的架构和内部版本，
复制可执行文件，并生成同一入库程序的部署ZIP。当前Windows检查不等于完成此龙芯构建。

上传新程序、启动脚本、`manual_capture.new_car.ini`、`parking_tuning.new_car.example.ini`、
`config/new_car_hardware.ini`、`config/new_car_vehicle.ini`到 `/home/root/smartcar/gy`。
若使用部署ZIP，解压后执行 `sha256sum -c PARKING_TUNING_SHA256SUMS`。
无需上传额外电机测试程序。已有 `config/parking_tuning.new_car.ini` 不会被启动脚本覆盖。

```sh
cd /home/root/smartcar/gy
unzip -o parking_rehearsal_20261010_1_verified.zip
sha256sum -c PARKING_TUNING_SHA256SUMS
chmod +x parking_rehearsal_20261007
sh ./run_new_car_parking.sh
```

启动版本必须显示2026-10-10.1、单位duty_ns，旧版会直接拒绝。
六个阶段的单阶段试验统一按以下顺序操作；两次都执行同一条 `sh ./run_new_car_parking.sh`：

1. 先关闭舵机电源开关，启动第一次。程序按参考代码顺序初始化PWM/GPIO、编码器、舵机及相机，不执行阶段动作、不创建试验录像或CSV。
2. 看到“第一次启动／硬件初始化完成”后，保持舵机开关关闭，按Ctrl+C结束；必须看到PREPARE_COMPLETE。初始化过程中提前中断不算完成。
3. 再次执行同一命令。看到“第二次启动：正式试验”后，再打开舵机电源开关。
4. 位置准备好后按一次Enter，开始执行及记录。首次无需快速抢按Ctrl+C；程序会一直等待。
5. 手动结束时，先关闭舵机开关，再Q加Enter；本次正式试验询问一次Y/N，直接按键。第一次没有试验记录，不询问保存。

准备标记是调参文件旁的 `.startup_ready` 文件，每次正式启动都会消费；下一次试验重新执行上述两次启动。标记绑定阶段、主转向/仅修正选择、single/full模式、程序版本、硬件/车辆/相机配置和本次开机ID。换阶段或重启板子需重新准备；修改同阶段角度/时间参数不会凭空执行动作。`--help`和`--check-config`不改变标记。该机制仅用于新车sysfs接口，旧车factory接口保持原启动方式。

`mode=full`仍按原逻辑在同一次正式试验内手动切段，不为每次内部切段另启进程；选任意单阶段启动均遵循两次启动。想重新走第一次，可在程序退出后删除 `config/parking_tuning.new_car.ini.startup_ready`。若意外留下同名 `.pending` 文件，确认无程序运行后只删除该文件，再重新准备。

按一次Enter授权后：记录、设舵机、等待settle_time_s，
阶段1确认主线后输出电机。运行时间从两轮PWM及GPIO73写入读回成功后开始，
不是从程序启动或按Enter时开始。默认阶段1左右3000、运行1秒；运行时必须位于可识别黑线上。
前相机请求1280×720；新车若返回其他可用尺寸，自动采用实际尺寸并统一缩放巡线，
不因旧车分辨率限制卡住。实际尺寸存入session.txt；运行中尺寸改变则停止。

## 只改这些参数

板上实际调参文件：`/home/root/smartcar/gy/config/parking_tuning.new_car.ini`。
首次启动自动创建；后续修改和上传该文件，保留原区段，不覆盖成不完整片段。

```ini
[session]
mode=single
stage=1

[stage_1]
motor_left_command=3000
motor_right_command=3000
motor_run_time_s=1
```

其他行保留。切换阶段改session里的stage，不要重复添加区段。

| 参数 | 意义及范围 |
|---|---|
| motor_left_command / motor_right_command | 左右PWM占空时间，整数0..50000纳秒，直接写入，不缩放 |
| motor_run_time_s | 本阶段总运行上限0..120秒；0完全禁用电机 |
| steer_command | 舵机命令，负左正右，当前±15；并非前轮实际转角 |
| hold_time_s | 主转向保持0..120秒；0不启用该转向计时 |
| correction_steer_command | 阶段2的右修正/阶段4的左修正命令 |
| correction_hold_time_s | 反向修正保持秒数，0关闭修正 |
| settle_time_s | 写舵机后静止等待0.5..5秒，不计入电机运行时间 |
| line_follow_enable | 仅阶段1，1启用前相机巡线，0固定角度前移 |

周期50000ns，即20kHz。3000=6%、10000=20%、20000=40%、50000=100%。
2000在新车是4%，不能沿用旧车“2000=20%”的理解；没有2000/12000人为上限。
电机运行时间和舵机最后一步结束时间，哪个先到就先停。
想完整执行“主转向5秒＋修正1秒”，电机运行时间至少覆盖两步；修正时间必须大于0。
新车主转向与反向修正需重新调，旧车-10/+12及各5秒只是配置起点。

六阶段仍为前移、第一倒弯、分支直退、第二倒弯、库内直退、停止确认。
不用分支直退时跳过阶段3。阶段2左→右→0，阶段4右→左→0；
阶段1前进，2..5倒退，6禁止电机。单阶段程序不会自动执行下一阶段或整场赛事。

## 执行与记录

- 一次Enter授权执行；P加Enter清零电机、关闭GPIO73并暂停记录。
- C加Enter只恢复记录；之后再按Enter才能恢复运动。
- Q加Enter提前结束。结束回正后只询问一次保存，直接按Y或N。
- 运行中保存当前段参数即授权重新执行，先停车再设置和等待；
  只改其他阶段不重启当前阶段。暂停状态下电机仍需Enter重新授权。
- 六个阶段各有CSV，另存录像、配置快照、阶段持续时间及速度汇总。
- `stage_timing_summary.csv`新增阶段名称、电机实际输出时长、主转向实际时长和反向修正实际时长；所有实际执行的阶段会打印持续时间。未执行阶段标为NOT_RUN且时长留空，绝不填成0秒。重复停机清零和Y/N等待不会延长已结束的时间。
- 时长统计旁路记录，不作为新阶段结束条件；既有hold_time_s、motor_run_time_s、手动结束和切段行为保持原样。
- 新车编码器采用原生rps（转/秒），不套用旧车delta counts，也不凭默认轮周长估算cm/s。
- 测量周期寄存器可能保留旧值；记录不会据此声称已经物理停稳。
  PWM=0与GPIO73=0均做软件读回，采集0.5秒尾段后结束。
- 新车资料未确认前后超声波及灰度接口，留空并标记UNCONFIGURED；
  阶段5按你设定的时间执行，本工具不自动按挡板距离停车。没有启用IMU。

## 接口依据和改动检查

用户提供的两个ZIP中，电机/编码器/舵机相关源码一致：
左PWM pwmchip8/pwm2、右PWM pwmchip8/pwm1、方向GPIO12/13、
总使能GPIO73；方向0前进/1倒退。舵机pwmchip2/pwm0、周期3040000ns，
脉宽1520000+(命令+中位偏置)*period/1800，中位偏置初始-15来自参考工程。
编码器基址0x1611B000，通道0/3，GPIO75/72，100MHz、1024线。

五项检查：共用原阶段逻辑避免复制；sysfs/nativeRps命名区分单位；
控制和I/O分层；电机PWM恢复参考顺序export→enable=1→period→duty=0，随后设置方向GPIO和总使能；舵机恢复period→初始脉宽1520000→enable=1。不再前置写enable=0或强制polarity；初始enable若被驱动拒绝会明确打印并按参考顺序继续，不能据此认定硬件已生效。正常阶段输出和独立定时停车线程保持原样；
新车物理转向和阶段时长需在实车调，下一步用记录的rps辅助分别调整左右PWM。硬件/车辆安装配置改变后须重新启动；运行中只重载调参INI。
离线模拟能验证输出顺序和错误路径，不能代替实车跑动确认。

本版本离线验证：347项阶段状态机、86项电机/时长、59项sysfs调用顺序、42项两次启动状态检查，共534项C++检查；另有7项部署/启动脚本检查通过。Linux入口使用实际OpenCV源头文件完成目标文件编译，未链接目标板OpenCV，也未执行实车硬件验证。
未在此Windows环境生成或运行LoongArch可执行文件，需按上面的虚拟机命令编译。
