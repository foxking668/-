# 六阶段入库调参：舵机与单阶段电机

版本 **2026-10-08.3**。保留六阶段及可选的两次反向修正，新增单阶段电机定时执行、阶段持续时间记录。没有IMU，不开启比赛主程序，尚未接成完整自动入库。起点/终点由你测量；程序只记录阶段1持续时间，不把已有手推起点当成第二次斑马线起点。

## 只改一个文件

板上 `/home/root/gy/deploy/config/parking_tuning.ini`，源码模板 `deploy/config/parking_tuning.ini`。保留所有六段，只改选中阶段。初始选阶段1，阶段1～5左右电机各2000，运行时间均0禁用；阶段6三项全0。第一弯-10、第二弯+12、各5秒，额外修正默认关闭，沿用成功手推记录。

相关区段示例（不能代替完整文件，也不要重复添加区段）：

```ini
[session]
mode=single
stage=1

[stage_1]
steer_command=0
settle_time_s=0.5
hold_time_s=0
motor_left_command=2000
motor_right_command=2000
motor_run_time_s=0
```

首次试电机，启动前把选中段 `motor_run_time_s` 改为短时间，例如1秒。时间0不初始化/输出电机，不改变方向GPIO；两个幅值都0也禁用。

| 参数 | 范围和含义 |
|---|---|
| `mode` | 有电机动作必须single；full只兼容全部电机禁用的手推流程 |
| `stage` | 1前移、2第一倒弯、3分支直退、4第二倒弯、5库内直退、6停止确认 |
| `motor_left_command` / `motor_right_command` | 非负幅值0～12000，并受车辆pwm_limit限制；左右可独立调，不填写负值 |
| `motor_run_time_s` | 0～120秒；0禁用，正时间配合非零幅值才执行；从实际输出成功开始计时 |
| `steer_command` | -15～15，受车辆上限限制；负左正右；这是命令，不是实测轮角 |
| `settle_time_s` | 设置舵机后静止等待0.5～5秒；电机模式保存重执行也等待 |
| `hold_time_s` | 动作允许后主转向保持0～120秒；0关闭舵机定时 |
| `correction_steer_command` | 第一弯右修正为正，第二弯左修正为负，也受舵机上限限制 |
| `correction_hold_time_s` | 0关闭修正；正数启用自动反向修正，范围0～120秒 |
| `speed_*_cm_per_count` | 0未标定，只报counts/s；有实测厘米/计数才输出cm/s |

当前硬件满量程50000：1000=2%、**2000=4%**、2500=5%、5000=10%、10000=20%、12000=24%占空比；最终按各通道PWM元数据换算。改变硬件满量程会改变对应比例。2000不保证克服静摩擦，也不是实际速度。先确认实际方向，再做短程试验；前进/倒退电平从已有硬件配置读取。

阶段1自动前进，阶段2～5自动倒退，阶段6禁止电机动作。**电机总时间上限与舵机最后一步完成，先到者结束本段。**例如阶段2未启用修正、hold_time_s=5，电机设置10秒也会在舵机最后动作完成时停；要测较长直行，把直行段hold_time_s保持0。自动反向修正不重启电机总计时。

## 执行与切段

1. 启动前修改选中阶段参数，车放在你量测的起点，保持静止，运行脚本。等待授权时不采样、不写舵机、不启动电机。
2. **按一次Enter**开始记录并设置舵机，等待settle_time_s；无须第二次Enter。新鲜前相机帧和有效编码器数据满足后执行电机，并从输出成功开始计算运行时间。禁用电机时提示手推/后拉。
3. 开启修正时：第一弯左打→右修正，第二弯右打→左修正，方向始终倒退；关闭时主动作到时回正。36.7°是线路夹角，不能直接填为舵机命令。
4. 时间到写两路电机0并回正，继续采集停机尾段。有效编码器约0.5秒连续零增量后结束；两秒内未确认则记录STOP_UNCONFIRMED并结束，不宣称停稳。零增量是观测条件，传感器故障仍可能影响判断。
5. 结束只显示一次保存提示，**直接按Y或N，无需回车**。Y保留本次全部记录，N丢弃目录；退出后你手动重启。
6. 切段：退出后修改stage再启动，分别试1→2→（需要时3）→4→5→6。实际不需要分支直退可直接选stage=4。第6段只静止记录，Q结束。

目前先分别标定电机阶段，不自动串行六段，不按虚线或超声波自动停车。旧手推full流程继续兼容。

## 控制键和保存更新

- P再Enter：停电机并暂停视频/传感器；电机试验同时取消舵机计时并回正，不能自动续跑。
- C再Enter：只恢复记录；之后再按Enter，重新设置舵机、静止等待并开始新试验。
- Q再Enter或Ctrl+C：提前结束、尝试电机零输出和舵机回正，再询问Y/N。人工退出不推断物理停稳。
- 正常运行中保存当前段参数即重新执行授权；先停当前电机，重新打角度并静止等待，再运行新参数。定时完成后结束，下一次手动启动。**P之后保存/C均不能绕过新的Enter授权。**改其他阶段、速度尺度、只改注释不重启当前段。
- 停机尾段确认期间拒绝新动作和配置更新，Q仍可提前退出，P/C仍可暂停/恢复记录。暂停后若两秒内未确认零增量尾段，以STOP_UNCONFIRMED结束；不会重新启动电机。
- 电机禁用的历史手推模式：P只暂停记录，舵机计时继续，C恢复记录；没有电机可停。缺少全部电机字段的旧文件按0处理；只补部分字段拒绝，三项需一起补。
- 非法/超限文件更新拒绝，合法旧参数和定时上限继续；稳定至少0.3秒才应用。旧试次计时事件不会结束新试次。

非零电机输出由主线程授权，独立线程只能写0。它在时间到或相机/编码器心跳超过frame_timeout_s（当前0.4秒）时停机，避免摄像头循环延迟无限续跑。它不能中断阻塞的内核驱动写入；软件零写入也不等于物理断电证明。实车电机极性、静摩擦、停止距离仍要短程确认。

## 记录文件

每次运行建立独立 `captures/parking_tuning/rehearsal_.../`：

```text
01_advance.csv
02_reverse_first.csv
03_reverse_branch.csv
04_reverse_second.csv
05_reverse_straight.csv
06_stop_confirmation.csv
stage_speed_summary.csv
stage_timing_summary.csv
camera_*.avi
config_*.ini、capture_config.ini、hardware_config.ini、vehicle_config.ini
session.txt
```

原始CSV包含帧时间、传感器读起止、编码器/超声波原始值、舵机写入/回正、按键和配置事件，以及电机启动/停止原因及左右带方向命令。各事件含阶段/试次/版本，未使用阶段只有表头；视频用CSV单调时钟定位，不依赖名义FPS。

速度表记录各阶段/步骤/试次的两轮窗口均速、运动窗口均速、峰值等；未标定只能比较counts/s，不是速度PID目标。

时间表记录允许动作时刻、电机输出成功时刻、零输出完成时刻、**stage_duration_s阶段执行时间**、首次/最后编码器运动观测时刻、运动观测跨度、连续零增量尾段和结束原因。阶段1执行时间会打印出来；授权等待、舵机静止等待、停机确认尾段和保存Y/N不计入电机阶段执行时间。运动观测跨度受采样频率影响，不是严格测量的物理运动持续时间。直接复用已有两轮样本，不额外读取或清空编码器。

## 构建、升级、启动

源码已同步共享目录。Windows只能离线验证，真正龙芯程序需在已有构建环境编译：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
sh deploy/build_parking_rehearsal.sh
```

将生成的 `deploy/parking_rehearsal_20261008_3_verified.zip` 上传到板上，再执行：

```sh
cd /home/root/gy/deploy
unzip -o parking_rehearsal_20261008_3_verified.zip
sha256sum -c PARKING_TUNING_SHA256SUMS
chmod +x parking_rehearsal_20261007
sh ./run_parking_rehearsal.sh
```

启动须显示2026-10-08.3；可执行文件保留历史名称，脚本核对内部版本。包只含程序、启动脚本、示例、说明和校验表，**不会覆盖原config/parking_tuning.ini**。已有配置需在各原区段补三项电机字段，或备份后安装新版完整模板。启动显示实际调参文件绝对路径及参数，避免改错文件。

沿用config/calibration_hardware.ini和config/calibration_vehicle.ini，不修改motion_calibrated=0，不使用未标定轮周长/PPR。检查配置无需硬件：

```sh
export LD_LIBRARY_PATH="/home/root/opencv-4.11-loongarch/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./parking_rehearsal_20261007 --check-config --allow-partial
```
