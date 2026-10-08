# 六阶段手推入库调参

当前版本 **2026-10-08.2**。第一弯：**2A左打倒弯→2B右打修正→零位**；第二弯：**4A右打倒弯→4B左打修正→零位**。四个动作分别调角度和保持时间，两处弯道共用执行逻辑。人手推/后拉，程序只写舵机、录前相机并读取已配置的编码器/前后超声波；没有电机、GPIO输出或IMU初始化，不自动判断车身已经摆正或入库成功。

## 调参只改一个文件

板上 `/home/root/gy/deploy/config/parking_tuning.ini`；源码模板 `deploy/config/parking_tuning.ini`。终端不接受角度数值。以下只展示相关区段，文件中其余四个阶段仍须保留，不能重复创建同名区段：

```ini
[session]
mode=single
stage=2
stage_2_step=first_bend
stage_4_step=second_bend

[stage_2]
steer_command=-10
settle_time_s=0.5
hold_time_s=5
correction_steer_command=5
correction_hold_time_s=1

[stage_4]
steer_command=12
settle_time_s=0.5
hold_time_s=5
correction_steer_command=-5
correction_hold_time_s=1
```

模板第四阶段沿用用户录像中的主转弯命令+12，其他主阶段参数仍须按实车结果调整。修正±5/1秒只是初始试值；两处弯道参数独立，不要求左右角度或时间相等。36.7°是线路夹角，不能直接填为舵机命令。

| 字段 | 含义 |
|---|---|
| `mode=single` | 只测选中阶段；最后一步计时完成后结束并询问Y/N |
| `mode=full` | 普通回车路径逐段走六阶段；运行中保存直接执行的试次最后一步完成后仍结束整个程序 |
| `stage=2` / `stage=4` | 选择第一弯 / 第二弯；`stage=1`从前移开始 |
| `stage_2_step=first_bend` | 测完整第一弯；`right_correction`只测2B |
| `stage_4_step=second_bend` | 测完整第二弯；`left_correction`只测4B |
| `steer_command` | 主转弯命令：第一弯负数左打，第二弯正数右打；范围±15并受车辆配置上限限制，不是实测前轮角度 |
| `settle_time_s` | 按一次Enter开始记录并设置舵机后静止等待0.5～5秒；运行中保存直接执行不等待此时间 |
| `hold_time_s` | 主转弯保持时间，范围0～120秒；启用反向修正时须大于0 |
| `correction_steer_command` | 反向修正命令：第一弯须正数右打，第二弯须负数左打，同样受角度上限限制 |
| `correction_hold_time_s` | 修正保持时间，范围0～120秒；0关闭对应修正，启用时修正命令不能为0 |

旧调参文件仍兼容，缺少某处修正字段时该动作默认关闭，程序不擅自加入新的转向动作。解压升级包不会覆盖原参数文件。请在已有`[session]`中补入两个步骤选择字段，在已有`[stage_2]`和`[stage_4]`中各补入两个`correction_...`字段；可以保留已经调好的主转弯角度和时间。

启动明确显示 `VERSION`、`TUNING_FILE`的绝对路径、模式/阶段、两处主转弯角度/时间、修正是否启用及修正角度/时间。看到“修正=关闭”时不会自动反向打轮；即使程序版本正确，也要检查是否修改了实际读取的文件。

## 测一个弯道的操作

1. 把车摆到该弯道起点，保持静止，启动脚本。等待授权时只显示状态，不录视频/传感器、不写舵机；没有等待超时。
2. **按一次Enter**：开始记录，写入本段角度。`WAIT_SERVO_SETTLE`期间保持静止。
3. 等待`settle_time_s`结束，自动显示**【开始推/拉】**并启动保持计时。无须第二次Enter，此时按车轮自然滚动推/拉，不横向掰动车身。第六阶段提示保持静止。
4. 主转弯时间到，自动接反向修正，无须Enter。第一弯左→右，第二弯右→左；继续向后拉，运动方向不变。修正计时从命令成功写入开始。
5. 最后一步时间到回零；你停止推/拉。single模式停止采集，只询问一次Y/N，直接按键，无需回车。Y保留、N丢弃，然后退出；下一次手动启动。

自动切换实现为主转弯到时先写零位，再由主循环取得新鲜相机帧后写修正角度，可能存在短暂间隔，不承诺无间隙或精确总用时。回零失败或相机帧失效会终止，不继续反向修正。计时线程只可写零位；非零修正仍由主循环在已授权的两步动作范围内执行。最后零位只表示软件写入，不证明车身已对齐。

关闭反向修正时，主转弯到时直接回零并收尾。普通阶段命令为0且保持时间大于0也计时；保持时间0关闭定时结束，需要人工Q结束。程序没有电机停车能力，Enter/P/Q不是传感器已经证明物理停稳。

## 单独调修正与走完整流程

| 试验 | 启动前选择与车体位置 |
|---|---|
| 第一弯完整两步 | `single`、`stage=2`、`stage_2_step=first_bend`，摆到第一弯起点 |
| 只调2B右修正 | `single`、`stage=2`、`stage_2_step=right_correction`，手动摆到左弯完成的位置 |
| 第二弯完整两步 | `single`、`stage=4`、`stage_4_step=second_bend`，摆到第二弯起点 |
| 只调4B左修正 | `single`、`stage=4`、`stage_4_step=left_correction`，手动摆到右弯完成的位置 |
| 六阶段完整流程 | `full`、`stage=1`，两处步骤选择分别为`first_bend`、`second_bend` |

只调修正也只按一次Enter，记录并设置修正角度，静止等待后自动提示推车并计时，不补走主转弯。只能把当前选中弯道设为修正单独测试，另一个步骤选择恢复完整弯道；不能同时选择两个单独修正或在full模式跳过主转弯。

full普通回车路径：当前段完成后保持记录，实际停稳再Enter结束本段，下一次只按一次Enter即设置下一段角度；静止等待结束自动开始计时并提示推/拉。正在执行左右两步时Enter不会跳过修正，提前结束用Q。第六段保持静止，结束后仍停留在第六段，用Q退出。

六阶段依次为：1前移→2左弯+右修正→3分支直退→4右弯+左修正→5库内直退→6停止确认。交点、起倒位置、分支对齐、库内终点仍由人判断。本工具没有闭环视觉位置/朝向控制。

## 暂停、保存参数与退出

- **P再Enter**只暂停录像和传感器样本，左右计时、自动反向修正继续；P不是停止动作。
- **C再Enter**只恢复记录，不重新执行角度或计时。
- **Q再Enter**或Ctrl+C提前结束、尝试写零位，再询问Y/N。
- 运行中保存当前弯道参数就是执行授权，不需要R/C或额外Enter。主转弯期间修改该弯道参数，重新执行整对动作；修正期间只修改修正字段，仅重新执行修正；修改主转弯字段则重新执行整对动作。修正单独模式始终只测修正。
- 修正期间把修正时间改为0，结束并回零，不返回主转弯。修改阶段/模式/当前步骤选择会直接执行新的选择。
- 修改其他阶段不重启当前动作；相同值或只改注释不执行。每次有效执行取消旧计时，旧试次/旧步骤事件不会结束新试次。
- **保存直接执行的试次，无论single/full，最后一步完成后都结束程序并询问Y/N。**要不中途退出地走full，启动前改好参数，运行时用回车逐段授权。

文件约每0.2秒检查一次，稳定至少0.3秒且全量校验通过才应用；速度随相机循环耗时变化。非法角度、错误方向、字段缺失、文件暂时清空或读取失败均拒绝更新，保留旧参数与计时。试验控制键后需Enter，最终Y/N直接按键；无效保存按键静默忽略，提示不重复、不默认选择。确认前清除排队输入，临时关闭整行输入和回显，回答或异常退出时恢复终端模式。

## 数据记录

每次启动创建独立目录 `/home/root/gy/deploy/captures/parking_tuning/rehearsal_.../`，固定六份CSV：

```text
01_advance.csv
02_reverse_first.csv
03_reverse_branch.csv
04_reverse_second.csv
05_reverse_straight.csv
06_stop_confirmation.csv
```

2A/2B共用第二份，4A/4B共用第四份。`segment`分别为`FIRST_LEFT_BEND`、`RIGHT_CORRECTION`、`SECOND_RIGHT_BEND`、`LEFT_CORRECTION`；其他阶段为`PRIMARY`。一对自动动作沿用同一个`trial`，重新保存执行增加试次编号。未进入阶段只有表头。

CSV记录按键、状态、步骤、试次、配置版本、配置命令与最近软件写入命令、相机读帧区间、传感器原始数值/单位/有效性/返回值、舵机写入、HOLD_START和AUTO_CENTER等事件。自动右/左修正计时来源分别为`AUTO_RIGHT_CORRECTION`、`AUTO_LEFT_CORRECTION`。`AUTO_CENTER`表示软件写零成功，失败为`AUTO_CENTER_FAILED`，不证明机械角度。原始编码器计数保留；增加编码器速度窗口，没有将计数当作实测朝向；超声波未自动换算厘米。

### 各阶段速度

六份阶段CSV新增`event=SPEED`行：`speed_left`、`speed_right`单位为counts/s，方向通过硬件和车辆配置的编码器符号统一，前进正、后退负。`speed_left_window`、`speed_right_window`记录对应窗口秒数。每个窗口至少0.5秒，按两轮各自的真实读取间隔累计计数计算，不用视频标称FPS，也不增加编码器读取次数。

另有`stage_speed_summary.csv`，按阶段、步骤、试次、配置版本分别汇总：有效观测时间、净计数、累计计数绝对值、含停顿平均速度、运动窗口平均速度、窗口峰值。`segment=0`为主动作、`segment=1`为反向修正。没有有效窗口的阶段不会伪造0速度汇总，原阶段文件仍保留表头。

在现有`[session]`中可选添加（旧文件不添加也能运行）：

```ini
speed_left_cm_per_count=0
speed_right_cm_per_count=0
```

0表示未标定，此时只报告counts/s，厘米速度列留空。实测沿直线移动D厘米，分别以D除以对应轮累计计数绝对值，得到两轮cm/count；标定距离必须与累计计数取自同一记录区间，并包含区间内每次有效增量。只有两项均大于0才输出中心速度估计；单轮已标定时只输出该轮cm/s。标定值范围0～10，不能使用calibration_vehicle.ini中的未验证轮周长/PPR默认值。修改速度标定只更新记录配置，不触发舵机或重新计时。

速度只统计已进入推/拉状态的观测窗口；等待舵机期间仍录视频/原始传感器但不计速度。首读、切段/切步骤/切试次/配置版本变化、暂停恢复及读取错误后的首读作为基线，不纳入速度；超过2秒的读取间隔不产生速度。暂停期间不补录速度。不足0.5秒的尾部窗口不进汇总，CSV原始样本仍保留。运动窗口平均值排除全零窗口，仍可能包含窗口内短停顿；峰值是窗口平均峰值，不是瞬时峰值。无打滑条件下，双轮厘米速度均值是后轴中心速度估计，不是实测车头速度。

后续调电机先对照各步骤的运动窗口平均速度；不能把手推速度直接当PWM，也不能把左右轮在弯道中的正常速度差当电机故障。电机恢复后需在相同线路和舵机动作下记录实际速度再匹配。本工具仍不写电机。

保存实际应用的`config_1.ini`及后续版本、传感器/硬件/车辆配置副本，共用`camera_1.avi`等录像每1800帧换片，CSV的clip/video_frame定位画面；实际时间看CSV单调时钟，不能仅用标称FPS。暂停不补录图像/传感器，控制与计时事件暂存，C恢复或最终收尾时补写实际发生时间，必要时按单调时钟排序。

收尾关闭/fsync文件后询问Y/N。N只删除本次拥有所有权标记的会话目录，保护调参文件和旧记录；路径或所有权验证失败不删除。Y完成摘要后才报告SAVED。设备/存储失败仍尝试回零并询问是否保留；终端断开/EOF无法回答时保留.pending_session并报告SAVE_UNCONFIRMED，异常文件标记KEPT_INCOMPLETE。帮助、配置检查或创建会话前失败没有记录，不询问；断电/强杀无法弹出询问。缺少允许省略的灰度通道为UNCONFIGURED、正常退出码2，不能算全部传感器有效。

## 编译、安装与启动

本机提供源码、配置和脚本，未生成或执行新的LoongArch二进制。共享目录同步后，在已有龙芯构建虚拟机运行：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
sh deploy/build_parking_rehearsal.sh
```

构建使用已有 `/home/gy/builds/SMART_CAR_2026-loongarch` CMake缓存及目标OpenCV，生成`deploy/parking_rehearsal_20261008_2_verified.zip`。打包只接受含当前版本/功能标记的64位小端LoongArch ELF；不会把Windows测试exe当成板上程序。可执行文件名称沿用`parking_rehearsal_20261007`，**以VERSION输出判断版本**。

上传ZIP到板上 `/home/root/gy/deploy/`：

```sh
cd /home/root/gy/deploy && \
unzip -o parking_rehearsal_20261008_2_verified.zip && \
sha256sum -c PARKING_TUNING_SHA256SUMS && \
chmod +x parking_rehearsal_20261007
```

补好原调参文件的新字段，先检查版本和启用状态（不访问硬件）：

```sh
cd /home/root/gy/deploy
export LD_LIBRARY_PATH="/home/root/opencv-4.11-loongarch/install/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
./parking_rehearsal_20261007 --config manual_capture.ini \
  --hardware-config config/calibration_hardware.ini \
  --vehicle-config config/calibration_vehicle.ini \
  --tuning-config config/parking_tuning.ini --allow-partial --check-config
```

实际启动：

```sh
sh /home/root/gy/deploy/run_parking_rehearsal.sh
```

启动脚本沿用已有OpenCV库路径，只在调参文件缺失时复制新模板，不覆盖已有参数。查看程序版本也可用`./parking_rehearsal_20261007 --help`。首次安装没有调参文件时，先运行启动脚本生成模板再退出等待授权，或手动复制`parking_tuning.example.ini`后进行配置检查。

## 五项代码检查与验证

1. 重复代码：两处弯道复用同一执行流程；两轮共用速度窗口和统计逻辑，不另读设备。
2. 命名：主转弯与correction字段分清；屏幕和CSV区分2A/2B/4A/4B；速度区分counts/s与已标定cm/s，平均值与运动窗口平均值分别命名。
3. 结构：纯状态机、仅可定时写零的舵机线程、采集执行入口分离；自动反向写入须有前序授权与新鲜相机帧，保持六主阶段；一次Enter授权，记录开始与等待后计时分开，速度估算独立于舵机输出。
4. 隐藏bug：验证两弯方向、旧配置默认关闭、部分字段拒绝、单独测试选择冲突、暂停继续切换、Q取消待执行修正、回零/写入失败、修改另一弯不重启当前、旧计时事件不结束新动作，以及full六阶段全部经过两处修正；增加暂停恢复基线、非法计数、长间隔、真实时间窗口、累计计数回绕、标定不写舵机和旧配置不伪造厘米速度。
5. 优化建议：先固定主转弯，单独调2B和4B，再测完整弯道，最后走六段；每次只改角度或时间，保持后拉速度接近。电机恢复后再以距离/视觉姿态控制替代单纯时间控制。

本版增加一次Enter开始记录/等待后自动计时，以及编码器速度窗口与统计测试。Windows纯逻辑、计时及回归检查和打包测试通过后同步；Linux入口只做对象编译检查。实车交互、目标LoongArch链接和厘米速度标定仍需目标环境验证；本地模拟不能证明机械执行或入库成功。
