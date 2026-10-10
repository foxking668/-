# 新车入库直接调参

使用 `run_new_car_parking.sh`，调参文件为 **config/parking_tuning.new_car.ini**。
这是原入库调参程序的新车接口版本 **2026-10-10.5**，没有另建电机测试程序。

### 本版舵机范围

新车 `steer_command`、两弯的 `correction_steer_command` 及可调巡线限幅统一支持±30命令；小数也可用。仍受车辆配置 `max_steer_deg` 限制，超出会拒绝，不静默截断调参值。旧Factory模式保持±15。默认转弯命令、保持时间和 `line_max_command=15` 不自动加大。

初始化顺序、3040000ns周期、中位偏置和cc脉宽公式不变。偏置为-15时，命令-30输出1444000ns、命令+30输出1545333ns；这是计算值，不是实测轮角或机械安全行程。

解压不会覆盖已调整的车辆配置。已有板子需在程序退出后保留其他参数，只将 `/home/root/smartcar/gy/config/new_car_vehicle.ini` 中的 `max_steer_deg` 改为30，例如：

```sh
cd /home/root/smartcar/gy
cp config/new_car_vehicle.ini config/new_car_vehicle.ini.before_steer30
sed -i 's/^max_steer_deg=.*/max_steer_deg=30/' config/new_car_vehicle.ini
sh ./run_new_car_parking.sh
```

启动应显示 `VERSION 2026-10-10.5` 和 `STEER_COMMAND_LIMIT +/-30`。如仍显示15，检查脚本打印的配置路径及车辆限幅。更换车辆配置后重新走两次启动流程。
旧可执行文件不能读写新车接口；首次须编译并替换，之后直接修改INI并运行。
本版直接调用与用户提供的 `cc(1).zip` 相同的增量PID源码，而不是继续固定输出3000。
只需修改 `[session]` 的 `motor_target_rps` 一个数，即可一起调节左右轮和各行驶阶段的快慢。

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
无需上传额外电机测试程序。已有 `config/parking_tuning.new_car.ini` 会由新版程序先备份，再移除旧左右电机字段及阶段1旧PD字段；角度、持续时间、线路启用和最终舵机限幅保留，缺少共用速度时补9。升级不访问硬件、不消费准备标记；格式错误会拒绝替换。已升级文件不会反复改写或生成备份。

```sh
cd /home/root/smartcar/gy
unzip -o parking_rehearsal_20261010_5_verified.zip
sha256sum -c PARKING_TUNING_SHA256SUMS
chmod +x parking_rehearsal_20261007
sh ./run_new_car_parking.sh
```

启动版本必须显示2026-10-10.5及cc速度PID，旧版会直接拒绝。必须替换编译后的程序，单改INI不能升级。
六个阶段的单阶段试验统一按以下顺序操作；两次都执行同一条 `sh ./run_new_car_parking.sh`：

1. 先关闭舵机电源开关，启动第一次。程序按参考代码顺序初始化PWM/GPIO、编码器、舵机及相机，不执行阶段动作、不创建试验录像或CSV。
2. 看到“第一次启动／硬件初始化完成”后，保持舵机开关关闭，按Ctrl+C结束；必须看到PREPARE_COMPLETE。初始化过程中提前中断不算完成。
3. 再次执行同一命令。看到“第二次启动：正式试验”后，再打开舵机电源开关。
4. 位置准备好后按一次Enter，开始执行及记录。首次无需快速抢按Ctrl+C；程序会一直等待。
5. 手动结束时，先关闭舵机开关，再Q加Enter；本次正式试验询问一次Y/N，直接按键。第一次没有试验记录，不询问保存。

准备标记是调参文件旁的 `.startup_ready` 文件，每次正式启动都会消费；下一次试验重新执行上述两次启动。标记绑定阶段、主转向/仅修正选择、single/full模式、程序版本、硬件/车辆/相机配置和本次开机ID。换阶段或重启板子需重新准备；修改同阶段角度/时间参数不会凭空执行动作。`--help`和`--check-config`不改变标记。该机制仅用于新车sysfs接口，旧车factory接口保持原启动方式。

`mode=full`仍按原逻辑在同一次正式试验内手动切段，不为每次内部切段另启进程；选任意单阶段启动均遵循两次启动。想重新走第一次，可在程序退出后删除 `config/parking_tuning.new_car.ini.startup_ready`。若意外留下同名 `.pending` 文件，确认无程序运行后只删除该文件，再重新准备。

按一次Enter授权后：记录、设舵机、等待settle_time_s，
阶段1按cc原巡线确认有效赛道后输出电机。运行时间从两轮PWM及GPIO73写入读回成功后开始，
不是从程序启动或按Enter时开始。默认共用目标9rps、阶段1运行1秒；9来自cc原配置，不是本车停车标定值。
阶段1运行时必须位于原cc算法可识别的线路上。PID在独立电机线程中每50ms更新，使用采集线程最新的原生编码器反馈；
反馈失效或相机/编码器心跳超时，清零两路输出。反向修正接续同一次速度控制，不重置电机总计时。
前相机请求1280×720；新车若返回其他可用尺寸，自动采用实际尺寸并统一缩放巡线，
不因旧车分辨率限制卡住。实际尺寸存入session.txt；运行中尺寸改变则停止。

## 只改这些参数

板上实际调参文件：`/home/root/smartcar/gy/config/parking_tuning.new_car.ini`。
首次启动自动创建；后续修改和上传该文件，保留原区段，不覆盖成不完整片段。

```ini
[session]
mode=single
stage=1
motor_target_rps=9

[stage_1]
motor_run_time_s=1
```

其他行保留。切换阶段改session里的stage，不要重复添加区段。
快慢只改 `motor_target_rps`：数值增大加快、减小减慢、0保持两路PWM为零。
这是轮转速（转/秒），不是PWM也不是厘米/秒。先沿用cc的9；需要慢速入库就只减小此值。
保存同一个参数会让当前段按已有授权规则重新执行；不改变任何阶段的角度或持续时间。
首次使用新版启动脚本，会自动备份旧配置、清理左右电机参数并补上共用速度。无需手动删除3000；已有共用速度值会保留。运行中重载也只认这个共用速度，旧左右PWM不再参与控制。

| 参数 | 意义及范围 |
|---|---|
| motor_target_rps | `[session]`共用速度，0..100转/秒，左右同步；方向由阶段决定；0使PWM输出为零 |
| motor_pid_kp / motor_pid_ki / motor_pid_kd | `[session]`高级参数，原cc为64/32/48；一般只调共用速度，不改它们 |
| motor_pid_pwm_limit | `[session]`PWM限幅，原cc为12000ns，可调100..50000；并非目标速度 |
| motor_run_time_s | 本阶段总运行上限0..120秒；大于0自动驱动两轮，0完全禁用电机；阶段6必须为0 |
| steer_command | 舵机命令，负左正右，当前±30；并非前轮实际转角 |
| hold_time_s | 主转向保持0..120秒；0不启用该转向计时 |
| correction_steer_command | 阶段2的右修正/阶段4的左修正命令 |
| correction_hold_time_s | 反向修正保持秒数，0关闭修正 |
| settle_time_s | 写舵机后静止等待0.5..5秒，不计入电机运行时间 |
| line_follow_enable | 仅阶段1，1启用前相机巡线，0固定角度前移 |

周期50000ns，即20kHz。PID输出3000=6%、10000=20%、20000=40%、50000=100%。
2000在新车是4%，不能沿用旧车“2000=20%”的理解。12000仅为原cc的可调默认限幅，没有锁死2000或12000。
程序根据原生速度误差更新PWM；同一目标转速下，左右PWM可以不同。
新车速度模式只允许 `[session] motor_target_rps` 作为目标速度；左右独立目标或阶段目标覆盖会报错，避免“改了全局值却没有生效”。原cc的左右独立编码器反馈和PID输出仍然保留，两轮PWM不必相同。旧车factory模式继续使用原始PWM配置。
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
- CSV增加MOTOR_PID_START/MOTOR_PID_UPDATE：目标来自配置快照/开始事件，逐次记录实测rps、左右有符号PWM、实际PWM幅值及方向GPIO。
- 开始事件还记录左右PWM的enable和GPIO73值；开始前确认两路PWM使能，读回失败则清零并报错。软件读回不等于已经测得物理转动。
- `stage_timing_summary.csv`新增阶段名称、电机实际输出时长、主转向实际时长和反向修正实际时长；所有实际执行的阶段会打印持续时间。未执行阶段标为NOT_RUN且时长留空，绝不填成0秒。重复停机清零和Y/N等待不会延长已结束的时间。
- 时长统计旁路记录，不作为新阶段结束条件；既有hold_time_s、motor_run_time_s、手动结束和切段行为保持原样。
- 新车编码器采用原生rps（转/秒），不套用旧车delta counts，也不凭默认轮周长估算cm/s。
- 测量周期寄存器可能保留旧值；记录不会据此声称已经物理停稳。
  PWM=0与GPIO73=0均做软件读回，采集0.5秒尾段后结束。
- 新车资料未确认前后超声波及灰度接口，留空并标记UNCONFIGURED；
  阶段5按你设定的时间执行，本工具不自动按挡板距离停车。没有启用IMU。

## 接口依据和改动检查

用户提供的新 `cc(1).zip` 与前一份cc的电机/PWM/GPIO实现一致：
左PWM pwmchip8/pwm2、右PWM pwmchip8/pwm1、方向GPIO12/13、
总使能GPIO73；方向0前进/1倒退。舵机pwmchip2/pwm0、周期3040000ns，
脉宽1520000+(命令+中位偏置)*period/1800，中位偏置初始-15来自参考工程。
编码器基址0x1611B000，通道0/3，GPIO75/72，100MHz、1024线。

五项检查：共用原阶段逻辑避免复制；sysfs/nativeRps命名区分单位；
控制和I/O分层；电机PWM恢复参考顺序export→enable=1→period→duty=0，随后设置方向GPIO和总使能；舵机恢复period→初始脉宽1520000→enable=1。不再前置写enable=0或强制polarity；初始enable若被驱动拒绝会明确打印并按参考顺序继续，不能据此认定硬件已生效。正常阶段输出和独立定时停车线程保持原样；
速度控制复用已有legacy/Contral/PID/PID.c；同原cc逐轮计算，Kp/Ki/Kd=64/32/48，每50ms更新。
共用速度只解析一次并继承给各段；新车阶段启停仅由运行时间决定，不依赖旧左右PWM；正常转弯修正不重置控制器，重新授权/保存重执行才清空PID历史。
独立停车锁阻止超时、暂停后产生新的PID输出；初始PWM使能失败后，在正式电机启动前再次检查和补写。
新车物理转向和阶段时长需在实车调。硬件/车辆安装配置改变后须重新启动；运行中只重载调参INI。
离线模拟能验证输出顺序和错误路径，不能代替实车跑动确认。

本版本离线验证覆盖原cc PID计算和内存池复用、共用速度解析/保存执行、阶段状态机、独立PID更新/停机、PWM使能补写及失败清零、两次启动和部署脚本；原cc普通巡线源码/配置64项对照通过，相对舵机命令及限幅31项测试通过。Linux入口和图像模块使用实际OpenCV头文件完成目标文件编译，未链接目标板OpenCV，也未执行实车硬件验证。
未在此Windows环境生成或运行LoongArch可执行文件，需按上面的虚拟机命令编译。

## 阶段1原cc巡线（2026-10-10.5）

直接复用cc(1).zip普通NewTrack/7.25巡线链路：相机帧先按原Camera最近邻缩放80×60，裁剪顶部5%、底部2%；原HandleImage用HSV范围(0,0,46)至(180,48,255)提取白色区域，11×1横向闭运算用于边界。原FindLine/FindStartPos/FixLine找边和补线，关闭图像边缘代理，原NewTrack逐行历史路宽补单边界。中心黑线可靠时用近区加权黑线，否则按原单边界偏置、局部黑线70%混合兜底。

原center_pratio=0.42；舵机误差为4×atan2(目标x−宽×0.42,高/2)，转换为度并按原代码转float。原Steer接受90+误差，本程序相对角接口直接接收误差，沿用既有机械偏置和脉宽换算。只保留最后的line_max_command及车辆±30限制，没有原先的PD/滤波/预瞄/变化率限速。底层电机仍是cc原速度PID，motor_target_rps只控制轮速。

原有效边界也可在黑线暂时不可靠时巡线。现有连续3帧且0.1秒启动确认、持续丢线停车、时间上限、暂停和结束锁仍然保留。原比赛的发车右偏、蓝锥、环岛、斑马线、计圈及自动车库不在普通巡线范围内，也不会覆盖阶段1指定速度或持续时间。阶段2～5继续使用既有角度/时间倒车。

新版启动自动备份并清理阶段1不再生效的旧PD字段；保留line_follow_enable、line_max_command、line_acquire_s、line_lost_s以及成功的角度和时间。老文件若保留line_max_command=8，最终仍按8限幅；新模板为15。原FindLine的三个数组释放改为delete[]，不改变算法；其余来源及核对方法见legacy/cc_lane/ORIGIN.md。

新车LINE_FEEDBACK的line_target_x为了兼容旧记录统一换算到160像素宽，算法内部仍用80像素；line_cc_error单位command_deg，表示原cc计算出的相对舵机命令，不是实际前轮角度或车体航向。新车不输出没有独立计算的line_near_x/line_far_x和旧line_filtered_error。LINE_STEER_WRITE额外记录CC_725、CENTER_BLACK/SINGLE_EDGE/BOUNDARY来源、限幅前误差；舵机列记录实际下发命令，便于检查限幅是否发生。

构建脚本恢复交叉编译→校验ELF架构/程序版本→打包流程，只构建parking_rehearsal，不在x86虚拟机执行龙芯程序。reference_cc_lane_tests和reference_cc_steering_tests源码保留，可在匹配架构及OpenCV环境中单独构建运行；其图像测试包括空帧、错误格式、丢线、中心黑线坐标/正负误差、缩放一致性及1000帧重复处理。当前Windows已执行无硬件控制/几何测试并完成入口目标文件编译；虚拟机交叉构建已完成主程序编译和链接，完整OpenCV图像测试尚未实际执行，不能视为实车效果验证。部署包名中的verified表示ELF/版本/文件校验，不表示全部测试或实车验证通过。
