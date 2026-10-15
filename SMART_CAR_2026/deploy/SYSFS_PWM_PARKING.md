# 7.25 sysfs适配入库：2026-10-16.1

问题：当前4.19.190+系统有pwmchip2/8，没有/dev/zf_*。用户确认当前板子7.25能跑。本版复用7.25的sysfs电机和编码器，不创建假的设备节点，不改另一个XX工程。

## 构建和上传
在已经配好龙芯工具链与目标OpenCV的虚拟机：
```sh
cd /mnt/hgfs/share/SMART_CAR_2026
sh deploy/build_sysfs_pwm_parking.sh
```
使用现有/home/gy/builds/SMART_CAR_2026-loongarch配置；全新环境须先指定cmake/loongarch64.cmake及龙芯OpenCV。脚本不在x86主机运行LoongArch二进制。
上传 deploy/parking_sysfs_pwm_20261016_1_verified.zip 到板子/home/root/gy：
```sh
cd /home/root/gy
unzip -o parking_sysfs_pwm_20261016_1_verified.zip
sha256sum -c SYSFS_PWM_SHA256SUMS
chmod +x parking_sysfs_pwm_20261016
sh ./run_sysfs_pwm_parking.sh
```
之后只运行run_sysfs_pwm_parking.sh，旧run_new_car_parking.sh仍是XX入口，不覆盖它，也不覆盖旧二进制。

## 配置与执行
继续读取 config/parking_tuning.new_car.ini，保留既有阶段/速度/角度/时间。没有该文件才复制全部居中、时间0的模板。格式或混用参数错误直接报错，绝不把用户配置重置为模板。
新硬件文件独立放 config/sysfs_pwm_hardware.ini、config/sysfs_pwm_vehicle.ini；原new_car_hardware.ini和new_car_vehicle.ini保留。新配置只在缺失时创建。
本版继承sysfs两次启动：第一次关舵机电源，等硬件初始化完成后Ctrl+C，看到PREPARE_COMPLETE；再次运行同一命令，正式试验提示后开舵机，Enter执行。
仍用P+Enter停机/暂停，C+Enter仅恢复记录，Enter再授权运动；Q+Enter结束，Y/N直接保存。运行中保存当前段参数仍会重新授权执行，编辑前最好先停机退出。
六阶段、两弯主转向和反向修正、0..120秒小数时长、录像及六个CSV均复用原状态机。第3阶段时间0表示不驱动，可直接选择stage=4。不会自动跑完六阶段。

## 单位必须区分
- servo_pwm保持3370右、4470中、5170左。新后端把它明确规定为10000逻辑刻度，300Hz周期3333333ns。脉宽=round(servo_pwm*3333333/10000)，三点分别1123333/1490000/1723333ns。没有读取或声称读取到不存在的厂家duty_max。数值不是实测轮角。
- 如果原厂使用的duty_max不是10000，该逻辑刻度不能保证复现原标定。当前保留参数窗口，不扩大机械行程；上车须观察回中/方向。旧7.25命令公式仍保留在原模式，本模式直接写逻辑PWM换算的脉宽。
- 电机PWM周期仍为7.25的50000ns，左pwmchip8/pwm2、右pwmchip8/pwm1；GPIO12/13控制方向，GPIO73使能。前进两侧GPIO0，倒退GPIO1，零PWM停车。
- 编码器仍为7.25的0x1611b000周期寄存器0/3和方向GPIO75/72：rps=100000000/period_ticks/1024*方向。不是XX的读取清零delta，不重复把右侧反馈取负。
- motor_target_counts保留名称，定义为按PPR1024换算的等效脉冲/5ms：rps=counts/(1024*0.005)。40=7.8125rps；范围整数1..400。采样记录真实rps，绝不把周期估计伪装成原生delta，也不把它当停稳证据。
- motor_raw_pwm保留为输出限幅，逻辑刻度10000，整数20..3500。限幅ns=值*50000/10000；1500=7500ns，3500=17500ns。motor_control只支持speed；raw模式会拒绝，防止原始工厂PWM单位混用。
- 速度控制使用7.25原增量PID64/32/48，每50ms执行。XX原15/4/12和20/3/12属于另一单位/周期，不能直接搬用。本版两轮前进/倒退，没有右轮独驱开关。
- 巡线复用7.25的80x60图像识别，保留XX的line_steer_kp/kd、死区和每帧PWM变化上限。识别中心参考为原7.25的0.42宽；每帧输出仍经过3370..5170边界限制。连续有效确认后启动，持续丢线停机，反馈不延长电机总时间。

## 输出与限制
原调参文件完整保存为config_N.ini，日志含逻辑PWM、实际脉宽、目标和反馈rps、实际下发PWM纳秒、方向GPIO、使能及六阶段时间。CSV configured_command/last_written_command是内部center-minus-PWM差值，用户PWM=4470-command；SERVO_PWM_WRITE同时打印可直接核对的PWM和pulse_ns。
参数示例（修改已有段，不重复添加section）：
```ini
[session]
mode=single
stage=2
motor_control=speed
motor_target_counts=40
motor_raw_pwm=1500
```
第一倒弯servo_pwm>4470向左，反向修正servo_pwm<4470向右；第二倒弯反之。correction_hold_time_s=0关闭修正；电机时间0禁用。电机总时间与最后一步舵机计时先到者停机。
本版未启用IMU、未配置超声波，也没有自动测距停车。周期寄存器可能保留旧速度，终端不声明已物理停稳。
Windows模拟设备、状态机及单位换算测试不等于龙芯实车验证。部署ZIP仅在真正LoongArch编译并通过架构/版本校验后生成，不能用旧XX可执行文件代替。

## 本次五项复核
1. 重复代码：未复制停车主程序、阶段状态机、速度PID或sysfs驱动；只新增有明确单位的转换层、PWM巡线控制和独立部署入口。
2. 命名：sysfs_pwm与XX/旧CC入口分开；内部delta、逻辑PWM、脉宽ns、真实rps明确区分。
3. 结构：配置转换->原阶段模型；7.25设备后端->原电机线程；阶段时间及退出保存共用原逻辑。
4. 隐藏bug：验证了负左/正右与原始PWM方向、端点越界不写输出、关机回4470而不是PWM0、两轮使能、错周期拒绝、非法文件重载后恢复、旧配置不覆盖、错误架构二进制拒绝。
5. 优化建议：先固定能稳定运动的目标速度，再调每段时间；最终物理脉宽和编码器响应需上板确认，不能把离线测试等同整车跑通。

本次离线结果：1662项控制检查通过（含原状态机440、启动42、巡线95、舵机35、电机99、sysfs模拟100、cc PID593、cc转向63、新PWM适配47、核心任务148）；12项Python打包/脚本检查；原cc源码对照64项；6个Linux目标文件编译通过。
未完成：真正LoongArch链接和实车运行，需在用户现有虚拟机使用上面的构建命令完成。没有生成或冒充可上板的新二进制。
