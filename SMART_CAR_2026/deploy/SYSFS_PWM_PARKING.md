# 7.25 sysfs适配入库：2026-10-16.3

问题：当前4.19.190+系统有pwmchip1/2/8，没有/dev/zf_*。用户确认当前板子7.25能跑。本版复用7.25的sysfs电机和编码器，不创建假的设备节点，不改另一个XX工程。

## 构建和上传
在已经配好龙芯工具链与目标OpenCV的虚拟机：
```sh
cd /mnt/hgfs/share/SMART_CAR_2026
sh deploy/build_sysfs_pwm_parking.sh
```
使用现有/home/gy/builds/SMART_CAR_2026-loongarch配置；全新环境须先指定cmake/loongarch64.cmake及龙芯OpenCV。脚本不在x86主机运行LoongArch二进制。
上传 deploy/parking_sysfs_pwm_20261016_3_verified.zip 到板子/home/root/gy：
```sh
cd /home/root/gy
unzip -o parking_sysfs_pwm_20261016_3_verified.zip
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

版本.1离线结果：1662项控制检查通过（含原状态机440、启动42、巡线95、舵机35、电机99、sysfs模拟100、cc PID593、cc转向63、新PWM适配47、核心任务148）；12项Python打包/脚本检查；原cc源码对照64项；6个Linux目标文件编译通过。
未完成：真正LoongArch链接和实车运行，需在用户现有虚拟机使用上面的构建命令完成。没有生成或冒充可上板的新二进制。

## 2026-10-16.2：GPIO输出回读兼容修复
用户实车在gpio12初始化写1后回读不一致，版本.1因此中止。核对原7.25 GPIO.cpp：setValue检查write(fd, value, 1)==1，不把随后value读取作为写入成功条件。
本模式对三个输出GPIO（方向12/13和总使能73）统一采用该判定：打开失败、短写、写入失败继续报错；value读失败或不一致只作诊断，每个引脚每个实例最多报告一次GPIO_REFERENCE_WRITE，记录requested和observed。
输入方向GPIO75/72、GPIO direction属性、PWM period/duty_cycle/enable都保持严格检查。旧CC sysfs模式也保持原来的严格输出回读，改动只作用于--sysfs-pwm-control。
MOTOR事件增加gpio_verification=write_ack_only；左右gpio字段是成功下发的方向命令，gpio73_enable仍是独立读到的值，可能为0或-1。不能把write成功、GPIO回读和物理电平混为一谈。停机先严格写零并回读两路PWM，仍尝试GPIO73写0。
对“写1读0”、读取异常、真正写入失败、两侧方向、使能、停机清零和PWM错误均有模拟覆盖。未修改转向/速度/持续时间/阶段参数；未进行新的实车验证。需要重新编译版本.2再部署，修改INI不能修复版本.1。

版本.2复核结果：1678项控制检查通过（sysfs模拟116项，其余1562项）；12项Python打包和启动脚本检查通过；6个Linux目标文件编译通过。针对本次变更的五项检查：共用一个输出GPIO写入函数，无三份重复逻辑；命名明确write_ack_only；严格PWM与GPIO诊断分层；覆盖读不一致、读失败、写失败及停机清零；诊断每引脚限一次防止PID循环刷屏。仅离线验证，不声明实车已跑通。

## 2026-10-16.3：按指定7.25.zip使用PWM1

已核对2026-09的7.25.zip：STEER_PWM_CHIP=1、STEER_PWM_INDEX=0。2026-10的“自动排期选方法”版本是PWM2，两个版本不能混称。本次仅修改此适配模式的舵机通道为/sys/class/pwm/pwmchip1/pwm0/duty_cycle；保持现有300Hz换算、3370/4470/5170窗口、电机、阶段和时间逻辑。没有把旧7.25的3040000ns周期直接套入现有逻辑PWM换算。

启动器会保留已有调参文件。若独立sysfs硬件配置仍为已知旧PWM2路径，先生成带时间和进程号的before_pwm1备份，在临时文件只替换servo_pwm路径，通过不访问硬件的--check-config检查并确认原文件没有同时被编辑后再替换。缺失、重复、自定义舵机路径或检查失败直接退出，不猜测引脚。不修改encoder_right_sign或其他硬件项；再次启动不重复迁移。直接运行新程序也会检查PWM1路径，防止绕过启动器仍误用PWM2。

目前板上已有版本.2也可直接改配置使用PWM1，无须为单独切换通道等待重新编译。程序停止且舵机开关关闭后执行：
```sh
cd /home/root/gy
cp config/sysfs_pwm_hardware.ini "config/sysfs_pwm_hardware.ini.before_pwm1_$(date +%Y%m%d_%H%M%S)"
sed -i 's#^servo_pwm=.*#servo_pwm=/sys/class/pwm/pwmchip1/pwm0/duty_cycle#' config/sysfs_pwm_hardware.ini
sh ./run_sysfs_pwm_parking.sh
```
按原两次启动流程执行；先在选中阶段把motor_run_time_s设0，确认转向后再恢复原电机时间。开启和脉宽回读只说明软件设置，仍需实际观察舵机动作。旧PWM2不再作为本程序舵机输出；启动器迁移本身不操作硬件。

本次五项检查：复用原硬件和状态机，无新增转向控制副本；PWM1路径和备份名称清晰；先验证再替换；覆盖旧配置残留、重复路径、检查失败和并发修改；建议先只改通道，确认舵机正常后再调动作参数，避免同时变更多项。

版本.3验证：1484项相关控制回归检查通过（含117项sysfs模拟）；13项Python打包/启动测试通过，其中迁移测试覆盖6种情形；6个Linux源文件编译通过；两个shell脚本语法检查通过。未进行实车验证，也未在本机生成龙芯部署二进制。
