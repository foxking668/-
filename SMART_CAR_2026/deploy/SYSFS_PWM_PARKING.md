# 原版7.25舵机公式入库：2026-10-16.5

当前板子使用sysfs接口、无/dev/zf_*。本版按用户指定的2026-09/7.25.zip复用舵机公式、PWM1通道、周期和-15运行偏移。默认跳过相机，沿用已有电机PID、阶段时间和两次启动流程。旧CC/XX入口、另一个SMART_CAR_2026 -2工程不修改。

## 舵机公式与调用顺序

已核对压缩包内Steer.hpp、Steer.cpp、Global.cpp和ImageHandle.cpp的活动代码。注释中的0.52..2.52ms说明与实际表达式不一致，本版以活动代码为准：

```cpp
// Steer.hpp
period = 3040000;
// ImageHandle.cpp -> Steer::setAngle
float angle = 90.0f + float(steer_command);
angle += -15.0f;
float duty = (((angle - 90.0f) / 90.0f) * 5.0f)
           * (float)(period / 100.0f) + 1520000.0f;
// 原PwmController::setDutyCycle接收unsigned int：截断，不四舍五入。
```

Global.cpp先steer->init()，内部setAngle(90)、写入1520000ns后enable；之后才setSteerOffset(-15)。本版保留该顺序，因此初始化脉宽与运行时零命令脉宽不同：

| 状态/命令 | 写入脉宽ns |
|---|---:|
| 原版初始化，偏移尚为0 | 1520000 |
| steer_command=-10 | 1477777 |
| steer_command=0，运行和自动回正 | 1494666 |
| steer_command=10 | 1511555 |

通道/sys/class/pwm/pwmchip1/pwm0/duty_cycle；周期3040000ns，约328.947Hz。禁止再次通过车辆steer_offset_deg叠加-15；配置仍保持steer_offset_deg=0、steer_sign=1，原偏移在专用输出层只应用一次。

方向改用steer_command，0名义回正、负左正右、可用小数。软件命令范围-75..105来自最终angle=75+command限制0..180，另受车辆max_steer_deg约束；这是命令范围，不是实测车轮转角或机械安全行程。初始化、阶段命令、自动回正、停机回正均走同一份原公式函数，初始化显式使用偏移0，运行/回正使用偏移-15。

不再把servo_pwm=3370/4470/5170解释为当前板子的实际舵机标定，也不按10000刻度计算舵机脉宽。10000刻度仅保留给原有电机输出上限换算。

## 构建与部署

源码已位于共享工程SMART_CAR_2026；在已有工具链和OpenCV配置的虚拟机运行：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
sh deploy/build_sysfs_pwm_parking.sh
```

默认复用/home/gy/builds/SMART_CAR_2026-loongarch。脚本只编译目标程序并校验LoongArch ELF/版本后生成deploy/parking_sysfs_pwm_20261016_5_verified.zip，不在x86主机执行LoongArch测试。

上传上述ZIP到板子/home/root/gy后：

```sh
cd /home/root/gy
unzip -o parking_sysfs_pwm_20261016_5_verified.zip
sha256sum -c SYSFS_PWM_SHA256SUMS
chmod +x parking_sysfs_pwm_20261016
sh ./run_sysfs_pwm_parking.sh
```

应看到VERSION 2026-10-16.5、SERVO_725_COMMAND_LIMIT和CAMERA_DISABLED。旧二进制与新脚本不能混用。本机没有新生成可上板的LoongArch二进制，需由上述虚拟机构建命令完成。

## 原有配置的保留、备份与迁移

主调参文件仍是/home/root/gy/config/parking_tuning.new_car.ini，存在时绝不覆盖为模板。新模板使用steer_command、correction_steer_command；全部居中且电机时间0。

如果已有文件仍含servo_pwm/correction_servo_pwm，启动器先生成.before_original725_时间_进程号.bak完整备份。只有旧中位4470或4470.0可明确迁移为命令0；速度、阶段、时间和其他文字保留。候选文件通过不访问硬件的--check-config检查，并确认原文件未被并发编辑后替换；失败则保留原文件和备份、清理临时文件，不执行试验。

旧非中位PWM值（如4900、5000、3370）没有经过验证的角度映射，启动器显示OLD_PWM_REQUIRES_ANGLE，保留原文件和备份后退出。用户在原文件把它改成steer_command或correction_steer_command，并重新填写角度命令。不能把旧4900直接写成steer_command=4900，也不能同时保留同一阶段的两个主转向字段。解析器只兼容旧中位别名，不接受非中位猜测转换。

独立硬件文件config/sysfs_pwm_hardware.ini只自动迁移已知PWM1/PWM2与3040000/3333333组合到PWM1、3040000，改前生成.before_725_servo_备份并校验。编码器符号、GPIO、电机等其余项不变。未知/重复路径或周期直接拒绝。硬件和调参迁移分别校验与备份，后续硬件校验失败不会执行试验；已经成功迁移的调参字段仍为同义零命令。迁移操作本身不访问硬件，check-config不消耗第一次启动的准备标记。

## 参数示例与运行

原文件中修改对应区段，不重复添加同名section。首次先静止确认名义回中：

```ini
[stage_2]
steer_command=0
settle_time_s=0.5
hold_time_s=2
motor_run_time_s=0
correction_steer_command=0
correction_hold_time_s=0
```

观察正常后再重新启动，逐步调主转弯负命令及右修正正命令。例如主转弯-10、hold_time_s=2，右修正5、correction_hold_time_s=1；主转弯时间到后自动反向修正，再回零。第二弯主命令为正、修正为负。是否符合实际机械方向仍需本车确认。

电机启停仍由motor_run_time_s控制，0禁用；正值可用小数0..120秒。第一段前进，第二至第五段倒车，第六段停车。电机总时间和最后一个舵机动作计时先到者停机，不会因修正动作重置电机总时间。

1. 关舵机开关，第一次运行，等准备提示后Ctrl+C退出。
2. 再运行相同命令，正式试验提示后开舵机，按一次Enter授权本阶段。
3. 先设置舵机、等待settle_time_s，再执行阶段电机和保持计时。
4. P+Enter停机并暂停记录；C+Enter只恢复记录，Enter再授权运动。Q+Enter结束；最后Y/N直接按键保存，不自动重启。
5. 运行中保存当前阶段参数仍是执行授权，会停下原动作后重新设置/计时。编辑前建议先P暂停或退出。

保持原两次启动身份校验：阶段、版本、硬件配置或相机模式变化均需重新准备。不会自动串行完成六个阶段。用户不需要量角器，但代码命令与车轮实际角度不能混称。

## 无相机与数据记录

启动器默认--no-camera：准备和正式运行均不打开相机、不取帧、不录像、不巡线。阶段1即使配置line_follow_enable=1，也按设定舵机命令和电机时间前进。有效编码器采样刷新原电机看门狗；主循环每次最多等20ms终端输入，不空转。frame_timeout_s仍约束控制/编码器循环，不扩大、不取消。

保留六阶段CSV、stage_speed_summary.csv、stage_timing_summary.csv、配置快照和session.txt。无相机使用CONTROL_TICK，不伪造FRAME/视频文件；配置命令、最近命令都为steer_command单位。SERVO_725_WRITE打印命令、脉宽、周期、偏移。元数据记录原公式标识、初始化脉宽、运行偏移及camera_enabled=0。不使用IMU、未配置超声波，不推断物理停车成功。

## 电机逻辑保持原样

电机仍为原7.25增量PID64/32/48、50ms，双轮驱动；左PWM8/2、右PWM8/1，周期50000ns。方向GPIO12/13、使能GPIO73保留。输出GPIO采用完整write判定、回读只诊断；PWM和方向属性严格读回。输入方向GPIO75/72与0x1611b000周期寄存器0/3仍为原生rps采样，不当成读清零delta。

motor_target_counts保留为PPR1024等效脉冲/5ms，rps=counts/(1024*0.005)，整数1..400，40=7.8125rps。motor_raw_pwm仍仅作输出上限，20..3500按10000刻度转成100..17500ns；motor_control=speed。没有把舵机公式变化扩散到速度/方向符号或电机参数。

## 检查与限制

重复代码：删除旧舵机10000刻度换算及XX巡线PWM控制副本，原公式仅在一个专用函数实现，继续共用阶段/电机/PID线程。命名：reference725明确原公式来源，区分角度命令、实际脉宽和电机占空比刻度。结构：配置转换只处理电机单位，舵机命令原值进入原公式。隐藏bug：覆盖-15重复偏移、初始化/回正区别、浮点截断、旧周期拒绝、旧字段冲突、非中位误转换、备份与并发编辑。优化建议：先验证静止零命令，再小步调整转向，保持已有电机参数，避免同时修改多项。

原公式的732项离线检查覆盖初始化、运行零位、边界、无效输入及整个命令区间每0.25一档与原表达式的对照。另有原阶段、电机、GPIO、启动及迁移测试。离线计算和sysfs回读不能证明重烧内核后实际波形正确；当前原版7.25尚未在重烧后的板子复测，不声称已排除硬件/内核因素。

本次结果：2286项控制/配置/原公式检查通过（原公式732、PWM配置53、sysfs模拟121）；14项Python打包/启动测试通过（含旧配置备份、零位迁移、非中位拒绝、未知配置和并发修改）；6个Linux源文件编译通过；两个shell脚本语法通过。真正LoongArch链接和实车波形仍由部署验证。
