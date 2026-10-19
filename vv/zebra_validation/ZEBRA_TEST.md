# VV17 人行横道独立验证程序

本程序基于用户提供的 vv(17).zip，单独生成 vv_zebra_test。普通找线、转向、PID、电机/舵机/编码器引脚及单位沿用 VV17；新增模块集中在 src/ZebraTest/。02(1).zip 没有源码，其程序与 vv(17) 二进制不同，因此本交付不声称复现了02二进制。

## 测试范围

自动巡线 → 远区候选接近限速 → 近区连续确认 → 左轮0、右轮反转制动 → 双轮0且编码器接近静止 → 保持至少3秒 → 自动起步巡线 → 实际测到轮子运动且横道连续离开有效画面后重新使能。

不检测横杆。本独立程序运行时强制关闭比赛环岛/锥桶/倒Y计圈/终点停车/倒车入库/发车定时偏转，不依赖每圈7秒屏蔽。比赛 image_params.txt 原样保留，不会为了测试写入这些开关。重新加载参数也不能把这些比赛任务重新打开。experiment_enable 必须为0。

## 文件与参数

- config/zebra_test.ini：全部横道检测、右轮反转、停车保持、驶离锁存、看门狗和记录参数，有逐项中文说明。只在启动读取，修改后重启；重复键、未知键和越界值拒绝启动。
- image_params.txt：VV17原始现场参数，包含 motor_aim_speed、steer_offset_deg、转向/PID及裁剪。该文件的原有热加载继续保留。上车时优先保留车上已调好的版本，不要用包中样例覆盖现场参数。
- src/ZebraTest/ZebraDetector.cpp：多行黑条、白间隔、宽度一致性、灰度对比、条纹组方向与分叉发散检验。
- src/ZebraTest/ZebraTest.cpp：不依赖硬件的控制状态机。
- src/ZebraTest/ZebraRuntime.cpp：与原图像/电机线程连接，后台录像、后台日志及事件记录。

本机位实拍有明显阴影，原单一Otsu阈值会把近处白路面判黑。因此新增 illumination_band_rows=9：按每行上下9行窗口估计Otsu，巡线和横道使用同一份补偿后的原始二值图。设置0回到原ROI单阈值。该设置只在独立程序生效。并未把闭运算/补线后的图当作条纹输入。

原7.25找线发现顶部只有两个轮廓点时访问第三个元素的越界，两处同类代码均作边界修正；不改变点数大于2时的选择规则。

## 编译

在原Linux虚拟机（已安装LoongArch交叉工具链与目标OpenCV）进入本交付文件夹：

```bash
OPENCV_DIR="/home/gy/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4" bash build_zebra.sh
```

也可用 bash build_zebra.sh 自动检查原工程常用的OpenCV路径；若工具链目录不同：

```bash
TOOLCHAIN_DIR="/实际/LoongArch工具链目录" OPENCV_DIR="/实际/OpenCV/lib/cmake/opencv4" bash build_zebra.sh
```

默认工具链目录与vv(17)一致，为 /opt/loongson-gnu-toolchain-8.3-x86_64-loongarch64-linux-gnu-rc1.6。这里需要目标LoongArch OpenCV，不能使用虚拟机x86版OpenCV。

若在小车Linux系统本机已经安装编译器、CMake、OpenCV开发文件，可选择本机编译：

```bash
ZEBRA_NATIVE=1 OPENCV_DIR="/本机/OpenCV/lib/cmake/opencv4" bash build_zebra.sh
```

本次交付为源码，不附带冒充新版本的旧vv二进制。build_zebra.sh 生成 ./vv_zebra_test，不修改原 build.sh，不清空旧构建目录。

## 小车运行

例如上传到 /home/root/smartcar/vv_zebra_test/，至少保留：

```text
vv_zebra_test
run_zebra_test.sh
image_params.txt                # 使用本车已调好的文件
config/zebra_test.ini
```

在小车终端执行：

```bash
cd /home/root/smartcar/vv_zebra_test
chmod +x vv_zebra_test run_zebra_test.sh
./run_zebra_test.sh
```

沿用原程序启动调试窗口，正常验证无需按C/D。录像与日志自动开启，不需要再运行vvv，也不要同时运行其他小车控制程序。保留与原程序兼容的 /tmp/cc_terminal_run.lock 进程锁。

结束按 Ctrl+C：先等待原图像/电机线程退出、双轮停止，再排空录像队列并逐帧校验视频；等 [ZebraTest] CLOSED 后再断电。该提示表示关闭流程完成，成功与否还要看recording_status.txt的CLOSED/INCOMPLETE及各错误字段。

## 记录目录

记录在二进制旁边，不依赖启动时工作目录：

```text
logs/run_日期_时间_单调时钟毫秒/
  run_info.json
  frames.csv
  control.csv
  events.jsonl
  recording_status.txt
  config_snapshot/
    image_params.txt
    zebra_test.ini
  camera/
    camera.avi
    video_timestamps.csv
    recording_status.txt
  diagnostic/
    diagnostic.avi
    video_timestamps.csv
    recording_status.txt
```

两路视频分子目录，避免原录像模块的同名时间戳/状态文件互相覆盖。每帧以frame_id/capture_ns关联；视频抽样上限默认10fps，控制不受该采样率限制。实际视频播放时间与真实采集时间不等价，使用各路video_timestamps.csv。

frames.csv每处理帧记录输入有效性、道路有效性、远近区结果、条纹/合格行数、灰度对比、最近条纹行、转向误差指令、状态及拒绝原因。raw_submit/diagnostic_submit取值来自RecordSubmit：0禁用、1入队、2按帧率跳过、3队列满丢帧、4失败。入队不能冒充已写盘；最终落盘帧以video_timestamps.csv为准。

control.csv记录目标RPS、实测编码器RPS、左右PWM命令及写入成功标志，含单调时钟毫秒。PWM是命令，不是实测电压/转矩；转向误差不是实测轮角。读数成功只说明寄存器和方向GPIO可读取，不能证明编码器物理上绝无冻结或接线故障。

events.jsonl记录状态转换、制动结束原因、语音发送结果、摄像头/编码器/执行器/写盘异常。设备错误尽可能包含操作、节点路径、errno。VOICE_SENT只证明接口返回成功，是否能被裁判清晰听见还需现场检查。

后台每storage_check_ms观察一次image_params.txt变动，保存image_params.observed_N.txt并记PARAM_FILE_CHANGE_OBSERVED。该记录表示发现文件变化的时刻，不冒充两个原参数加载器恰好应用变化的时刻；极短时间连续多次保存可能只观察到最后一个版本。需要严格比较参数时，每次修改后重启，使用启动快照。

## 调参顺序

1. 先保留现场巡线与舵机参数，确认原图、诊断图都正常保存。
2. 检查二值图：阴影中白路面是否保留，黑条和白间隔是否分开。先调illumination_band_rows，再考虑原二值化偏移，不先放宽条纹条件。
3. 观察far_reason/near_reason与红色候选扫描段。STRIPES_LOW看区域/条纹宽度；ROWS_LOW看连续行；GROUP_SUPPORT_LOW看同一组条纹支持行；BRANCH_DIVERGENCE看分叉发散。
4. 调trigger_y及approach_speed，让车头停在横道前。trigger_y越小越早制动；这是图像比例，不能直接换算厘米。
5. 右轮反转保持用户当前参数12000、最大800ms作为初值，结合两轮反馈与姿态画面标定。左轮始终0，不能用本模块参数使其反转。
6. 核对静止确认与3秒保持。stationary_rps过大会过早判静止，过小会受量化噪声影响。反转/滑行/停稳确认不计入hold_ms。
7. 核对驶离锁存：同一处不反复停车，下一次仍可触发。没有有效图像时不累计消失。

不建议同时修改识别宽松度、速度、制动PWM和触发行，否则难以判断变化原因。所有阈值是可验证初值，未宣称已完成实车停车位置标定。

## 状态与异常

- WAITING：录像与连续有效图像准备期间，双轮0。
- CRUISE / APPROACH：原巡线控制，接近时仅限制速度。
- COAST：双轮0，等待coast_ms后换向。
- RIGHT_BRAKE：左轮0、右轮负PWM；成功写入后开始计时，接近零速或最大时间到则退出。
- SETTLE：双轮0，等待两轮低速持续stationary_ms；超时故障。
- HOLD_3S：实际停稳后保持至少hold_ms；再运动会重新确认，不把运动时间算静止。
- DEPART：原巡线恢复，横道锁存，必须实际测到轮子运动且远近区连续有效消失才解锁。
- LANE_PAUSE：短时有图但巡线失效，双轮0；连续有效后恢复，超lane_loss_timeout_ms则故障。
- FAULT：双轮0并锁存，不能自动起步。退出、检查events.jsonl后重新运行。

记录失败/空间不足也会结束有效测试并停车。录像队列满只丢录像帧并计数；状态日志队列满则故障停车。异常断电/kill -9可能损坏视频和末尾日志，不能保证保存进程崩溃后的数据。

## 验证边界

具体主机测试、650帧实拍回放及Linux编译检查见VALIDATION.md。未执行真实龙芯链接、刷写和实车闭环，不能据此保证制动距离与真实停车位置。当前不具备横杆等待功能，故不代表已完成整套比赛规则。
