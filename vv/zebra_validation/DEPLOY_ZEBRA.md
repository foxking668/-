# 横道独立模块更新：VV17_ZEBRA_TEST_20261020.2

Windows源码：F:\共享文件夹\vv_zebra_test。虚拟机共享挂载按本次现场确认为 /mnt/hgfs/share。
源码包：vv_zebra_test_20261020_2_源码.zip。需在原Linux虚拟机重新编译；不附旧二进制。

## 1. 虚拟机编译、自动打包

```bash
cd /mnt/hgfs/share/vv_zebra_test &&
bash build_zebra.sh
```

如自动查找不到目标OpenCV：

```bash
cd /mnt/hgfs/share/vv_zebra_test &&
OPENCV_DIR="/home/gy/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4" bash build_zebra.sh
```

成功末尾会打印版本号和 Built vv_zebra_test_board.zip。build_zebra.sh自动调用tools/package_zebra.py生成全新的ZIP，并检查程序中含本版本标记，防止误打包旧二进制。打包需要Python3标准库，无需pip安装。

把该目录这次新生成的 vv_zebra_test_board.zip 上传至板子 /home/root/gy2/vv_zebra_test。
ZIP只含新程序、启动脚本、默认横道配置、构建版本文件和程序SHA256清单，不含image_params.txt。
程序校验清单不校验现场可修改的横道配置，避免保留调参后出现伪校验失败。

## 2. 板子更新并检查

退出原控制程序、关闭舵机开关后执行：

```bash
cd /home/root/gy2/vv_zebra_test &&
unzip -n vv_zebra_test_board.zip config/zebra_test.ini &&
unzip -o vv_zebra_test_board.zip -x config/zebra_test.ini &&
sha256sum -c ZEBRA_PROGRAM_SHA256SUMS &&
chmod +x vv_zebra_test run_zebra_test.sh &&
cat ZEBRA_BUILD_INFO.txt &&
test -s image_params.txt &&
grep -nE '^[[:space:]]*(steer_offset_deg|experiment_enable)[[:space:]]*=' image_params.txt
```

版本应为 VV17_ZEBRA_TEST_20261020.2；本次现场应仍有 steer_offset_deg=20、experiment_enable=0。
命令保留板上image_params.txt和已有config/zebra_test.ini；首次安装才提取默认横道配置。
若现场参数文件缺失，此检查会停止，需要先从本车已调好的原程序复制，不能用源码模板覆盖。

## 3. 正式运行

在原来能显示摄像头画面的板子桌面终端，按本车原开关操作准备后执行：

```bash
cd /home/root/gy2/vv_zebra_test &&
sh ./run_zebra_test.sh
```

沿用当前自动启动方式：硬件成功后约3秒自动启动，不需要Enter，没有新增两次启动授权。Ctrl+C取消。
正常应看到 [PWM_INIT] ... mode=reused（或首次导出的exported）、两轮和舵机Initialize Success、Hardware Initialize Success。
看到STARTUP REJECTED则不会创建巡线/电机线程；events.jsonl记HARDWARE_INIT_FAILED和具体设备路径、errno。

实际模式提示应为mode=INDEPENDENT_ZEBRA，无环岛/锥桶/比赛计圈/旧横道门控/倒库任务。
流程仍为自动巡线→横道确认和语音指令→右轮反转、左轮0→停稳确认→至少3秒→自动巡线驶离。

退出Ctrl+C，等[ZebraTest] CLOSED后检查当次recording_status.txt。CLOSED是关闭流程完成提示，实际成功看state=CLOSED、raw_verified=1、diagnostic_verified=1和错误字段为空。
硬件初始化失败时尚未开始录像，状态会记为INCOMPLETE；这不代表已经有可用录像。

## 改动及边界

- 已导出PWM验证period/duty_cycle/enable的读写可用性和数值后复用；不会仅凭目录存在判成功。
- 首次导出等待节点就绪；即使export报错，也必须通过节点验证才允许继续。
- 启动配置顺序为禁止输出、清零旧占空比、设置周期和目标占空比、回读确认、使能并回读。
- 两轮初始化期间nSLEEP保持低；任何一步失败关闭已持有输出，主线程拒绝发车。
- GPIO同样复用已有节点，检查实际写入；两轮和舵机共用一套sysfs写入错误记录。
- 舵机首次脉冲先应用现场steer_offset_deg。保留VV17原公式、3040000ns周期和引脚，没有引入其他车的PWM参数。
- 本次未改识别阈值、速度、右轮反转参数、3秒时间或现场图像模板。没有实现横杆检测。

新增回归用模拟文件节点，不是实际Linux内核驱动或电气行为；本修复版还需要在龙芯板重新编译并实车确认。

首次导出的0周期/0占空比/禁用通道会先建立非零周期，再进入统一配置流程；0周期却启用的异常元数据拒绝。
此分支依据Linux PWM core.c pwm_apply_state的非零周期校验及sysfs.c的状态应用方式处理：
https://github.com/torvalds/linux/blob/v6.6/drivers/pwm/core.c#L481-L483
https://github.com/torvalds/linux/blob/v6.6/drivers/pwm/sysfs.c#L125-L138
这是通用内核接口依据，并非声称已核对板子当前内核源码。
