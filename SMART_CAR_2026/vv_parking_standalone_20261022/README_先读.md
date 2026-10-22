# VV 独立入库测试 2026-10-22.1

唯一基线：`vv_20261022_1809_3a31be32.zip`。参考 2026-10-11 交付的组织方式，实际算法、硬件接口和参数取自本次 VV，不使用旧交付的 cc 巡线或旧转向参数。

默认流程：**手动 Enter → 原巡线寻找人行道 → 原横道反转及停车保持 → 原巡线前进 4000 ms → S2 → S4 → S5 → 停止并退出。**

程序标识必须是 `VV_STANDALONE_20261022.1`，可执行文件名是 `vv_parking_test`。原压缩包里的比赛二进制 `vv` 没有放进本工程，也不能改名当作独立程序使用。`vv/` 内保留基线完整源码和依赖；本工程唯一构建入口是根目录的 `build_test.sh`，唯一参数入口是根目录的 `image_params.txt`。

## 编译

需要 CMake 3.16+、C++17 编译器、与目标平台匹配的 OpenCV 开发库。龙芯交叉编译器默认目录与上传源码一致。

在原来编译 VV 的 Linux 虚拟机中执行，`OpenCV_DIR` 填该机器上龙芯 OpenCV 的真实目录：

```bash
cd /你的路径/vv_parking_standalone_20261022
OpenCV_DIR=/实际的龙芯OpenCV安装目录/lib/cmake/opencv4 bash ./build_test.sh
```

工具链安装位置不同可同时设置 `TOOLCHAIN_DIR`。不能使用 x86 OpenCV 编译龙芯运行包。编译成功后，根目录生成新的 `vv_parking_test`；上传整个工程目录或至少上传该程序、根参数文件和运行脚本。源码中的 `vv/build.sh` 和 `vv/CMakeLists.txt` 是原始参考文件，不用于本独立入口。

如果龙芯板上已经有本机 C++ 编译器、CMake 和 OpenCV 开发库，也可以直接编译：

```bash
bash ./build_test.sh --native
```

本交付不声称已完成龙芯交叉链接或上车验证；实测范围见 `VALIDATION.md`。不会把仅通过语法检查的代码称为已生成板端程序。

## 板端操作

示例把工程放到 `/home/root/smartcar/vv_parking_standalone_20261022`，进入该目录后运行：

```bash
bash ./run_parking_test.sh --check-config
bash ./run_parking_test.sh
```

`--check-config` 只打印实际参数并校验，不初始化硬件，不发车。正常启动沿用当前 VV 的电机、GPIO、编码器、舵机和相机初始化；打印“初始化完成”后等待。把车放在人行道前可巡线的位置，按 Enter 才开始寻找人行道。无需先跑两圈，也不用手动伪造斑马线触发。

Q 加 Enter 或 Ctrl+C 提前结束；流程完成后自动输出零动力、结束控制线程，再排空并验证录像。每次试验重新启动一次，参数保存不会自动重新发车。使用本工具时结束原比赛程序，避免两个程序同时访问同一硬件。脚本的文件锁只保证本独立目录内不重复启动。

## 测试选择

```bash
bash ./run_parking_test.sh --mode full
bash ./run_parking_test.sh --mode approach-only
bash ./run_parking_test.sh --mode reverse-only
bash ./run_parking_test.sh --mode s2
bash ./run_parking_test.sh --mode s4
bash ./run_parking_test.sh --mode s5
```

- `full`（默认）：找人行道后按完整流程执行。
- `approach-only`：同样先找人行道、执行反转及保持、前进 4 秒；在 S2 前停止，用于比较你现在关心的前进终点。
- `reverse-only`：手动摆好倒车起点，Enter 后直接执行 S2→S4→S5。
- `s2/s4/s5`：使用当前对应阶段原参数，只执行所选阶段。若该阶段在参数中关闭，则不会替你重新开启。
- 可加 `--no-video` 关闭录像；默认记录原四宫格处理录像。录像启动失败时不启动运动。

这些模式共用一个原始倒车控制器；前进阶段共用原 VV 巡线和速度 PID。附加模式只裁剪测试范围，不另写一套转向或计时算法。

## 当前默认值及时间含义

根目录 `image_params.txt` 与上传 ZIP 字节一致，195 个参数项完整保留。`vv/image_params.txt` 是基线参考副本，运行不读取它；调参只改根目录文件。

| 参数 | 默认值 |
| --- | ---: |
| `motor_aim_speed` | 7 |
| `center_pratio` | 0.48 |
| `error_gain` | 3.5 |
| `steer_offset_deg` | 95 |
| `new_track_zebra_reverse_pwm` | 12000 |
| `new_track_zebra_reverse_ms` | 800 |
| `new_track_zebra_reverse_left_enable` | 0 |
| `new_track_zebra_stop_ms` | 3200 |
| `new_track_zebra_finish_stop_delay_ms` | 4000 |
| `new_track_post_stop_center_deg` | 102 |
| `new_track_post_stop_settle_ms` | 200 |
| `new_track_post_stop_left_motor_enable` | 0 |
| `new_track_post_stop_right_motor_enable` | 1 |

| 阶段 | 相对中位偏移 | 右轮倒车 PWM | 总时间 ms | 驱动后偏置保持 ms |
| --- | ---: | ---: | ---: | ---: |
| S2 | -107 | -12700 | 2450 | 2000 |
| S4 | +108 | -12700 | 3500 | 2000 |
| S5 | 0 | -12700 | 1700 | 0，全程保持 |

S2/S4/S5 的总时间包含各阶段开始的 200 ms 停驱动时间，倒车输出时间分别约 2250/3300/1500 ms。保持时间从开始驱动计；S2 后 250 ms、S4 后 1300 ms 在入库中位继续倒车。循环调度和原舵机 PWM 重启事务会产生实际时间差，日志用于观察这些差异。

舵机 `setAngle` 命令分别为 -5、210、102，入库中位约 1700711 ns。命令不是实测车轮角度。PWM、GPIO、编码器读取、舵机换算及速度 PID 实现均与上传基线相同。

横道 3200 ms 保持仍从识别时开始，包含 800 ms 反转；本次没有顺带改成“反转结束后再保持 3200 ms”。前进 4000 ms 仍从电机线程第一次下发正向速度控制指令后开始，包含加速过程；没有加入新的减速、滤波、距离停车或停稳判断。S2/S4/S5 继续冻结触发时的参数，视觉不控制倒车转向、动力或完成条件。

## 独立运行所需的场景隔离

只在这个新入口中生效：建立明确的“测试用第二圈上下文”，跳过圈数累积；不执行环岛、锥桶、旧自动车库、出库右偏和按圈/首次横道触发的临时中心偏置；人行道检测无需等待每圈 7 秒门控。原横道远近区判据、确认帧数、图像裁剪和普通巡线计算均保留。

这些改变是让车从人行道前直接测试所需的外层调度隔离，不是巡线优化。比赛任务原始入口仍在 `vv/src/main.cpp`，但独立目标不编译它。根参数文件未被脚本修改，原工程目录也未修改。

## 每次记录

`runs/毫秒时间戳_进程号/`：

- `image_params.start.txt`：启动时完整参数快照。
- `events.txt`：测试模式、实际冻结的倒车参数、阶段变化及异常。
- `motor.csv`：单调时钟相对时间、阶段、PWM 命令、目标速度、编码器实际速度和由 RPS 积分得到的圈数估计。
- `frames.csv`：有效性和原巡线误差，供对齐控制记录。
- `processed.avi`、`video_timestamps.csv`、`recording_status.txt`：原处理管线四宫格及逐帧数据，默认 10 fps、队列 4。
- `result.txt`：退出原因、录像帧数及验证状态。

`SEQUENCE_DONE_NOT_PHYSICAL_SUCCESS` 只表示动作序列结束，不表示车体已经实际入库或未碰挡板。圈数积分是编码器速度积分估计，不是 IMU 航向或直接位移测量；PWM 列是下发命令，不是电气实测。启动到触发间仍保持原配置热加载行为；倒车参数以 `events.txt` 的冻结值为准。

## 源码核对与测试

`ORIGIN_SHA256.json` 记录原 ZIP 和 142 个基线文件的 SHA256。基线只改了 4 个接入文件：NewTrack 头/源、ImageHandle、MotorHandle。新增测试运行状态/旁路记录头文件；`ParkingController.hpp`、`ParkingRuntime.hpp`、舵机、电机、编码器、PID、相机、二值化和横道确认实现保持字节一致。配置加载节流状态纳入原互斥锁，防止两个线程同时访问加载时间。

```bash
python3 tests/check_extraction.py
cmake -S . -B build-tests -DVV_HOST_TESTS_ONLY=ON
cmake --build build-tests
ctest --test-dir build-tests --output-on-failure
```

电脑测试检查原控制器边界、4 秒起算点、模式选择、阶段零输出、原横道确认与参数来源。完整相机、驱动、板端链接和实车轨迹仍需在目标设备验证。
