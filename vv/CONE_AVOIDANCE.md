# vv 锥桶避障与两段偏置（2026-10-15）

本次只修改 `vv`。以实跑目录 `run_20210324_102632_2214_434` 的34.08秒录像、111个已保存帧及逐帧CSV定位问题；录像在约5.18秒、6.61秒、23.43秒有锥桶，而记录的锥桶检测为0。录像队列丢178帧，原始彩色信息未保存，不能据此宣称HSV已完成现场标定。

## 运行与文件

在原来的龙芯 Linux 构建环境中：

```bash
cd /home/root/smartcar/vv
bash build.sh
# 普通运行与录像运行二选一
./vv
# 或
./vvv
```

运行目录必须同时有 `vv`、`image_params.txt`、`cone_avoidance.ini` 和原来的环岛模板。`build.sh` 仍使用工程已有的LoongArch工具链及OpenCV路径，并未改为Windows构建。本机测试程序不是龙芯可执行文件；目录中原有 `vv` 二进制没有冒充新版本。

`image_params.txt` 中 `new_track_cone_sequence_enable=1` 开启动作。独立的 `cone_avoidance.ini` 约500ms检查一次；启动一轮动作时锁定参数，行驶中保存的新值用于下一轮。文件缺失、字段不全、重复键、未知键、非数值、越界或NaN会明确停驱动，CSV的 `cone_tuning_valid=0`。恢复合法文件后可恢复；不要通过删除文件关闭功能，使用总开关。

## 只在 cone_avoidance.ini 调整这套动作

|阶段|参数|初始值|
|---|---|---|
|左偏|`left_pratio`、`left_min_error`|0.65、30|
|左偏|`left_duration_ms`、`left_speed`|500、1|
|正常巡线等待|`wait_duration_ms`、`wait_speed`|200、7|
|右偏|`right_min_error`|58|
|右偏|`right_duration_ms`、`right_speed`|700、3|
|后续速度上限|`post_duration_ms`、`post_speed`|5500、7|
|启动位置|`trigger_y_percent`|45，待实车验证|

误差为现有舵机控制链的输入，不是实测轮角、PWM或航向。左偏采用pratio计算并保留更大的左转误差；右偏保留更大的右转误差。等待阶段正常巡线，车辆继续行驶。后续阶段释放强制方向、继续视觉避障；速度7是上限，原目标更低时保持更低值。

触发行45表示锥桶底部达到处理图像高度45%，不是45厘米。越大越晚触发。这个数值是初始试调值，不能当作已经验证的车体净空。左/等待/右时长允许0～10000毫秒，后续时长0～30000毫秒；两段强制动作不能同时为0。速度0.5～20沿用旧车控制尺度，不是m/s。`left_pratio` 为0.05～0.95，误差下限0～90，触发行30～85；字段必须完整，时长与触发行必须为整数。

## 确认、门控和中断

- 保持连续2个真实新帧确认。相邻确认帧最大间隔初值100ms；没有增加至少60ms等额外等待。重复时间戳、明显断档、缓存保持帧都不能凑够2帧。
- 先检测锥桶，再区分LEFT/CENTER/RIGHT。居中目标不会直接丢弃。道路暂时未知时仍记录彩色候选，但不能生成未经确认的绕行路径。
- 新增近锥上限：面积16%、宽度38%、高度80%；仅对下半幅且上窄下宽的紧凑候选放宽。挡板过滤保留；这些上限尚需彩色实跑验证。
- 当前圈首次确认RIGHT/CENTER、达到触发行、有可行通道、发车右偏结束且没有其他任务/停车接管时，启动一次先左后右动作。LEFT目标继续原视觉绕行，不擅自镜像动作。左偏期间若确认到左侧障碍，暂停并报告 `LEFT_ROUTE_MISMATCH`。
- 动作开始后冻结本次参数；停车、道路/图像不可靠时暂停计时，相机150ms没有更新时停驱动。恢复时不把暂停或断帧间隔计入动作。计时是允许行驶时间，不能替代真实距离反馈。
- 动作期间禁止新开环岛模板/入口动作，但已经开始的环岛停车与转向仍正常推进，避免锁死。两段结束后锥桶视觉补线继续工作；本圈不重复启动定时序列，实际计圈后重新允许。
- 中心黑线检测屏蔽已确认锥桶的包围框，避免把锥体暗色轮廓当作中心黑线。原始二值图保留，不被诊断绘图修改。

## 录像检查

`./vvv` 的第一格改为彩色观察图；第三格右下角显示BLUE MASK，不遮挡彩色原图与RAW BINARY。录像目录增加两个启动快照：`image_params.snapshot.txt`、`cone_avoidance.snapshot.ini`。这是启动时文件内容，行驶中热加载后的值应结合阶段日志/实际调参文件核对。

CSV保留原字段顺序并追加：`cone_confirmed`、`cone_confirmation_count`、`cone_blue_pixels`、`cone_components`、`cone_rejected_size`、`cone_rejected_road`、`cone_reason`、`cone_phase`、`cone_remaining_ms`、`cone_sequence_used`、`cone_tuning_valid`、`stop_reason`。

先看蓝色像素数，再看SIZE/OUTSIDE_ROAD/ROAD_UNKNOWN/CONFIRMING/CORRIDOR_BLOCKED等原因；`HELD`表示缓存，不是本帧重新检测。阶段LEFT/WAIT/RIGHT/POST分别对应独立调参文件中的四段。

## 检查范围

新增/扩展检测器、独立时序、实际视觉管线、录像和参数快照测试。运行 `bash tests/run_tests.sh --cones-only` 可执行本次7组专项检查；不加选项仍执行原来的完整测试入口；OpenCV相关检查要求 `pkg-config opencv4`。主机测试使用生产视觉算法；仅Linux设备声明、全局数据初始化和原测试中的标定getter作Windows主机适配，不访问车辆硬件。

本机验证结果、原版本遗留测试问题及五项代码检查另见 `CONE_VALIDATION.md`。Windows检查和Linux对象编译都不等同于龙芯板链接、相机HSV实测或实车绕障通过。
