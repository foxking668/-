# 前进直线循迹：图像误差与建议命令

2026-10-08。当前实现为独立的前进直线观察模块，复用正式 C++ Vision 黑线结果。没有 IMU、设备访问、舵机/电机输出或速度决策；未接入 Mission 正式驾驶。目标须由调用方显式提供，不把起步画面当作正确姿态。

## 目标、误差和状态

拟合沿用已有观察器的规则：图像高度40%～90%，拟合 x=a+b*y；有效支持至少80%，通常最大横向残差0.008（320像素宽约2.56像素），最多排除两个有限的小像素偏移，排除点仍限制0.012。支持可能包含 Vision 原有的短缺口插值，不等同于独立实测像素。近、中、远三处均须有足够支持；线评分至少0.65，并拒绝已有歧义/不连续标志。

- target_x：正确对正姿态下，拟合线在图像高度84%的归一化横向位置。
- target_heading：正确对正姿态下，拟合线在45%与84%高度的归一化横向位置差。
- lateral_error_image：当前84%位置减 target_x。
- heading_error_image：当前远近位置差减 target_heading；不是弧度或实测车身航向。

偏在画面一侧的直线、斜向直线仍可通过线形检查，是否居中/对正另由原始误差判断。straight_aligned只表示两个图像误差均在0.015内；不证明后轴位姿、实际厘米精度、已经停稳或可以加速。

命令使用横向与方向两个比例项，没有积分、时间微分或陀螺仪项。分别按真实新帧 dt 做一阶滤波，然后应用连续死区、幅度限制及每秒变化限制。首次调用没有 dt 时命令为0；恢复后从0重新限制建议变化。以下参数可写入现有车辆 INI，默认值只是诊断起点：

| 参数 | 默认值 | 含义 |
|---|---:|---|
| straight_lateral_gain | 8 | 横向图像误差增益 |
| straight_heading_gain | 12 | 方向图像特征误差增益 |
| straight_filter_s | 0.15 | 滤波时间常数，秒；0表示不滤波 |
| straight_deadband_image | 0.005 | 归一化图像连续死区 |
| straight_max_command | 5 | 建议命令幅度上限，命令单位 |
| straight_command_rate_s | 3 | 每秒建议命令变化上限 |

命令单位不是已测得的前轮角度；正负约定沿用前进图像纠偏方向。增益与目标都尚未完成动力实车闭环验证。

WAIT_STRAIGHT表示连续确认不足；TRACKING_STRAIGHT才有建议。NOT_STRAIGHT、PARTIAL_PATH、LOW_QUALITY、AMBIGUOUS、DISCONTINUOUS撤销建议；重复时间、倒退时间、新帧间隔或处理年龄超过 frame_timeout_s也撤销。相邻接受特征横向/方向变化超过0.06时输出FEATURE_JUMP，保留原接受位置，避免丢线后无声换成远处分支。处理高度改变需显式 reset。

这些检查是图像连续性约束，尚未完成交点主线/分支语义识别。长时间运动后的旧参考可能需要人工确认并显式开始新会话；不能用自动 reset 吸收偏移或掩盖错误换线。

## Windows 回放

在 SMART_CAR_2026 目录运行：

```powershell
.\build_windows.ps1
python tests\straight_follow_integration.py
python tools\replay.py "C:\Users\巩钰\Desktop\captures\manual_push_steer_0\manual_20210324T102813Z_2001_190255851097\camera.avi" --config "C:\Users\巩钰\Desktop\captures\manual_push_steer_0\manual_20210324T102813Z_2001_190255851097\vehicle_config.ini" --frame-times "C:\Users\巩钰\Desktop\captures\manual_push_steer_0\manual_20210324T102813Z_2001_190255851097\frames.csv" --stage ToCones --observe-straight 0.5 0 --output build\straight_replay.jsonl
```

0.5、0是显式的名义诊断目标，不能据此声称摄像头中心与车轴对齐。回放必须按每段录像重新建立模块状态；实际目标应来自车体正确居中/对正时的画面。观察模式与车辆遥测、停车参考观察互斥，仅允许普通前进黑线阶段。

重复检查四段前进录像：

```powershell
python tools\check_straight_follow_recordings.py --capture-root "C:\Users\巩钰\Desktop\captures" --output build\straight_recordings --target-x 0.5 --target-heading 0
```

输出 speed、steer、actuator_writes始终为0。straight_suggested_command与实际执行命令分开；无效建议用JSON null，不暴露上一帧命令。旧原始录像和配置保持不变。

## 龙芯虚拟机构建与板上接口

沿用已有 LoongArch CMake 构建缓存：

```sh
cd /mnt/hgfs/share/SMART_CAR_2026
cmake -S . -B /home/gy/builds/SMART_CAR_2026-loongarch -DBUILD_VEHICLE=ON
cmake --build /home/gy/builds/SMART_CAR_2026-loongarch --target vision_stream straight_follow_tests --parallel 2
/home/gy/builds/SMART_CAR_2026-loongarch/straight_follow_tests
```

上述虚拟机路径来自项目既有部署文档；本机尚未生成/运行 LoongArch 二进制。vision_stream读取标准输入中的遥测行+P6帧，**不直接打开摄像头或控制小车**。板上上传该目标与现有车辆 INI 后，仅观察的接口是：

```sh
cd /home/root/gy/deploy
./vision_stream config/calibration_vehicle.ini ToCones --observe-straight 0.5 0 < straight_frames.ppmstream > straight_observation.jsonl
```

straight_frames.ppmstream需先准备：每帧先一行 time distance speed yaw yaw_valid yaw_measured，再P6头及RGB字节；图像尺寸应与配置一致，时间必须为真实递增记录时钟。这个命令是离线输入接口，不是已有实车启动命令；没有实时摄像头巡线或动力输出入口。

## 验证结果与五项检查

- 新模块129项检查通过：起步偏差、斜直线、显式目标、弯线、三处支持、歧义、失线后远处分支、尺寸变化、时序、死区、滤波、限幅/限速及非法配置。
- 全部Windows构建脚本测试通过，共1538项。硬件写失败日志是模拟注入用例。
- 原回放集成与新前后端/Vision/直线控制集成通过；7个Linux x86_64翻译单元编译通过，无警告。这不是LoongArch链接或板上运行。
- 四段前进录像442帧回放通过，时钟匹配，原录像/配置哈希不变，实际命令始终为0。
- 旧参考观察模式另用既有原帧及记录时钟回归833帧，直线/弯线拒绝、参考编号及零输出检查通过；本次提取共用拟合没有改变旧观察器的目标含义。

| 录像 | 帧数 | 可接受直线帧 | 有效建议帧 |
|---|---:|---:|---:|
| 前推0 | 123 | 123 | 121 |
| 前推+5 | 123 | 28 | 26 |
| 前推-5 | 123 | 52 | 50 |
| 已有前推观察 | 73 | 72 | 69 |

这是覆盖统计，缺少全帧独立真值，不能称作识别准确率或实车闭环成功率。转弯录像仍有大量低质量和歧义帧，不通过放宽阈值提高表面覆盖率。

1. 重复代码：黑线仍只调用Vision；原观察器与新模块共用straight_image_features.hpp，车辆配置继续共用Params解析器。
2. 命名：image特征、显式target、建议command和实际steer分开；不把图像方向量称作IMU航向，不把命令幅度称作实际轮角。
3. 结构逻辑：新帧时间→路径质量→线形→连续确认→显式目标误差→滤波/死区→限幅/限速；模块无设备接口。
4. 隐藏bug：起步偏斜被学为目标、旧帧累积、丢线后远处分支、尺寸变化、NaN、短数组、非法目标和零滤波时间均有检查；主线语义及真实曝光年龄仍未验证。
5. 优化建议：拟合为O(h)，不复制图像、不引入采样线程；先核对车辆正确姿态下的图像目标及动态反馈，再决定是否需要微分项或速度联动。
