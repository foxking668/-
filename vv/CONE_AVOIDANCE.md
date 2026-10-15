# 锥桶强制转向限时修正 VV_CONE_PULSE_20261015_5

维护目录F:\共享文件夹\vv。用户批准：锥桶区域速度设为6，其余按“强制打方向限时、到时转视觉绕行、视觉不可用停车”修正。原main.cpp、原build.sh、vvv保留，普通巡线参数与硬件中位不改。

## 本次修正的原因

实跑run_20210324_103009_5158_160在5003ms由时间兜底进入LEFT_1，持续误差-30、速度9，7563ms才超时停车。旧duration=200被实现成最短时间，导致等待视觉通过期间继续强制左打。本版把它改为强制输出上限，绝不以2500ms超时作为持续固定打方向的目标时长。

## 进入规则与时间原点

原程序START!!!后第一次图像控制调用为首圈时间原点，不是第一条初始化日志，也不是实测车轮起步时刻。原起点倒Y同步不会重置首圈进入窗口。后续圈沿用原倒Y计圈起点，每圈最多一次，第二圈结束停车保留。

```ini
format_version=2
entry_time_ms=4500
entry_early_ms=500
entry_late_ms=500
cone_zone_speed=6
```

4000ms以前普通巡线；4000~5000ms连续2个有效新帧确认完整灰度锥桶即可进入；5000ms仍未确认，下一可用控制帧时间兜底进入。人工停车或无有效相机画面时不强行驱动，恢复后再处理。前后浮动可分别调整，不随机取时间，两个范围都为0即中心时刻兜底。窗口初值未重新实车标定。

## 四段转向、时间与速度

|阶段|duration强制转向上限|error幅度|timeout整段最大驱动预算|
|---|---|---|---|
|first_left 左绕第1桶|200ms|30|2500ms|
|second_right 右绕第2桶|200ms|58|2500ms|
|third_left 左绕第3桶|200ms|30|2500ms|
|return_right 右偏找路|200ms|58|2500ms|

每段的duration_ms、error、timeout_ms均独立调参，例如first_left_duration_ms/first_left_error/first_left_timeout_ms。速度统一由cone_zone_speed=6控制，停车和更低的其他任务限速仍优先。普通巡线本地参考为7，上次实车为9；部署保留车上主参数，不用本地7覆盖现场9。

- PULSE：输出该段固定误差，左为负、右为正。duration是这段固定输出的上限，累计驱动时间达到它便解除；视觉提前确认通过也可提前结束。
- GUIDE：不再固定输出-30或58，而是根据道路边界和当前确认的桶构造可通行一侧的视觉目标。未见当前候选且道路可靠时沿道路几何中心寻找；有候选但未确认、边界无效、图像质量不足或通道不足时停止驱动。
- GUIDE_STOP：停止驱动等待视觉恢复，不把丢失目标直接当成通过；恢复后继续视觉引导，不重置已经结束的强制输出。
- 确认当前桶曾到近处，随后连续2个有效帧离开，且最后一次近处观测后确有允许驱动时间，才进入下一方向。时间到不会凭空计桶；第四段要求道路有效、居中、连续2帧稳定，才恢复普通巡线并开放环岛。
- FAULT：整段最大驱动预算耗尽仍未通过则停车，转向修正归0，不自动跳段。修正归0是原软件中位指令，不能当作实测车轮回正。

error是加到舵机软件角度指令上的修正量，不是实测轮角或PWM。只影响PULSE；GUIDE使用当帧视觉目标，不套最小误差下限。不要为调锥桶随意改全局steer_offset_deg或error_gain。速度字段也不是实测m/s。

wait_duration_ms必须为0。这里的视觉引导没有新增一段静止等待；只有视觉不可用才停车。找回道路后即开放环岛，post_duration_ms=5500/post_speed=7仅作后续速度上限，保留更低速度及停车。

## 截止保护、暂停与参数版本

图像线程在下一次控制调用切换PULSE/GUIDE；电机线程另检查强制输出截止时间，到时尚无新视觉结果就先停止驱动，防止带着固定方向继续走。舵机位置更新仍由原图像线程执行，本版不新增独立舵机线程或改启动流程。

原150ms相机看门狗保留。整段预算按上一帧实际允许驱动情况累计，并受看门狗和脉冲截止双重限制；已停车的等待不消耗驱动预算。重复时间戳、相机无效、长断帧会清除视觉确认，不能复制图片凑两帧。暂停恢复不重置该段已累计的强制时长。

新INI共27项，format_version=2必填，用于区分旧的“最短时间”格式。旧26项INI会被拒绝，不能只替换二进制后沿用旧文件。未知/重复/缺项、NaN、分数毫秒、timeout不大于duration等均停止驱动并打印ConeTuning STOP。约500ms检查更新，整次避障进入时锁定参数，新值用于下一次；触发前修改进入窗口会影响尚未开始的动作。

## 灰度识别与近处截断

entry_y_percent=45、pass_near_y_percent=75分别控制进入和近区比例。gray_threshold_percent=55及向上20/40个百分点的有限多阈值合并候选，不增加帧数。gray_min_contrast=12、gray_min_area_80x60=5、gray_taper_percent=80控制对比、归一化面积和锥形。

对顶部截断并与横向暗带连接的大近处目标，增加小核开运算及局部身体分离，检查足够大的对称收缩底部；它只作近处候选，不能单独触发视觉进入，必须在已进入的状态机中经两帧关联确认。新段不会直接重用刚通过的同一目标。平板、细线等拒绝规则仍有测试，但灰度形状不是可靠的物体语义，现场仍需验证误检。

第四段return_center_tolerance_percent=20，还要求连续两帧中心变化不超过画面宽5%。环岛侧模板停车/入口定位停车两条旧分支保持默认关闭，主状态机保留。

## 编译、部署与运行

先正常停止旧程序，等待ProcessedVideo CLOSED。不要同时运行两个车程序。

虚拟机：

```bash
cd "/mnt/hgfs/share/vv" &&
OPENCV_DIR="/home/gy/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4" bash build_cone.sh
```

成功标志BUILD_OK VV_CONE_PULSE_20261015_5，包名vv_cone_pulse_20261015_5.zip。本轮仅主机验证，目录中的旧vv/旧ZIP不是新程序，需用原虚拟机龙芯交叉工具链构建。

上传新包到板子/home/root/smartcar/vv/，如需保留旧INI先另存，再执行：

```bash
cd /home/root/smartcar/vv &&
unzip -o vv_cone_pulse_20261015_5.zip &&
sha256sum -c VV_SHA256SUMS &&
cat VV_VERSION &&
sh apply_cone_config.sh &&
chmod +x vv vvv
```

包内image_params.reference.txt不覆盖现场image_params.txt；迁移脚本先备份，再仅合并锥桶开启、旧蓝锥关闭、两个旧停车分支关闭。启动仍为：

```bash
./vvv
```

结束Ctrl+C，等ProcessedVideo CLOSED。不新增init-only或新的发车阶段。

## 日志判断

新增CSV：cone_control_mode=PULSE/GUIDE/GUIDE_STOP/NONE，cone_pulse_remaining_ms为固定输出剩余；cone_remaining_ms为整段超时预算剩余。source对应CONE_PULSE_*、CONE_GUIDE_*、CONE_GUIDE_STOP_*、CONE_FAULT。

CONE_GUIDANCE_UNAVAILABLE表示视觉不可用停车，CONE_VISUAL_TIMEOUT表示预算超时停车。cone_entry_reason仍区分VISUAL_WINDOW/TIME_FALLBACK。lane_valid如实表示道路，valid只表示存在可返回控制结果；cone_route_complete只在回路确认后为1。详细回放限制见CONE_VALIDATION.md。
