# 锥桶时间窗口与视觉换段 VV_WINDOW_CONE_20261015_4

维护目录：`F:\共享文件夹\vv`。最新批准方案覆盖此前固定延时/纯定时换段方案。本车是灰度摄像头，使用灰度形状与亮度对比检测，不使用HSV蓝色来触发。

## 首圈时间起点与进入规则

原main.cpp、原build.sh、vvv未修改。初始化阶段已经会打印日志，不能以第一条终端日志计时。原程序打印 `[INFO] START!!!` 后创建控制线程；本版以其后第一次调用图像控制管线作为首圈时间原点。这与日志有线程启动时间差，不是编码器实测起步时刻，也不受首次找到道路、起点倒Y同步重置影响。

```ini
entry_time_ms=4500
entry_early_ms=500
entry_late_ms=500
```

|相对首圈起点|行为|
|---|---|
|4000ms之前|正常巡线，不触发锥桶状态机|
|4000ms起至5000ms|灰度锥桶连续2个有效新采集帧确认即可进入|
|到5000ms仍未确认|下一次可用控制帧强制进入，记录TIME_FALLBACK|

±范围是允许触发窗口，不是随机时间。前后可分别调整，均设0则在中心时间兜底进入。相机无有效画面、人工停车或已有任务停车时不强行发车，恢复后若已过截止时刻则进入。以上4500±500ms只是试调初值。

每圈最多进入一次；后续圈沿用原倒Y计圈起点重新计算窗口。原第二圈结束停车保留。

## 四段如何换向

固定方向为左→右→左→右回路，段间等待为0，但各段结束由视觉证据决定。

|阶段|最短时间|最大超时|转向误差幅度|视觉条件|
|---|---|---|---|---|
|first_left 第1段左绕|200ms|2500ms|30|跟踪第1只桶，先接近近区，再连续2帧消失|
|second_right 第2段右绕|200ms|2500ms|58|重新确认左侧下一只桶，同样确认通过|
|third_left 第3段左绕|200ms|2500ms|30|重新确认右侧下一只桶，同样确认通过|
|return_right 第4段右偏回路|200ms|2500ms|58|双边道路有效、目标靠近画面中部且连续2帧稳定|

上述每段均对应3项INI：例如first_left_duration_ms、first_left_timeout_ms、first_left_error。duration是最短持续时间；到了仍须视觉确认，不能靠200ms自动跳段。最大超时没有取得证据则进入FAULT停车，不继续盲目换向。时间均为有效连续控制时间，停车/相机中断不消耗动作时间。

左误差在代码中取负、右误差取正，INI只写非负幅度。它是控制误差，不是实测轮角/PWM。保持此前误差30/58，但移除了旧pratio参考中心偏移组合。错误进入或超时后，应先检查录像与参数，再重新启动；不能把FAULT当作已过桶。

## 区域速度与后续限速

```ini
cone_zone_speed=9
wait_duration_ms=0
post_duration_ms=5500
post_speed=7
```

四段统一使用独立cone_zone_speed，先沿用上次实车普通巡线目标9；取代旧各段速度1/3。停车保护仍优先。普通巡线参数未改：本地参考image_params.txt为motor_aim_speed=7，而上次实车日志为9；部署不会以本地7覆盖车上9。速度字段是控制目标，不是实测m/s。

第四段确认找回道路后立即恢复普通巡线并允许环岛主状态机观察。post_duration_ms只是后续巡线速度上限窗口，不再额外等待5500ms才开放环岛，也不会把已有更低速度提高到7。环岛仍需原标志、形状和图像质量条件。

## 灰度判据及防误换段

INI中的entry_y_percent=45决定进入候选的底部行比例；pass_near_y_percent=75决定近区。gray_threshold_percent=55是大津阈值的基础百分比，再增加20、40个百分点作有限的亮度兼容，最多100%；相同候选合并，多个阈值命中不能凑成多帧。

gray_min_contrast=12、gray_min_area_80x60=5、gray_taper_percent=80分别调整局部对比、归一化最小面积和上窄下宽条件。细线、平板、边缘大块不能直接启动新目标；下沿截断候选只允许继续已有近处跟踪。

只有先连续确认当前桶到近区，再连续2个有效帧失去它，才作为过桶证据；远处消失不会换段。下一段重新找预期侧的桶，不能直接获取画面底部刚通过的桶。相邻时间戳必须递增且间隔不超过150ms，重复时间戳、停车、无有效灰度画面、长断帧都会重置视觉证据；不会用缓存帧凑两帧。

第四段的return_center_tolerance_percent=20控制道路中心容差，还要求连续两帧中心变化不超过画面宽5%。道路短时有效或偏到画面边缘不能立即结束回路。

这些低分辨率灰度证据是软件过桶判据，不能等同于外部实测车体净空；详细实跑数据限制见CONE_VALIDATION.md。

## 参数与保护

全部26项必须存在且不重复；不接受旧两段/旧纯时间INI、未知键、NaN、分数毫秒、最短时间不小于超时、前浮动超过中心时刻等错误。文件缺失/非法会打印 `[ConeTuning] STOP` 并停止驱动。约500ms检查更新，进入动作后锁定本次参数，下一次动作使用新值；触发前修改进入窗口会影响尚未开始的动作。

有效图像暂时没有可用道路时，前三段仍可按灰度锥桶控制；灰度画面无效、相机中断、人工停车和其他任务停车仍停止驱动。结束后恢复普通视觉巡线，道路无效会停车等待恢复。

两条曾提前停车的旧分支保持默认关闭：new_track_ring_side_enable=0、new_track_ring_entry_cue_enable=0。普通巡线轮廓、八邻域、环岛控制器、PID和电机/舵机接口没有改；改变范围包括锥桶触发/控制优先级、环岛放行时机及上述两个停车开关。

## 虚拟机编译

```bash
cd "/mnt/hgfs/share/vv" &&
OPENCV_DIR="/home/gy/下载/opencv-4.11.0/loongson/install/lib/cmake/opencv4" bash build_cone.sh
```

成功标志：`BUILD_OK VV_WINDOW_CONE_20261015_4`。
生成：`vv_window_cone_20261015_4.zip`。必须在原龙芯交叉编译环境构建；Windows目录的旧vv、旧ZIP、旧版本标记不代表本版已编译。新脚本只编译打包，不改变原启动流程。

## 板上部署与原方式运行

将新ZIP上传到 `/home/root/smartcar/vv/`；如需保存车上已有INI，在解压前另行备份。执行：

```bash
cd /home/root/smartcar/vv &&
unzip -o vv_window_cone_20261015_4.zip &&
sha256sum -c VV_SHA256SUMS &&
cat VV_VERSION &&
sh apply_cone_config.sh &&
chmod +x vv vvv
```

包内主参数是image_params.reference.txt，不覆盖车上image_params.txt。apply_cone_config.sh先备份现场参数，再只合并锥桶状态机开启、旧蓝锥关闭、两个旧停车分支关闭四个开关，保留现场中心、巡线速度和阈值。新INI会覆盖旧格式INI，不能再用旧INI替换回来。

沿用原录像启动：

```bash
./vvv
```

不要同时启动另一个./vv；不增加--init-only或新的初始化阶段。结束Ctrl+C，等待 `[ProcessedVideo] CLOSED` 后断电。录像仍位于程序目录video/run_.../。

## 日志和CSV

控制台 `[ConeRoute] phase=... entry=... reason=... route_ms=...` 记录换段。entry为VISUAL_WINDOW或TIME_FALLBACK，故障为VISUAL_STAGE_TIMEOUT。

CSV末尾包含cone_phase、cone_remaining_ms、cone_tuning_valid、stop_reason、lane_valid、cone_route_complete，以及cone_entry_reason、cone_visual_reason、cone_near_seen、cone_gray_base_threshold。cone_remaining_ms为本段最大超时剩余，不是必然换段倒计时。cone_route_complete只在找回道路后为1。

固定转向阶段valid=1仅表示有可用转向命令，lane_valid仍如实区分道路；CONE_VISUAL_*表示锥桶状态机控制。cone_detected表示当帧灰度候选，旧cone_blue_pixels/旧蓝锥确认列不作为本版触发判据。相机停帧、参数错误和过桶超时可通过stop_reason区分。
