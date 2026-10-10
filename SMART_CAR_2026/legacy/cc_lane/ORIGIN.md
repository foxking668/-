# cc(1)普通巡线源码来源

来源为用户提供的 `cc(1).zip`，路径 `cc/src`。直接复用并更新仓库已有 `legacy/ImageProcess` 模块，不另复制一套轮廓实现。`source_sha256.json` 保存九个依赖文件的原始字节 SHA-256。用 `tools/check_cc_lane_origin.py --reference-zip 路径` 对照原压缩包复核。

当前接入原程序启用的 NewTrack `laneMethod=4`（7.25）：80×60最近邻缩放，上裁5%、下裁2%，原HSV白色掩码，11×1闭运算，原FindLine/FixLine轮廓找边补线、历史路宽单边界恢复、中心黑线优先及局部黑线辅助。`center_pratio=0.42`，原误差换算及float精度保持一致。

`lane_core.inc` 截取 NewTrack.cpp 的普通巡线辅助函数。仅删除未启用的method=3分支及其多余参数；Config仅保留普通巡线使用的原默认值。`steering_geometry.hpp` 单独保存原centerToError函数，供实现和无硬件测试共同调用。适配器 `tools/reference_cc_lane.cpp` 按原普通道路targetX选择顺序接入。

九个依赖文件中，八个文件保持原内容；FindLine.cpp只将三个int数组的delete改为delete[]，修复原源码释放方式错误。`headfile.hpp`为最小依赖头，避免引入原应用硬件全局对象。未定义斑马线元素宏，不执行FixLine中的比赛元素分支。

复用范围是普通巡线。原应用的发车右偏、蓝锥、环岛、斑马线、计圈、交叉口和车库任务调度不接入本工具；阶段2～6继续使用当前停车调参流程。误差直接作为相对舵机命令，最终限幅同时受line_max_command和车辆max_steer_deg约束；这是当前车辆接口约束，区别于原程序未限幅的90+error调用。原有确认线路、丢线停车、速度PID、授权、录制、阶段计时均由当前工具负责。

路宽记忆是单个图像线程中的静态状态。每次试验/阶段重新授权时清空，不允许并行调用多个适配器实例。
