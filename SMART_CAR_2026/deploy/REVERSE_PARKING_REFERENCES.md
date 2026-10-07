# 倒车入库开源项目核查

2026-10-07通过GitHub插件检索并读取公开源码。以下是可借鉴的设计，不是已经移植或实车验证的功能；没有引入第三方依赖。

## 最接近竞赛小车：matgrand/bfmc_2022

- [完整停车状态机](https://github.com/matgrand/bfmc_2022/blob/main/Simulator/brain.py)：parking()区分车位定位、车位检查及T/S停车子阶段。每段先设置转向，等待执行，再倒车或前进，根据局部距离进入下一段，并检查超距。
- [车辆位置接口](https://github.com/matgrand/bfmc_2022/blob/main/Simulator/automobile_data_interface.py)：yaw来自IMU，局部位置结合编码器距离与yaw计算；定位还包含GPS/EKF。不能把这些yaw值换成我们并不存在的IMU反馈。
- [试验动作代码](https://github.com/matgrand/bfmc_2022/blob/main/dei_ws/src/tests/lane_keeping_6_cv_camera/maneuvers.py)：展示固定转向、倒车、反向转向的分段动作，但不是可直接套用的停车控制器；部分动作有阻塞循环，必须重新设计退出与超时。

适合借鉴阶段划分、距离推进、转向执行等待和超距检测。T/S车位的具体几何、固定角度、距离、IMU/GPS定位不直接适用于我们的斜分支入库。也不能因为上层有视觉巡线就认定这些停车转弯是视觉闭环。

## 车辆几何和轨迹：AtsushiSakai/PythonRobotics

- [Hybrid A*](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/HybridAStar/hybrid_a_star.py)：包含前进/后退运动、转向代价、换向代价和解析路径连接。
- [车辆模型与车体碰撞检查](https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathPlanning/HybridAStar/car.py)：move()按有符号距离、实际转角与轴距更新位置/航向；check_car_collision()检查车辆矩形轮廓。

低速几何模型中，航向变化近似为 Δψ=Δs·tan(δ)/L。我们轴距L=0.22m；δ应是实际等效前轮转角，不能把舵机命令或路线36.7°直接代入。此式不是IMU实测，也不能在尚未标定距离与转向时声称得到准确姿态。

适合先在离线环境检查起倒位置、圆弧连接、车头外摆和车身是否越界。当前赛道几何固定，优先采用简洁的分段轨迹和反馈，不必把完整Hybrid A*搜索器部署到龙芯板。车辆模型默认尺寸是大车，必须换成我们的实际尺寸；19cm是轮胎外侧宽度，车体包络还应包含突出部件及前后悬。

## 感知、几何与执行分层：FaustoArroyoM/ros2-autonomous-parking

- [项目说明](https://github.com/FaustoArroyoM/ros2-autonomous-parking/blob/main/README.md)：ROS 2/Nav2方案，使用激光扫描检测车位并生成停车目标；还有外部硬件包依赖。
- [目标执行代码](https://github.com/FaustoArroyoM/ros2-autonomous-parking/blob/main/src/parking_tas2/parking_tas2/parking_service_node.py)：生成waypoints后实际只把最后一个目标发送给NavigateToPose，并非逐个执行整条停车路径。

适合借鉴检测、几何计算、执行模块分离。传感器和运行环境与我们的前摄像头、编码器、超声波不同，不直接移植ROS 2/Nav2或把两个目标点当成已完成的转弯轨迹。

## 对本车的设计建议

保留“第二次交点触发→沿主线到起倒点→倒车切入分支→反向转向对正库轴→直退→停车”的流程，但每次转向切换应由目标姿态、位置及有效观测决定，不能只用固定时间或假定两次舵机命令相等就能回正。

1. 交点计数需先进入后离开识别区域再允许计下一个交点，避免同一个交点重复触发。
2. 前移距离应相对交点的地面位置与后轴起倒位置计算。22cm轴距不等于整车长度，不能直接作为“一车身”距离。
3. 用已标定的车辆几何给出初始转向/距离，再用前相机可见的库外线和交点纠偏；相机看不到车后，需要同时检查车体包络和观测是否可用。当前图像相对直线观察器不是地面厘米/航向估计器，也不能直接控制整个转弯入库。
4. 前、后距离传感器用于各自覆盖范围内的距离约束；单个后超声波不能唯一判断车身是否偏斜。接近库底时必须有有效竖直挡板回波；地面蓝色区域不提供同样的测距条件。
5. 车身与库轴对正后再回零直退；不是一到库口就必然回零。终点结合前相机虚线几何、有效后距离及低速停车余量判断，停止阈值需用实际安装位置和停车数据标定。
6. 视觉参考丢失、传感器无效、超距或超时应有明确停止处理，不沿用旧观测继续倒车。

当前电机不可用，因此先进行人工后拉、舵机响应与观测验证；完整自动停车控制器尚未接入。新的一次+5命令/10cm人工后拉试验见[AUTO_REVERSE_PROBE.md](AUTO_REVERSE_PROBE.md)。
