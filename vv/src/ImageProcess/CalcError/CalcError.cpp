/**
 * @file CalcError.cpp
 * @brief 赛道中心线误差计算模块
 *
 * @details
 * 本文件主要实现 ImageProcess::CalcError 类，用于根据图像处理模块
 * 已经识别出来的“赛道中心线 centerLine”计算小车当前的转向误差。
 *
 * 这个模块本身不负责摄像头采集、不负责图像二值化、不负责寻找赛道线，
 * 也不直接控制舵机。它只负责一件事情：
 *
 *     根据赛道中心线的位置，计算出一个“角度误差值”。
 *
 * 该误差值通常会在图像处理线程中被使用，例如：
 *
 *     steer->setAngle(90.0f + gHandleImageError);
 *
 * 其中：
 *
 *     90.0f              表示舵机中位角，也就是小车直行方向；
 *     gHandleImageError  表示本模块计算出来的误差修正量；
 *     90 + 误差值        表示最终舵机目标角度。
 *
 * ------------------------------------------------------------
 * 一、centerLine 的含义
 * ------------------------------------------------------------
 *
 * 本模块的核心输入是 centerLine 数组：
 *
 *     centerLine[y] = 第 y 行图像中，赛道中心线对应的 x 坐标
 *
 * 例如：
 *
 *     centerLine[30] = 82
 *
 * 表示图像第 30 行处，赛道中心线的横坐标 x 为 82。
 *
 * 如果某一行没有识别到有效中心线，通常会被设置为：
 *
 *     centerLine[y] = -1
 *
 * 本代码在计算时会跳过这些无效点。
 *
 * ------------------------------------------------------------
 * 二、calcErrorAngleByCenterSimple 的计算思想
 * ------------------------------------------------------------
 *
 * calcErrorAngleByCenterSimple(start, end, ratio, centerPratio)
 * 的主要思路是：
 *
 *     1. 从 centerLine 中取 start 到 end 范围内的有效点；
 *     2. 计算这些中心线 x 坐标的平均值 avgX；
 *     3. 将 avgX 与目标中心位置 width * centerPratio 比较；
 *     4. 使用 atan2() 计算赛道中心线相对图像竖直方向的偏角；
 *     5. 将偏角从弧度转换为角度；/////////////////////////////
 *     6. 乘以 ratio 放大，得到最终转向误差。
 *
 * 公式核心为：
 *
 *     angleRad = atan2(avgX - width * centerPratio, height - deltaY)
 *
 * 其中：
 *
 *     avgX                  表示选定区域内赛道中心线的平均 x 坐标；
 *     width * centerPratio  表示期望的小车中心位置；
 *     height - deltaY       表示参考点到采样点之间的纵向距离。
 *
 * 如果 avgX 大于目标中心位置，说明赛道中心线偏右；
 * 如果 avgX 小于目标中心位置，说明赛道中心线偏左；
 * 如果两者相等，说明车辆基本处于目标中心位置。
 *
 * ------------------------------------------------------------
 * 三、calcErrorAngleByCenterRelativeSimple 的计算思想
 * ------------------------------------------------------------
 *
 * calcErrorAngleByCenterRelativeSimple(start, end, ratio)
 * 与上一个函数类似，但参考中心不同。
 *
 * calcErrorAngleByCenterSimple 使用的是固定参考中心：
 *
 *     width * centerPratio
 *
 * 而 calcErrorAngleByCenterRelativeSimple 使用的是图像底部的中心线位置：
 *
 *     center_line[height - 1]
 *
 * 也就是说，它不是判断赛道中心线相对于图像固定中心偏多少，
 * 而是判断前方赛道中心线相对于近处赛道中心线偏多少。
 *
 * 这种方式更像是在判断赛道走势：
 *
 *     前方赛道相对于当前车头方向，是往左弯还是往右弯。
 *
 * ------------------------------------------------------------
 * 四、输出结果的意义
 * ------------------------------------------------------------
 *
 * 本模块返回的 float 值一般表示转向误差角度。
 *
 *     返回值 > 0：表示需要向一侧修正；
 *     返回值 < 0：表示需要向另一侧修正；
 *     返回值 = 0：表示无需修正，或者没有有效中心线数据。
 *
 * 具体正方向对应左转还是右转，需要结合舵机安装方向、
 * Steer::setAngle() 的角度定义以及车辆机械结构判断。
 *
 * ------------------------------------------------------------
 * 五、注意事项
 * ------------------------------------------------------------
 *
 * 1. 本代码默认 center_line 已经被 submitCenterLine() 正确提交。
 *    如果 center_line 是空指针，直接调用计算函数可能导致程序崩溃。
 *
 * 2. 本代码默认 start 和 end 没有超出图像高度范围。
 *    如果传入的 start 或 end 超过 centerLine 数组范围，可能发生越界访问。
 *
 * 3. ratio 是误差放大倍数。
 *    ratio 越大，舵机修正越激烈；
 *    ratio 过大可能导致车辆左右抖动。
 *
 * 4. centerPratio 用来设置期望中心位置。
 *    centerPratio = 0.5 表示图像正中心；
 *    centerPratio = 0.55 表示图像宽度 55% 的位置。
 *
 * 5. 代码中的循环变量虽然命名为 x，
 *    但它实际表示的是 centerLine 数组下标，也就是图像的 y 行号。
 */

 #include "CalcError.hpp"

 // using namespace Other;
 
 namespace ImageProcess
 {
     /**
      * @brief 构造 CalcError 对象
      *
      * @details
      * 创建 ImageProcess / CalcError 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
      */
     CalcError::CalcError() = default;
     /**
      * @brief 提交 submitCenterLine 输入数据
      *
      * @details
      * 把调用方提供的实时数据写入 ImageProcess / CalcError 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
      *
      * @param centerLine 中心线数组指针，保存每一行对应的赛道中心位置。
      */
     void CalcError::submitCenterLine(int *centerLine)
     {
         /*
          * 提交中线数组。
          * 数组下标表示 y，高度方向逐行存储对应的中心 x 坐标。
          */
         this->center_line = centerLine;
     }
 
     /**
      * @brief 计算 calcErrorAngleByCenterSimple 结果
      *
      * @details
      * 按照 ImageProcess / CalcError 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
      *
      * @param start 参与计算的起始行或起始索引。
      * @param end 参与计算的结束行或结束索引。
      * @param ratio 误差计算的采样权重或比例系数。
      * @param centerPratio 中心点参考比例，默认 0.5 表示图像水平中点。
      *
      * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
      */
     float CalcError::calcErrorAngleByCenterSimple(int start, int end, int ratio, float centerPratio)
     {
         int *centerLine = this->center_line;
         float sumX = 0.0f; // Y 值的累加
         int count = 0;     // 点的数量
 
         const int height = this->image_height;
         const int width = this->image_width;
 
         if (start > end)
         {
             swap(start, end);
         }
 
         for (int x = start; x <= end; ++x)
         {
             if (centerLine[x] == -1) // 如果 y 值为 -1，跳过该点
                 continue;
 
             sumX += centerLine[x]; // 累加 Y 值
             count++;
         }
 
         // 如果没有有效的点，避免除以零
         if (count == 0)
             return 0.0f;
 
         float avgX = sumX / count; // 平均 X 值
 
         // gTestRightPointContours.clear();
 
         // 计算与Y轴的夹角
         float deltaX = avgX;                      // 计算 X 方向的差值
         float deltaY = start + (end - start) / 2; // 计算 Y 方向的差值
 
         // gTestRightPointContours.push_back(Point(deltaX, deltaY));
 
         // gTestRightPointContours.push_back(Point(deltaX, deltaY));
 
         // 使用 atan2 来计算夹角，这里我们希望的是 Y 轴为基准
         float angleRad = atan2(deltaX - (width * centerPratio), height - deltaY); // 计算弧度角，以 Y 轴为基准
 
         // 将弧度转换为角度
         float angleDeg = angleRad * 180.0f / M_PI;
 
         // 3. 将夹角与放大倍数 ratio 相乘
         float result = angleDeg * ratio;
 
         return result;
     }
 
     /**
      * @brief 计算 calcErrorAngleByCenterRelativeSimple 结果
      *
      * @details
      * 按照 ImageProcess / CalcError 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
      *
      * @param start 参与计算的起始行或起始索引。
      * @param end 参与计算的结束行或结束索引。
      * @param ratio 误差计算的采样权重或比例系数。
      *
      * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
      */
     float CalcError::calcErrorAngleByCenterRelativeSimple(int start, int end, int ratio)
     {
         int *centerLine = this->center_line;
         float sumX = 0.0f; // Y 值的累加
         int count = 0;     // 点的数量
 
         const int height = this->image_height;
         const int centerPointX = center_line[height - 1];
 
         if (start > end)
         {
             swap(start, end);
         }
 
         for (int x = start; x <= end; ++x)
         {
             if (centerLine[x] == -1) // 如果 y 值为 -1，跳过该点
                 continue;
 
             sumX += centerLine[x]; // 累加 Y 值
             count++;
         }
 
         // 如果没有有效的点，避免除以零
         if (count == 0)
             return 0.0f;
 
         float avgX = sumX / count; // 平均 X 值
 
         // gTestRightPointContours.clear();
 
         // 计算与Y轴的夹角
         float deltaX = avgX;                      // 计算 X 方向的差值
         float deltaY = start + (end - start) / 2; // 计算 Y 方向的差值
 
         // gTestRightPointContours.push_back(Point(deltaX, deltaY));
 
         // gTestRightPointContours.push_back(Point(deltaX, deltaY));
 
         // 使用 atan2 来计算夹角，这里我们希望的是 Y 轴为基准
         float angleRad = atan2(deltaX - centerPointX, height - deltaY); // 计算弧度角，以 Y 轴为基准
 
         // 将弧度转换为角度
         float angleDeg = angleRad * 180.0f / M_PI;
 
         // 3. 将夹角与放大倍数 ratio 相乘
         float result = angleDeg * ratio;
 
         return result;
     }
 
     /**
      * @brief 析构 CalcError 对象
      *
      * @details
      * 释放 ImageProcess / CalcError 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
      */
     CalcError::~CalcError() = default;
 }