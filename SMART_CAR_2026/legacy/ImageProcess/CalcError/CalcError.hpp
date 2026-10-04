#pragma once

#include "headfile.hpp"

namespace ImageProcess
{
    /*
     * 偏差计算类。
     *
     * 通过中线在图像中的位置，估算车辆当前相对赛道中心的转向偏差，
     * 最终输出一个可直接映射到舵机角度的误差值。
     */
    class CalcError
    {
    private:

        int *center_line = nullptr;

    public:
        int image_height = 0;
        int image_width = 0;

        /**
         * @brief 构造 CalcError 对象
         *
         * @details
         * 创建 ImageProcess / CalcError 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        CalcError();

        /**
         * @brief 提交 submitCenterLine 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / CalcError 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param centerLine 中心线数组指针，保存每一行对应的赛道中心位置。
         */
        void submitCenterLine(int *centerLine);

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
        float calcErrorAngleByCenterSimple(int start, int end, int ratio, float centerPratio = 0.5);

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
        float calcErrorAngleByCenterRelativeSimple(int start, int end, int ratio);

        /**
         * @brief 析构 CalcError 对象
         *
         * @details
         * 释放 ImageProcess / CalcError 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~CalcError();
    };
}
