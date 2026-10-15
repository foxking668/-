#pragma once
#include "headfile.hpp"

namespace Contral
{
    /*
     * 舵机控制类。
     *
     * 通过 PWM 占空比控制转向舵机角度，
     * 并支持额外的机械安装偏移补偿。
     */
    class Steer
    {
    private:
        void *pwmCtrl = nullptr;
        const int period = 3040000;
        //const int period = 2000000;
        float steer_offset = 0;

    public:
        /**
         * @brief 构造 Steer 对象
         *
         * @details
         * 创建 Contral / Steer 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        Steer();
        /**
         * @brief 初始化 init 相关资源
         *
         * @details
         * 完成 Contral / Steer 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
         */
        void init();
        /**
         * @brief 设置 setAngle 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Steer 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param angle 舵机目标角度。
         */
        void setAngle(float angle);
        /**
         * @brief 设置 setSteerOffset 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Steer 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param offsetAngle 舵机机械安装偏差补偿角度。
         */
        void setSteerOffset(float offsetAngle);
        /**
         * @brief 析构 Steer 对象
         *
         * @details
         * 释放 Contral / Steer 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Steer();
    };

} // namespace Contral