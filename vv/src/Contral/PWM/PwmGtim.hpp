#pragma once
/*
 * 基于 LS2K0300 GTIMER 的 PWM 控制器类。
 *
 * 与 `PwmAtim` 类似，该类直接操作通用定时器寄存器产生 PWM，
 * 适合特定 SoC 硬件平台上的低层定时输出控制。
 */

#include "headfile.hpp"

namespace Contral
{

    class PwmGtim
    {
    public:
        /**
         * @brief 构造 PwmGtim 对象
         *
         * @details
         * 创建 Contral / PWM 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param gpio 需要复用为 PWM 功能的 GPIO 编号。
         * @param mux GPIO 复用功能编号。
         * @param chNum_ 定时器 PWM 通道号。
         * @param period_ 初始 PWM 周期。
         * @param duty_cycle_ 初始 PWM 占空时间。
         */
        PwmGtim(int gpio, int mux, int chNum_, int period_, int duty_cycle_);
        /**
         * @brief 析构 PwmGtim 对象
         *
         * @details
         * 释放 Contral / PWM 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~PwmGtim(void);

        /**
         * @brief 使能 enable 功能
         *
         * @details
         * 打开 Contral / PWM 模块对应的硬件输出或软件功能，让控制信号开始生效。
         */
        void enable(void);
        /**
         * @brief 关闭 disable 功能
         *
         * @details
         * 关闭 Contral / PWM 模块对应的硬件输出或软件功能，用于停止输出或进入安全状态。
         */
        void disable(void);
        /**
         * @brief 设置 setPeriod 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param period_10ns_ PWM 周期，单位为 10ns。
         */
        void setPeriod(unsigned int period_10ns_);
        /**
         * @brief 设置 setDutyCycle 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param duty_cycle_10ns_ PWM 占空时间，单位为 10ns。
         */
        void setDutyCycle(unsigned int duty_cycle_10ns_);
        uint32_t period_10ns, duty_cycle_10ns;

    private:
        uint32_t chNum;
        void *ccmr_buffer[2];
        void *ccer_buffer;
        void *period_buffer;
        void *duty_cycle_buffer;
        void *cnt_buffer;
    };

}