#pragma once
/*
 * 基于 LS2K0300 ATIMER 的 PWM 控制器类。
 *
 * 该类通过直接访问高级定时器寄存器输出 PWM，
 * 适用于需要绕过 sysfs、追求更底层控制能力的场景。
 * 当前文件注释中已标明“未测试”。
 */
#include "headfile.hpp"

namespace Contral
{

    class PwmAtim
    {
    public:
        /**
         * @brief 构造 PwmAtim 对象
         *
         * @details
         * 创建 Contral / PWM 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param gpio 需要复用为 PWM 功能的 GPIO 编号。
         * @param mux GPIO 复用功能编号。
         * @param chNum_ 定时器 PWM 通道号。
         * @param period_ 初始 PWM 周期。
         * @param duty_cycle_ 初始 PWM 占空时间。
         * @param NEG_ 互补/反相输出选择位。
         */
        PwmAtim(int gpio, int mux, int chNum_, int period_, int duty_cycle_, int NEG_);
        /**
         * @brief 析构 PwmAtim 对象
         *
         * @details
         * 释放 Contral / PWM 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~PwmAtim(void);

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
        uint32_t chNum, NEG;
        void *ccmr_buffer[2];
        void *ccer_buffer;
        void *period_buffer;
        void *duty_cycle_buffer;
        void *cnt_buffer;
        void *bdtr_buffer;
    };

}