#include "PwmGtim.hpp"

namespace Contral
{
    /*
     * 构造函数负责将指定 GPIO 复用到 GTIMER 通道，
     * 并完成寄存器映射、PWM 模式配置以及初始周期/占空比写入。
     */

    /**
     * @brief 构造 PwmGtim 对象
     *
     * @details
     * 创建 Contral / PWM 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * @param gpio 需要复用为 PWM 功能的 GPIO 编号。
     * @param mux GPIO 复用功能编号。
     * @param chNum_ 定时器 PWM 通道号。
     * @param period_10ns_ PWM 周期，单位为 10ns。
     * @param duty_cycle_10ns_ PWM 占空时间，单位为 10ns。
     */
    PwmGtim::PwmGtim(int gpio, int mux, int chNum_, int period_10ns_, int duty_cycle_10ns_)
        : period_10ns(period_10ns_), duty_cycle_10ns(duty_cycle_10ns_), chNum(chNum_ - 1)
    {
        { // 配置功能复用
            void *gpio_mux_buffer = mapRegister(GPIO_MUX_BASE_ADDR + (gpio / 16) * 0x04, PAGE_SIZE);
            REG_WRITE(gpio_mux_buffer, (REG_READ(gpio_mux_buffer) & ~(0b11 << (gpio % 16 * 2))) | (mux << (gpio % 16 * 2)));
        }

        // 初始化所有寄存器
        REG_WRITE(mapRegister(GTIM_BASE_ADDR + GTIM_EGR_OFFSET, PAGE_SIZE), 0x01);

        // 启动计数器
        REG_WRITE(mapRegister(GTIM_BASE_ADDR + GTIM_CR1_OFFSET, PAGE_SIZE), 0x01);

        period_buffer = mapRegister(GTIM_BASE_ADDR + GTIM_ARR_OFFSET, PAGE_SIZE);
        duty_cycle_buffer = mapRegister(GTIM_BASE_ADDR + GTIM_CCR1_OFFSET + chNum * 0x04, PAGE_SIZE);
        ccmr_buffer[0] = mapRegister(GTIM_BASE_ADDR + GTIM_CCMR1_OFFSET, PAGE_SIZE);
        ccmr_buffer[1] = mapRegister(GTIM_BASE_ADDR + GTIM_CCMR2_OFFSET, PAGE_SIZE);
        ccer_buffer = mapRegister(GTIM_BASE_ADDR + GTIM_CCER_OFFSET, PAGE_SIZE);
        cnt_buffer = mapRegister(GTIM_BASE_ADDR + GTIM_CNT_OFFSET, PAGE_SIZE);

        // 清除chNum的PWM模式
        REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) & ~(0x7 << (chNum % 2 * 8 + 4)));
        // 配置chNum的PWM模式 0x6为模式1 0x7为模式2
        REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) | (0x7 << (chNum % 2 * 8 + 4)));

        // 清除chNum的输出极性
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 1))); // 1为反相
        // 配置chNum的输出极性
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 1))); // 1为反相

        REG_WRITE(period_buffer, period_10ns);
        REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

        REG_WRITE(cnt_buffer, 0);
    }

    /**
     * @brief 析构 PwmGtim 对象
     *
     * @details
     * 释放 Contral / PWM 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    PwmGtim::~PwmGtim(void)
    {
        munmap(ccmr_buffer[0], PAGE_SIZE);
        munmap(ccmr_buffer[1], PAGE_SIZE);
        munmap(period_buffer, PAGE_SIZE);
        munmap(duty_cycle_buffer, PAGE_SIZE);
        munmap(ccer_buffer, PAGE_SIZE);
        munmap(cnt_buffer, PAGE_SIZE);
    }

    /**
     * @brief 使能 enable 功能
     *
     * @details
     * 打开 Contral / PWM 模块对应的硬件输出或软件功能，让控制信号开始生效。
     */
    void PwmGtim::enable(void)
    {
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 0)));
    }

    /**
     * @brief 关闭 disable 功能
     *
     * @details
     * 关闭 Contral / PWM 模块对应的硬件输出或软件功能，用于停止输出或进入安全状态。
     */
    void PwmGtim::disable(void)
    {
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 0)));
    }

    // 设置周期（以10纳秒为单位）
    /**
     * @brief 设置 setPeriod 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param period_10ns_ PWM 周期，单位为 10ns。
     */
    void PwmGtim::setPeriod(unsigned int period_10ns_)
    {
        period_10ns = period_10ns_;
        REG_WRITE(period_buffer, period_10ns);

        REG_WRITE(cnt_buffer, 0);
    }

    // 设置低电平时间（以10纳秒为单位）
    /**
     * @brief 设置 setDutyCycle 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param duty_cycle_10ns_ PWM 占空时间，单位为 10ns。
     */
    void PwmGtim::setDutyCycle(unsigned int duty_cycle_10ns_)
    {
        duty_cycle_10ns = duty_cycle_10ns_;
        REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

        REG_WRITE(cnt_buffer, 0);
    }

}