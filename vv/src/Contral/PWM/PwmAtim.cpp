#include "PwmAtim.hpp"
namespace Contral
{
    /*
     * 构造函数完成：
     * 1. GPIO 复用设置；
     * 2. 高级定时器寄存器映射；
     * 3. PWM 模式与极性配置；
     * 4. 周期和占空比初值写入。
     */

    /**
     * @brief 构造 PwmAtim 对象
     *
     * @details
     * 创建 Contral / PWM 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * @param gpio 需要复用为 PWM 功能的 GPIO 编号。
     * @param mux GPIO 复用功能编号。
     * @param chNum_ 定时器 PWM 通道号。
     * @param period_10ns_ PWM 周期，单位为 10ns。
     * @param duty_cycle_10ns_ PWM 占空时间，单位为 10ns。
     * @param NEG_ 互补/反相输出选择位。
     */
    PwmAtim::PwmAtim(int gpio, int mux, int chNum_, int period_10ns_, int duty_cycle_10ns_, int NEG_ = 0)
        : period_10ns(period_10ns_), duty_cycle_10ns(duty_cycle_10ns_), chNum(chNum_ - 1), NEG(NEG_)
    {
        { // 配置功能复用
            void *gpio_mux_buffer = mapRegister(GPIO_MUX_BASE_ADDR + (gpio / 16) * 0x04, PAGE_SIZE);
            REG_WRITE(gpio_mux_buffer, REG_READ(gpio_mux_buffer) | (mux << (gpio % 16 * 2)));
        }

        // 初始化所有寄存器
        REG_WRITE(mapRegister(ATIM_BASE_ADDR + ATIM_EGR_OFFSET, PAGE_SIZE), 0x01);

        // 启动计数器
        REG_WRITE(mapRegister(ATIM_BASE_ADDR + ATIM_CR1_OFFSET, PAGE_SIZE), 0x01);

        period_buffer = mapRegister(ATIM_BASE_ADDR + ATIM_ARR_OFFSET, PAGE_SIZE);
        duty_cycle_buffer = mapRegister(ATIM_BASE_ADDR + ATIM_CCR1_OFFSET + chNum * 0x04, PAGE_SIZE);
        ccmr_buffer[0] = mapRegister(ATIM_BASE_ADDR + ATIM_CCMR1_OFFSET, PAGE_SIZE);
        ccmr_buffer[1] = mapRegister(ATIM_BASE_ADDR + ATIM_CCMR2_OFFSET, PAGE_SIZE);
        ccer_buffer = mapRegister(ATIM_BASE_ADDR + ATIM_CCER_OFFSET, PAGE_SIZE);
        cnt_buffer = mapRegister(ATIM_BASE_ADDR + ATIM_CNT_OFFSET, PAGE_SIZE);
        bdtr_buffer = mapRegister(ATIM_BASE_ADDR + ATIM_BDTR_OFFSET, PAGE_SIZE);

        // 清除chNum的PWM模式
        REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) & ~(0x7 << (chNum % 2 * 8 + 4)));
        // 配置chNum的PWM模式 0x6为模式1 0x7为模式2
        REG_WRITE(ccmr_buffer[chNum / 2], REG_READ(ccmr_buffer[chNum / 2]) | (0x7 << (chNum % 2 * 8 + 4)));

        // 清除chNum的输出极性
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 1 + NEG * 2))); // 1为反相
        // 配置chNum的输出极性
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 1 + NEG * 2))); // 1为反相

        REG_WRITE(period_buffer, period_10ns);
        REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

        REG_WRITE(bdtr_buffer, 0x1 << 15); // 主输出使能

        REG_WRITE(cnt_buffer, 0);
        /* quiet: registers mapped */
    }

    /**
     * @brief 析构 PwmAtim 对象
     *
     * @details
     * 释放 Contral / PWM 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    PwmAtim::~PwmAtim(void)
    {
        munmap(ccmr_buffer[0], PAGE_SIZE);
        munmap(ccmr_buffer[1], PAGE_SIZE);
        munmap(period_buffer, PAGE_SIZE);
        munmap(duty_cycle_buffer, PAGE_SIZE);
        munmap(ccer_buffer, PAGE_SIZE);
        munmap(cnt_buffer, PAGE_SIZE);
        munmap(bdtr_buffer, PAGE_SIZE);
    }

    /**
     * @brief 使能 enable 功能
     *
     * @details
     * 打开 Contral / PWM 模块对应的硬件输出或软件功能，让控制信号开始生效。
     */
    void PwmAtim::enable(void)
    {
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) | (0x1 << (chNum * 4 + 0 + NEG * 2)));
    }

    /**
     * @brief 关闭 disable 功能
     *
     * @details
     * 关闭 Contral / PWM 模块对应的硬件输出或软件功能，用于停止输出或进入安全状态。
     */
    void PwmAtim::disable(void)
    {
        REG_WRITE(ccer_buffer, REG_READ(ccer_buffer) & ~(0x1 << (chNum * 4 + 0 + NEG * 2)));
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
    void PwmAtim::setPeriod(unsigned int period_10ns_)
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
    void PwmAtim::setDutyCycle(unsigned int duty_cycle_10ns_)
    {
        duty_cycle_10ns = duty_cycle_10ns_;
        REG_WRITE(duty_cycle_buffer, duty_cycle_10ns);

        REG_WRITE(cnt_buffer, 0);
    }

}
