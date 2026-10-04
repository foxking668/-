#include "Encoder.hpp"

using namespace Other;

namespace Contral
{
    /*
     * 构造时根据 PWM 通道号推导寄存器基地址，
     * 并初始化方向引脚与计数寄存器映射。
     */
    Encoder::Encoder(int pwmNum, int gpioNum) : base_addr(PWM_BASE_ADDR + pwmNum * PWM_OFFSET)
    {
        Log log(LOG_LEVEL_GLOBAL);

        log.logOutputConsole("Initialize Encoder", LOG_INFO);

        this->direction_gpio = new GPIO(gpioNum);
        GPIO *directionGPIO = (GPIO *)this->direction_gpio;
        directionGPIO->setDirection("in");

        this->control_buffer = mapRegister(base_addr + CONTROL_REG_OFFSET, PAGE_SIZE);
        this->low_buffer = mapRegister(base_addr + LOW_BUFFER_OFFSET, PAGE_SIZE);
        this->full_buffer = mapRegister(base_addr + FULL_BUFFER_OFFSET, PAGE_SIZE);

        pwmInit();
    }

    Encoder::~Encoder(void)
    {
        // mapRegister returns an offset address; munmap requires the mapping base.
        munmap(reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(control_buffer) & ~(uintptr_t(PAGE_SIZE)-1)), PAGE_SIZE);
        munmap(reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(low_buffer) & ~(uintptr_t(PAGE_SIZE)-1)), PAGE_SIZE);
        munmap(reinterpret_cast<void*>(reinterpret_cast<uintptr_t>(full_buffer) & ~(uintptr_t(PAGE_SIZE)-1)), PAGE_SIZE);

        delete (GPIO *)this->direction_gpio;
    }

    /*
     * 初始化 PWM 控制器为计数/脉冲测量模式。
     *
     * 和官方逻辑一致：
     * 只打开 CNTR_ENABLE_BIT 和 MEASURE_PULSE_BIT。
     */
    void Encoder::pwmInit(void)
    {
        uint32_t control_reg = 0;

        control_reg |= CNTR_ENABLE_BIT;
        control_reg |= MEASURE_PULSE_BIT;

        REG_WRITE(control_buffer, control_reg);
    }

    void Encoder::resetCounter(void)
    {
        uint32_t control_reg = REG_READ(control_buffer);
        control_reg |= COUNTER_RESET_BIT;
        REG_WRITE(control_buffer, control_reg);
    }

    /*
     * 返回编码器 RPS。
     * 不再打印 EncoderRaw，避免终端刷屏。
     */
    double Encoder::pulseConterUpdate(void)
    {
        Log log(LOG_LEVEL_GLOBAL);
        GPIO *directionGPIO = (GPIO *)this->direction_gpio;

        uint32_t encoderValue = REG_READ(this->full_buffer);

        int dirRaw = directionGPIO->readValue();

        double value = 0;
        if (encoderValue != 0)
        {
            value = 100000000.0 / encoderValue / pulses_per_revolution * (dirRaw * 2 - 1);
        }

        log.logOutputConsole("GET ENCODER VALUE: " + std::to_string(value), LOG_DEBUG);

        return value;
    }
}
