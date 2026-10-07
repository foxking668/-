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

        try {
        this->direction_gpio = new GPIO(gpioNum);
        GPIO *directionGPIO = (GPIO *)this->direction_gpio;
        directionGPIO->setDirection("in");

        this->control_buffer = mapRegister(base_addr + CONTROL_REG_OFFSET, ENCODER_MAP_BYTES);
        this->low_buffer = mapRegister(base_addr + LOW_BUFFER_OFFSET, ENCODER_MAP_BYTES);
        this->full_buffer = mapRegister(base_addr + FULL_BUFFER_OFFSET, ENCODER_MAP_BYTES);

        pwmInit();
        } catch(...) {releaseResources();throw;}
    }

    Encoder::~Encoder(void)
    {
        releaseResources();
    }

    void Encoder::releaseResources() noexcept
    {
        // Virtual mmap base need not be 64K-aligned: subtract the PHYSICAL offset.
        void* addresses[]={control_buffer,low_buffer,full_buffer};
        uint32_t offsets[]={CONTROL_REG_OFFSET,LOW_BUFFER_OFFSET,FULL_BUFFER_OFFSET};
        for(int i=0;i<3;++i) if(addresses[i])
            munmap(static_cast<char*>(addresses[i])-((base_addr+offsets[i])&(ENCODER_MAP_BYTES-1)),ENCODER_MAP_BYTES);

        delete (GPIO *)this->direction_gpio;
        direction_gpio=control_buffer=low_buffer=full_buffer=nullptr;
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
