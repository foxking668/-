#ifndef Encoder_HPP
#define Encoder_HPP

#include "headfile.hpp"

namespace Contral
{
    /*
     * 编码器读取类。
     *
     * 当前实现通过寄存器映射方式读取脉冲计数结果，
     * 并结合方向引脚推算电机当前转速与方向。
     */
    class Encoder
    {
    public:
        /**
         * @brief 构造 Encoder 对象
         *
         * @details
         * 创建 Contral / Encoder 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param pwmNum 编码器 STEP 信号接入的 PWM 通道编号，用于推导计数寄存器基地址。
         * @param gpioNum GPIO 编号，用于配置方向或电平读写引脚。
         */
        Encoder(int pwmNum, int gpioNum);
        /**
         * @brief 析构 Encoder 对象
         *
         * @details
         * 释放 Contral / Encoder 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Encoder(void);

        /**
         * @brief 更新 pulseConterUpdate 状态
         *
         * @details
         * 刷新 Contral / Encoder 模块的缓存、控制结果或运行状态，使对象内部数据与当前传感器/算法结果保持一致。
         *
         * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
         */
        double pulseConterUpdate(void);

        void setPulsesPerRevolution(double ppr) { pulses_per_revolution = ppr; }

    private:
        uint32_t base_addr;
        double pulses_per_revolution = Encoder_PPR;
        void *direction_gpio; // 这里直接使用GPIO类会报未找到类名的错误
        void *low_buffer;
        void *full_buffer;
        void *control_buffer;
        /**
         * @brief 初始化 pwmInit 相关资源
         *
         * @details
         * 完成 Contral / Encoder 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
         */
        void pwmInit(void);
        /**
         * @brief resetCounter 函数说明
         *
         * @details
         * 该函数属于 Contral / Encoder 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         */
        void resetCounter(void);
    };

}

#endif
