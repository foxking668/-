#pragma once

#include "headfile.hpp"

namespace Contral
{
    /*
     * 基于 Linux sysfs 的通用 PWM 控制封装。
     *
     * 用于完成 PWM 通道导出、启停、周期设置和占空比设置，
     * 是电机和舵机控制所依赖的底层输出类。
     */
    class PwmController
    {
    public:
        /**
         * @brief 构造 PwmController 对象
         *
         * @details
         * 创建 Contral / PWM 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param pwmChip PWM 芯片编号，对应 /sys/class/pwm/pwmchipN。
         * @param pwmNumber PWM 通道编号，对应 pwmchip 下的 pwmN。
         */
        PwmController(int pwmChip, int pwmNumber);
        /**
         * @brief 析构 PwmController 对象
         *
         * @details
         * 释放 Contral / PWM 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~PwmController();

        /**
         * @brief 使能 enable 功能
         *
         * @details
         * 打开 Contral / PWM 模块对应的硬件输出或软件功能，让控制信号开始生效。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool enable();
        /**
         * @brief 关闭 disable 功能
         *
         * @details
         * 关闭 Contral / PWM 模块对应的硬件输出或软件功能，用于停止输出或进入安全状态。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool disable();
        /**
         * @brief 设置 setPeriod 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param period_ns PWM 周期，单位为纳秒。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool setPeriod(unsigned int period_ns);
        /**
         * @brief 设置 setDutyCycle 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param duty_cycle_ns PWM 占空时间，单位为纳秒。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool setDutyCycle(unsigned int duty_cycle_ns);
        /**
         * @brief 初始化 initialize 相关资源
         *
         * @details
         * 完成 Contral / PWM 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool initialize();
        /**
         * @brief 获取 readPeriod 对应数据
         *
         * @details
         * 从 Contral / PWM 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int readPeriod();
        /**
         * @brief 获取 readDutyCycle 对应数据
         *
         * @details
         * 从 Contral / PWM 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int readDutyCycle();

    private:
        std::string pwm_path; // PWM设备的路径
        int pwm_chip;
        int pwm_number;
        int period;
        int duty_cycle;
        /**
         * @brief 写入 writeToFile 对应数据
         *
         * @details
         * 将调用方提供的数据写入 Contral / PWM 模块管理的文件、设备节点或硬件寄存器，并返回写入结果。
         *
         * @param path 目标文件路径或设备节点路径。
         * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool writeToFile(const std::string &path, const std::string &value);
    };

}