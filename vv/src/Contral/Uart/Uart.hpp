#pragma once

#include "headfile.hpp"

namespace Contral
{
    /*
     * 串口通信封装类。
     *
     * 基于 Linux `termios` 接口，支持设置：
     * - 波特率；
     * - 停止位；
     * - 数据位；
     * - 校验方式；
     * 并提供基础收发接口。
     */
    enum
    {
        UART_STOP1, // 1位停止位
        UART_STOP2, // 2位停止位
    };

    enum
    {
        UART_DATA5, // 5位数据位
        UART_DATA6, // 6位数据位
        UART_DATA7, // 7位数据位
        UART_DATA8  // 8位数据位
    };

    // 校验位相关宏
    enum
    {
        UART_NONE, // 无校验
        UART_ODD,  // 偶校验
        UART_EVEN  // 奇校验
    };

    class Uart
    {
    private:
        int uart_fd;
        struct termios ts;

        speed_t baud_rate;
        uint8_t stop;
        uint8_t data;
        uint8_t check;

    public:
        /**
         * @brief 构造 Uart 对象
         *
         * @details
         * 创建 Contral / Uart 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param uart 串口设备路径，例如 /dev/ttyS* 或 /dev/ttyUSB*。
         * @param baudRate 串口波特率常量。
         * @param stopBit 停止位配置。
         * @param dataBit 数据位配置。
         * @param checkBit 校验位配置。
         */
        Uart(const std::string &uart, speed_t baudRate, uint8_t stopBit, uint8_t dataBit, uint8_t checkBit);

        /**
         * @brief 设置 setBaudrate 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param baudrate 串口波特率常量。
         */
        void setBaudrate(speed_t baudrate);
        /**
         * @brief 设置 setStopBit 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param stop 停止位配置。
         */
        void setStopBit(uint8_t stop);
        /**
         * @brief 设置 setDataBit 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param data 数据位配置。
         */
        void setDataBit(uint8_t data);
        /**
         * @brief 设置 setChecBit 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param check 校验位配置。
         */
        void setChecBit(uint8_t check);
        /**
         * @brief 写入 writeData 对应数据
         *
         * @details
         * 将调用方提供的数据写入 Contral / Uart 模块管理的文件、设备节点或硬件寄存器，并返回写入结果。
         *
         * @param buf 读写数据缓冲区指针。
         * @param len 期望读写的字节数。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        ssize_t writeData(const char *buf, ssize_t len);
        /**
         * @brief 获取 readData 对应数据
         *
         * @details
         * 从 Contral / Uart 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param buf 读写数据缓冲区指针。
         * @param len 期望读写的字节数。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        ssize_t readData(char *buf, ssize_t len);

        /**
         * @brief 析构 Uart 对象
         *
         * @details
         * 释放 Contral / Uart 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Uart();
    };

}