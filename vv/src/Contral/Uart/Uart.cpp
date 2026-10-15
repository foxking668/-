#include "Uart.hpp"

using namespace std;

namespace Contral
{
    /*
     * 打开指定串口设备并完成基础参数初始化。
     *
     * 初始化后会进一步调用各 set 方法应用波特率、停止位、数据位和校验位配置。
     */
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
    Uart::Uart(const string &uart, speed_t baudRate, uint8_t stopBit, uint8_t dataBit, uint8_t checkBit)
    {

        this->baud_rate = baudRate;
        this->stop = stopBit;
        this->data = dataBit;
        this->check = checkBit;

        this->uart_fd = open(uart.c_str(), O_RDWR);
        if (this->uart_fd == -1)
        {
            /* quiet: unable to open dev */
            return;
        }
        // 串口配置初始化
        memset(&this->ts, 0, sizeof(this->ts));
        if (tcgetattr(this->uart_fd, &this->ts) != 0)
        {
            perror("Error tcgetattr");
            close(this->uart_fd);
            return;
        }
        this->ts.c_cflag &= ~CRTSCTS;                        // 无硬件流控制
        this->ts.c_cflag |= CREAD | CLOCAL;                  // 打开接收器，忽略modem控制线
        this->ts.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG); // 原始输入
        this->ts.c_iflag &= ~(IXON | IXOFF | IXANY);         // 禁用软件流控
        this->ts.c_oflag &= ~OPOST;                          // 原始输出
        this->ts.c_cc[VMIN] = 0;                             // 读取时不等待
        this->ts.c_cc[VTIME] = 5;                            // 0.5秒超时

        this->setBaudrate(this->baud_rate);
        this->setStopBit(this->stop);
        this->setDataBit(this->data);
        this->setChecBit(this->check);
    }

    /**
     * @brief 设置 setBaudrate 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param baudRate 串口波特率常量。
     */
    void Uart::setBaudrate(speed_t baudRate)
    {
        this->baud_rate = baudRate;
        cfsetispeed(&this->ts, baudRate); // 设置输入波特率
        cfsetospeed(&this->ts, baudRate); // 设置输出波特率
        if (tcsetattr(this->uart_fd, TCSANOW, &this->ts) != 0)
        {
            perror("Error set Baudrate");
            return;
        }
    }

    /**
     * @brief 设置 setStopBit 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param stopBit 停止位配置。
     */
    void Uart::setStopBit(uint8_t stopBit)
    {
        this->stop = stopBit;
        switch (this->stop)
        {
        case UART_STOP1:
            this->ts.c_cflag &= ~CSTOPB;
            break; // 停止位为1
        case UART_STOP2:
            this->ts.c_cflag |= CSTOPB;
            break; // 停止位为2
        default:
            /* quiet: stop bit setting error */
            break;
        }
        if (tcsetattr(this->uart_fd, TCSANOW, &this->ts) != 0)
        {
            perror("Error set Stop Bit");
            return;
        }
    }

    /**
     * @brief 设置 setDataBit 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param dataBit 数据位配置。
     */
    void Uart::setDataBit(uint8_t dataBit)
    {
        this->data = dataBit;
        this->ts.c_cflag &= ~CSIZE; // 清除数据位大小设置
        switch (this->data)
        {
        case UART_DATA5: // 数据位为5
            this->ts.c_cflag |= CS5;
            break;
        case UART_DATA6: // 数据位为6
            this->ts.c_cflag |= CS6;
            break;
        case UART_DATA7: // 数据位为7
            this->ts.c_cflag |= CS7;
            break;
        case UART_DATA8: // 数据位为8
            this->ts.c_cflag |= CS8;
            break;
        default:
            /* quiet: data bit setting error */
            break;
        }
        if (tcsetattr(this->uart_fd, TCSANOW, &this->ts) != 0)
        {
            perror("Error set Stop Bit");
            return;
        }
    }

    /**
     * @brief 设置 setChecBit 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / Uart 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param checkBit 校验位配置。
     */
    void Uart::setChecBit(uint8_t checkBit)
    {
        this->check = checkBit;
        this->ts.c_cflag &= ~PARENB; // 清除校验位
        switch (this->check)
        {
        case UART_NONE: // 无校验
            break;
        case UART_ODD: // 偶校验
            this->ts.c_cflag |= PARENB;
            this->ts.c_cflag &= PARODD;
            break;
        case UART_EVEN: // 奇校验
            this->ts.c_cflag |= PARENB;
            this->ts.c_cflag |= PARODD;
            break;
        default:
            /* quiet: check bit setting error */
            break;
        }
        if (tcsetattr(this->uart_fd, TCSANOW, &this->ts) != 0)
        {
            perror("Error set Check Bit");
            return;
        }
    }

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
    ssize_t Uart::writeData(const char *buf, ssize_t len)
    {
        return write(this->uart_fd, buf, len);
    }

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
    ssize_t Uart::readData(char *buf, ssize_t len)
    {
        return read(this->uart_fd, buf, len);
    }

    /**
     * @brief 析构 Uart 对象
     *
     * @details
     * 释放 Contral / Uart 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    Uart::~Uart()
    {
        close(this->uart_fd);
    }
}