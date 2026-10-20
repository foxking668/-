/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 15:02:10
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2024-12-01 03:50:49
 * @FilePath: /smartcar/src/GPIO.cpp
 * @Description:
 *
 * Copyright (c) 2024 by ${git_name_email}, All Rights Reserved.
 */
#include "GPIO.hpp"
#include "ZebraTest/SysfsIo.hpp"
#include <chrono>
#include <filesystem>
#include <thread>

namespace Contral
{
    /*
     * 构造函数负责导出指定 GPIO，并打开其 value 节点，
     * 供后续高频读写直接复用文件描述符。
     */
    /**
     * @brief 构造 GPIO 对象
     *
     * @details
     * 创建 Contral / GPIO 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * @param gpioNum GPIO 编号，用于配置方向或电平读写引脚。
     */
    GPIO::GPIO(int gpioNum, const std::string& root) : gpioNum(gpioNum), fd(-1)
    {
        gpioPath=root+"/gpio"+std::to_string(gpioNum);
        if(gpioNum<0) {
            ZebraTest::IoStatus::set("GPIO_NUMBER",gpioPath,EINVAL);
            ZebraTest::IoStatus::require(false,"GPIO initialize");
        }
        std::error_code error;
        const bool exists=std::filesystem::is_directory(gpioPath,error);
        if(error && error!=std::errc::no_such_file_or_directory) {
            ZebraTest::IoStatus::set("GPIO_DIRECTORY",gpioPath,error.value());
            ZebraTest::IoStatus::require(false,"GPIO initialize");
        }
        std::string exportError;
        if(!exists && !writeToFile(root+"/export",std::to_string(gpioNum)))
            exportError=ZebraTest::IoStatus::lastError;
        for(int attempt=0;attempt<50;++attempt) {
            fd=::open((gpioPath+"/value").c_str(),O_RDWR);
            if(fd>=0) return;
            ZebraTest::IoStatus::set("GPIO_VALUE_OPEN",gpioPath+"/value",errno);
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        if(!exportError.empty())ZebraTest::IoStatus::lastError+=" export_error={"+exportError+"}";
        ZebraTest::IoStatus::require(false,"GPIO initialize");
    }

    /**
     * @brief 析构 GPIO 对象
     *
     * @details
     * 释放 Contral / GPIO 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    GPIO::~GPIO()
    {
        if (fd != -1)
        {
            close(fd); // 关闭文件描述符
        }
    }

    /**
     * @brief 设置 setDirection 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / GPIO 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param direction 要写入 sysfs direction 节点的方向字符串，通常为 "in" 或 "out"。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool GPIO::setDirection(const std::string &direction)
    {
        return writeToFile(gpioPath + "/direction", direction);
    }

    /**
     * @brief 设置 setEdge 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / GPIO 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param edge 要写入 sysfs edge 节点的边沿触发类型，例如 none、rising、falling 或 both。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool GPIO::setEdge(const std::string &edge)
    {
        return writeToFile(gpioPath + "/edge", edge);
    }

    /**
     * @brief 设置 setValue 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / GPIO 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool GPIO::setValue(bool value)
    {
        if (fd == -1)
        {
            ZebraTest::IoStatus::set("GPIO_FD",gpioPath+"/value",EBADF);
            return false;
        }

        // 使用文件描述符写入 GPIO 值 ('1' 或 '0')
        const char *val_str = value ? "1" : "0";
        if(lseek(fd,0,SEEK_SET)<0) {
            ZebraTest::IoStatus::set("GPIO_SEEK",gpioPath+"/value",errno);return false;
        }
        decltype(write(fd,val_str,1)) written;
        do { written=write(fd,val_str,1); } while(written<0 && errno==EINTR);
        if (written != 1)
        {
            ZebraTest::IoStatus::set("GPIO_WRITE",gpioPath+"/value",written<0?errno:EIO);
            return false;
        }
        return true;
    }

    /**
     * @brief 获取 readValue 对应数据
     *
     * @details
     * 从 Contral / GPIO 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool GPIO::readValue()
    {
        last_read_ok_=false;
        if (fd == -1)
        {
            ZebraTest::IoStatus::set("GPIO_FD",gpioPath+"/value",EBADF);
            return false;
        }

        char value;
        if(lseek(fd,0,SEEK_SET)<0){ZebraTest::IoStatus::set("GPIO_SEEK",gpioPath+"/value",errno);return false;}
        if (read(fd, &value, 1) != 1)
        {
            ZebraTest::IoStatus::set("GPIO_READ",gpioPath+"/value",errno);
            return false;
        }
        last_read_ok_ = value == '0' || value == '1';
        if(!last_read_ok_)ZebraTest::IoStatus::set("GPIO_INVALID_VALUE",gpioPath+"/value",0);
        return value == '1'; // 如果读取的值为 '1'，则返回 true，否则返回 false
    }

    /**
     * @brief 获取 getFileDescriptor 对应数据
     *
     * @details
     * 从 Contral / GPIO 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int GPIO::getFileDescriptor() const
    {
        return fd;
    }

    /**
     * @brief 写入 writeToFile 对应数据
     *
     * @details
     * 将调用方提供的数据写入 Contral / GPIO 模块管理的文件、设备节点或硬件寄存器，并返回写入结果。
     *
     * @param path 目标文件路径或设备节点路径。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool GPIO::writeToFile(const std::string &path, const std::string &value)
    {
        return ZebraTest::SysfsIo::writeValue(path,value);
    }
}