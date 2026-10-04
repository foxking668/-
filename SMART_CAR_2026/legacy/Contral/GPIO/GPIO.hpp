/*
 * @Author: ilikara 3435193369@qq.com
 * @Date: 2024-10-10 15:02:00
 * @LastEditors: ilikara 3435193369@qq.com
 * @LastEditTime: 2024-12-01 03:49:47
 * @FilePath: /smartcar/lib/GPIO.h
 * @Description:
 *
 * Copyright (c) 2024 by ilikara 3435193369@qq.com, All Rights Reserved.
 */

#pragma once

#include "headfile.hpp"

namespace Contral
{
    /*
     * GPIO 控制封装类。
     *
     * 通过 Linux sysfs 接口完成 GPIO 的导出、方向设置、
     * 电平读写以及边沿触发配置。
     */
    class GPIO
    {
    public:
        /**
         * @brief 构造 GPIO 对象
         *
         * @details
         * 创建 Contral / GPIO 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param gpioNum GPIO 编号，用于配置方向或电平读写引脚。
         */
        GPIO(int gpioNum);
        /**
         * @brief 析构 GPIO 对象
         *
         * @details
         * 释放 Contral / GPIO 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~GPIO();

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
        bool setDirection(const std::string &direction);
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
        bool setEdge(const std::string &edge);
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
        bool setValue(bool value);     // 设置 GPIO 输出值
        /**
         * @brief 获取 readValue 对应数据
         *
         * @details
         * 从 Contral / GPIO 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool readValue();              // 读取 GPIO 输入值
        /**
         * @brief 获取 getFileDescriptor 对应数据
         *
         * @details
         * 从 Contral / GPIO 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int getFileDescriptor() const; // 获取 GPIO 文件描述符

    private:
        int gpioNum;
        int fd; // 文件描述符
        std::string gpioPath;

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
        bool writeToFile(const std::string &path, const std::string &value);
    };
}

