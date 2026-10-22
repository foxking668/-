#pragma once
#include "headfile.hpp"

namespace Contral
{
    /*
     * 电机控制类。
     *
     * 负责封装左右电机的：
     * - PWM 输出；
     * - 方向控制 GPIO；
     * - 使能引脚；
     * - PID 参数提交与计算结果输出。
     */
    class Motor
    {
    private:
        void *pwm_left;
        void *dir_left;
        void *pwm_right;
        void *dir_right;
        void *motor_enable_gpio;

        st_PID_Attr *left_pid_attr;
        st_PID_Attr *right_pid_attr;

        int pwm_left_value;
        int pwm_right_value;

        float aim_value;

        float left_true_speed;
        float right_true_speed;

        /**
         * @brief constrainPwmValue 函数说明
         *
         * @details
         * 该函数属于 Contral / Motor 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param pwm 待约束或输出的 PWM 数值，正负号通常表示电机方向。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int constrainPwmValue(int pwm);

    public:
        /**
         * @brief 构造 Motor 对象
         *
         * @details
         * 创建 Contral / Motor 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param pwm_left_chip 左电机 PWM 控制器芯片编号。
         * @param pwm_left_index 左电机 PWM 通道编号。
         * @param dir_left_index 左电机方向 GPIO 编号。
         * @param pwm_right_chip 右电机 PWM 控制器芯片编号。
         * @param pwm_right_index 右电机 PWM 通道编号。
         * @param dir_right_index 右电机方向 GPIO 编号。
         * @param motor_enable_pin 电机驱动使能 GPIO 编号。
         */
        Motor(int pwm_left_chip, int pwm_left_index, int dir_left_index, int pwm_right_chip, int pwm_right_index, int dir_right_index, int motor_enable_pin);

        /**
         * @brief 设置 setLeftPwm 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Motor 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param pwm 待约束或输出的 PWM 数值，正负号通常表示电机方向。
         */
        void setLeftPwm(int pwm);
        /**
         * @brief 设置 setRightPwm 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Motor 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param pwm 待约束或输出的 PWM 数值，正负号通常表示电机方向。
         */
        void setRightPwm(int pwm);

        /**
         * @brief 提交 submitLeftPidCtrlAttr 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 Contral / Motor 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param pidAttr PID 属性结构体指针，用于传入或保存控制器状态。
         */
        void submitLeftPidCtrlAttr(st_PID_Attr *pidAttr);
        /**
         * @brief 提交 submitRightPidCtrlAttr 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 Contral / Motor 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param pidAttr PID 属性结构体指针，用于传入或保存控制器状态。
         */
        void submitRightPidCtrlAttr(st_PID_Attr *pidAttr);

        /**
         * @brief 设置 setAimValue 对应参数
         *
         * @details
         * 根据传入参数更新 Contral / Motor 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param aimValue 目标控制值，例如目标速度、目标角度或目标误差。
         */
        void setAimValue(float aimValue);

        /**
         * @brief 提交 submitRightTrueSpeed 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 Contral / Motor 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param trueSpeed 编码器反馈得到的实际速度值。
         */
        void submitRightTrueSpeed(float trueSpeed);
        /**
         * @brief 提交 submitLeftTrueSpeed 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 Contral / Motor 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param trueSpeed 编码器反馈得到的实际速度值。
         */
        void submitLeftTrueSpeed(float trueSpeed);

        /**
         * @brief 提交 submitLeftPwm 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 Contral / Motor 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param pwm 待约束或输出的 PWM 数值，正负号通常表示电机方向。
         */
        void submitLeftPwm(int pwm);
        /**
         * @brief 提交 submitRightPwm 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 Contral / Motor 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param pwm 待约束或输出的 PWM 数值，正负号通常表示电机方向。
         */
        void submitRightPwm(int pwm);

        /**
         * @brief 获取 getLeftPwm 对应数据
         *
         * @details
         * 从 Contral / Motor 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int getLeftPwm();
        /**
         * @brief 获取 getRightPwm 对应数据
         *
         * @details
         * 从 Contral / Motor 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int getRightPwm();

        /**
         * @brief 计算 updateCalcPidCtrl 结果
         *
         * @details
         * 按照 Contral / Motor 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
         *
         * @param offsetLeft 左轮 PID 输出的附加修正量。
         * @param offsetRight 右轮 PID 输出的附加修正量。
         */
        void updateCalcPidCtrl(int offsetLeft, int offsetRight);

        /**
         * @brief 析构 Motor 对象
         *
         * @details
         * 释放 Contral / Motor 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Motor();
    };

} // namespace Contral