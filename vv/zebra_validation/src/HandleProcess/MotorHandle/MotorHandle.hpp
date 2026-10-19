#pragma once

#include "headfile.hpp"

/*
 * 电机控制线程入口。
 *
 * 负责周期性读取编码器速度、更新 PID 参数、
 * 并将速度闭环计算结果输出到左右电机。
 */
/**
 * @brief 电机控制线程入口
 *
 * @details
 * 循环读取编码器速度、刷新 PID 输出并更新左右电机 PWM，实现车辆速度闭环控制。
 */
void threadMotorHandle(void);
