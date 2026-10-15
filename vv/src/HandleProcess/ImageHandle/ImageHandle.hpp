#pragma once

#include "headfile.hpp"

/*
 * 单帧图像处理接口。
 *
 * 返回值为当前图像计算得到的赛道偏差角，
 * 同时可按需回传处理结果、图像数据以及图像尺寸。
 */
/**
 * @brief 执行一次图像处理流程
 *
 * @details
 * 从摄像头读取图像并完成赛道识别、中心线计算和转向误差输出，同时可回传调试图像数据。
 *
 * @param result 用于接收处理结果的输出参数指针。
 * @param imageDate 用于回传图像数据地址的输出参数。
 * @param height 目标图像高度或裁剪高度。
 * @param width 目标图像宽度或裁剪宽度。
 *
 * @return 返回浮点数结果，通常表示速度、角度、误差或比例计算值。
 */
float getImageHandle(bool *result = nullptr, void **imageDate = nullptr, int *height = nullptr, int *width = nullptr);

/*
 * 图像处理线程入口。
 *
 * 循环完成：取帧、图像预处理、赛道识别、偏差计算、舵机控制、
 * 以及图像调试信息缓存更新。
 */
/**
 * @brief 图像处理线程入口
 *
 * @details
 * 循环获取摄像头画面，执行裁剪、赛道线识别、误差计算和舵机控制，并根据全局关闭标志退出。
 */
void threadRunImageHandle(void);
