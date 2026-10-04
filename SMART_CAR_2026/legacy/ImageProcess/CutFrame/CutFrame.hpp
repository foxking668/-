#pragma once
#include "headfile.hpp"

using namespace cv;
using namespace std;

namespace ImageProcess
{
    /*
     * 图像裁剪工具类。
     *
     * 提供按像素或按比例对图像上下左右区域进行裁剪，
     * 常用于提前去除赛道图像中无用区域，降低后续算法负担。
     */
    class CutFrame
    {
    private:
    public:
        /**
         * @brief 构造 CutFrame 对象
         *
         * @details
         * 创建 ImageProcess / CutFrame 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        CutFrame();

        /**
         * @brief cutFrameUpByHeight 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param height 目标图像高度或裁剪高度。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameUpByHeight(Mat frame, int height);

        /**
         * @brief cutFrameUpByPercent 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param percent percent 参数，参与 cutFrameUpByPercent 的业务处理或状态更新。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameUpByPercent(Mat frame, float percent);

        /**
         * @brief cutFrameDownByHeight 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param height 目标图像高度或裁剪高度。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameDownByHeight(Mat frame, int height);

        /**
         * @brief cutFrameDownByPercent 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param percent percent 参数，参与 cutFrameDownByPercent 的业务处理或状态更新。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameDownByPercent(Mat frame, float percent);

        /**
         * @brief cutFrameLeftByWidth 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param width 目标图像宽度或裁剪宽度。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameLeftByWidth(Mat frame, int width);

        /**
         * @brief cutFrameLeftByPercent 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param percent percent 参数，参与 cutFrameLeftByPercent 的业务处理或状态更新。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameLeftByPercent(Mat frame, float percent);

        /**
         * @brief cutFrameRightByWidth 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param width 目标图像宽度或裁剪宽度。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameRightByWidth(Mat frame, int width);

        /**
         * @brief cutFrameRightByPercent 函数说明
         *
         * @details
         * 该函数属于 ImageProcess / CutFrame 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param frame 输入或输出的 OpenCV 图像帧。
         * @param percent percent 参数，参与 cutFrameRightByPercent 的业务处理或状态更新。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat cutFrameRightByPercent(Mat frame, float percent);

        /**
         * @brief 析构 CutFrame 对象
         *
         * @details
         * 释放 ImageProcess / CutFrame 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~CutFrame();
    };

}
