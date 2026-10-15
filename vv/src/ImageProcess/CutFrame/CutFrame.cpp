#include "CutFrame.hpp"

namespace ImageProcess
{
    /*
     * 图像裁剪工具实现。
     *
     * 每个函数都返回裁剪后的新图像副本，
     * 这样调用方可以安全地在后续流程中继续修改结果图像。
     */

    /**
     * @brief 构造 CutFrame 对象
     *
     * @details
     * 创建 ImageProcess / CutFrame 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     */
    CutFrame::CutFrame() = default;

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
    cv::Mat CutFrame::cutFrameUpByHeight(cv::Mat frame, int height)
    {
        if (frame.empty())
        {
            return cv::Mat();
        }

        height = std::min(height, frame.rows);

        cv::Rect roi(0, height, frame.cols, frame.rows - height);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameUpByPercent(cv::Mat frame, float percent)
    {
        if (frame.empty() || percent <= 0.0f || percent >= 1.0f)
        {
            return cv::Mat();
        }

        int height = static_cast<int>(frame.rows * percent);

        cv::Rect roi(0, height, frame.cols, frame.rows - height);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameDownByHeight(cv::Mat frame, int height)
    {
        if (frame.empty())
        {
            return cv::Mat();
        }

        height = std::min(height, frame.rows);

        cv::Rect roi(0, 0, frame.cols, frame.rows - height);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameDownByPercent(cv::Mat frame, float percent)
    {
        if (frame.empty() || percent <= 0.0f || percent >= 1.0f)
        {
            return cv::Mat();
        }

        int height = static_cast<int>(frame.rows * percent);

        cv::Rect roi(0, 0, frame.cols, frame.rows - height);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameLeftByWidth(cv::Mat frame, int width)
    {
        if (frame.empty())
        {
            return cv::Mat();
        }

        width = std::min(width, frame.cols);

        cv::Rect roi(width, 0, frame.cols - width, frame.rows);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameLeftByPercent(cv::Mat frame, float percent)
    {
        if (frame.empty() || percent <= 0.0f || percent >= 1.0f)
        {
            return cv::Mat();
        }

        int width = static_cast<int>(frame.cols * percent);

        cv::Rect roi(width, 0, frame.cols - width, frame.rows);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameRightByWidth(cv::Mat frame, int width)
    {
        if (frame.empty())
        {
            return cv::Mat();
        }

        width = std::min(width, frame.cols);

        cv::Rect roi(0, 0, frame.cols - width, frame.rows);

        return frame(roi).clone();
    }

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
    cv::Mat CutFrame::cutFrameRightByPercent(cv::Mat frame, float percent)
    {
        if (frame.empty() || percent <= 0.0f || percent >= 1.0f)
        {
            return cv::Mat();
        }

        int width = static_cast<int>(frame.cols * percent);

        cv::Rect roi(0, 0, frame.cols - width, frame.rows);

        return frame(roi).clone();
    }

    /**
     * @brief 析构 CutFrame 对象
     *
     * @details
     * 释放 ImageProcess / CutFrame 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    CutFrame::~CutFrame() = default;
}
