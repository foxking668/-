#pragma once
#include "headfile.hpp"

using namespace cv;
using namespace std;

namespace ImageProcess
{
    /*
     * 图像预处理类。
     *
     * 封装灰度化、Canny、Otsu、自适应阈值、HSV 白色提取、
     * 轮廓查找等常用操作，为赛道线提取提供统一接口。
     */
    class HandleImage
    {
    private:
        Mat origin_frame;
        Mat grayscale_frame;
        Mat canny_frame;
        Mat bin_frame;

        Mat find_contour_frame;

        vector<vector<Point>> contours;

    public:
        /**
         * @brief 构造 HandleImage 对象
         *
         * @details
         * 创建 ImageProcess / HandleImage 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         */
        HandleImage();

        /**
         * @brief 提交 submitOriginFrame 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param originFrame originFrame 参数，参与 submitOriginFrame 的业务处理或状态更新。
         * @param isGray 布尔开关参数，用于启用或关闭对应功能。
         */
        void submitOriginFrame(Mat originFrame, bool isGray = false);
        /**
         * @brief 提交 submitCannyFrame 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param cannyFrame cannyFrame 参数，参与 submitCannyFrame 的业务处理或状态更新。
         */
        void submitCannyFrame(Mat cannyFrame);
        /**
         * @brief 提交 submitBinFrame 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param binFrame binFrame 参数，参与 submitBinFrame 的业务处理或状态更新。
         */
        void submitBinFrame(Mat binFrame);
        /**
         * @brief 提交 submitFindContourFrame 输入数据
         *
         * @details
         * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
         *
         * @param findContourFrame findContourFrame 参数，参与 submitFindContourFrame 的业务处理或状态更新。
         */
        void submitFindContourFrame(Mat findContourFrame);

        /**
         * @brief 获取 getGrayscaleFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getGrayscaleFrame();
        /**
         * @brief 获取 getCannyFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getCannyFrame();
        /**
         * @brief 获取 getOtsuBinFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getOtsuBinFrame();
        /**
         * @brief 获取 getOtsuBinFrameByGaussian 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getOtsuBinFrameByGaussian();
        /**
         * @brief 获取 getOtsuBinFrameBySharpness 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getOtsuBinFrameBySharpness();
        /**
         * @brief 获取 getWhiteBinFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getWhiteBinFrame();
        /**
         * @brief 获取 getBinFrameByFixThreshold 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @param threshold 阈值参数，用于判断误差区间或算法分段。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getBinFrameByFixThreshold(u_char threshold);
        /**
         * @brief 获取 getGreyBinaryFromOriginalFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getGreyBinaryFromOriginalFrame();
        /**
         * @brief 获取 getGreyAndWhiteBinaryFromOriginalFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getGreyAndWhiteBinaryFromOriginalFrame();
        /**
         * @brief 获取 getWhiteBinaryFromOriginalFrame 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getWhiteBinaryFromOriginalFrame();
        /**
         * @brief 获取 getGreyBinaryFromOriginalFrameByGaussian 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getGreyBinaryFromOriginalFrameByGaussian();
        /**
         * @brief 获取 getAdaptiveBinFrameByGaussian 对应数据
         *
         * @details
         * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
         */
        Mat getAdaptiveBinFrameByGaussian();

        /**
         * @brief 查找 findWhiteContourBigToSmall 对应节点
         *
         * @details
         * 在 ImageProcess / HandleImage 模块维护的链表、缓存或图像数据中查找符合条件的节点/位置，并将结果返回给调用方。
         *
         * @param choose choose 参数，参与 findWhiteContourBigToSmall 的业务处理或状态更新。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        vector<Point> findWhiteContourBigToSmall(int choose = 0);

        /**
         * @brief 析构 HandleImage 对象
         *
         * @details
         * 释放 ImageProcess / HandleImage 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~HandleImage();
    };

}