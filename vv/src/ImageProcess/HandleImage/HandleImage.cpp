#include "HandleImage.hpp"

using namespace Other;

namespace ImageProcess
{
    /**
     * @brief 构造 HandleImage 对象
     *
     * @details
     * 创建 ImageProcess / HandleImage 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     */
    HandleImage::HandleImage() = default;

    /**
     * @brief 提交 submitOriginFrame 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param originFrame originFrame 参数，参与 submitOriginFrame 的业务处理或状态更新。
     * @param isGray 布尔开关参数，用于启用或关闭对应功能。
     */
    void HandleImage::submitOriginFrame(Mat originFrame, bool isGray)
    {
        /*
         * 提交原始图像。
         * 如果调用方已经确保输入是灰度图，可通过 `isGray` 直接复用，
         * 避免重复颜色空间转换。
         */
        this->origin_frame = originFrame;
        if (isGray)
        {
            this->grayscale_frame = originFrame;
        }
    }

    /**
     * @brief 提交 submitCannyFrame 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param cannyFrame cannyFrame 参数，参与 submitCannyFrame 的业务处理或状态更新。
     */
    void HandleImage::submitCannyFrame(Mat cannyFrame)
    {
        this->canny_frame = cannyFrame;
    }

    /**
     * @brief 提交 submitBinFrame 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param binFrame binFrame 参数，参与 submitBinFrame 的业务处理或状态更新。
     */
    void HandleImage::submitBinFrame(Mat binFrame)
    {
        this->bin_frame = binFrame;
    }

    /**
     * @brief 提交 submitFindContourFrame 输入数据
     *
     * @details
     * 把调用方提供的实时数据写入 ImageProcess / HandleImage 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
     *
     * @param findContourFrame findContourFrame 参数，参与 submitFindContourFrame 的业务处理或状态更新。
     */
    void HandleImage::submitFindContourFrame(Mat findContourFrame)
    {
        this->find_contour_frame = findContourFrame;
    }

    /**
     * @brief 获取 getGrayscaleFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getGrayscaleFrame()
    {
        if (this->grayscale_frame.empty())
        {
            cvtColor(this->origin_frame, this->grayscale_frame, COLOR_BGR2GRAY);
        }

        return this->grayscale_frame;
    }

    /**
     * @brief 获取 getCannyFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getCannyFrame()
    {
        if (this->canny_frame.empty())
        {
            if (this->grayscale_frame.empty())
            {
                this->getGrayscaleFrame();
            }
            Canny(this->grayscale_frame, this->canny_frame, 100, 250);
        }

        return this->canny_frame;
    }

    /**
     * @brief 获取 getOtsuBinFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getOtsuBinFrame()
    {
        if (this->bin_frame.empty())
        {
            if (this->grayscale_frame.empty())
            {
                this->getGrayscaleFrame();
            }

            cv::threshold(this->grayscale_frame, this->bin_frame, 255, gBinWhilePointValue, cv::THRESH_OTSU);
        }

        return this->bin_frame;
    }

    /**
     * @brief 获取 getOtsuBinFrameByGaussian 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getOtsuBinFrameByGaussian()
    {
        if (this->bin_frame.empty())
        {
            if (this->grayscale_frame.empty())
            {
                this->getGrayscaleFrame();
            }

            // 预处理：高斯模糊降噪
            cv::Mat processed;
            cv::GaussianBlur(this->grayscale_frame, processed, cv::Size(3, 3), 0);

            // 增强对比度：使用直方图均衡化提高图像的对比度
            cv::Mat equalized;
            cv::equalizeHist(processed, equalized); // 直方图均衡化

            // 使用 Otsu 方法进行阈值二值化
            cv::threshold(equalized, this->bin_frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            // 二次处理：填充小的孔洞
            cv::morphologyEx(this->bin_frame, this->bin_frame, cv::MORPH_CLOSE,
                             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        }
        return this->bin_frame;
    }

    /**
     * @brief 获取 getOtsuBinFrameBySharpness 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getOtsuBinFrameBySharpness()
    {
        if (this->bin_frame.empty())
        {
            if (this->grayscale_frame.empty())
            {
                this->getGrayscaleFrame();
            }

            // 预处理：高斯模糊降噪
            cv::Mat processed;
            cv::GaussianBlur(this->grayscale_frame, processed, cv::Size(3, 3), 0);

            // 锐化图像：使用拉普拉斯滤波器进行锐化
            cv::Mat sharpened;
            cv::Mat kernel = (cv::Mat_<float>(3, 3) << 0, -1, 0,
                              -1, 5, -1,
                              0, -1, 0);
            cv::filter2D(processed, sharpened, processed.depth(), kernel);

            // 使用 Otsu 方法进行阈值二值化
            cv::threshold(sharpened, this->bin_frame, 0, 255, cv::THRESH_BINARY | cv::THRESH_OTSU);

            // 二次处理：填充小的孔洞
            cv::morphologyEx(this->bin_frame, this->bin_frame, cv::MORPH_CLOSE,
                             cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3)));
        }
        return this->bin_frame;
    }

    /**
     * @brief 获取 getWhiteBinFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getWhiteBinFrame()
    {
        if (this->bin_frame.empty())
        {
            if (this->origin_frame.empty())
            {
                throw runtime_error("original_frame is empty");
            }

            // 转换为 HSV 色彩空间
            Mat hsv;
            cvtColor(this->origin_frame, hsv, COLOR_BGR2HSV);

            // 白色区域的 HSV 范围
            Scalar lower_white(0, 0, 200);
            Scalar upper_white(180, 50, 255);  /////////////////////////////

            // 提取白色区域
            Mat mask;
            inRange(hsv, lower_white, upper_white, mask);

            // 形态学闭操作去噪
            Mat kernel = getStructuringElement(MORPH_RECT, Size(5, 5));
            morphologyEx(mask, mask, MORPH_CLOSE, kernel);

            // 保存二值图像以供后续处理
            this->bin_frame = mask.clone();
        }
        return this->bin_frame;
    }

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
    Mat HandleImage::getBinFrameByFixThreshold(u_char threshold)
    {
        if (this->bin_frame.empty())
        {
            if (this->grayscale_frame.empty())
            {
                this->getGrayscaleFrame();
            }

            cv::threshold(this->grayscale_frame, this->bin_frame, threshold, gBinWhilePointValue, THRESH_BINARY);
        }

        return this->bin_frame;
    }

    /**
     * @brief 获取 getGreyBinaryFromOriginalFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getGreyBinaryFromOriginalFrame()
    {
        if (this->origin_frame.empty())
        {
            return Mat();
        }

        if (this->bin_frame.empty())
        {

            Mat hsv_frame;
            cvtColor(this->origin_frame, hsv_frame, COLOR_BGR2HSV);

            Scalar lower_white = Scalar(0, 0, 46); // H:任意(0~180), S:低(0~50), V:高(200~255)
            Scalar upper_white = Scalar(180, 43, 220);

            inRange(hsv_frame, lower_white, upper_white, this->bin_frame);
        }
        return bin_frame;
    }

    /**
     * @brief 获取 getGreyAndWhiteBinaryFromOriginalFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getGreyAndWhiteBinaryFromOriginalFrame()
    {
        if (this->origin_frame.empty())
        {
            return Mat();
        }

        if (this->bin_frame.empty())
        {

            Mat hsv_frame;
            cvtColor(this->origin_frame, hsv_frame, COLOR_BGR2HSV);

            Scalar lower_white = Scalar(0, 0, 46); // H:任意(0~180), S:低(0~50), V:高(200~255)
            Scalar upper_white = Scalar(180, 48, 255);

            inRange(hsv_frame, lower_white, upper_white, this->bin_frame);
        }
        return bin_frame;
    }

    /**
     * @brief 获取 getWhiteBinaryFromOriginalFrame 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getWhiteBinaryFromOriginalFrame()
    {
        if (this->origin_frame.empty())
        {
            return Mat();
        }

        if (this->bin_frame.empty())
        {

            Mat hsv_frame;
            cvtColor(this->origin_frame, hsv_frame, COLOR_BGR2HSV);

            Scalar lower_white = Scalar(0, 0, 173); // H:任意(0~180), S:低(0~50), V:高(200~255)
            Scalar upper_white = Scalar(180, 30, 255);

            inRange(hsv_frame, lower_white, upper_white, this->bin_frame);
        }
        return bin_frame;
    }

    /**
     * @brief 获取 getGreyBinaryFromOriginalFrameByGaussian 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getGreyBinaryFromOriginalFrameByGaussian()
    {
        if (this->origin_frame.empty())
        {
            return Mat();
        }

        if (this->bin_frame.empty())
        {

            Mat blurred_frame;
            GaussianBlur(this->origin_frame, blurred_frame, Size(7, 7), 0);

            Mat hsv_frame;
            cvtColor(this->origin_frame, hsv_frame, COLOR_BGR2HSV);

            Scalar lower_white = Scalar(0, 0, 46); // H:任意(0~180), S:低(0~50), V:高(200~255)
            Scalar upper_white = Scalar(180, 43, 220);

            inRange(hsv_frame, lower_white, upper_white, this->bin_frame);
        }
        return bin_frame;
    }

    /**
     * @brief 获取 getAdaptiveBinFrameByGaussian 对应数据
     *
     * @details
     * 从 ImageProcess / HandleImage 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    Mat HandleImage::getAdaptiveBinFrameByGaussian()
    {
        if (this->bin_frame.empty())
        {
            if (this->grayscale_frame.empty())
            {
                this->getGrayscaleFrame();
            }

            // 高斯模糊去噪
            Mat blurred_frame;
            GaussianBlur(this->grayscale_frame, blurred_frame, Size(7, 7), 0);

            // 自适应二值化
            adaptiveThreshold(blurred_frame, this->bin_frame, 255, ADAPTIVE_THRESH_GAUSSIAN_C, THRESH_BINARY, 159, 12);
        }

        return this->bin_frame;
    }

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
    vector<Point> HandleImage::findWhiteContourBigToSmall(int choose) // 将函数改为类成员函数
    {
        if (!this->contours.empty())
        {
            if (choose >= 0 && choose < static_cast<int>(this->contours.size()))
            {
                return this->contours[choose];
            }
            else
            {
                return vector<Point>();
            }
        }

        if (this->find_contour_frame.empty())
        {
            throw runtime_error("find_contour_frame is empty"); // 使用标准异常类
        }

        vector<vector<Point>> contours;
        vector<Vec4i> hierarchy;

        // 查找轮廓
        findContours(this->find_contour_frame.clone(), contours, hierarchy, RETR_LIST, CHAIN_APPROX_NONE);

        // 按面积从大到小排序
        sort(contours.begin(), contours.end(), [](const vector<Point> &c1, const vector<Point> &c2)
             { return contourArea(c1) > contourArea(c2); });

        // 如果找到了轮廓，则返回最大的那个或指定索引的轮廓
        if (!contours.empty() && choose >= 0 && choose < contours.size())
        {
            this->contours = contours;
            return this->contours[choose];
        }
        else
        {
            // 没有找到任何轮廓时返回空vector
            return vector<Point>();
        }
    }

    /**
     * @brief 析构 HandleImage 对象
     *
     * @details
     * 释放 ImageProcess / HandleImage 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    HandleImage::~HandleImage() = default;
}