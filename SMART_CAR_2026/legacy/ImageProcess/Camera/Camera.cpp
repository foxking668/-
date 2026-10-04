#include "Camera.hpp"

using namespace Other;

namespace ImageProcess
{
    /*
     * 通过设备路径初始化 OpenCV 摄像头采集对象。
     *
     * 典型输入示例：`/dev/video0`。
     */
    /**
     * @brief 初始化 initOpencvCaptureByPath 相关资源
     *
     * @details
     * 完成 ImageProcess / Camera 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
     *
     * @param source 摄像头设备路径或视频流地址。
     */
    void Camera::initOpencvCaptureByPath(const std::string &source)
    {
        this->device_path = source;

        cv::VideoCapture *cap = new cv::VideoCapture();

        Log log(LOG_LEVEL_GLOBAL);

        log.logOutputConsole("choose CAMERA_OPENCV mode to load", LOG_INFO);

        switch (this->type)
        {
        case CAMERA_OPENCV:
            log.logOutputConsole("Opening camera with path: " + source, LOG_DEBUG);
            cap->open(source);
            break;
        case CAMERA_OPENCV_ANY:
            log.logOutputConsole("Opening camera with path (any): " + source, LOG_DEBUG);
            cap->open(source, cv::CAP_ANY);
            break;
        case CAMERA_OPENCV_FFMPEG:
            log.logOutputConsole("Opening camera with path (ffmpeg): " + source, LOG_DEBUG);
            cap->open(source, cv::CAP_FFMPEG);
            break;
        case CAMERA_OPENCV_V4L2:
            log.logOutputConsole("Opening camera with path (v4l2): " + source, LOG_DEBUG);
            cap->open(source, cv::CAP_V4L2);
            break;
        default:
            delete cap;
            this->camera = nullptr;
            log.logOutputConsole("type error can't load next", LOG_ERROR);
            return;
        }

        std::string log_tmp = "source " + source;

        if (!cap->isOpened())
        {
            delete cap;
            this->camera = nullptr;
            log_tmp = log_tmp + " failed to open";
            log.logOutputConsole(log_tmp.c_str(), LOG_ERROR);
            return;
        }

        log_tmp = log_tmp + " opened";
        log.logOutputConsole(log_tmp.c_str(), LOG_INFO);

        this->setCamera(cap);

        this->camera = (void *)cap;
    }

    /**
     * @brief 初始化 initOpencvCaptureByIndex 相关资源
     *
     * @details
     * 完成 ImageProcess / Camera 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
     *
     * @param index 摄像头设备索引。
     */
    void Camera::initOpencvCaptureByIndex(int index)
    {
        this->device_index = index;

        cv::VideoCapture *cap = new cv::VideoCapture();

        Log log(LOG_LEVEL_GLOBAL);
        log.logOutputConsole("choose CAMERA_OPENCV mode to load", LOG_DEBUG);

        switch (this->type)
        {
        case CAMERA_OPENCV:
            log.logOutputConsole("Opening camera with index: " + std::to_string(index), LOG_DEBUG);
            cap->open(index);
            break;
        case CAMERA_OPENCV_ANY:
            log.logOutputConsole("Opening camera with index (any): " + std::to_string(index), LOG_DEBUG);
            cap->open(index, cv::CAP_ANY);
            break;
        case CAMERA_OPENCV_FFMPEG:
            log.logOutputConsole("Opening camera with index (ffmpeg): " + std::to_string(index), LOG_DEBUG);
            cap->open(index, cv::CAP_FFMPEG);
            break;
        case CAMERA_OPENCV_V4L2:
            log.logOutputConsole("Opening camera with index (v4l2): " + std::to_string(index), LOG_DEBUG);
            cap->open(index, cv::CAP_V4L2);
            break;
        default:
            delete cap;
            this->camera = nullptr;
            log.logOutputConsole("type error can't load next", LOG_ERROR);
            return;
        }

        std::string log_tmp = "device id " + std::to_string(index);

        if (!cap->isOpened())
        {
            delete cap;
            this->camera = nullptr;
            log_tmp = log_tmp + " failed to open";
            log.logOutputConsole(log_tmp.c_str(), LOG_ERROR);
            return;
        }

        log_tmp = log_tmp + " opened";
        log.logOutputConsole(log_tmp.c_str(), LOG_INFO);

        this->setCamera(cap);

        this->camera = (void *)cap;
    }

    /**
     * @brief 设置 setCamera 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param cap OpenCV VideoCapture 对象指针。
     */
    void Camera::setCamera(cv::VideoCapture *cap)
    {
        // cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc(this->video_Writer_fourcc[0], this->video_Writer_fourcc[1], this->video_Writer_fourcc[2], this->video_Writer_fourcc[3]));
        // cap->set(cv::CAP_PROP_FRAME_WIDTH, this->frame_width);
        // cap->set(cv::CAP_PROP_FRAME_HEIGHT, this->frame_height);
        // cap->set(cv::CAP_PROP_FPS, this->frame_fps);

        // cap->set(cv::CAP_PROP_BUFFERSIZE, this->buffer_size);
        // cap->set(cv::CAP_PROP_AUTOFOCUS, (int)this->is_auto_focus);
        // cap->set(cv::CAP_PROP_AUTO_EXPOSURE, (int)(!this->is_auto_exposure));
        // cap->set(cv::CAP_PROP_EXPOSURE, this->exposure_time); // 固定曝光值
        // cap->set(cv::CAP_PROP_AUTO_WB, (int)this->is_auto_white_balance);
        // cap->set(cv::CAP_PROP_WB_TEMPERATURE, this->white_balance_temperature); // 固定白平衡

        Log log(LOG_LEVEL_GLOBAL);

        std::string info = "Camera actual parameters:\n";
        info += "       Format: " + std::to_string((int)cap->get(cv::CAP_PROP_FOURCC)) + "\n";
        info += "       Resolution: " + std::to_string((int)cap->get(cv::CAP_PROP_FRAME_WIDTH)) + "x" +
                std::to_string((int)cap->get(cv::CAP_PROP_FRAME_HEIGHT)) + "\n";
        info += "       Frame rate: " + std::to_string(cap->get(cv::CAP_PROP_FPS)) + "\n";
        info += "       Buffer size: " + std::to_string((int)cap->get(cv::CAP_PROP_BUFFERSIZE));

        log.logOutputConsole(info, LOG_INFO);
    }

    /**
     * @brief 构造 Camera 对象
     *
     * @details
     * 创建 ImageProcess / Camera 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * @param type 对象类型、计算类型或回调类型，具体取决于函数所在模块。
     */
    Camera::Camera(CameraType type)
    {
        this->camera = nullptr;
        this->type = type;
        Log log(LOG_LEVEL_GLOBAL);
        log.logOutputConsole("Camera object created", LOG_INFO);
    }

    /**
     * @brief 获取 getFrame 对应数据
     *
     * @details
     * 从 ImageProcess / Camera 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @param isGetNew 是否强制抓取新帧，true 表示先 grab 再 retrieve。
     *
     * @return 返回 OpenCV 图像矩阵；当输入无效或读取失败时可能返回空矩阵。
     */
    cv::Mat Camera::getFrame(bool isGetNew)
    {
        /*
         * 获取一帧图像。
         *
         * - `isGetNew = true` 时，尽量抓取最新帧，减少缓冲延迟；
         * - `isGetNew = false` 时，按普通 `read` 方式读取。
         */
        cv::Mat frame;

        Log log(LOG_LEVEL_GLOBAL);

        if (this->type / 10 < 20) // OpenCV的摄像头
        {
            log.logOutputConsole("Retrieving frame from OpenCV capture", LOG_DEBUG);

            cv::VideoCapture *capture = static_cast<cv::VideoCapture *>(this->camera);
            if (!capture || !capture->isOpened())
            {
                log.logOutputConsole("Video capture is not initialized or opened!", LOG_ERROR);
                return frame;
            }

            if (isGetNew)
            {
                while (!capture->grab())
                    ;
                capture->retrieve(frame, cv::CAP_OPENNI_BGR_IMAGE);
            }
            else
            {
                if (!capture->read(frame))
                {
                    log.logOutputConsole("Failed to read frame from video capture!", LOG_ERROR);
                    return frame;
                }
            }

            cv::resize(frame, frame, cv::Size(this->image_width, this->image_height), 0, 0, cv::INTER_NEAREST);
        }

        // 更新图像处理属性

        log.logOutputConsole("Frame retrieved successfully", LOG_DEBUG);

        return frame;
    }

    /**
     * @brief 设置 setCameraFrameWidth 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param width 目标图像宽度或裁剪宽度。
     */
    void Camera::setCameraFrameWidth(int width)
    {
        this->frame_width = width;
    }

    /**
     * @brief 设置 setCameraFrameHeight 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param height 目标图像高度或裁剪高度。
     */
    void Camera::setCameraFrameHeight(int height)
    {
        this->frame_height = height;
    }

    /**
     * @brief 设置 setCameraFps 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param fps 摄像头目标帧率。
     */
    void Camera::setCameraFps(int fps)
    {
        this->frame_fps = fps;
    }

    /**
     * @brief 设置 setCameraBufferSize 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param bufferSize 摄像头缓冲区帧数。
     */
    void Camera::setCameraBufferSize(int bufferSize)
    {
        this->buffer_size = bufferSize;
    }

    /**
     * @brief 设置 setCameraAutoFocus 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param isAutoFocus 是否启用摄像头自动对焦。
     */
    void Camera::setCameraAutoFocus(bool isAutoFocus)
    {
        this->is_auto_focus = isAutoFocus;
    }

    /**
     * @brief 设置 setCameraAutoExposure 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param isAutoExposure 是否启用摄像头自动曝光。
     */
    void Camera::setCameraAutoExposure(bool isAutoExposure)
    {
        this->is_auto_exposure = isAutoExposure;
    }

    /**
     * @brief 设置 setCameraAutoWhiteBalance 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param isAutoWhiteBalance 是否启用摄像头自动白平衡。
     */
    void Camera::setCameraAutoWhiteBalance(bool isAutoWhiteBalance)
    {
        this->is_auto_white_balance = isAutoWhiteBalance;
    }

    /**
     * @brief 设置 setCameraExposureTime 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param time 曝光时间或等待时间参数。
     */
    void Camera::setCameraExposureTime(int time)
    {
        this->exposure_time = time;
    }

    /**
     * @brief 设置 setCameraWhiteBalanceTemperature 对应参数
     *
     * @details
     * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param temperature 白平衡色温参数。
     */
    void Camera::setCameraWhiteBalanceTemperature(int temperature)
    {
        this->white_balance_temperature = temperature;
    }

    /**
     * @brief 获取 getCameraBufferSize 对应数据
     *
     * @details
     * 从 ImageProcess / Camera 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int Camera::getCameraBufferSize()
    {
        cv::VideoCapture *capture = (cv::VideoCapture *)this->camera;
        return (int)capture->get(cv::CAP_PROP_BUFFERSIZE);
    }

    /**
     * @brief 检查 checkCamera 状态
     *
     * @details
     * 检测 ImageProcess / Camera 模块当前资源或设备状态是否可用，并将检查结果返回给调用方。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool Camera::checkCamera()
    {
        if (this->camera == nullptr)
        {
            return false;
        }
        return true;
    }

    /**
     * @brief 重新加载 reload 资源
     *
     * @details
     * 在 ImageProcess / Camera 模块出现设备异常或配置变化时释放并重新打开资源，恢复到可继续运行的状态。
     */
    void Camera::reload()
    {
        this->~Camera();

        if (this->device_index != -1)
        {
            this->open(this->device_index);
        }
        else if (this->device_path != "")
        {
            this->open(this->device_path);
        }
    }

    /**
     * @brief 析构 Camera 对象
     *
     * @details
     * 释放 ImageProcess / Camera 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    Camera::~Camera()
    {
        Log log(LOG_LEVEL_GLOBAL);

        if (this->type / 10 < 20)
        {
            cv::VideoCapture *capture = (cv::VideoCapture *)this->camera;

            if (capture != nullptr && capture->isOpened())
            {
                log.logOutputConsole("Releasing OpenCV capture resources", LOG_INFO);
                capture->release(); // 确保释放摄像机资源
                delete capture;
            }
        }
        log.logOutputConsole("Camera object destroyed", LOG_INFO);
    }
}
