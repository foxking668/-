#pragma once
#include "headfile.hpp"

using namespace cv;

namespace ImageProcess
{
    /*
     * 摄像头抽象类。
     *
     * 当前主要封装 OpenCV `VideoCapture`，支持：
     * - 通过设备路径或设备序号打开；
     * - 获取帧图像；
     * - 设置采集参数；
     * - 重新加载摄像头。
     */
    enum CameraType
    {
        CAMERA_OPENCV = 0,
        CAMERA_OPENCV_ANY = 1,
        CAMERA_OPENCV_FFMPEG = 2,
        CAMERA_OPENCV_V4L2 = 3
    };

    class Camera
    {
    private:
        void *camera;
        CameraType type;

        int device_index = -1;
        std::string device_path;

        char video_Writer_fourcc[4] = {'M', 'J', 'P', 'G'};
        int frame_width = 320;
        int frame_height = 240;
        int frame_fps = 120;
        int buffer_size = 1;
        bool is_auto_focus = true;
        bool is_auto_exposure = true;
        int exposure_time = 40;
        bool is_auto_white_balance = true;
        int white_balance_temperature = 5000;

        /**
         * @brief 初始化 initOpencvCaptureByPath 相关资源
         *
         * @details
         * 完成 ImageProcess / Camera 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
         *
         * @param source 摄像头设备路径或视频流地址。
         */
        void initOpencvCaptureByPath(const std::string &source);
        /**
         * @brief 初始化 initOpencvCaptureByIndex 相关资源
         *
         * @details
         * 完成 ImageProcess / Camera 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
         *
         * @param index 摄像头设备索引。
         */
        void initOpencvCaptureByIndex(int index);

        /**
         * @brief 设置 setCamera 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param cap OpenCV VideoCapture 对象指针。
         */
        void setCamera(cv::VideoCapture *cap);

    public:
        int image_width = 0;
        int image_height = 0;

        /**
         * @brief 构造 Camera 对象
         *
         * @details
         * 创建 ImageProcess / Camera 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param type 对象类型、计算类型或回调类型，具体取决于函数所在模块。
         */
        Camera(CameraType type);

        template <typename T>
        /**
         * @brief 打开 open 相关资源
         *
         * @details
         * 打开 ImageProcess / Camera 模块依赖的设备、缓存或功能开关，为后续读写和控制操作建立可用通道。
         *
         * @param device 摄像头设备索引或设备路径，模板参数允许两种形式。
         */
        void open(T device = 0);

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
        Mat getFrame(bool isGetNew = false);

        /**
         * @brief 设置 setCameraFrameWidth 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param width 目标图像宽度或裁剪宽度。
         */
        void setCameraFrameWidth(int width);
        /**
         * @brief 设置 setCameraFrameHeight 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param height 目标图像高度或裁剪高度。
         */
        void setCameraFrameHeight(int height);
        /**
         * @brief 设置 setCameraFps 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param fps 摄像头目标帧率。
         */
        void setCameraFps(int fps);
        /**
         * @brief 设置 setCameraBufferSize 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param bufferSize 摄像头缓冲区帧数。
         */
        void setCameraBufferSize(int bufferSize);
        /**
         * @brief 设置 setCameraAutoFocus 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param isAutoFocus 是否启用摄像头自动对焦。
         */
        void setCameraAutoFocus(bool isAutoFocus);
        /**
         * @brief 设置 setCameraAutoExposure 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param isAutoExposure 是否启用摄像头自动曝光。
         */
        void setCameraAutoExposure(bool isAutoExposure);
        /**
         * @brief 设置 setCameraAutoWhiteBalance 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param isAutoWhiteBalance 是否启用摄像头自动白平衡。
         */
        void setCameraAutoWhiteBalance(bool isAutoWhiteBalance);
        /**
         * @brief 设置 setCameraExposureTime 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param time 曝光时间或等待时间参数。
         */
        void setCameraExposureTime(int time);
        /**
         * @brief 设置 setCameraWhiteBalanceTemperature 对应参数
         *
         * @details
         * 根据传入参数更新 ImageProcess / Camera 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
         *
         * @param temperature 白平衡色温参数。
         */
        void setCameraWhiteBalanceTemperature(int temperature);

        /**
         * @brief 获取 getCameraBufferSize 对应数据
         *
         * @details
         * 从 ImageProcess / Camera 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
         *
         * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
         */
        int getCameraBufferSize();

        /**
         * @brief 检查 checkCamera 状态
         *
         * @details
         * 检测 ImageProcess / Camera 模块当前资源或设备状态是否可用，并将检查结果返回给调用方。
         *
         * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
         */
        bool checkCamera();

        /**
         * @brief 重新加载 reload 资源
         *
         * @details
         * 在 ImageProcess / Camera 模块出现设备异常或配置变化时释放并重新打开资源，恢复到可继续运行的状态。
         */
        void reload();

        /**
         * @brief 析构 Camera 对象
         *
         * @details
         * 释放 ImageProcess / Camera 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Camera();
    };

    template <typename T>
    /**
     * @brief 打开 open 相关资源
     *
     * @details
     * 打开 ImageProcess / Camera 模块依赖的设备、缓存或功能开关，为后续读写和控制操作建立可用通道。
     *
     * @param device 摄像头设备索引或设备路径，模板参数允许两种形式。
     *
     * @return 返回 inline void 类型结果，表示 open 的处理结果。
     */
    inline void Camera::open(T device)
    {
        if (this->type / 10 < 20)
        {
            if constexpr (std::is_same_v<T, const char *> || std::is_same_v<T, std::string>)
            {
                std::string path = device;
                this->initOpencvCaptureByPath(path);
            }
            else if constexpr (std::is_same_v<T, int>)
            {
                this->initOpencvCaptureByIndex(device);
            }
            else
            {
                throw std::invalid_argument("Camera type error");
            }
        }
        else
        {
            throw std::invalid_argument("Camera type error");
        }
    }

}