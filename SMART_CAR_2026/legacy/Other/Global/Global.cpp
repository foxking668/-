#include "Global.hpp"

using namespace ImageProcess;
using namespace Contral;
using namespace Other;

namespace Other
{
    /*
     * 全局共享状态定义区。
     *
     * 这些变量会被主线程、图像线程、电机线程和控制模块共同访问，
     * 用于共享处理结果、设备对象和运行状态。
     */
    // 初始化全局变量
    ImageProcessAttribute gImageProcessAttribute = {};
    ImageInfoAttribute gImageInfoDate = {};

    st_PID_Attr gLeftPidMotor = {};
    st_PID_Attr gRightPidMotor = {};

    // 速度控制相关
    float gMotorAimSpeed = 0;
    float gMotorLeftTrueSpeed = 0;
    float gMotorRightTrueSpeed = 0;
    float gMotorSpeedParamPvalue = 0;
    float gMotorSpeedParamIvalue = 0;
    float gMotorSpeedParamDvalue = 0;
    float gHandleImageError = 0;
    float gHandleImageErrorRotia = 0;
    float gImageAllowError = 0;

    void *gMotor = nullptr;
    void *gLeftEncoder = nullptr;
    void *gRightEncoder = nullptr;
    void *gCamera = nullptr;
    void *gSteer = nullptr;

    bool gIsClose = false;
    bool gIsStart = false;
    bool gIsReload = false;
    bool gMotorStop = false;
    bool gMotorReverseRequest = false;
    bool gZebraResumeEvent = false;
    bool gBlueBoardLineCropFinishedEvent = false;
    bool gBlueReturnSlowMode = false;
    bool gZebraSlowMode = false;
    bool gImageStop = false;
    bool gImageInfoIsShow = false;

    void *gThreadImageDate = nullptr;

    /**
     * @brief 初始化全局硬件与算法资源
     *
     * @details
     * 统一创建电机、编码器、相机、舵机、PID 等全局对象，并完成它们之间的参数绑定与初始配置。
     */
    void globalInit()
    {
        /*
         * 初始化全局硬件对象和运行参数。
         *
         * 包括：
         * - PID 控制器；
         * - 电机对象；
         * - 左右编码器；
         * - 摄像头；
         * - 舵机。
         */
        /*
         * 第一步：初始化左右电机的 PID 控制器结构体。
         * 后续电机线程会反复使用这两个结构体计算左右轮 PWM 修正量。
         */
        PID_Initialize(&gLeftPidMotor);
        PID_Initialize(&gRightPidMotor);

        /*
         * 第二步：加载电机速度环的默认 PID 参数。
         * 这里先保存到全局变量，电机线程会检测这些值是否变化，
         * 一旦变化就重新写入 PID 控制器，实现运行时调参。
         */
        gMotorSpeedParamPvalue = PID_DEFAULT_KP;
        gMotorSpeedParamIvalue = PID_DEFAULT_KI;
        gMotorSpeedParamDvalue = PID_DEFAULT_KD;

        /*
         * 第三步：初始化左右电机 PID 的增量式参数与输出限幅。
         * 输出限幅用于避免 PID 计算结果过大，导致 PWM 输出突变或超过安全范围。
         */
        PID_Incremental_Set_Kpid(&gLeftPidMotor, 0, 0, 0);
        PID_Incremental_Open_OutLimit(&gLeftPidMotor);
        PID_Incremental_Set_OutLimitValue(&gLeftPidMotor, PID_LIMLIT_PWM);

        PID_Incremental_Set_Kpid(&gRightPidMotor, 0, 0, 0);
        PID_Incremental_Open_OutLimit(&gRightPidMotor);
        PID_Incremental_Set_OutLimitValue(&gRightPidMotor, PID_LIMLIT_PWM);

        /*
         * 第四步：创建底盘运动相关硬件对象。
         * Motor 负责 PWM 与方向 GPIO；Encoder 负责读取左右轮速度反馈。
         */
        gMotor = new Motor(MOTOR_LEFT_PWM_CHIP, MOTOR_LEFT_PWM_INDEX, MOTOR_LEFT_DIR_NUMBER, MOTOR_RIGHT_PWM_CHIP, MOTOR_RIGHT_PWM_INDEX, MOTOR_RIGHT_DIR_NUMBER, MOTOR_ENABLE_PIN);
        gLeftEncoder = new Encoder(ENCODER_LEFT_PIN, ENCODER_LEFT_DIR);
        gRightEncoder = new Encoder(ENCODER_RIGHT_PIN, ENCODER_RIGHT_DIR);

        /*
         * 第五步：创建感知与转向相关硬件对象。
         * Camera 提供赛道图像输入；Steer 通过 PWM 控制转向舵机。
         */
        gCamera = new Camera(CAMERA_OPENCV);
        gSteer = new Steer();

        /*
         * 第六步：把全局 PID 结构体提交给电机对象。
         * Motor 本身不拥有 PID 内存，只保存指针并在控制循环中使用。
         */
        Motor *motor = (Motor *)gMotor;
        motor->submitLeftPidCtrlAttr(&gLeftPidMotor);
        motor->submitRightPidCtrlAttr(&gRightPidMotor);

        /*
         * 第七步：初始化舵机并设置机械偏移补偿。
         * 由于舵机安装可能不是绝对居中，因此通过 offset 修正中位角。
         */
        Steer *steer = (Steer *)gSteer;
        steer->init();
        steer->setSteerOffset(-15);

        /*
         * 第八步：初始化语音播报模块。
         * 语音模块使用 I2C：/dev/i2c-2，地址 0x34。
         * 初始化失败不会阻塞小车运行；后续播报时也会尝试懒加载重试。
         */
        WonderEchoInit();

        /*
         * 第八步：打开摄像头并设置图像处理目标尺寸。
         * 图像尺寸较小，有利于降低每帧处理耗时，提高控制频率。
         */
        Camera *camera = (Camera *)gCamera;
        camera->open(CAMERA_PATH_OR_INDEX);
        camera->image_width = CAMERA_OPENCV_WIDTH;
        camera->image_height = CAMERA_OPENCV_HEIGHT;

        /*
         * 第九步：初始化图像误差相关全局状态。
         * gImageInfoDate.data 初始为空，只有开启图像调试显示时才会分配缓存。
         */
        gHandleImageErrorRotia = 1;

        gImageInfoDate.data = nullptr;
    }

    /**
     * @brief 销毁全局硬件与算法资源
     *
     * @details
     * 在工作线程结束后释放全局对象，防止摄像头、PWM、GPIO、PID 相关资源发生泄漏或悬空访问。
     */
    void globalDestroy()
    {
        /*
         * 关闭语音播报 I2C 设备。
         */
        WonderEchoDestroy();

        /*
         * 按创建顺序释放全局对象。
         *
         * 这里负责清理所有通过 `new` 创建的底层控制对象，
         * 防止程序退出时泄漏硬件资源。
         */
        delete (Motor *)gMotor;
        delete (Encoder *)gLeftEncoder;
        delete (Encoder *)gRightEncoder;
        delete (Camera *)gCamera;
        delete (Steer *)gSteer;
    }
}