#pragma once
#include "headfile.hpp"

namespace Other
{
    /*
     * 图像基础属性。
     * 用于描述一帧原始图像的裸数据地址、尺寸和通道数。
     */
    struct ImageAttribute
    {
        unsigned char *data;
        int width;
        int height;
        int channel;
    };

    struct ImageInfoAttribute
    {
        unsigned char *data;
        int width;
        int height;
        float errorValue;
        float fps;
        bool success;
    };

    /*
     * 图像处理过程中的共享数据结构。
     *
     * 该结构在各图像算法模块之间传递：
     * - 原始图像；
     * - 各阶段中间图；
     * - 起止点；
     * - 左/中/右线结果。
     */
    struct ImageProcessAttribute
    {
        ImageAttribute origin_image;

        Mat *bin_frame;
        Mat *canny_frame;
        Mat *grayscale_frame;

        Point left_start_pos;
        Point right_start_pos;
        Point left_end_pos;
        Point right_end_pos;

        vector<Point> *left_line_contours;
        vector<Point> *right_line_contours;

        int *left_line;
        int *center_line;
        int *right_line;
    };

    constexpr uchar gBinWhilePointValue = 255;
    constexpr uchar gBinBlackPointValue = 0;

    extern ImageProcessAttribute gImageProcessAttribute;
    extern ImageInfoAttribute gImageInfoDate;

    extern st_PID_Attr gLeftPidMotor;
    extern st_PID_Attr gRightPidMotor;

    extern float gMotorAimSpeed;
    extern float gMotorLeftTrueSpeed;
    extern float gMotorRightTrueSpeed;
    extern float gMotorSpeedParamPvalue;
    extern float gMotorSpeedParamIvalue;
    extern float gMotorSpeedParamDvalue;
    extern float gHandleImageError;
    extern float gHandleImageErrorRotia;
    extern float gImageAllowError;

    extern void *gMotor;
    extern void *gLeftEncoder;
    extern void *gRightEncoder;
    extern void *gCamera;
    extern void *gSteer;

    /*
     * main 通过这些全局标志协调程序生命周期：
     * - gIsClose：关闭总开关，Ctrl+C 后会被置为 true；
     * - gIsStart：启动开关，main 会等待它变为 true；
     * - gIsReload：退出后是否重启 systemd 服务；
     * - gMotorStop：电机暂停标志，图像识别到特殊元素时会使用；
     * - gImageStop：图像线程暂停标志；
     * - gImageInfoIsShow：是否维护图像调试信息。
     */
    extern bool gIsClose;
    extern bool gIsStart;
    extern bool gIsReload;
    extern bool gMotorStop;
    extern bool gMotorReverseRequest;
    /* 由电机线程在“人行横道实际重新起步”那一刻置位，图像线程据此开始动态巡线裁剪计时。 */
    extern bool gZebraResumeEvent;
    /* ImageHandle 在动态上裁剪窗口结束、已恢复正常裁剪时置位；MotorHandle 据此开始后置低速。 */
    extern bool gBlueBoardLineCropFinishedEvent;
    extern bool gBlueReturnSlowMode;
    extern bool gZebraSlowMode;
    extern bool gImageStop;
    extern bool gMotoInfoIsShow;
    extern bool gImageInfoIsShow;

    /*
     * main 调用的全局初始化函数。
     *
     * 负责创建 PID、电机、编码器、摄像头和舵机等全局资源。
     */
    /**
     * @brief 初始化全局硬件与算法资源
     *
     * @details
     * 统一创建电机、编码器、相机、舵机、PID 等全局对象，并完成它们之间的参数绑定与初始配置。
     */
    void globalInit();

    /*
     * main 调用的全局销毁函数。
     *
     * 在线程退出后释放 `globalInit()` 创建的硬件对象。
     */
    /**
     * @brief 销毁全局硬件与算法资源
     *
     * @details
     * 在工作线程结束后释放全局对象，防止摄像头、PWM、GPIO、PID 相关资源发生泄漏或悬空访问。
     */
    void globalDestroy();

}