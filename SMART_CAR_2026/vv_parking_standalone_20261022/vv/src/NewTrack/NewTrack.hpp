#pragma once

#include <opencv2/core.hpp>
#include <string>
#include "Parking/ParkingRuntime.hpp"

namespace NewTrack
{
    enum class RingStage
    {
        NORMAL = 0,
        APPROACH = 1,
        ENTRY = 2,
        INSIDE = 3,
        EXIT = 4,
        RECOVER = 5
    };

    enum class GarageStage
    {
        IDLE = 0,
        ARMED = 1,
        ADVANCE = 2,
        TURN_RIGHT = 3,
        REVERSE_SEARCH = 4,
        REVERSE = 5,
        STRAIGHTEN = 6,
        DONE = 7
    };

    struct FrameResult
    {
        bool valid = false;
        float steerError = 0.0f;
        bool centerBlackReliable = false;
        bool zebraTriggered = false;
        // Permission belongs to this event, before it may submit parking.
        bool zebraVoiceAllowed = false;
        bool crossingArmed = false;
        bool crossingDetected = false;
        int lapElapsedMs = 0;
        int leftDownY = -1, leftMidY = -1, leftUpY = -1, rightOutY = -1;
        bool ringCompleted = false;
        int markerBlocks = 0;
        bool markerArmed = false;
        int binaryThreshold = -1;
        double binaryWhitePercent = 0;
        bool binaryQuality = false;
        bool lowCameraCandidate = false, lowCameraConfirmed = false;
        bool ringGateOpen = false;
        int virtualDownX = -1, virtualDownY = -1;
        // 【修正6】区分"暂停/无效证据导致观测被冻结"与"本帧真实观测"。
        // CSV 末尾也会追加同名列（A1：不改动原有字段顺序）。
        bool ringObservationPaused = false;
        RingStage ringStage = RingStage::NORMAL;
        GarageStage garageStage = GarageStage::IDLE;
        cv::Mat debugFrame;
        std::string diagnosticCsv;
    };

    // 新赛道总开关。参数来自 ./image_params.txt，约500ms热加载。
    bool enabled();

    // 普通巡线中心黑线优先；环岛拐点独立采用Moore八邻域追踪，左探及环内屏蔽中心黑线。
    // lane_method=3/4 切换普通巡线
    // M3八邻域式双边界基线 / 7.25完整轮廓双边界。
    FrameResult processFrame(const cv::Mat &frameBlock, bool buildDebugFrame);
    // Offline replay: run the same pipeline using this frame's capture timestamp.
    // Timestamps must increase within a run; reset() starts a new replay.
    FrameResult processFrameAt(const cv::Mat &frameBlock, bool buildDebugFrame, long long timestampMs);
    std::string diagnosticCsvHeader();

    // MotorHandle 使用：返回 true 表示新赛道状态机正在覆盖正常目标速度。
    bool speedOverride(double &targetSpeed);
    bool forceMotorStop();

/* 【环岛入口-停错位置后退】MotorHandle 每周期调用；true 时直接用 pwm 写电机（调用方加负号 = 后退） */
bool reversePwm(int &pwm, int &leftEnable, int &rightEnable);

/* ==================== 【人行横道·右电机反向转动】 ==================== */
/*
 * 识别到横道后：右电机 PWM = -zebraReversePwmValue()，持续 zebraReverseMsValue() 毫秒。
 * NewTrack 置一次请求，MotorHandle 每周期取一次：
 *   zebraReverseTakeRequest()   取走后返回 true（保证只执行一次）
 */
bool zebraReverseTakeRequest();
bool zebraReverseRequestPending();
int  zebraReversePwmValue();
int  zebraReverseMsValue();
int  zebraReverseLeftEnabled();

// Second-lap parking requests and steering ownership: Parking/ParkingRuntime.hpp.

    // Independent test entry: same detector/controller, without competition prerequisites.
    void beginStandaloneParkingTest();
    Parking::Request standaloneParkingRequest();

    int lapCount();
    RingStage ringStage();
    GarageStage garageStage();

    // 调试/测试时可手动复位新赛道状态机。
    void reset();
}
