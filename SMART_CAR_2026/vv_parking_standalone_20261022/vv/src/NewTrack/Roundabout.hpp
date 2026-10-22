#pragma once

#include <vector>

namespace NewTrack { namespace Roundabout {

// Coordinates are (x,y), origin at top left. -1 is missing; x=0 is a coordinate.
struct Point {
    int x = -1, y = -1;
    bool valid() const { return x >= 0 && y >= 0; }
};

struct Settings {
    int jumpPx80 = 4;
    int minGapRows60 = 2;
    int separationRows60 = 6;
    int confirmFrames = 2;
    int disappearFrames = 3;
    int rightContinuousPercent = 80;
    int enterYPercent = 33;
    int entryTimeoutMs = 1600;
    int approachTimeoutMs = 6000;
    int exitSearchDelayMs = 800;
    int bothMissingRows60 = 5; // Trigger is strictly greater than this threshold.
    int cooldownMs = 2000;
    int exitTargetXPercent = 0; // 0: fit real right edge below corner; otherwise fixed percentage.
    // 【修正】入环/环内阶段的中线横向比例（%）。
    // 新的入环补线把左边线定为画面最左侧(x=0)、右边线定为"左上拐点->右下角"的连线，
    // 50 = 取这两条线的正中，也就是通常意义上的中线。
    // 若实车仍不肯转进环岛，可下调到 35/30/25 让中线更靠近内侧。
    int ringCenterPratio = 50;
    bool entryValidationRequired = false;
    bool lowCameraEntryEnabled = false;
    int lowCameraOpeningRows60 = 6;
    int lowCameraSupportRows60 = 3;
    int lowCameraLeftClippedRows60 = 4;
    int lowCameraConfirmFrames = 3;
    int lowCameraStablePx80 = 5;
    int lowCameraStableRows60 = 6;
    // 【修复】低相机入口确认窗口(ms)。实车环岛入口常常只闪现 1~2 帧，
    // 原实现要求"连续 N 帧"且把下限硬编码成 3，实际永远凑不满。
    // 现在允许在这个时间窗口内累计候选帧（中间可夹杂非候选帧）。
    int lowCameraWindowMs = 1500;

    // ===== 【入环停车左移】=====
    // 阶段1检测到"左中拐点消失"后：先停车 creepStopMs，
    // 再向左侧低速移动，边移动边找左上拐点；找到就进入入环补线(阶段2)，
    // 或者行程达到 creepMaxMm 毫米也进入阶段2（按轮径换算成圈数累计）。
    // 默认关闭，保持原有"立即切换"行为；实车由 new_track_ring_creep_enable=1 打开。
    bool creepEnable = false;
    int creepStopMs = 300;          // 停车保持时间
    double creepSpeed = 3.0;        // 左移时的目标速度
    double creepMaxMm = 400.0;      // 最多移动距离(mm)
    double wheelDiameterMm = 65.0;  // 轮径，用于把编码器圈数换成毫米
    int creepTimeoutMs = 4000;      // 左移最长持续时间
    int creepSteerXPx80 = 20;       // 左移时强制的中线横向位置(80宽基准)，越小越左

    // ===== 【入环模板匹配】=====
    // 相机看不到环岛左侧拐点时(实车 lap_ms≈11.5s 帧)，用 OpenCV 模板匹配识别
    // "到达环岛区域"和"入环点"。模板命中 -> 直接进入上面的停车左移流程。
    bool templateEntryEnable = true;

    // ===== 【强制左转】=====
    // 实车现象：进环(stage2/3)后很快就被 stage3->4 的出环判据拉出去，
    // "刚进去就拐出来"。这里给一段"进环后强制左打"的时间窗：
    //   forceLeftMs 内 -> 上层把 error 固定成 forceLeftError（负=左），速度用 forceLeftSpeed，
    //   并且这段时间内禁止 stage3->4 出环。
    // 默认 0 = 关闭，行为与原来完全一致。
    int forceLeftMs = 0;
    double forceLeftError = -35.0;  // 强制期间的固定转向误差(度)，负=左
    double forceLeftSpeed = 4.0;    // 强制期间的目标速度
};

struct Evidence {
    Point down, mid, up, rightOut;
    int leftBreaks = 0;
    int bothMissingRows = 0; // Longest continuous two-sided loss run, not scattered-row total.
    bool rightContinuous = false;
    bool usable = false;
    // Separate entrance evidence. A virtual down point is geometry, never a detected corner.
    bool lowCameraEntry = false;
    Point rightEntry, virtualDown, lowCameraMid, lowCameraUp;
    int rightOpeningRows = 0;
};

struct Result {
    int stage = 0; // 1=first sight, 2=enter, 3=inside, 4=exit, 5=straight after exit.
    bool changed = false;
    bool completed = false;
    bool active = false;
    bool lowCameraEntryConfirmed = false;
    // 【入环停车左移】供上层(NewTrack/MotorHandle)决定速度与转向。
    int creepPhase = 0;        // 0=未启动, 1=停车中, 2=左移中
    bool creepStop = false;    // 请求停车
    bool creepMove = false;    // 请求低速左移
    double creepTurns = 0.0;   // 本轮已累计圈数
    double creepMm = 0.0;      // 本轮已累计毫米
    // 【强制左转】进环后的一段固定左打时间窗（由 Settings::forceLeftMs 决定）
    bool forceLeft = false;
    double forceLeftError = 0.0;
    double forceLeftSpeed = 0.0;
    Evidence evidence;
    std::vector<int> left, right, center;
};

bool fitSegment(std::vector<int>& bounds, Point a, Point b, int width);
Evidence detect(const std::vector<int>& left, const std::vector<int>& right,
                int width, const Settings& settings,
                const std::vector<Point>& leftContour = {},
                const std::vector<Point>& rightContour = {});

class Controller {
public:
    Result update(const std::vector<int>& rawLeft, const std::vector<int>& rawRight,
                  int width, long long nowMs, bool allowStart, bool enabled,
                  const Settings& settings, const std::vector<Point>& leftContour = {},
                  const std::vector<Point>& rightContour = {}, bool enterNow = false,
                  bool paused = false, bool templateHit = false);
    // 【入环停车左移】由上层每帧喂入本帧车轮转过的圈数(可为负，内部取绝对值)。
    void feedTurns(double turns) { pendingTurns_ += turns<0?-turns:turns; }
    void reset();
    int stage() const { return stage_; }
    bool completed() const { return completed_; }
private:
    int stage_ = 0, confirm_ = 0, absent_ = 0;
    int width_ = 0, height_ = 0;
    long long stageAt_ = 0, cooldownUntil_ = 0;
    bool completed_ = false, recoveryPairSeen_ = false;
    Point entryAnchor_, exitAnchor_;
    bool lowCameraMode_ = false, haveClock_ = false, wasPaused_ = false;
    int lowConfirm_ = 0;
    long long lastNow_ = 0;
    long long lowEntryAt_ = 0;   // 【修复】最近一次低相机候选的时间戳
    Point lowEntryPosition_, virtualDown_, lowMid_, lowUp_;
    // 【入环停车左移】子状态
    int creepPhase_ = 0;
    long long creepAt_ = 0;
    double creepTurns_ = 0.0;
    double pendingTurns_ = 0.0;
    // 【入环模板匹配】连续命中计数
    int templateConfirm_ = 0;
    // 【强制左转】进入 stage2 的时刻；0 = 还没进过环
    long long forceLeftAt_ = 0;
    Result lastResult_;
};

} }
