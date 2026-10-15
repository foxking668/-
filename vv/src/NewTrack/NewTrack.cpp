#include "NewTrack.hpp"
#include "Roundabout.hpp"
#include "TrackGates.hpp"
#include "TrackPolicy.hpp"
#include "EightNeighbor.hpp"
#include "EntranceProbe.hpp"
#include "ConeBoundary.hpp"
#include "ConeSequence.hpp"
#include "GrayCone.hpp"
#include "DiagnosticView.hpp"
#include "Binarization.hpp"

#include "headfile.hpp"
#include "Experiment/BaselineAdapter.hpp"

#include <atomic>
#include <fstream>
#include <map>
#include <numeric>

using namespace ImageProcess;
using namespace Other;

namespace NewTrack
{
namespace
{
    using Clock = std::chrono::steady_clock;

    // One capture clock per frame; the motor thread still reads the real clock.
    thread_local bool frameClockActive = false;
    thread_local Clock::time_point frameClock;
    Clock::time_point frameNow() { return frameClockActive ? frameClock : Clock::now(); }
    struct FrameClockGuard {
        bool previousActive; Clock::time_point previous;
        explicit FrameClockGuard(Clock::time_point now)
            : previousActive(frameClockActive), previous(frameClock) {
            frameClockActive=true; frameClock=now;
        }
        ~FrameClockGuard() {frameClockActive=previousActive;frameClock=previous;}
    };

    struct Config
    {
        int enable = 1;
        int laneMethod = 4;                 // 3=M3, 4=最新版7.25自动排期核心
        // 新赛道宽视野：只轻裁顶部/底部，左右不裁，避免S弯/环岛把真实边线裁掉。
        double topCropRatio = 0.05;
        double bottomCropRatio = 0.02;
        double leftCropRatio = 0.00;
        double rightCropRatio = 0.00;
        int boundaryCloseKernel = 11;       // 横向闭运算，填掉中心黑线，避免把白色赛道切成两半
        Binarization::Settings binarySettings;

        // 发车出库：仅程序第一次起步短时右偏，之后所有圈不重复。
        int startupRightEnable = 1;
        int startupRightDurationMs = 400;
        double startupRightError = 5.0;    // 正误差=右转
        double startupRightSpeed = 7.0;

        // 蓝锥避障沿用最新版7.25自动排期里的蓝色目标检测器；只在人行横道前普通道路启用。
        int blueConeEnable = 0;
        double blueConeSpeed = 7.0;
        ConeBoundary::Settings coneSettings;
        ConeSequence::Settings coneSequenceSettings;
        bool coneTuningValid=true;

        int centerBlackEnable = 1;
        int centerBlackMinRun = 2;
        int centerBlackMaxRun = 18;
        int centerBlackMaxStep = 24;
        int centerBlackMinValidRows = 8;
        int centerBlackMinNearRows = 2;
        int centerBlackMinWhiteMargin = 2;
        // 【黑线丢边兜底】白色路面两侧都丢失时，仍在画面中间 60% 区域检测中心黑线。
        // 参考 cc 版做法。原来是 l<0||r<0 直接 continue，会连"临时丢边时本来还能
        // 看到的中心线"一起丢掉，目标退化成垃圾。1=启用（参考版行为），0=原行为。
        int centerBlackLooseBounds = 1;

        // 单边界弯道兜底：一侧边界丢失时，不再把缺失侧粗暴补到图像边缘，
        // 而是使用已学习的道路宽度估算虚拟边界。右边界单独存在时自然向左修正，反之向右。
        int singleEdgeEnable = 1;
        int singleEdgeBiasPx160 = 3;
        int partialBlackAssistEnable = 1;
        int partialBlackMinValidRows = 3;
        int partialBlackMinNearRows = 1;
        int partialBlackBlendPercent = 70;

        // 【转向目标限速】中心黑线不可靠时，限制 targetX 每帧最多变化 N 像素，
        // 把"一帧跳 40 像素 -> error 跳 ±300 度"的抖动掐掉。参考 cc 版默认 12。
        // 中心黑线可靠时直接跟线（不抹平真弯道）；环岛/蓝锥阶段不限速。0=关闭。
        int steerTargetMaxStepPx = 12;
        // 【中线空间平滑】对修好的中线做 window 行中值平滑，再与原值取平均
        // （保留真弯道曲率）。参考 cc 版默认 5。1=关闭。
        int centerSpatialWindow = 5;
        // 【丢线停车】中心黑线和白路边界都丢 -> 连续 N 帧停车，恢复 M 帧才解除。
        // 参考 cc 版 8/4。0=关闭。仅在普通巡线段生效，环岛/蓝锥阶段不生效。
        int lostLaneStopFrames = 8;
        int lostLaneRecoverFrames = 4;

        int workMode = 1;                  // 0=普通巡线，1=八邻域五阶段环岛模式
        int ringEnable = 1;
        int ringStartupBlockMs = 2900;        // 本圈起点后2.9秒开放黑块门控，再等待三拐点
        double ringSpeed = 4.0;
        int ringRequireMarker = 1;
        int ringMarkerConfirmFrames = 2;
        int ringMarkerHoldMs = 7000;
        double ringApproachSpeed = 4.0;
        TrackGates::MarkerSettings markerSettings;
        int ringProbeEnable=1, ringMarkerWindowMs=14000, ringTaskTimeoutMs=14000;
        int ringEntryAssistMs=800;
        TrackPolicy::ProbeSettings probeSettings;
        int ringZebraBlockAfterExitMs = 500;
        Roundabout::Settings ringSettings;
        // 【入环模板匹配】当几何检测看不到环岛左侧拐点时(实车进入环岛区域的那几帧
        // leftBreaks=0, mid/up 全空)，用离线从录像截取的灰度模板做 OpenCV 模板匹配，
        // 连续命中若干帧即判定到达入环点，交由 Roundabout 的停车左移流程处理。
        int ringTemplateEnable = 1;
        int ringTemplateFrames = 3;          // 需要命中的帧数（窗口内累计，不要求连续）
        double ringTemplateThreshold = 0.82; // TM_CCOEFF_NORMED 阈值
        // 【窗口累计】命中帧不必连续：只要在 ringTemplateWindowMs 时间窗内累计到
        // ringTemplateFrames 次就触发。实测（三段 speed6 录像离线打分）tmpl 峰值只到
        // 0.6~0.72，而且帧间分数跳变大，"严格连续 N 帧"几乎永远凑不满 —— 这是
        // "8 次只成功一次进环"的直接原因。
        int ringTemplateWindowMs = 1500;
        std::string ringTemplateFile = "ring_tmpl_entry.png";
        // 【模板时间窗】模板匹配只在本圈 [ringStartupBlockMs, ringTemplateMaxMs] 之间生效。
        // 下界复用 startup_block_ms(开头屏蔽)，上界单独给一个，避免被几何那条
        // marker_window_ms 的时间窗连带限制住。
        int ringTemplateMaxMs = 25000;
        // 【阈值向下浮动】命中判据用 (threshold - float)，即基准 0.78 时可下浮到 0.75。
        double ringTemplateFloat = 0.03;

        // ============ 【环岛右侧多模板触发】 ============
        // 用 f033~f039（车沿环岛右侧行进时的画面）裁出的 7 张 80x56 模板做匹配。
        // 判据：7 张里任意一张命中即算"本帧在环岛右侧"，窗口内累计 ringSideFrames
        //       帧（默认 2）即触发。
        // 触发后固定序列：完全断电停车 ringSideStopMs（默认3s）
        //              -> 强制左转 ringSideLeftError（-60度）持续 ringSideLeftMs（默认2s）
        //              -> 全程速度压到 ringSideSpeed（默认2）
        // 匹配在【原生 80x56 灰度图】上做（不是 452x316 诊断面板），模板文件直接可用。
        int ringSideEnable = 0;
        std::string ringSideTmplPrefix = "ring_side_";
        int ringSideTmplCount = 7;
        // 模板中央裁剪尺寸。80x56=整帧原样匹配（零平移容忍）；60x44=允许横向±10/纵向±6像素平移。
        int ringSideCropW = 60;
        int ringSideCropH = 44;
        double ringSideThreshold = 0.60;
        double ringSideFloat = 0.0;
        int ringSideFrames = 2;              // 窗口内累计命中帧数
        int ringSideWindowMs = 3000;         // 累计窗口
        int ringSideStartMs = 0;             // 本圈下界（0=不设下界）
        int ringSideMaxMs = 30000;           // 本圈上界
        int ringSideStopMs = 3000;           // 完全断电停车时长
        int ringSideLeftMs = 2000;           // 强制左转时长
        double ringSideLeftError = -60.0;    // 强制左转误差（负=左）
        double ringSideSpeed = 2.0;          // 认出环岛后的速度
        int ringSideCooldownMs = 8000;       // 序列结束后多久才能再次触发
        int ringSideLogMs = 900;             // 观测日志间隔

        // ============ 【环岛入口定位·虚线段判据】 ============
        // 黑色虚线段是环岛右侧独有的特征（黑弧出现不固定，不能当判据）。
        // 流程：检出虚线段(阶段A) -> 按 mode 决定停车时刻(阶段B) -> 立即完全断电停车。
        // mode=2：检出虚线后延时 ringEntryDelayMs 停车（对应 f039 附近，delay≈870ms）
        // mode=3：检出虚线后等最大暗块变成「竖直窄条」(高>=narrowH 宽<=narrowW) 停车
        //         （对应 f041 附近，16x56）
        // ringEntryStopMs=0 表示停下来就不走了（给你走过去判断位置）。
        int ringEntryCueEnable = 0;          // 1=启用
        int ringEntryCueMode = 2;            // 2=延时定位  3=窄条定位
        int ringEntryDashMinBlocks = 2;      // 至少几个共线小暗块才算"虚线"
        int ringEntryDashTolPx = 6;          // 共线容差(像素)
        int ringEntryDashNeedFrames = 2;     // 窗口内需要检出几次
        int ringEntryDashWindowMs = 3000;    // 虚线累计窗口
        // 【排除斑马线】虚线段必须【纵向排开】(跨很多行)且接近竖直；
        // 斑马线条纹是横向排开(几乎同一行)，用 ySpan 和斜率就能排除。
        int ringEntryDashMinYSpan = 20;      // 内点块的最小 y 跨度(行)（旧拟合判据用，已不生效）
        double ringEntryDashMaxSlope = 1.5;  // 拟合直线 |dx/dy| 上限（旧判据用，已不生效）
        // 【特征①·x 值不变】虚线段在同一条竖线上，质心 x 跨度上限。
        int ringEntryDashMaxXSpread = 20;
        // 【特征③·不连续】相邻两段之间的最小空隙(行)，小于它就算"太密=实线"。
        int ringEntryDashMinGapRows = 1;
        int ringEntryDelayMs = 0;          // mode=2 的延时（②≈870 ③≈1460）
        int ringEntryNarrowW = 20;           // mode=3 宽度上限
        int ringEntryNarrowH = 45;           // mode=3 高度下限
        int ringEntryStartMs = 6000;        // 本圈下界（前10s屏蔽）
        int ringEntryMaxMs = 90000;          // 本圈上界
        int ringEntryStopMs = 0;             // 0=永久停车；>0=停这么久后恢复
        int ringEntryLogMs = 500;            // 观测日志间隔

        int zebraEnable = 1;
        int zebraStartupBlockMs = 12000;    // 每圈前12秒屏蔽人行横道
        int zebraStopMs = 3200;             // 识别后原地停车，禁用旧版斑马线倒退
        // 【重复停车】1=每次见到人行横道都播报+停车（默认）；
        // 0=旧行为，本圈只停一次（依赖车库倒Y计圈来复位，而倒Y实测识别不到，
        //   所以旧行为实际上整场只停一次，第二圈见到不再播报）。
        // 解锁条件：这处斑马线连续"离开视野" zebraReleaseFrames 帧，或冷却到期。
        int zebraStopEveryTime = 1;
        // 人行横道改用新车 SMART_CAR2026 思路，但针对旧车80x60重新分层：
        // 远区预警；近区宽条纹确认，强近区可在远区漏检时独立确认。
        int zebraCropLeftPercent = 8;
        int zebraCropRightPercent = 8;
        int zebraFarYStartPercent = 20;
        int zebraFarYEndPercent = 55;
        int zebraNearYStartPercent = 38;
        int zebraNearYEndPercent = 92;
        // 远区：距离远时横向黑条被压扁，不再要求黑白跳变/条纹数，只看黑色面积连续多帧。
        int zebraFarBlackAreaMinPercent = 4;
        int zebraFarBlackAreaMaxPercent = 35;
        int zebraFarConfirmFrames = 3;
        double zebraApproachSpeed = 6.0;    // 远区确认后先降速，等待近区
        // 近区：真正停车确认，要求明显的横向黑条结构。
        int zebraNearMinTransitions = 3;
        int zebraNearMinStripes = 2;
        int zebraNearMinRows = 2;
        int zebraNearBlackAreaMinPercent = 5;
        int zebraNearBlackAreaMaxPercent = 65;
        int zebraFarNearHoldMs = 6000;
        int zebraNearAfterFarMinDelayMs = 150;
        int zebraNearFallbackConfirmFrames=4;
        int zebraRequireFarNear = 1;
        int zebraConfirmFrames = 2;
        int zebraReleaseFrames = 24;
        int zebraCooldownMs = 5000;

        // 车库右侧交叉口：不再依赖“必须先识别人行横道”，避免漏检人行横道后整圈无法计圈。
        // 改为本圈独立时间门控；赛道顺序固定，目前18秒以后才开放交叉口判断。
        int crossingEnable = 1;
        int crossingRequireZebra = 0;
        int crossingStartupBlockMs = 18000;
        // 发车出库后会很快经过起点倒Y交叉口：先尝试视觉确认；若漏检，约3秒自动按“已通过起点交叉口”同步本圈计时。
        int startupCrossingSyncEnable = 1;
        int startupCrossingFallbackMs = 2800;
        int startupCrossingMinDetectMs = 700;
        int startupCrossingConfirmFrames = 3;
        int crossingAfterZebraDelayMs = 1200; // From the actual scheduled stop END, always enforced.
        int crossingNearYPercent = 70;
        int crossingFarConfirmFrames=2, crossingFarHoldMs=8000;
        int crossingNearMinSplitRows = 2;
        int crossingOpenConfirmFrames = 3;
        int crossingCooldownMs = 5000;
        int crossingForkMinSplitRows = 2;
        int crossingForkMinStemRows = 3;
        int crossingForkMinSeparationPx160 = 12;
        int crossingRequireRingComplete = 0;

        // 第二圈倒Y交叉口确认后，若暂不开自动倒车入库，则在近区确认后短时继续前进再停车。
        // 程序保持运行，方便先关闭车上的舵机电源开关，再按 Ctrl+C。
        int finishStopEnable = 1;
        int finishStopDelayMs = 200;

        int garageAutoEnable = 0;            // 第二圈交叉口确认后可自动进入倒车入库；首测建议仍保持0
        double garageSpeed = 5.0;
        double garageTurnSpeed = 4.0;
        double garageReverseSpeed = 4.0;
        int garageArmDelayMs = 1200;
        int garageOpenConfirmFrames = 4;
        int garageForwardMs = 650;
        int garageRightTurnMs = 650;
        double garageRightTurnError = 28.0;
        int garageReverseSearchMs = 1200;
        int garageReverseMs = 1000;
        int garageStraightenMs = 500;
        double garageStraightenError = -18.0;
        int garageBlackConfirmFrames = 3;
        double garageReverseBlackGain = 1.0;

        int debugPrint = 1;
        Config() { ringSettings.lowCameraEntryEnabled=true; }
    };

    std::mutex configMutex;
    Config config;
    Clock::time_point lastConfigLoad = Clock::time_point{};
    bool configPrinted = false;

    std::atomic<double> speedOverrideValue{0.0};
    std::atomic<bool> speedOverrideActive{false};
    std::atomic<bool> motorStopForced{false};
    std::atomic<bool> coneStopForced{false};
    // 【丢线停车】中心黑线和白路边界都丢时的独立停车标志，和上面两个是或的关系。
    std::atomic<bool> lostLaneStopForced{false};
    // 【环岛入口/右侧】跨线程停车标志：图像线程写、电机线程读，必须是 atomic。
    // 原来放在普通 struct 里的 bool，实测停车不生效。
    std::atomic<bool> ringEntryStopForced{false};
    std::atomic<bool> ringSideStopForced{false};
    std::atomic<long long> zebraStopUntilNs{0};
    std::atomic<int> lapCounter{0};
    std::atomic<int> ringStageAtomic{0};
    std::atomic<int> garageStageAtomic{0};

    Clock::time_point processStart = frameNow();
    // 第一圈从第一帧有效图像开始计时，避免程序初始化耗时吃掉出库右偏/4秒/12秒窗口。
    bool processStarted = false;
    Clock::time_point lapEpoch = processStart;
    bool startupCrossingSynced = false;
    int startupCrossingConfirm = 0;
    bool finishStopPending = false;
    bool finishStopped = false;
    Clock::time_point finishStopAt = Clock::time_point{};

    struct RingState
    {
        RingStage stage = RingStage::NORMAL;
        Roundabout::Controller controller;
        TrackPolicy::EntranceProbe probe;
        TrackPolicy::ProbeResult probeResult;
        bool attemptedThisLap=false;
        long long taskAtMs=0, entryAssistUntilMs=0;
        long long lastFrameMs=0,routePausedMs=0;
        bool gateOpen=false, pausedPreviously=false, physicalPausePreviously=false;
        bool observationPaused=false;   // 【修正6】本帧环岛观测是否因暂停/证据无效被冻结
        TrackPolicy::MarkerLatch marker;
        TrackPolicy::MarkerLatchResult markerResult;
        TrackGates::MarkerEvidence markerEvidence;
        Roundabout::Result result;
        Clock::time_point zebraBlockUntil = Clock::time_point{};
        // 【入环模板匹配】相机看不到环岛左侧拐点时的备用入口识别
        double templateScore=0.0;
        int templateConfirm=0;
        long long templateHitMs=0;
        long long templateWindowStartMs=0;   // 【窗口累计】本次命中窗口的起点(本圈计时)
    } ring;

    struct ZebraState
    {
        TrackPolicy::ZebraConfirmation sequence;
        bool latched = false;
        bool seenThisLap = false;
        Clock::time_point cooldownUntil = Clock::time_point{};
        Clock::time_point lastTrigger = Clock::time_point{};
        long long stopEndMs = 0;
        // 【重复停车】上一次触发后，这处斑马线连续"离开视野"的帧数。
        // 达到 zebraReleaseFrames 才解锁，允许下一次触发。
        int absentFrames = 0;
    } zebra;

    struct CrossingState
    {
        TrackPolicy::CrossingCounter confirmation;
        TrackPolicy::ForkConfirmation farNear;
        int openConfirm = 0;
        bool armed = false;
        bool justDetected = false;
        Clock::time_point cooldownUntil = Clock::time_point{};
    } crossing;

    struct GarageStateData
    {
        GarageStage stage = GarageStage::IDLE;
        int openConfirm = 0;
        int blackConfirm = 0;
        Clock::time_point stageAt = frameNow();
        Clock::time_point armedAt = Clock::time_point{};
    } garage;

    // 使用最新版7.25自动排期源码中的蓝色障碍检测器，硬件/HSV接口不另造一套。
    ConeBoundary::Detector coneDetector;
    ConeSequence::Controller coneSequence;
    long long coneLaunchAtMs=-1;
    std::atomic<bool> coneInputStop{false},coneSequenceActive{false},coneGuideStop{false};
    std::atomic<long long> coneLastFrameNs{-1};
    std::atomic<long long> conePulseUntilNs{0};

    // 新赛道人行横道使用新车 SMART_CAR2026 的黑条纹远近双区逻辑；
    // 不再调用7.25旧地图中面向白色斑马条纹的 FixLine/Zebra 检测器。

    constexpr const char *ANSI_RED = "\033[1;31m";
    constexpr const char *ANSI_BLUE = "\033[1;34m";
    constexpr const char *ANSI_RESET = "\033[0m";

    static inline int clampi(int v, int lo, int hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    static inline double clampd(double v, double lo, double hi)
    {
        return std::max(lo, std::min(v, hi));
    }

    static std::string trim(std::string s)
    {
        const char *ws = " \t\r\n";
        const auto a = s.find_first_not_of(ws);
        if (a == std::string::npos) return {};
        const auto b = s.find_last_not_of(ws);
        return s.substr(a, b - a + 1);
    }

    static void loadConfigIfNeeded()
    {
        // 【修正1】配置热加载必须用独立的真实时钟，不能用帧时钟 frameNow()。
        //
        // 原实现的问题：用 frameNow()，且 reset() 没清 lastConfigLoad。
        //   进程先用真实时钟跑一段 -> lastConfigLoad 是个很大的值；
        //   reset() 后从较小的时间戳开始离线回放 -> now - lastConfigLoad 为负；
        //   负数 < 500ms 成立 -> 判定"刚加载过" -> 整段回放都不再更新配置。
        //
        // 现在：热加载只看真实时钟；时间差为负也强制重新加载。
        // 比赛状态与阶段计时仍然用帧时钟（frameNow()），两者互不干扰。
        const auto now = Clock::now();
        if (lastConfigLoad != Clock::time_point{})
        {
            const auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
                                  now - lastConfigLoad).count();
            if (diff >= 0 && diff < 500) return;   // 正常 500ms 节流
            // diff < 0（时钟源被切换/重置）-> 不 return，落到下面强制加载
        }
        lastConfigLoad = now;

        Config next;
        std::ifstream in("./image_params.txt");
        std::string line;
        while (std::getline(in, line))
        {
            const auto hash = line.find('#');
            if (hash != std::string::npos) line = line.substr(0, hash);
            const auto eq = line.find('=');
            if (eq == std::string::npos) continue;
            const std::string k = trim(line.substr(0, eq));
            const std::string v = trim(line.substr(eq + 1));
            if (k.rfind("new_track_", 0) != 0) continue;
            try
            {
                auto I=[&](int &dst,int lo,int hi){dst=clampi(std::stoi(v),lo,hi);};
                auto D=[&](double &dst,double lo,double hi){dst=clampd(std::stod(v),lo,hi);};
                if(k=="new_track_enable") I(next.enable,0,1);
                else if(k=="new_track_lane_method") I(next.laneMethod,3,4);
                else if(k=="new_track_top_crop_ratio") D(next.topCropRatio,0.0,0.60);
                else if(k=="new_track_bottom_crop_ratio") D(next.bottomCropRatio,0.0,0.30);
                else if(k=="new_track_left_crop_ratio") D(next.leftCropRatio,0.0,0.40);
                else if(k=="new_track_right_crop_ratio") D(next.rightCropRatio,0.0,0.40);
                else if(k=="new_track_side_crop_ratio") { double legacy=0.0; D(legacy,0.0,0.40); next.leftCropRatio=legacy; next.rightCropRatio=legacy; }
                else if(k=="new_track_boundary_close_kernel") I(next.boundaryCloseKernel,1,31);
                else if(k=="new_track_binary_mode") I(next.binarySettings.mode,0,1);
                else if(k=="new_track_binary_roi_top_percent") I(next.binarySettings.roiTopPercent,0,90);
                else if(k=="new_track_binary_roi_bottom_percent") I(next.binarySettings.roiBottomPercent,10,100);
                else if(k=="new_track_binary_threshold_offset") I(next.binarySettings.thresholdOffset,-40,40);
                else if(k=="new_track_binary_denoise") {int b=next.binarySettings.denoise;I(b,0,1);next.binarySettings.denoise=b!=0;}
                else if(k=="new_track_start_right_enable") I(next.startupRightEnable,0,1);
                else if(k=="new_track_start_right_duration_ms") I(next.startupRightDurationMs,0,5000);
                else if(k=="new_track_start_right_error") D(next.startupRightError,0.0,45.0);
                else if(k=="new_track_start_right_speed") D(next.startupRightSpeed,0.5,20.0);
                else if(k=="new_track_blue_cone_enable") I(next.blueConeEnable,0,1);
                else if(k=="new_track_blue_cone_speed") D(next.blueConeSpeed,0.5,20.0);
                else if(k=="new_track_cone_sequence_enable") {
                    int value=0;I(value,0,1);next.coneSequenceSettings.enabled=value!=0;
                }
                else if(k=="new_track_blue_cone_max_confirm_gap_ms") I(next.coneSettings.maxConfirmGapMs,40,300);
                else if(k=="new_track_blue_cone_near_max_area_percent") I(next.coneSettings.nearMaxAreaPercent,8,25);
                else if(k=="new_track_blue_cone_near_max_width_percent") I(next.coneSettings.nearMaxWidthPercent,24,45);
                else if(k=="new_track_blue_cone_near_max_height_percent") I(next.coneSettings.nearMaxHeightPercent,55,90);
                else if(k=="new_track_blue_cone_h_min") I(next.coneSettings.hMin,0,179);
                else if(k=="new_track_blue_cone_h_max") I(next.coneSettings.hMax,0,179);
                else if(k=="new_track_blue_cone_s_min") I(next.coneSettings.sMin,0,255);
                else if(k=="new_track_blue_cone_v_min") I(next.coneSettings.vMin,0,255);
                else if(k=="new_track_blue_cone_min_area80x60") I(next.coneSettings.minArea80x60,3,100);
                else if(k=="new_track_blue_cone_max_area_percent") I(next.coneSettings.maxAreaPercent,1,15);
                else if(k=="new_track_blue_cone_margin_px80") I(next.coneSettings.marginPx80,1,15);
                else if(k=="new_track_blue_cone_min_corridor_px80") I(next.coneSettings.minCorridorPx80,6,35);
                else if(k=="new_track_blue_cone_confirm_frames") I(next.coneSettings.confirmFrames,1,10);
                else if(k=="new_track_blue_cone_hold_ms") I(next.coneSettings.holdMs,0,500);
                else if(k=="new_track_blue_cone_near_y_percent") I(next.coneSettings.nearYPercent,10,80);
                else if(k=="new_track_center_black_enable") I(next.centerBlackEnable,0,1);
                else if(k=="new_track_center_black_min_run") I(next.centerBlackMinRun,1,20);
                else if(k=="new_track_center_black_max_run") I(next.centerBlackMaxRun,2,80);
                else if(k=="new_track_center_black_max_step") I(next.centerBlackMaxStep,2,80);
                else if(k=="new_track_center_black_min_valid_rows") I(next.centerBlackMinValidRows,2,40);
                else if(k=="new_track_center_black_min_near_rows") I(next.centerBlackMinNearRows,1,20);
                else if(k=="new_track_center_black_min_white_margin") I(next.centerBlackMinWhiteMargin,1,10);
                else if(k=="new_track_center_black_loose_bounds") I(next.centerBlackLooseBounds,0,1);
                else if(k=="new_track_steer_target_max_step_px") I(next.steerTargetMaxStepPx,0,80);
                else if(k=="new_track_center_spatial_window") I(next.centerSpatialWindow,1,11);
                else if(k=="new_track_lost_lane_stop_frames") I(next.lostLaneStopFrames,0,60);
                else if(k=="new_track_lost_lane_recover_frames") I(next.lostLaneRecoverFrames,1,60);
                else if(k=="new_track_binary_blur_kernel") I(next.binarySettings.blurKernel,1,9);
                else if(k=="new_track_gray_threshold") I(next.binarySettings.fixedThreshold,0,254);
                else if(k=="new_track_single_edge_enable") I(next.singleEdgeEnable,0,1);
                else if(k=="new_track_single_edge_bias_px") I(next.singleEdgeBiasPx160,0,30);
                else if(k=="new_track_partial_black_assist_enable") I(next.partialBlackAssistEnable,0,1);
                else if(k=="new_track_partial_black_min_valid_rows") I(next.partialBlackMinValidRows,1,20);
                else if(k=="new_track_partial_black_min_near_rows") I(next.partialBlackMinNearRows,0,10);
                else if(k=="new_track_partial_black_blend_percent") I(next.partialBlackBlendPercent,0,100);
                else if(k=="new_track_work_mode") I(next.workMode,0,1);
                else if(k=="new_track_ring_enable") I(next.ringEnable,0,1);
                else if(k=="new_track_ring_startup_block_ms") I(next.ringStartupBlockMs,0,60000);
                else if(k=="new_track_ring_speed") D(next.ringSpeed,0.5,30.0);
                else if(k=="new_track_ring_require_marker") I(next.ringRequireMarker,0,1);
                else if(k=="new_track_ring_approach_speed") D(next.ringApproachSpeed,0.5,30.0);
                else if(k=="new_track_ring_probe_enable") I(next.ringProbeEnable,0,1);
                else if(k=="new_track_ring_marker_window_ms") I(next.ringMarkerWindowMs,4000,60000);
                else if(k=="new_track_ring_task_timeout_ms") I(next.ringTaskTimeoutMs,4000,60000);
                else if(k=="new_track_ring_entry_assist_ms") I(next.ringEntryAssistMs,0,3000);
                else if(k=="new_track_ring_probe_wait_ms") I(next.probeSettings.waitMs,0,10000);
                else if(k=="new_track_ring_probe_require_opening") {int v=next.probeSettings.requireOpening?1:0;I(v,0,1);next.probeSettings.requireOpening=v!=0;}
                else if(k=="new_track_ring_probe_require_width_growth") {int v=next.probeSettings.requireWidthGrowth?1:0;I(v,0,1);next.probeSettings.requireWidthGrowth=v!=0;}
                else if(k=="new_track_ring_probe_width_growth_px80") I(next.probeSettings.widthGrowthPx80,1,15);
                else if(k=="new_track_ring_probe_settle_ms") I(next.probeSettings.settleMs,50,1000);
                else if(k=="new_track_ring_probe_timeout_ms") I(next.probeSettings.timeoutMs,300,3000);
                else if(k=="new_track_ring_probe_shift_px80") I(next.probeSettings.shiftPx80,1,15);
                else if(k=="new_track_ring_probe_confirm_frames") I(next.probeSettings.confirmFrames,2,10);
                else if(k=="new_track_ring_probe_min_rows60") I(next.probeSettings.minRows60,3,25);
                else if(k=="new_track_ring_entry_confirm_frames") I(next.probeSettings.entryConfirmFrames,2,12);
                else if(k=="new_track_ring_entry_stable_px80") I(next.probeSettings.entryStablePx80,1,15);
                else if(k=="new_track_ring_entry_stable_rows60") I(next.probeSettings.entryStableRows60,1,12);
                else if(k=="new_track_ring_probe_left_error") D(next.probeSettings.leftError,3,40);
                else if(k=="new_track_ring_probe_right_error") D(next.probeSettings.rightError,1,30);
                else if(k=="new_track_ring_probe_recover_ms") I(next.probeSettings.recoverMs,100,1000);
                else if(k=="new_track_ring_probe_cooldown_ms") I(next.probeSettings.cooldownMs,1000,10000);
                else if(k=="new_track_ring_marker_confirm_frames") I(next.ringMarkerConfirmFrames,1,12);
                else if(k=="new_track_ring_marker_hold_ms") I(next.ringMarkerHoldMs,500,15000);
                else if(k=="new_track_ring_marker_min_blocks") I(next.markerSettings.minBlocks,2,10);
                else if(k=="new_track_ring_marker_center_tolerance_px") I(next.markerSettings.centerTolerancePx160,4,60);
                else if(k=="new_track_ring_marker_min_span_rows") I(next.markerSettings.minSpanRows82,4,80);
                else if(k=="new_track_ring_marker_vertical_ratio_percent") I(next.markerSettings.verticalRatioPercent,0,500);
                else if(k=="new_track_ring_marker_min_angle_deg") D(next.markerSettings.minAngleDeg,0.0,89.0);
                else if(k=="new_track_ring_marker_max_width_percent") I(next.markerSettings.maxWidthPercent,10,80);
                else if(k=="new_track_ring_jump_px80") I(next.ringSettings.jumpPx80,2,30);
                else if(k=="new_track_ring_low_camera_enable") {int b=next.ringSettings.lowCameraEntryEnabled;I(b,0,1);next.ringSettings.lowCameraEntryEnabled=b!=0;}
                else if(k=="new_track_ring_low_camera_opening_rows60") I(next.ringSettings.lowCameraOpeningRows60,3,20);
                else if(k=="new_track_ring_low_camera_support_rows60") I(next.ringSettings.lowCameraSupportRows60,2,10);
                else if(k=="new_track_ring_low_camera_left_clipped_rows60") I(next.ringSettings.lowCameraLeftClippedRows60,3,20);
                else if(k=="new_track_ring_low_camera_confirm_frames") I(next.ringSettings.lowCameraConfirmFrames,1,12);
                else if(k=="new_track_ring_low_camera_window_ms") I(next.ringSettings.lowCameraWindowMs,100,5000);
                else if(k=="new_track_ring_low_camera_stable_px80") I(next.ringSettings.lowCameraStablePx80,1,15);
                else if(k=="new_track_ring_low_camera_stable_rows60") I(next.ringSettings.lowCameraStableRows60,1,12);
                // 【修正】环岛入环/环内的中线横向比例(%)。50=几何正中(原行为)，
                // 默认 30 让中线偏向内侧，保证车真的转进环岛。
                else if(k=="new_track_ring_center_pratio") I(next.ringSettings.ringCenterPratio,10,90);
                // 【入环停车左移】检测到左中拐点消失后：停车 -> 向左低速移动(最多 creep_max_mm)
                // 边移动边找左上拐点；找到或走满距离即进入入环补线。
                else if(k=="new_track_ring_creep_enable") {int b=next.ringSettings.creepEnable;I(b,0,1);next.ringSettings.creepEnable=b!=0;}
                else if(k=="new_track_ring_creep_stop_ms") I(next.ringSettings.creepStopMs,0,3000);
                else if(k=="new_track_ring_creep_speed") D(next.ringSettings.creepSpeed,0.5,20.0);
                else if(k=="new_track_ring_creep_max_mm") D(next.ringSettings.creepMaxMm,20.0,1500.0);
                else if(k=="new_track_ring_creep_wheel_mm") D(next.ringSettings.wheelDiameterMm,20.0,200.0);
                else if(k=="new_track_ring_creep_timeout_ms") I(next.ringSettings.creepTimeoutMs,200,15000);
                else if(k=="new_track_ring_creep_steer_px80") I(next.ringSettings.creepSteerXPx80,0,79);
                // 【入环模板匹配】
                else if(k=="new_track_ring_template_enable") {int b=next.ringTemplateEnable;I(b,0,1);next.ringTemplateEnable=b!=0;}
                else if(k=="new_track_ring_template_frames") I(next.ringTemplateFrames,1,20);
                else if(k=="new_track_ring_template_threshold") D(next.ringTemplateThreshold,0.30,0.999);
                else if(k=="new_track_ring_template_file") next.ringTemplateFile=v;
                else if(k=="new_track_ring_template_max_ms") I(next.ringTemplateMaxMs,1000,60000);
                else if(k=="new_track_ring_template_float") D(next.ringTemplateFloat,0.0,0.3);
                else if(k=="new_track_ring_template_window_ms") I(next.ringTemplateWindowMs,100,8000);
                // ---- 环岛右侧多模板触发 ----
                else if(k=="new_track_ring_side_enable") I(next.ringSideEnable,0,1);
                else if(k=="new_track_ring_side_tmpl_prefix") next.ringSideTmplPrefix=v;
                else if(k=="new_track_ring_side_tmpl_count") I(next.ringSideTmplCount,1,20);
                else if(k=="new_track_ring_side_crop_w") I(next.ringSideCropW,8,80);
                else if(k=="new_track_ring_side_crop_h") I(next.ringSideCropH,8,56);
                else if(k=="new_track_ring_side_threshold") D(next.ringSideThreshold,0.10,0.999);
                else if(k=="new_track_ring_side_float") D(next.ringSideFloat,0.0,0.30);
                else if(k=="new_track_ring_side_frames") I(next.ringSideFrames,1,10);
                else if(k=="new_track_ring_side_window_ms") I(next.ringSideWindowMs,100,10000);
                else if(k=="new_track_ring_side_start_ms") I(next.ringSideStartMs,0,60000);
                else if(k=="new_track_ring_side_max_ms") I(next.ringSideMaxMs,1000,120000);
                else if(k=="new_track_ring_side_stop_ms") I(next.ringSideStopMs,0,10000);
                else if(k=="new_track_ring_side_left_ms") I(next.ringSideLeftMs,100,10000);
                else if(k=="new_track_ring_side_left_error") D(next.ringSideLeftError,-90.0,0.0);
                else if(k=="new_track_ring_side_speed") D(next.ringSideSpeed,0.0,20.0);
                else if(k=="new_track_ring_side_cooldown_ms") I(next.ringSideCooldownMs,0,60000);
                else if(k=="new_track_ring_side_log_ms") I(next.ringSideLogMs,100,5000);
                // ---- 环岛入口定位·虚线段判据 ----
                else if(k=="new_track_ring_entry_cue_enable") I(next.ringEntryCueEnable,0,1);
                else if(k=="new_track_ring_entry_cue_mode") I(next.ringEntryCueMode,2,3);
                else if(k=="new_track_ring_entry_dash_min_blocks") I(next.ringEntryDashMinBlocks,1,12);
                else if(k=="new_track_ring_entry_dash_tol_px") I(next.ringEntryDashTolPx,1,20);
                else if(k=="new_track_ring_entry_dash_need_frames") I(next.ringEntryDashNeedFrames,1,10);
                else if(k=="new_track_ring_entry_dash_window_ms") I(next.ringEntryDashWindowMs,100,10000);
                else if(k=="new_track_ring_entry_dash_min_y_span") I(next.ringEntryDashMinYSpan,4,56);
                else if(k=="new_track_ring_entry_dash_max_slope") D(next.ringEntryDashMaxSlope,0.2,8.0);
                else if(k=="new_track_ring_entry_dash_max_x_spread") I(next.ringEntryDashMaxXSpread,2,60);
                else if(k=="new_track_ring_entry_dash_min_gap_rows") I(next.ringEntryDashMinGapRows,0,10);
                else if(k=="new_track_ring_entry_delay_ms") I(next.ringEntryDelayMs,0,5000);
                else if(k=="new_track_ring_entry_narrow_w") I(next.ringEntryNarrowW,4,80);
                else if(k=="new_track_ring_entry_narrow_h") I(next.ringEntryNarrowH,8,56);
                else if(k=="new_track_ring_entry_start_ms") I(next.ringEntryStartMs,0,120000);
                else if(k=="new_track_ring_entry_max_ms") I(next.ringEntryMaxMs,1000,180000);
                else if(k=="new_track_ring_entry_stop_ms") I(next.ringEntryStopMs,0,60000);
                else if(k=="new_track_ring_entry_log_ms") I(next.ringEntryLogMs,100,5000);
                else if(k=="new_track_ring_template_entry_enable") {int b=next.ringSettings.templateEntryEnable?1:0;I(b,0,1);next.ringSettings.templateEntryEnable=b!=0;}
                // 【强制左转】进环后强制左打的时长/角度/速度
                else if(k=="new_track_ring_force_left_ms") I(next.ringSettings.forceLeftMs,0,8000);
                else if(k=="new_track_ring_force_left_error") D(next.ringSettings.forceLeftError,-90.0,0.0);
                else if(k=="new_track_ring_force_left_speed") D(next.ringSettings.forceLeftSpeed,0.5,20.0);
                else if(k=="new_track_ring_min_gap_rows60") I(next.ringSettings.minGapRows60,1,12);
                else if(k=="new_track_ring_separation_rows60") I(next.ringSettings.separationRows60,3,20);
                else if(k=="new_track_ring_confirm_frames") I(next.ringSettings.confirmFrames,1,12);
                else if(k=="new_track_ring_disappear_frames") I(next.ringSettings.disappearFrames,1,15);
                else if(k=="new_track_ring_right_continuous_percent") I(next.ringSettings.rightContinuousPercent,60,100);
                else if(k=="new_track_ring_enter_y_percent") I(next.ringSettings.enterYPercent,15,70);
                else if(k=="new_track_ring_entry_timeout_ms") I(next.ringSettings.entryTimeoutMs,300,8000);
                else if(k=="new_track_ring_approach_timeout_ms") I(next.ringSettings.approachTimeoutMs,1000,15000);
                else if(k=="new_track_ring_exit_search_delay_ms") I(next.ringSettings.exitSearchDelayMs,0,8000);
                else if(k=="new_track_ring_both_missing_rows60") I(next.ringSettings.bothMissingRows60,1,20);
                else if(k=="new_track_ring_post_exit_cooldown_ms") I(next.ringSettings.cooldownMs,0,10000);
                else if(k=="new_track_ring_exit_target_x_percent") I(next.ringSettings.exitTargetXPercent,0,95);
                else if(k=="new_track_ring_zebra_block_after_exit_ms") I(next.ringZebraBlockAfterExitMs,0,15000);
                else if(k=="new_track_zebra_enable") I(next.zebraEnable,0,1);
                else if(k=="new_track_zebra_stop_every_time") I(next.zebraStopEveryTime,0,1);
                else if(k=="new_track_zebra_startup_block_ms") I(next.zebraStartupBlockMs,0,60000);
                else if(k=="new_track_zebra_stop_ms") I(next.zebraStopMs,2500,10000);
                else if(k=="new_track_zebra_crop_left_percent") I(next.zebraCropLeftPercent,0,30);
                else if(k=="new_track_zebra_crop_right_percent") I(next.zebraCropRightPercent,0,30);
                else if(k=="new_track_zebra_far_y_start_percent") I(next.zebraFarYStartPercent,0,95);
                else if(k=="new_track_zebra_far_y_end_percent") I(next.zebraFarYEndPercent,1,100);
                else if(k=="new_track_zebra_near_y_start_percent") I(next.zebraNearYStartPercent,0,95);
                else if(k=="new_track_zebra_near_y_end_percent") I(next.zebraNearYEndPercent,1,100);
                else if(k=="new_track_zebra_far_black_area_min_percent") I(next.zebraFarBlackAreaMinPercent,1,90);
                else if(k=="new_track_zebra_far_black_area_max_percent") I(next.zebraFarBlackAreaMaxPercent,2,99);
                else if(k=="new_track_zebra_far_confirm_frames") I(next.zebraFarConfirmFrames,1,15);
                else if(k=="new_track_zebra_approach_speed") D(next.zebraApproachSpeed,0.5,20.0);
                else if(k=="new_track_zebra_near_min_transitions") I(next.zebraNearMinTransitions,1,20);
                else if(k=="new_track_zebra_near_min_stripes") I(next.zebraNearMinStripes,1,10);
                else if(k=="new_track_zebra_near_min_rows") I(next.zebraNearMinRows,1,30);
                else if(k=="new_track_zebra_near_black_area_min_percent") I(next.zebraNearBlackAreaMinPercent,1,90);
                else if(k=="new_track_zebra_near_black_area_max_percent") I(next.zebraNearBlackAreaMaxPercent,2,99);
                else if(k=="new_track_zebra_far_near_hold_ms") I(next.zebraFarNearHoldMs,200,10000);
                else if(k=="new_track_zebra_near_after_far_min_delay_ms") I(next.zebraNearAfterFarMinDelayMs,0,5000);
                else if(k=="new_track_zebra_require_far_near") I(next.zebraRequireFarNear,0,1);
                else if(k=="new_track_zebra_confirm_frames") I(next.zebraConfirmFrames,1,10);
                else if(k=="new_track_zebra_near_fallback_confirm_frames") I(next.zebraNearFallbackConfirmFrames,0,15);
                else if(k=="new_track_zebra_release_frames") I(next.zebraReleaseFrames,1,120);
                else if(k=="new_track_zebra_cooldown_ms") I(next.zebraCooldownMs,1000,30000);
                else if(k=="new_track_crossing_enable") I(next.crossingEnable,0,1);
                else if(k=="new_track_crossing_require_zebra") I(next.crossingRequireZebra,0,1);
                else if(k=="new_track_crossing_startup_block_ms") I(next.crossingStartupBlockMs,0,60000);
                else if(k=="new_track_startup_crossing_sync_enable") I(next.startupCrossingSyncEnable,0,1);
                else if(k=="new_track_startup_crossing_fallback_ms") I(next.startupCrossingFallbackMs,1000,8000);
                else if(k=="new_track_startup_crossing_min_detect_ms") I(next.startupCrossingMinDetectMs,0,5000);
                else if(k=="new_track_startup_crossing_confirm_frames") I(next.startupCrossingConfirmFrames,1,10);
                else if(k=="new_track_crossing_after_zebra_delay_ms") I(next.crossingAfterZebraDelayMs,0,10000);
                else if(k=="new_track_crossing_near_y_percent") I(next.crossingNearYPercent,30,85);
                else if(k=="new_track_crossing_far_confirm_frames") I(next.crossingFarConfirmFrames,1,10);
                else if(k=="new_track_crossing_far_hold_ms") I(next.crossingFarHoldMs,1000,20000);
                else if(k=="new_track_crossing_near_min_split_rows") I(next.crossingNearMinSplitRows,1,20);
                else if(k=="new_track_crossing_open_confirm_frames") I(next.crossingOpenConfirmFrames,1,20);
                else if(k=="new_track_crossing_cooldown_ms") I(next.crossingCooldownMs,1000,30000);
                else if(k=="new_track_crossing_fork_min_split_rows") I(next.crossingForkMinSplitRows,1,20);
                else if(k=="new_track_crossing_fork_min_stem_rows") I(next.crossingForkMinStemRows,1,30);
                else if(k=="new_track_crossing_fork_min_separation_px") I(next.crossingForkMinSeparationPx160,2,60);
                else if(k=="new_track_crossing_require_ring_complete") I(next.crossingRequireRingComplete,0,1);
                else if(k=="new_track_finish_stop_enable") I(next.finishStopEnable,0,1);
                else if(k=="new_track_finish_stop_delay_ms") I(next.finishStopDelayMs,0,10000);
                else if(k=="new_track_garage_auto_enable") I(next.garageAutoEnable,0,1);
                else if(k=="new_track_garage_speed") D(next.garageSpeed,0.5,20.0);
                else if(k=="new_track_garage_turn_speed") D(next.garageTurnSpeed,0.5,20.0);
                else if(k=="new_track_garage_reverse_speed") D(next.garageReverseSpeed,0.5,20.0);
                else if(k=="new_track_garage_arm_delay_ms") I(next.garageArmDelayMs,0,10000);
                else if(k=="new_track_garage_open_confirm_frames") I(next.garageOpenConfirmFrames,1,20);
                else if(k=="new_track_garage_forward_ms") I(next.garageForwardMs,100,5000);
                else if(k=="new_track_garage_right_turn_ms") I(next.garageRightTurnMs,100,5000);
                else if(k=="new_track_garage_right_turn_error") D(next.garageRightTurnError,1.0,60.0);
                else if(k=="new_track_garage_reverse_search_ms") I(next.garageReverseSearchMs,100,8000);
                else if(k=="new_track_garage_reverse_ms") I(next.garageReverseMs,100,8000);
                else if(k=="new_track_garage_straighten_ms") I(next.garageStraightenMs,100,5000);
                else if(k=="new_track_garage_straighten_error") D(next.garageStraightenError,-60.0,60.0);
                else if(k=="new_track_garage_black_confirm_frames") I(next.garageBlackConfirmFrames,1,20);
                else if(k=="new_track_garage_reverse_black_gain") D(next.garageReverseBlackGain,0.0,3.0);
                else if(k=="new_track_debug_print") I(next.debugPrint,0,1);
            }
            catch (...) {}
        }

        if(next.coneSequenceSettings.enabled) {
            std::string error;
            next.coneTuningValid=ConeSequence::loadFile("./cone_avoidance.ini",next.coneSequenceSettings,error);
            static std::string previousError;
            if(error!=previousError) {
                if(!error.empty()) std::cerr<<"[ConeTuning] STOP: "<<error<<std::endl;
                else std::cout<<"[ConeTuning] loaded ./cone_avoidance.ini; action parameters lock on entry"<<std::endl;
                previousError=error;
            }
        }
        std::lock_guard<std::mutex> lock(configMutex);
        const bool changed = next.enable != config.enable || next.laneMethod != config.laneMethod ||
            next.ringEnable != config.ringEnable || next.workMode != config.workMode || next.garageAutoEnable != config.garageAutoEnable ||
            next.centerBlackEnable != config.centerBlackEnable;
        config = next;
        if (!configPrinted || changed)
        {
            configPrinted = true;
            std::cout << "[NewTrack] revision=VV_CONE_PULSE_20261015_5 enable=" << config.enable
                      << " binary=" << (config.binarySettings.mode?"ROAD_ROI_OTSU":"LEGACY_HSV")
                      << " binary_roi=" << config.binarySettings.roiTopPercent << "/" << config.binarySettings.roiBottomPercent
                      << " lane=" << (config.workMode==0 && config.laneMethod==3 ? "M3-8neighborhood" : "7.25-contour")
                      << " center_black=" << config.centerBlackEnable
                      << " black_loose=" << config.centerBlackLooseBounds
                      << " white_margin=" << config.centerBlackMinWhiteMargin
                      << " tgt_step=" << config.steerTargetMaxStepPx
                      << " ctr_win=" << config.centerSpatialWindow
                      << " lost_stop=" << config.lostLaneStopFrames << "/" << config.lostLaneRecoverFrames
                      << " bin_blur=" << config.binarySettings.blurKernel
                      << " gray_T=" << config.binarySettings.fixedThreshold
                      << " side_tmpl=" << config.ringSideEnable << "/" << config.ringSideTmplCount
                      << "@" << config.ringSideThreshold << "x" << config.ringSideFrames
                      << "f/" << config.ringSideCropW << "x" << config.ringSideCropH
                      << " stop=" << config.ringSideStopMs << " left=" << config.ringSideLeftError
                      << "/" << config.ringSideLeftMs << " spd=" << config.ringSideSpeed
                      << " crop(t/b/l/r)=" << config.topCropRatio << "/" << config.bottomCropRatio
                      << "/" << config.leftCropRatio << "/" << config.rightCropRatio
                      << " start_right=" << config.startupRightEnable << "/" << config.startupRightDurationMs << "ms"
                      << " blue_cone=" << config.blueConeEnable
                      << " cone_sequence=" << config.coneSequenceSettings.enabled
                      << " cone_mode=" << (config.coneSequenceSettings.enabled?"BOUNDED_PULSE_GRAY_PASS":"LEGACY_VISUAL")
                      << " cone_entry=" << config.coneSequenceSettings.entryTimeMs << "-" << config.coneSequenceSettings.entryEarlyMs
                      << "+" << config.coneSequenceSettings.entryLateMs << "ms"
                      << " cone_zone_speed=" << config.coneSequenceSettings.zoneSpeed
                      << " ring_entry_stop=" << config.ringEntryCueEnable
                      << " cone_tuning_valid=" << config.coneTuningValid
                      << " cone_confirm=" << config.coneSettings.confirmFrames
                      << " work_mode=" << (config.workMode==1?"ROUNDABOUT":"NORMAL")
                      << " ring=" << config.ringEnable
                      << " ring_corner=MOORE_8 probe=" << config.ringProbeEnable
                      << " low_camera=" << config.ringSettings.lowCameraEntryEnabled
                      << " entry_frames=" << config.probeSettings.entryConfirmFrames
                      << " marker_required=" << config.ringRequireMarker
                      << " ring_approach_speed=" << config.ringApproachSpeed
                      << " ring_block=" << config.ringStartupBlockMs << "ms/lap"
                      << " zebra=" << config.zebraEnable
                      << " zebra_every=" << config.zebraStopEveryTime
                      << " zebra_release=" << config.zebraReleaseFrames
                      << " zebra_block=" << config.zebraStartupBlockMs << "ms/lap"
                      << " zebra_mode=" << (config.zebraRequireFarNear?"far-near":"near-only")
                      << " crossing=" << config.crossingEnable
                      << " crossing_block=" << config.crossingStartupBlockMs << "ms/lap"
                      << " fork_near_y=" << config.crossingNearYPercent << "% far_frames=" << config.crossingFarConfirmFrames
                      << " zebra_fallback_frames=" << config.zebraNearFallbackConfirmFrames
                      << " crossing_ring_required=" << config.crossingRequireRingComplete
                      << " garage_auto=" << config.garageAutoEnable
                      << " traffic_light=BYPASSED" << std::endl;
        }
    }

    static Config cfg()
    {
        loadConfigIfNeeded();
        std::lock_guard<std::mutex> lock(configMutex);
        return config;
    }

    static cv::Mat cropSafe(const cv::Mat &input, double top, double bottom, double left, double right)
    {
        if (input.empty()) return {};
        const int y0 = clampi(static_cast<int>(input.rows * top), 0, std::max(0,input.rows-1));
        const int y1cut = clampi(static_cast<int>(input.rows * bottom), 0, std::max(0,input.rows-1));
        const int x0 = clampi(static_cast<int>(input.cols * left), 0, std::max(0,input.cols-1));
        const int x1cut = clampi(static_cast<int>(input.cols * right), 0, std::max(0,input.cols-1));
        const int w = input.cols - x0 - x1cut;
        const int h = input.rows - y0 - y1cut;
        if (w < 8 || h < 8) return input.clone();
        return input(cv::Rect(x0,y0,w,h)).clone();
    }

    static double centerToError(double centerX, int h, int w, double centerPratio)
    {
        const double yMid = h * 0.50;
        const double rad = std::atan2(centerX - w * centerPratio, std::max(1.0, h - yMid));
        // 【误差增益】原来是写死的 4.0。画面只有 80x56，gain=4 时目标偏离中心
        // 5~6 个像素 error 就超过 40 度，实车 60% 的帧 |error|>60（舵机打到底）。
        // 现在由 error_gain 热加载控制，默认 4.0 保持原行为。
        return rad * 180.0 / std::acos(-1.0) * RuntimeConfig::getErrorGain();
    }

    // 【修正4】averageCenter 增加"近处备用窗口"。
    //
    // 原实现只平均 [h/4, 3h/4]。环岛阶段2若真实上锚点移到 3h/4 以下，
    // 这个固定窗口会整体落在被清空的远处区域 -> 一个有效点都没有 ->
    // 返回 false -> lane.valid=false -> 明明下方还有有效补线却判成无效。
    //
    // 现在：正常窗口为空时，允许在调用方指定的【近处】范围里，
    // 取"最长连续有效段"（至少 fallbackMinRun 行）求平均。
    // 只往下找，不把已经清掉的远处支路补回来；找不到就仍返回 false。
    static bool averageCenter(const std::vector<int> &center, double &out,
                              int fallbackBegin=-1, int fallbackEnd=-1,
                              int fallbackMinRun=3)
    {
        if (center.empty()) return false;
        const int h = static_cast<int>(center.size());

        // ① 正常窗口 [h/4, 3h/4]
        {
            const int y0 = h/4, y1 = h*3/4;
            long long sum=0; int n=0;
            for(int y=y0;y<=y1 && y<h;++y) if(center[y]>=0){sum+=center[y];++n;}
            if(n>0){ out=static_cast<double>(sum)/n; return true; }
        }

        // ② 【修正4】正常窗口全空 -> 在近处备用范围里找最长连续有效段
        if(fallbackBegin>=0 && fallbackEnd>fallbackBegin && fallbackBegin<h)
        {
            const int end=std::min(fallbackEnd,h-1);
            int bestS=-1,bestE=-1,curS=-1;
            for(int y=fallbackBegin;y<=end;++y)
            {
                if(center[y]>=0) { if(curS<0) curS=y; }
                else {
                    if(curS>=0 && (bestS<0 || (y-1-curS)>(bestE-bestS))){bestS=curS;bestE=y-1;}
                    curS=-1;
                }
            }
            if(curS>=0 && (bestS<0 || (end-curS)>(bestE-bestS))){bestS=curS;bestE=end;}
            if(bestS>=0 && (bestE-bestS+1)>=fallbackMinRun)
            {
                long long sum=0; int n=0;
                for(int y=bestS;y<=bestE;++y) if(center[y]>=0){sum+=center[y];++n;}
                if(n>0){ out=static_cast<double>(sum)/n; return true; }
            }
        }
        return false;
    }

    struct LaneLines
    {
        bool valid=false;
        std::vector<int> left, center, right;
        std::vector<int> rawLeft, rawRight; // Before FixLine and width-based virtual repair.
        std::vector<Roundabout::Point> rawLeftContour;
        std::vector<Roundabout::Point> rawRightContour;
        double aggregateCenter=0.0;
        int bilateralRows=0;
        int leftOnlyRows=0;
        int rightOnlyRows=0;
        bool singleEdgeUsed=false;
    };

    // 保存“正常道路”的逐行宽度记忆。这样转弯时一侧边线完全出画，
    // 可以用另一侧真实边线 + 历史道路宽度恢复虚拟边线，而不是直接把缺边补到0/图像最右端。
    static std::vector<double> laneWidthProfile;
    static double laneWidthMedian = 0.0;
    static int laneWidthProfileHeight = 0;

    // 【转向目标限速】上一帧真正下发给控制器的 targetX。-1 = 尚未初始化。
    static double lastTargetX = -1.0;
    // 【丢线停车】连续丢线段数 / 恢复计数 / 当前是否处于丢线停车状态。
    static int lostLaneCount = 0;
    static int lostLaneRecover = 0;
    static bool lostLaneStopActive = false;

    static double medianOf(std::vector<int> values)
    {
        if(values.empty()) return 0.0;
        const auto mid=values.begin()+values.size()/2;
        std::nth_element(values.begin(),mid,values.end());
        return *mid;
    }

    // 【中线空间平滑】对逐行中线做 window 行中值滤波，再与原值取平均。
    // 中值能滤掉单行跳变；和原值平均是为了保住真实弯道的曲率（照参考 cc 版）。
    static void smoothCenterRows(std::vector<int> &center, int window)
    {
        if(window<=1 || center.empty()) return;
        const int half=(window|1)/2, h=static_cast<int>(center.size());
        const auto original=center;
        for(int y=0;y<h;++y)
        {
            if(original[y]<0) continue;
            std::vector<int> neighbours;
            for(int j=std::max(0,y-half);j<=std::min(h-1,y+half);++j)
                if(original[j]>=0) neighbours.push_back(original[j]);
            if(neighbours.size()<3) continue;
            const auto mid=neighbours.begin()+neighbours.size()/2;
            std::nth_element(neighbours.begin(),mid,neighbours.end());
            center[y]=(*mid+original[y])/2;  // retain the real midpoint's curvature
        }
    }

    static void repairSingleEdge(LaneLines &lane, int w, int h, const Config &c, bool learnWidth=true,
                                 int repairBegin=0, int repairEnd=-1, const cv::Mat *mask=nullptr)
    {
        lane.bilateralRows=lane.leftOnlyRows=lane.rightOnlyRows=0;
        lane.singleEdgeUsed=false;
        if((int)laneWidthProfile.size()!=h)
        {
            laneWidthProfile.assign(h,0.0);
            laneWidthProfileHeight=h;
            laneWidthMedian=0.0;
        }

        std::vector<int> currentWidths;
        for(int y=h/4;y<h*9/10;++y)
        {
            const int l=lane.left[y], r=lane.right[y];
            if(l>=0 && r>l+8)
            {
                const int width=r-l;
                // 已有宽度记忆后，明显突然变宽（锥桶宽道/环岛开口）不用于刷新正常路宽记忆。
                bool sane = laneWidthMedian<=0.0 ||
                            (width>=laneWidthMedian*0.72 && width<=laneWidthMedian*1.28);
                if(sane && learnWidth)
                {
                    currentWidths.push_back(width);
                    if(laneWidthProfile[y]<=0.0) laneWidthProfile[y]=width;
                    else if(width>=laneWidthProfile[y]*0.68 && width<=laneWidthProfile[y]*1.32)
                        laneWidthProfile[y]=laneWidthProfile[y]*0.82+width*0.18;
                }
            }
        }
        if((int)currentWidths.size()>=4)
        {
            const double m=medianOf(currentWidths);
            laneWidthMedian = laneWidthMedian<=0.0 ? m : laneWidthMedian*0.88+m*0.12;
        }

        // Always rebuild the center, even when single-edge repair is disabled.

        auto widthAt=[&](int y)->double
        {
            if(y>=0 && y<h && laneWidthProfile[y]>8.0) return laneWidthProfile[y];
            for(int d=1;d<=8;++d)
            {
                if(y-d>=0 && laneWidthProfile[y-d]>8.0) return laneWidthProfile[y-d];
                if(y+d<h && laneWidthProfile[y+d]>8.0) return laneWidthProfile[y+d];
            }
            return laneWidthMedian;
        };

        for(int y=0;y<h;++y)
        {
            int &l=lane.left[y], &r=lane.right[y], &m=lane.center[y];
            // A ring stage deliberately masks distant branches. Never restore those rows
            // from the normal-road width memory or replace two conflicting real edges.
            if(y<repairBegin || (repairEnd>=0 && y>repairEnd) || (l>=0 && r>=0 && r<=l))
            {m=-1;continue;}
            if(l>=0 && r>l)
            {
                ++lane.bilateralRows;
                m=(l+r)/2;
                continue;
            }
            // 【A方案：图像边缘补线】参考 7.25 版 FixLine::fixLineSimple / FindLine::getCenterLine：
            // 边界缺失 = 路面一直延伸到画面外，虚拟边界直接用【画面边缘】，不再用历史路宽推算。
            // 原因：路宽记忆的 sane 只拦"突然变宽"、不拦变窄，窄路段会把 laneWidthMedian 拖小，
            //       于是"左界 = 右界 - 路宽"算出的中线偏右 —— 实车表现为循迹太靠右。
            // 统计口径(rightOnlyRows/leftOnlyRows/singleEdgeUsed)保持不变，
            // 供 single_edge_bias_px 与 partial_black 继续使用。
            if(!c.singleEdgeEnable) { m=-1; continue; }
            // 【边界缺失兜底·Q4/Q5】缺失的那一侧，优先取【该行最左/最右的白色像素】作为边界，
            // 连起来就是一条贴合路面的斜线；只有当整行都没有白色像素时，才退化成图像边缘
            // (左=x0 / 右=w-1)。这样绕环岛时看不到完整边界也不会把中线拉到画面正中。
            if(l<0 && r>=0)
            {
                int lw = 0;                                  // 退化值 = 图像最左列
                if(mask && !mask->empty() && y<mask->rows && mask->cols>=w)
                {
                    const uchar *row=mask->ptr<uchar>(y);
                    int k=0; while(k<w && row[k]==0) ++k;
                    lw = (k<w) ? k : 0;                      // 整行无白 -> 0
                }
                l = std::min(lw, std::max(0, r-2));
                m=(l+r)/2;
                ++lane.rightOnlyRows;
                lane.singleEdgeUsed=true;
            }
            else if(r<0 && l>=0)
            {
                int rw = w-1;                                // 退化值 = 图像最右列
                if(mask && !mask->empty() && y<mask->rows && mask->cols>=w)
                {
                    const uchar *row=mask->ptr<uchar>(y);
                    int k=w-1; while(k>=0 && row[k]==0) --k;
                    rw = (k>=0) ? k : (w-1);                 // 整行无白 -> w-1
                }
                r = std::max(rw, std::min(w-1, l+2));
                m=(l+r)/2;
                ++lane.leftOnlyRows;
                lane.singleEdgeUsed=true;
            }
            else m=-1;
        }
    }

    static LaneLines extractLane(const cv::Mat &mask, int method, double centerPratio, const Config &c)
    {
        LaneLines out;
        out.left.assign(mask.rows,-1); out.center.assign(mask.rows,-1); out.right.assign(mask.rows,-1);
        out.rawLeft=out.left;out.rawRight=out.right;
        if(method==3)
        {
            const auto base=Experiment::runBaseline(mask,3,centerPratio);
            for(int y=0;y<mask.rows;++y)
            {
                if(y<(int)base.left.size() && std::isfinite(base.left[y]) && base.left[y]>=0) out.left[y]=(int)std::lround(base.left[y]);
                if(y<(int)base.right.size() && std::isfinite(base.right[y]) && base.right[y]>=0) out.right[y]=(int)std::lround(base.right[y]);
            }
            out.rawLeft=out.left;out.rawRight=out.right;
            repairSingleEdge(out,mask.cols,mask.rows,c,ring.stage==RingStage::NORMAL,0,-1,&mask);
            if(!averageCenter(out.center,out.aggregateCenter)) return out;
            out.valid=true;
            return out;
        }

        // 725分支直接复用你最后给的“7.25自动排期”里的 FindLine + FixLine 核心。
        // 只把原来的图像边缘代理关闭，改由新赛道的历史路宽单边界补线接手。
        FindLine findLine;
        findLine.submitCountorFrame(mask);
        findLine.setMissingEdgeProxyEnabled(false);
        if(!findLine.handleContoursInTopFromCountorFrameSimple()) return out;
        findLine.findLineContoursSimple();
        int *l=findLine.getLeftLine(); int *r=findLine.getRightLine(); int *m=findLine.getCenterLine();
        if(!l || !r || !m) return out;
        out.rawLeft.assign(l,l+mask.rows);out.rawRight.assign(r,r+mask.rows);
        FixLine fix;
        fix.submitLeftStartPos(findLine.getLeftStartPos());
        fix.submitRightStartPos(findLine.getRightStartPos());
        fix.submitLeftEndPos(findLine.getLeftEndPos());
        fix.submitRightEndPos(findLine.getRightEndPos());
        fix.submitLeftLineContours(findLine.getLeftLineContours());
        fix.submitRightLineContours(findLine.getRightLineContours());
        fix.image_width=mask.cols; fix.image_height=mask.rows;
        fix.submitLeftLine(l); fix.submitRightLine(r); fix.submitCenterLine(m);
        fix.setMissingEdgeProxyEnabled(false);
        fix.fixLineSimple();
        for(int y=0;y<mask.rows;++y){out.left[y]=l[y];out.right[y]=r[y];out.center[y]=m[y];}
        repairSingleEdge(out,mask.cols,mask.rows,c,ring.stage==RingStage::NORMAL,0,-1,&mask);
        // 【中线空间平滑】修完边线后、求平均中线前，先做逐行中值平滑，掐掉单行跳变。
        smoothCenterRows(out.center,c.centerSpatialWindow);
        if(!averageCenter(out.center,out.aggregateCenter)) return out;
        out.valid=true;
        return out;
    }

    static void extractRingEvidence(const cv::Mat &binary, LaneLines &lane)
    {
        // Independent Moore eight-neighbor growth on the unclosed binary image.
        // Ordinary 7.25 steering and its closed image are preserved outside ring tasks.
        const auto boundaries=EightNeighbor::trace(binary.ptr<unsigned char>(),binary.cols,binary.rows,
                                                   binary.step,lane.left,lane.right);
        lane.rawLeft=boundaries.left;lane.rawRight=boundaries.right;
        lane.rawLeftContour=boundaries.leftPath;lane.rawRightContour=boundaries.rightPath;
    }

    static bool applyBlueObstacleToLane(const cv::Mat &frame, LaneLines &lane, const Config &c, ConeBoundary::Result &result)
    {
        if(!c.blueConeEnable || frame.empty()) {coneDetector.reset();result.reason="DISABLED";return false;}
        const long long ms=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow().time_since_epoch()).count();
        result=coneDetector.update(frame,lane.left,lane.right,lane.center,ms,c.coneSettings);
        if(result.active) averageCenter(lane.center,lane.aggregateCenter);
        return result.active;
    }

    struct BlackLineResult
    {
        bool reliable=false;
        int validRows=0;
        int nearRows=0;
        double weightedX=0.0;
        std::vector<int> x;
    };

    static BlackLineResult detectCenterBlack(const cv::Mat &rawMask, const LaneLines &lane, const Config &c)
    {
        BlackLineResult out; out.x.assign(rawMask.rows,-1);
        if(!c.centerBlackEnable || !lane.valid) return out;
        // 【修正3】长度一致性检查：lane 的 left/right/center 都必须不短于 rawMask.rows。
        // 暂停分支可能复用上一帧（旧尺寸）的缓存几何；热改 crop / 图像尺寸后
        // 直接按 rawMask.rows 索引会越界读。
        {
            const std::size_t need=static_cast<std::size_t>(rawMask.rows);
            if(lane.left.size()<need || lane.right.size()<need || lane.center.size()<need) return out;
        }
        int prediction=-1, predY=rawMask.rows-1;
        long long sum=0, weight=0;
        const int yNear=rawMask.rows*2/3;
        for(int y=rawMask.rows-3;y>=std::max(4,rawMask.rows/7);y-=2)
        {
            // 【黑线丢边兜底】参考 cc 版：白色路面两侧都丢时，不再直接跳过这一行，
            // 而是在画面中间 60% 区域里继续找中心黑线（丢边时黑线往往还在）。
            const bool haveBounds = lane.valid && y<(int)lane.left.size() &&
                lane.left[y]>=0 && lane.right[y]>lane.left[y]+18;
            int l, r;
            if(c.centerBlackLooseBounds)
            {
                l = haveBounds ? lane.left[y]  : rawMask.cols/5;
                r = haveBounds ? lane.right[y] : rawMask.cols*4/5;
                if(r-l<18) continue;
            }
            else
            {
                l=lane.left[y]; r=lane.right[y];
                if(l<0||r<0||r-l<18) continue;
            }
            l=clampi(l+2,1,rawMask.cols-3); r=clampi(r-2,2,rawMask.cols-2);
            int reference=prediction>=0?prediction:
                ((haveBounds && lane.center[y]>=0)?lane.center[y]:(l+r)/2);
            int maxStep=prediction>=0?c.centerBlackMaxStep*std::max(1,(predY-y)/2):rawMask.cols;
            int best=-1,bestScore=1<<30;
            const uchar *row=rawMask.ptr<uchar>(y);
            for(int x=l;x<=r;)
            {
                if(row[x]!=0){++x;continue;}
                int s=x; while(x<=r && row[x]==0) ++x; int e=x-1;
                int span=e-s+1;
                if(span<c.centerBlackMinRun || span>c.centerBlackMaxRun) continue;
                int m=std::max(c.centerBlackMinWhiteMargin,span/2);
                if(s-m<l || e+m>r) continue;
                if(row[s-m]==0 || row[e+m]==0) continue;
                int mid=(s+e)/2, d=std::abs(mid-reference);
                if(d>maxStep) continue;
                int score=d*10+span;
                if(score<bestScore){bestScore=score;best=mid;}
            }
            if(best>=0)
            {
                out.x[y]=best; prediction=best; predY=y; ++out.validRows;
                if(y>=yNear) ++out.nearRows;
                int w=(y>=yNear)?3:(y>=rawMask.rows/2?2:1); sum+=best*w; weight+=w;
            }
        }
        out.reliable=out.validRows>=c.centerBlackMinValidRows && out.nearRows>=c.centerBlackMinNearRows;
        if(weight) out.weightedX=(double)sum/weight;
        return out;
    }

    using ForkEvidence = TrackGates::ForkEvidence;

    static ForkEvidence crossingForkEvidence(const cv::Mat &raw,const LaneLines &lane,const Config &c)
    {
        TrackGates::ForkSettings s;
        s.minSplitRows=c.crossingForkMinSplitRows;s.minStemRows=c.crossingForkMinStemRows;
        s.minSeparationPx160=c.crossingForkMinSeparationPx160;
        s.nearYPercent=c.crossingNearYPercent;s.minNearSplitRows=c.crossingNearMinSplitRows;
        if(!lane.valid) return {};
        return TrackGates::detectFork(raw,lane.left,lane.right,lane.center,s);
    }

    struct FarAreaEvidence
    {
        int blackPct=0;
        int validRows=0;
        bool visible=false;
    };

    // 远区只看“赛道内部黑色面积”是否连续多帧达到范围。
    // 远处横向条纹在80x60处理图里常被压成1~2像素，强行要求黑白跳变会漏检。
    static FarAreaEvidence zebraFarArea(const cv::Mat &raw, const LaneLines &lane,
                                        int y0, int y1, const Config &c)
    {
        FarAreaEvidence e; long long black=0,area=0;
        y0=clampi(y0,0,raw.rows-1); y1=clampi(y1,y0,raw.rows-1);
        for(int y=y0;y<=y1;++y)
        {
            int l=lane.left[y],r=lane.right[y];
            if(l<0||r<=l+8) continue;
            int margin=std::max(2,(r-l)/20); l+=margin; r-=margin;
            l=std::max(l, raw.cols*c.zebraCropLeftPercent/100);
            r=std::min(r, raw.cols*(100-c.zebraCropRightPercent)/100-1);
            if(r<=l+8) continue;
            ++e.validRows;
            const uchar *row=raw.ptr<uchar>(y);
            for(int x=l;x<=r;++x){const bool isBlack=row[x]==0; black+=isBlack?1:0; ++area;}
        }
        if(area) e.blackPct=(int)(black*100/area);
        e.visible=e.validRows>=2 && e.blackPct>=c.zebraFarBlackAreaMinPercent &&
                  e.blackPct<=c.zebraFarBlackAreaMaxPercent;
        return e;
    }

    static bool zebraUpdate(const cv::Mat &raw, const LaneLines &lane, const Config &c, RingStage rs)
    {
        // 【重复停车】旧行为（zebraStopEveryTime=0）才用 seenThisLap 硬拦截。
        // 注意：seenThisLap 只在 updateCrossing() 识别到车库倒Y并计圈时复位，
        // 而倒Y实测识别不到，所以旧行为等于"整场只停一次"。
        if(!c.zebraEnable || !lane.valid) return false;
        if(!c.zebraStopEveryTime && zebra.seenThisLap) return false;
        const auto now=frameNow();
        const long long ms=std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        const long long lapMs=std::chrono::duration_cast<std::chrono::milliseconds>(now-lapEpoch).count();
        const bool allowed=lapMs>=c.zebraStartupBlockMs && rs==RingStage::NORMAL &&
            !ring.probe.controlsSteering() &&
            (ring.zebraBlockUntil==Clock::time_point{} || now>=ring.zebraBlockUntil);
        const auto far=zebraFarArea(raw,lane,raw.rows*c.zebraFarYStartPercent/100,raw.rows*c.zebraFarYEndPercent/100,c);
        TrackGates::ZebraSettings s;
        s.yStartPercent=c.zebraNearYStartPercent;s.yEndPercent=c.zebraNearYEndPercent;
        s.cropLeftPercent=c.zebraCropLeftPercent;s.cropRightPercent=c.zebraCropRightPercent;
        s.minTransitions=c.zebraNearMinTransitions;s.minStripes=c.zebraNearMinStripes;s.minRows=c.zebraNearMinRows;
        s.minBlackPercent=c.zebraNearBlackAreaMinPercent;s.maxBlackPercent=c.zebraNearBlackAreaMaxPercent;
        const auto near=TrackGates::detectZebraNear(raw,lane.left,lane.right,s);

        // 【重复停车·解锁】触发过一次后，要等这处斑马线【连续离开视野】
        // zebraReleaseFrames 帧才解锁，否则车还压在斑马线上就会被反复触发。
        // 解锁时必须同时 sequence.reset()：ZebraConfirmation::update() 开头有
        // "if(seen_) return out;"，不复位它第二道锁永远打不开。
        const bool zebraVisible = near.visible || near.strong || far.visible;
        if(zebra.latched)
        {
            if(!zebraVisible) { if(zebra.absentFrames<100000) ++zebra.absentFrames; }
            else zebra.absentFrames=0;
            if(zebra.absentFrames>=std::max(1,c.zebraReleaseFrames))
            {
                zebra.latched=false; zebra.absentFrames=0;
                zebra.sequence.reset();
                if(c.debugPrint)
                    std::cout<<ANSI_BLUE<<"【人行横道解锁】已离开视野"<<c.zebraReleaseFrames
                             <<"帧，可以识别下一处斑马线"<<ANSI_RESET<<std::endl;
            }
        }
        // 锁存中 / 冷却中：状态机照常更新（保留远区锁存和计时），但不允许再次触发。
        const bool canTrigger = !zebra.latched &&
            !(c.zebraCooldownMs>0 && now<zebra.cooldownUntil);

        const auto decision=zebra.sequence.update(far.visible,near.visible,near.strong,ms,allowed,
            c.zebraRequireFarNear,c.zebraFarConfirmFrames,c.zebraConfirmFrames,c.zebraNearFallbackConfirmFrames,
            c.zebraNearAfterFarMinDelayMs,c.zebraFarNearHoldMs);
        if(decision.justArmed)
            std::cout<<ANSI_BLUE<<"【人行横道远区预警】面积="<<far.blackPct<<"%，降速至="<<c.zebraApproachSpeed<<ANSI_RESET<<std::endl;
        if(decision.expired)
            std::cout<<ANSI_BLUE<<"【人行横道远区超时】解除锁存；强近区仍可独立确认"<<ANSI_RESET<<std::endl;
        if(decision.slow) {speedOverrideValue.store(c.zebraApproachSpeed);speedOverrideActive.store(true);}
        if(c.debugPrint && lapMs>=c.zebraStartupBlockMs)
        {
            static Clock::time_point lastPrint{};
            if(lastPrint==Clock::time_point{} || std::chrono::duration_cast<std::chrono::milliseconds>(now-lastPrint).count()>=900)
            {
                lastPrint=now;
                std::cout<<"【人行横道观察】允许="<<allowed<<" 环岛="<<(int)rs
                         <<" 远面积="<<far.blackPct<<"% 锁存="<<zebra.sequence.farArmed()
                         <<" 近连续行="<<near.maxConsec<<"/"<<c.zebraNearMinRows
                         <<" 宽黑条="<<near.maxStripes<<"/"<<c.zebraNearMinStripes
                         <<" 全区/条纹行面积="<<near.blackPct<<"/"<<near.stripeBlackPct
                         <<"% 横向黑带="<<near.transverseBands
                         <<" 近确认="<<zebra.sequence.nearFrames()<<"/"<<c.zebraConfirmFrames
                         <<" 强近确认="<<zebra.sequence.strongFrames()<<"/"<<c.zebraNearFallbackConfirmFrames
                         <<" 已停="<<zebra.seenThisLap
                         <<" 锁存中="<<zebra.latched
                         <<" 离视野="<<zebra.absentFrames<<"/"<<c.zebraReleaseFrames<<std::endl;
            }
        }
        if(!canTrigger) return false;
        if(!decision.triggered) return false;
        zebra.latched=true;zebra.seenThisLap=true;zebra.absentFrames=0;
        zebra.cooldownUntil=now+std::chrono::milliseconds(c.zebraCooldownMs);zebra.lastTrigger=now;
        crossing.confirmation.reset();crossing.farNear.reset();crossing.openConfirm=0;
        std::cout<<ANSI_RED<<"!!!【检测到人行横道】第"<<(lapCounter.load()+1)
                 <<"圈，近区条纹确认，停车"<<c.zebraStopMs<<"毫秒!!!"<<ANSI_RESET<<std::endl;
        return true;
    }

    // 【入环模板匹配】把运行时灰度裁剪图映射到诊断视图 GRAY 面板的坐标系。
    // DiagnosticView 的面板做法：scale=min(480/cols,316/rows)，INTER_NEAREST 缩放后
    // 居中放进 480x316 单元格(单元格原点 y=48)。这里完全复现，使离线从录像截取的
    // 模板(取自该面板坐标)可以直接在本函数输出上匹配。
    static cv::Mat toDiagnosticGrayPanel(const cv::Mat &frame)
    {
        if(frame.empty()) return cv::Mat();
        cv::Mat g;
        if(frame.channels()==3) cv::cvtColor(frame,g,cv::COLOR_BGR2GRAY);
        else g=frame;
        if(g.empty() || g.cols<2 || g.rows<2) return cv::Mat();
        const double sc=std::min(480.0/g.cols,316.0/g.rows);
        cv::Mat rs;
        cv::resize(g,rs,cv::Size(std::max(1,static_cast<int>(g.cols*sc)),
                                 std::max(1,static_cast<int>(g.rows*sc))),0,0,cv::INTER_NEAREST);
        cv::Mat panel=cv::Mat::zeros(400,480,CV_8UC1);
        const int x=(480-rs.cols)/2, y=48+(316-rs.rows)/2;
        if(x>=0 && y>=0 && x+rs.cols<=480 && y+rs.rows<=400)
            rs.copyTo(panel(cv::Rect(x,y,rs.cols,rs.rows)));
        return panel;
    }

    // 模板只加载一次(按文件名缓存)
    static const cv::Mat& ringTemplateCached(const std::string &file)
    {
        static std::string cachedName;
        static cv::Mat cached;
        if(file!=cachedName)
        {
            cachedName=file;
            cached=cv::imread(file,cv::IMREAD_GRAYSCALE);
            std::cout<<"[NewTrack][TMPL] load "<<file<<" -> "
                     <<(cached.empty()?"FAILED":(std::to_string(cached.cols)+"x"+std::to_string(cached.rows)))
                     <<std::endl;
        }
        return cached;
    }

    static double ringTemplateScore(const cv::Mat &panel,const cv::Mat &tmpl)
    {
        if(panel.empty() || tmpl.empty()) return 0.0;
        if(tmpl.rows>panel.rows || tmpl.cols>panel.cols) return 0.0;
        cv::Mat res;
        cv::matchTemplate(panel,tmpl,res,cv::TM_CCOEFF_NORMED);
        double mn=0.0,mx=0.0;cv::Point a,b;
        cv::minMaxLoc(res,&mn,&mx,&a,&b);
        return mx;
    }

    // ================= 【环岛入口定位·虚线段判据】 =================
    // phase: 0=IDLE  1=ARMED(已确认在环岛右侧)  2=STOPPED
    struct RingEntryCueState
    {
        int       phase        = 0;
        int       armedCount   = 0;
        long long lastDashMs   = 0;
        long long armedAtMs    = 0;
        int       dashBlocks   = 0;
        int       bigW = 0, bigH = 0, bigArea = 0;
        int       usedFrames   = 0;      // 触发时的帧号(仅用于日志)
        long long stopAtMs     = 0;
        long long lapMsAtStop  = 0;
        bool      stopForced   = false;
    } ringEntryCue;

    // 在运行时原始二值图上找【黑色虚线段】和【最大暗块】。
    // mask: CV_8UC1 80x56, 白=路面 黑=非路面
    //
    // 【黑色虚线段判据】三条（几何+邻域，比"拟合共线"可靠得多）：
    //   ① x 值不变    : 虚线段沿道路方向排列，质心 x 必须在同一条竖线上
    //   ② 两侧都是白色: 虚线是【细线】，其所在行的左右紧邻必须是路面。
    //        - 斑马线条纹横跨整个路宽 -> 同一行左右都是黑 -> 必然排除
    //        - 黑弧 / 环岛岛是实心大块   -> 同一行左右也是黑 -> 同样排除
    //   ③ 不连续      : 按 y 排序后，相邻两段之间必须有空隙（段间是路面）
    static void ringEntryMeasure(const cv::Mat &mask, const Config &c,
                                 int &dashCount, int &bigW, int &bigH, int &bigArea)
    {
        dashCount = 0; bigW = 0; bigH = 0; bigArea = 0;
        if(mask.empty() || mask.channels()!=1) return;
        cv::Mat dark;
        cv::compare(mask, 128, dark, cv::CMP_LT);
        cv::Mat lab, st, ct;
        const int n = cv::connectedComponentsWithStats(dark, lab, st, ct, 8);

        struct Blk { int cx, cy, w, h; };
        std::vector<Blk> raw;
        for(int i=1;i<n;++i)
        {
            const int a  = st.at<int>(i,cv::CC_STAT_AREA);
            const int ww = st.at<int>(i,cv::CC_STAT_WIDTH);
            const int hh = st.at<int>(i,cv::CC_STAT_HEIGHT);
            if(a>=200 && a>bigArea) { bigArea=a; bigW=ww; bigH=hh; }
            if(a>=3 && a<=70 && ww<=12 && hh<=14)
            {
                Blk b;
                b.cx=(int)std::lround(ct.at<double>(i,0));
                b.cy=(int)std::lround(ct.at<double>(i,1));
                b.w=ww; b.h=hh;
                raw.push_back(b);
            }
        }
        if(raw.size()<2) return;

        // 【特征②·两侧都是白色】虚线所在行的左右紧邻必须是路面。
        std::vector<Blk> keep;
        for(const Blk &b : raw)
        {
            if(b.cy<0 || b.cy>=mask.rows) continue;
            const int lx = b.cx - b.w/2 - 2;
            const int rx = b.cx + b.w/2 + 2;
            if(lx<0 || rx>=mask.cols) continue;
            const uchar *row = mask.ptr<uchar>(b.cy);
            if(row[lx] < 128 || row[rx] < 128) continue;   // 任一侧不是路面 -> 丢
            keep.push_back(b);
        }
        if(keep.size()<2) { dashCount=(int)keep.size(); return; }

        // 【特征①·x 值不变】虚线段在同一条竖线上，x 质心必须集中。
        int xmin=9999, xmax=-1;
        for(const Blk &b : keep) { xmin=std::min(xmin,b.cx); xmax=std::max(xmax,b.cx); }
        if(xmax-xmin > c.ringEntryDashMaxXSpread) return;

        // 【特征③·不连续】按 y 排序，相邻两段之间必须有空隙。
        std::sort(keep.begin(), keep.end(),
                  [](const Blk &A, const Blk &B){ return A.cy < B.cy; });
        int gaps=0;
        for(std::size_t k=1;k<keep.size();++k)
            if(keep[k].cy - (keep[k-1].cy + keep[k-1].h) >= c.ringEntryDashMinGapRows) ++gaps;
        if(gaps*2 < (int)keep.size()-1) return;            // 太密 -> 更像实线，丢弃

        dashCount=(int)keep.size();
    }

    static void ringEntryCueUpdate(const cv::Mat &rawMask, const Config &c, long long lapMs, long long ms, bool allowNew=true)
    {
        if(!c.ringEntryCueEnable)
        { ringEntryCue.phase=0; ringEntryStopForced.store(false); return; }

        // --- 已停车：可选自动恢复 ---
        if(ringEntryCue.phase==2)
        {
            ringEntryStopForced.store(true);
            if(c.ringEntryStopMs>0 && ms-ringEntryCue.stopAtMs>=c.ringEntryStopMs)
            {
                ringEntryCue.phase=0; ringEntryStopForced.store(false);
                ringEntryCue.armedCount=0;
                std::cout<<ANSI_BLUE<<"【环岛入口·停车结束】恢复了（停了"<<c.ringEntryStopMs<<"ms）"<<ANSI_RESET<<std::endl;
            }
            return;
        }

        if(!allowNew) {
            ringEntryCue.phase=0;ringEntryCue.armedCount=0;ringEntryCue.lastDashMs=0;return;
        }
        if(lapMs < c.ringEntryStartMs || lapMs > c.ringEntryMaxMs) return;

        int dash=0, bw=0, bh=0, ba=0;
        ringEntryMeasure(rawMask, c, dash, bw, bh, ba);
        ringEntryCue.dashBlocks=dash;
        ringEntryCue.bigW=bw; ringEntryCue.bigH=bh; ringEntryCue.bigArea=ba;

        // --- 阶段A：确认在环岛右侧（检出虚线段）---
        if(dash >= c.ringEntryDashMinBlocks)
        {
            if(ringEntryCue.lastDashMs==0 || ms-ringEntryCue.lastDashMs>c.ringEntryDashWindowMs)
                ringEntryCue.armedCount=1;
            else ++ringEntryCue.armedCount;
            ringEntryCue.lastDashMs=ms;

            if(ringEntryCue.phase==0 && ringEntryCue.armedCount>=std::max(1,c.ringEntryDashNeedFrames))
            {
                ringEntryCue.phase=1; ringEntryCue.armedAtMs=ms;
                std::cout<<ANSI_BLUE<<"!!!【环岛入口·检出虚线段】共线暗块="<<dash
                         <<" 累计"<<ringEntryCue.armedCount<<"帧 lap_ms="<<lapMs
                         <<" -> 等待"<<(c.ringEntryCueMode==2?"延时":"窄条")<<"判据!!!"<<ANSI_RESET<<std::endl;
            }
        }
        else if(ringEntryCue.lastDashMs!=0 && ms-ringEntryCue.lastDashMs>c.ringEntryDashWindowMs)
        {
            ringEntryCue.armedCount=0;
        }

        if(ringEntryCue.phase!=1) return;

        // --- 阶段B：决定停车时刻 ---
        bool fire=false; const char* why="";
        if(c.ringEntryCueMode==2)
        {
            if(ms-ringEntryCue.armedAtMs >= c.ringEntryDelayMs){ fire=true; why="延时到"; }
        }
        else
        {
            if(bh>=c.ringEntryNarrowH && bw<=c.ringEntryNarrowW && bw>0){ fire=true; why="竖直窄条"; }
        }
        if(!fire) return;

        ringEntryCue.phase=2; ringEntryStopForced.store(true);
        ringEntryCue.stopAtMs=ms; ringEntryCue.lapMsAtStop=lapMs; ++ringEntryCue.usedFrames;
        std::cout<<ANSI_RED<<"!!!【环岛入口·停车】"<<why<<" mode="<<c.ringEntryCueMode
                 <<" lap_ms="<<lapMs<<" 虚线块="<<dash
                 <<" 最大暗块="<<bw<<"x"<<bh<<"(area="<<ba<<")"
                 <<(c.ringEntryStopMs>0?(" 停"+std::to_string(c.ringEntryStopMs)+"ms后恢复"):" 永久停车")
                 <<"!!!"<<ANSI_RESET<<std::endl;
    }

    // ================= 【环岛右侧多模板触发】 =================
    // 状态机：0=IDLE  1=STOP(完全断电停车)  2=LEFT(强制左转)  3=刚结束(冷却中)
    struct RingSideState
    {
        int    phase        = 0;
        long long phaseAtMs = 0;
        long long cooldownUntilMs = 0;
        int    hits         = 0;
        long long windowStartMs = 0;
        double score        = -1.0;
        int    bestIdx      = -1;
        int    triggerCount = 0;
        bool   stopForced   = false;   // 给 forceMotorStop() 用
    } ringSide;
    static std::vector<cv::Mat> ringSideTmpls;   // 缓存，按 prefix+count 失效
    static std::string ringSideTmplKey;

    // 按 "prefix + 下标 + .png" 加载 ringSideTmplCount 张模板。文件缺失只警告一次。
    static void ringSideLoadTemplates(const Config &c)
    {
        const std::string key = c.ringSideTmplPrefix + "#" + std::to_string(c.ringSideTmplCount);
        if(key == ringSideTmplKey) return;
        ringSideTmplKey = key;
        ringSideTmpls.clear();
        for(int i=0;i<c.ringSideTmplCount;++i)
        {
            const std::string f = c.ringSideTmplPrefix + std::to_string(i) + ".png";
            cv::Mat t = cv::imread(f,cv::IMREAD_GRAYSCALE);
            if(t.empty())
                std::cout << "[NewTrack][RING_SIDE] 模板缺失/读取失败: " << f << std::endl;
            ringSideTmpls.push_back(t);
        }
        int ok=0; for(const auto &t:ringSideTmpls) if(!t.empty()) ++ok;
        std::cout << "[NewTrack][RING_SIDE] 载入 " << ok << "/" << c.ringSideTmplCount
                  << " 张模板, 前缀=" << c.ringSideTmplPrefix
                  << ", 裁剪=" << c.ringSideCropW << "x" << c.ringSideCropH
                  << ", 阈值=" << c.ringSideThreshold << ", 累计=" << c.ringSideFrames << "帧" << std::endl;
    }

    // 对 7 张模板各做一次滑动匹配，返回最高分与对应模板下标。
    static double ringSideScore(const cv::Mat &grey80, const Config &c, int &bestIdx)
    {
        bestIdx = -1;
        if(grey80.empty() || grey80.channels()!=1) return -1.0;
        double best = -1.0;
        for(std::size_t i=0;i<ringSideTmpls.size();++i)
        {
            const cv::Mat &t = ringSideTmpls[i];
            if(t.empty()) continue;
            int cw = std::min(c.ringSideCropW>0?c.ringSideCropW:t.cols, t.cols);
            int ch = std::min(c.ringSideCropH>0?c.ringSideCropH:t.rows, t.rows);
            if(cw<8 || ch<8) continue;
            if(grey80.cols<cw || grey80.rows<ch) continue;
            const int x0=(t.cols-cw)/2, y0=(t.rows-ch)/2;
            const cv::Mat sub = t(cv::Rect(x0,y0,cw,ch));
            cv::Mat res;
            cv::matchTemplate(grey80, sub, res, cv::TM_CCOEFF_NORMED);
            double mn=0.0,mx=0.0; cv::Point a,b;
            cv::minMaxLoc(res,&mn,&mx,&a,&b);
            if(mx>best){ best=mx; bestIdx=(int)i; }
        }
        return best;
    }

    // 每帧调用。grey80 = 裁剪后的 80x56 灰度图（和 80x56_native 同坐标）。
    static void ringSideUpdate(const cv::Mat &grey80, const Config &c, long long lapMs, long long ms, bool allowNew=true)
    {
        if(!c.ringSideEnable)
        {
            ringSide.phase=0; ringSideStopForced.store(false); return;
        }

        // --- 1) 先推进状态机（不受检测时间窗影响）---
        if(ringSide.phase==1 && ms-ringSide.phaseAtMs>=c.ringSideStopMs)
        {
            ringSide.phase=2; ringSide.phaseAtMs=ms; ringSideStopForced.store(false);
            std::cout<<ANSI_RED<<"!!!【环岛右侧·强制左转开始】error="<<c.ringSideLeftError
                     <<"度 持续"<<c.ringSideLeftMs<<"ms 速度="<<c.ringSideSpeed
                     <<"（停车"<<c.ringSideStopMs<<"ms 结束）!!!"<<ANSI_RESET<<std::endl;
        }
        else if(ringSide.phase==2 && ms-ringSide.phaseAtMs>=c.ringSideLeftMs)
        {
            ringSide.phase=3; ringSide.phaseAtMs=ms;
            ringSide.cooldownUntilMs = ms + c.ringSideCooldownMs;
            std::cout<<ANSI_BLUE<<"【环岛右侧·强制左转结束】冷却"<<c.ringSideCooldownMs
                     <<"ms 后可以再次识别（累计触发 "<<ringSide.triggerCount<<" 次）"<<ANSI_RESET<<std::endl;
        }
        if(ringSide.phase==3 && ms>=ringSide.cooldownUntilMs)
        {
            ringSide.phase=0; ringSide.hits=0; ringSide.windowStartMs=0;
        }
        if(ringSide.phase!=0) { return; }        // 序列执行中，不再检测
        if(!allowNew) {ringSide.hits=0;ringSide.windowStartMs=0;return;}

        // --- 2) 检测 ---
        if(lapMs < c.ringSideStartMs || lapMs > c.ringSideMaxMs) return;
        if(ms < ringSide.cooldownUntilMs) return;
        ringSideLoadTemplates(c);

        int bi=-1;
        const double s = ringSideScore(grey80, c, bi);
        ringSide.score=s; ringSide.bestIdx=bi;
        if(s < 0.0) return;

        const double eff = c.ringSideThreshold - c.ringSideFloat;
        if(s >= eff)
        {
            if(ringSide.windowStartMs==0 || ms-ringSide.windowStartMs>c.ringSideWindowMs)
            { ringSide.windowStartMs=ms; ringSide.hits=1; }
            else ++ringSide.hits;
        }
        else if(ringSide.windowStartMs!=0 && ms-ringSide.windowStartMs>c.ringSideWindowMs)
        { ringSide.hits=0; ringSide.windowStartMs=0; }

        if(ringSide.hits >= std::max(1,c.ringSideFrames))
        {
            ringSide.phase=1; ringSide.phaseAtMs=ms; ringSideStopForced.store(true);
            ringSide.hits=0; ringSide.windowStartMs=0; ++ringSide.triggerCount;
            std::cout<<ANSI_RED<<"!!!【环岛右侧模板命中】score="<<s<<" 模板T"<<bi
                     <<" 累计"<<c.ringSideFrames<<"帧 -> 完全断电停车"<<c.ringSideStopMs
                     <<"ms，随后强制左转"<<c.ringSideLeftError<<"度"<<c.ringSideLeftMs
                     <<"ms，速度压到"<<c.ringSideSpeed<<"!!!"<<ANSI_RESET<<std::endl;
        }
    }

    static void updateRing(const cv::Mat &raw, LaneLines &lane, int width, const Config &c,
                           bool coneVisible=false, bool binaryQuality=true,
                           const cv::Mat &srcFrame=cv::Mat())
    {
        const auto now=frameNow();
        const long long ms=std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        const bool routeReady=!c.coneSequenceSettings.enabled || coneSequence.routeComplete();
        const bool enabled=c.workMode==1 && c.ringEnable && routeReady;
        const bool physicalPause=finishStopped || coneStopForced.load() || ms*1000000<zebraStopUntilNs.load();
        const bool paused=physicalPause || !lane.valid || !binaryQuality ||
            (c.coneSequenceSettings.enabled && (gMotorStop || coneInputStop.load()));
        // 【修正6】把"暂停/证据无效"状态暴露给 FrameResult 和 CSV，
        // 这样上层能区分"观测被冻结"和"本帧真实检测结果"。
        ring.observationPaused=paused;
        const long long delta=ring.lastFrameMs>0?std::max(0LL,ms-ring.lastFrameMs):0;
        if(c.coneSequenceSettings.enabled && routeReady && (paused || ring.pausedPreviously))
            ring.routePausedMs+=delta;
        ring.lastFrameMs=ms;
        if(paused || ring.pausedPreviously)
        {
            if(ring.taskAtMs>0) ring.taskAtMs+=delta;
            if(ring.entryAssistUntilMs>0) ring.entryAssistUntilMs+=delta;
        }
        if(physicalPause || ring.physicalPausePreviously) lapEpoch+=std::chrono::milliseconds(delta);
        ring.pausedPreviously=paused;ring.physicalPausePreviously=physicalPause;
        const long long lapMs=c.coneSequenceSettings.enabled && coneSequence.releaseAtMs()>=0?
            c.ringStartupBlockMs+std::max(0LL,ms-coneSequence.releaseAtMs()-ring.routePausedMs):
            std::chrono::duration_cast<std::chrono::milliseconds>(now-lapEpoch).count();
        const bool candidateWindow=lapMs<=c.ringMarkerWindowMs || ring.markerResult.armed || ring.probe.phase()==2;
        const bool canSeek=lapCounter.load()<2 && lapMs>=c.ringStartupBlockMs && candidateWindow &&
            !zebra.seenThisLap && !zebra.sequence.farArmed() && !ring.attemptedThisLap && !paused && !coneVisible;
        if(enabled && !paused && ring.controller.stage()>=2 && ring.taskAtMs>0 && ms-ring.taskAtMs>=c.ringTaskTimeoutMs)
        {
            ring.controller.reset();ring.marker.reset();ring.probe.reset();ring.attemptedThisLap=true;
            std::cout<<ANSI_BLUE<<"【环岛任务超时】未形成完整出环证据，解除环岛控制；不标记完成，恢复巡线和横道检测"<<ANSI_RESET<<std::endl;
        }
        if((!enabled || (!paused && !canSeek && ring.controller.stage()<2 && !ring.probe.controlsSteering())) && ring.controller.stage()==1)
            ring.controller.reset();
        ring.markerEvidence=TrackGates::detectMarker(raw,lane.left,lane.right,lane.center,c.markerSettings);
        const bool latchOn=enabled && c.ringRequireMarker &&
            (ring.controller.stage()>=2 || ring.probe.phase()==2 ||
             (paused && ring.markerResult.armed) || (canSeek && ring.probe.phase()<3));
        ring.markerResult=ring.marker.update(ring.markerEvidence.passed,ms,latchOn,canSeek,
            ring.controller.stage()>=2 || ring.probe.phase()==2 || (paused && ring.markerResult.armed),ring.controller.completed(),
            c.ringMarkerConfirmFrames,c.ringMarkerHoldMs,
            paused);   // 【修正2】传入真实暂停标志：暂停期间冻结 holdMs 计时，恢复后不因长暂停立即过期
        if(ring.markerResult.justArmed)
            std::cout<<ANSI_BLUE<<"【环岛门控通过】黑块="<<ring.markerEvidence.blocks<<"，立即降速至="
                     <<c.ringApproachSpeed<<"；入口稳定"<<c.probeSettings.entryConfirmFrames<<"帧才左探；"
                     <<c.probeSettings.waitMs<<"ms兜底仍需有效开口"<<ANSI_RESET<<std::endl;
        const auto evidence=Roundabout::detect(lane.rawLeft,lane.rawRight,width,c.ringSettings,lane.rawLeftContour,lane.rawRightContour);
        const bool standardEntry=evidence.leftBreaks>=1 && evidence.up.valid() && evidence.up.x<width/2 && evidence.rightContinuous;
        const bool entrySeen=standardEntry || evidence.lowCameraEntry;
        const auto entryPoint=evidence.lowCameraEntry?evidence.rightEntry:evidence.up;
        // 【修正5】兜底开口：统计"最长连续贴边段"，而不是累计贴边行数。
        //   - 累计法会被分散噪点凑够行数；连续段法不会
        //   - 同时记录该段的稳定中心行，作为候选跟踪坐标
        //   - 该位置只是候选定位，绝不写入真实拐点证据
        int borderBestRun=0, borderBestMid=-1, borderRun=0, borderStart=-1;
        const int h=static_cast<int>(lane.rawLeft.size());
        for(int y=h/3;y<h-3;++y)
        {
            const bool clipped=(lane.rawLeft[y]>=0 && lane.rawLeft[y]<=1);
            if(clipped)
            {
                if(borderRun==0) borderStart=y;
                ++borderRun;
                if(borderRun>borderBestRun){borderBestRun=borderRun;borderBestMid=(borderStart+y)/2;}
            }
            else borderRun=0;
        }
        const int minRows=std::max(3,static_cast<int>(std::lround(c.probeSettings.minRows60*h/60.0)));
        // 兜底判据：用最长连续段，不用累计行数
        const bool fallbackOpening=evidence.rightContinuous && (evidence.leftBreaks>=1 || borderBestRun>=minRows);
        const bool probeGate=ring.markerResult.armed || (!c.ringRequireMarker && canSeek);
        const bool newProbeGate=ring.markerResult.justArmed || (!c.ringRequireMarker && canSeek && ring.probe.phase()==0);
        // 【修正5】兜底时若没有真实入口点，用"最长连续贴边段中心行"作候选定位。
        // 仅用于候选跟踪，不修改 evidence 里的真实拐点。
        auto probeEntry=entryPoint;
        if(!probeEntry.valid() && fallbackOpening && borderBestMid>=0)
        {
            probeEntry.y=borderBestMid;
            probeEntry.x=(lane.rawLeft[borderBestMid]>=0)?lane.rawLeft[borderBestMid]:0;
        }
        ring.probeResult=ring.probe.update(probeGate,newProbeGate,entrySeen,
            ring.controller.stage()>=2,ring.controller.completed(),enabled && c.ringProbeEnable,paused,
            lane.rawLeft,width,ms,c.probeSettings,fallbackOpening,&lane.rawRight,probeEntry.x,probeEntry.y);
        if(ring.probeResult.started)
            std::cout<<ANSI_BLUE<<"【环岛左探开始】原因="<<(evidence.lowCameraEntry?"低相机入口多帧确认":(entrySeen?"八邻域入口多帧确认":"等待结束且开口多帧确认"))
                     <<" 速度="<<c.ringApproachSpeed<<" 左偏="<<c.probeSettings.leftError<<ANSI_RESET<<std::endl;
        if(ring.probeResult.rejected || ring.probeResult.unknown)
        {
            ring.marker.reject();ring.markerResult.armed=false;
            if(ring.controller.stage()<=1) ring.controller.reset();
            std::cout<<ANSI_BLUE<<(ring.probeResult.rejected?"【环岛误检回正】左界x增大，短时右偏":"【环岛左探结束】未测到持续扩宽，恢复巡线")
                     <<" delta_x="<<ring.probeResult.deltaX<<" rows="<<ring.probeResult.sampleRows<<ANSI_RESET<<std::endl;
        }
        const bool allowStart=enabled && !paused && !ring.attemptedThisLap && ring.probe.phase()<3 &&
            (canSeek || ring.probe.phase()==2) && (!c.ringRequireMarker || ring.markerResult.armed);
        auto settings=c.ringSettings;settings.entryValidationRequired=c.ringProbeEnable!=0;
        ring.gateOpen=allowStart;
        const bool wasComplete=ring.controller.completed();const int previous=ring.controller.stage();
        // 【入环停车左移】把本帧车轮转过的圈数喂给控制器，用于 0.4m 行程判定。
        // 编码器速度单位是"转/秒"(与 emergency_reverse_turns 的积分方式一致)，
        // 用帧间隔积分得到本帧圈数；左右轮取平均。
        if(c.ringSettings.creepEnable)
        {
            const double dtSec=double(delta)/1000.0;
            if(dtSec>0.0 && dtSec<0.5)
            {
                const double turns=0.5*(double(gMotorLeftTrueSpeed)+double(gMotorRightTrueSpeed))*dtSec;
                ring.controller.feedTurns(turns);
            }
        }
        // 【入环模板匹配】只在阶段0/1、未完成任务、本圈未试过，且落在
        // [ringStartupBlockMs, ringTemplateMaxMs] 时间窗内时累积。
        // 这条路径不再经过 allowStart/canSeek，避免被几何那套时间窗+黑块门控+横道状态绑死。
        bool templateHit=false;
        if(c.ringTemplateEnable && !c.ringTemplateFile.empty()
           && ring.controller.stage()<=1 && !wasComplete
           && !ring.attemptedThisLap && !paused && enabled
           && lapMs>=c.ringStartupBlockMs && lapMs<=c.ringTemplateMaxMs)
        {
            const cv::Mat panel=toDiagnosticGrayPanel(srcFrame);
            if(!panel.empty())
            {
                const cv::Mat &tmpl=ringTemplateCached(c.ringTemplateFile);
                ring.templateScore=ringTemplateScore(panel,tmpl);
                // 命中判据用 (基准阈值 - 向下浮动)，如 0.60 - 0.03 = 0.57
                const double effThreshold=c.ringTemplateThreshold-c.ringTemplateFloat;
                // 【窗口累计】命中帧不必连续：同一次窗口内累计够 ringTemplateFrames 次即触发；
                // 窗口过期才清零。解决"分数跳变导致连续帧判据永远凑不满"的问题。
                if(ring.templateScore>=effThreshold)
                {
                    if(ring.templateWindowStartMs==0 ||
                       lapMs-ring.templateWindowStartMs>c.ringTemplateWindowMs)
                    { ring.templateWindowStartMs=lapMs; ring.templateConfirm=1; }
                    else ++ring.templateConfirm;
                }
                else if(ring.templateWindowStartMs!=0 &&
                        lapMs-ring.templateWindowStartMs>c.ringTemplateWindowMs)
                { ring.templateConfirm=0; ring.templateWindowStartMs=0; }
                if(ring.templateConfirm>=std::max(1,c.ringTemplateFrames))
                {
                    templateHit=true;
                    ring.templateHitMs=ms;
                }
            }
        }
        else { ring.templateConfirm=0; ring.templateWindowStartMs=0; }
        ring.result=ring.controller.update(lane.rawLeft,lane.rawRight,width,ms,allowStart,enabled,settings,
            lane.rawLeftContour,lane.rawRightContour,ring.probeResult.accepted,paused,templateHit);
        if(previous<2 && ring.result.stage>=2)
        {
            ring.taskAtMs=ms;ring.entryAssistUntilMs=ms+c.ringEntryAssistMs;ring.attemptedThisLap=true;
            std::cout<<ANSI_BLUE<<"【环岛入口确认】左界向左扩宽或已有连续贴边入口；进入无中心黑线环岛，delta_x="
                     <<ring.probeResult.deltaX<<" width_delta="<<ring.probeResult.deltaWidth<<" rows="<<ring.probeResult.sampleRows
                     <<" border_rows="<<ring.probeResult.clippedRows<<ANSI_RESET<<std::endl;
        }
        ring.stage=static_cast<RingStage>(ring.result.stage);ringStageAtomic.store(ring.result.stage);
        speedOverrideActive.store(false);
        // 【入环停车左移】停车/左移优先于常规环岛速度与几何：
        //   停车阶段：目标速度 0；
        //   左移阶段：用 creepSpeed，并把中线整列压到左侧(creepSteerXPx80)，
        //             让车头朝环岛方向转，同时继续每帧检测左上拐点。
        if(ring.result.creepStop || ring.result.creepMove)
        {
            if(ring.result.creepStop)
            {
                speedOverrideValue.store(0.0);
                speedOverrideActive.store(true);
            }
            else
            {
                speedOverrideValue.store(c.ringSettings.creepSpeed);
                speedOverrideActive.store(true);
                const int rows=static_cast<int>(lane.center.size());
                if(rows>0 && width>1)
                {
                    const int tx=std::max(0,std::min(width-1,c.ringSettings.creepSteerXPx80*width/80));
                    std::fill(lane.center.begin(),lane.center.end(),tx);
                    lane.valid=true;
                    lane.aggregateCenter=double(tx);
                }
            }
        }
        else if(ring.result.active)
        {
            // 【修正3】尺寸一致性检查。
            // 控制器的暂停分支会返回上一帧缓存的几何；若期间热改了 crop 或图像尺寸，
            // 缓存的 left/right/center 长度会和本帧 rawLeft 对不上。
            // 直接拿去覆盖 lane 会让后面的 repairSingleEdge / averageCenter 越界。
            const int expectRows=static_cast<int>(lane.rawLeft.size());
            const bool geomSizeOk=
                static_cast<int>(ring.result.left.size())==expectRows &&
                static_cast<int>(ring.result.right.size())==expectRows &&
                static_cast<int>(ring.result.center.size())==expectRows;

            // 【修正3】只有"本帧图像质量可信 + 复用几何尺寸一致 + 本帧道路有效"时，
            // 才允许用环岛几何覆盖当前控制线。
            // 二值化质量差时不能用缓存的环岛线冒充本帧控制线：
            //   保留 lane 中本帧已算出的普通道路边界做短时兜底（若有），
            //   否则明确保持无效、走受限丢线处理，绝不用固定左偏无限沿用缓存。
            if(geomSizeOk && binaryQuality && lane.valid)
            {
                lane.left=ring.result.left;lane.right=ring.result.right;lane.center=ring.result.center;
                const int rows=static_cast<int>(lane.left.size());
                int repairBegin=0;
                if(ring.result.stage==2)
                {
                    // The entry controller clears both edges above its real upper anchor.
                    while(repairBegin<rows && lane.left[repairBegin]<0 && lane.right[repairBegin]<0) ++repairBegin;
                }
                else if(ring.result.stage==3 || ring.result.stage==4) repairBegin=rows/3;
                repairSingleEdge(lane,width,rows,c,false,repairBegin);
                // 【修正4】环岛阶段：正常窗口 [h/4,3h/4] 全空时（真实上锚点已移到下方），
                // 用近处最长连续有效段兜底，避免 lane.valid 被误判成 false。
                // 只在下 1/3 找，绝不把已经清掉的远处支路补回来；找不到仍为无效。
                lane.valid=averageCenter(lane.center,lane.aggregateCenter,
                                         rows*2/3, rows-1, 3);
                speedOverrideValue.store(c.ringSpeed);speedOverrideActive.store(true);
            }
            else
            {
                // 本帧证据不足（质量差 / 尺寸不匹配 / 道路无效）：
                // 不采用缓存环岛几何，改为受限兜底速度。
                if(lane.valid) {speedOverrideValue.store(c.ringApproachSpeed);speedOverrideActive.store(true);}
                else           {speedOverrideActive.store(false);}
                // 关闭本帧新的主动左探/入环助转（allowStart 已含 !paused，这里再兜一层）
                ring.gateOpen=false;
            }
        }
        else if(ring.markerResult.armed || ring.probe.controlsSteering())
        {speedOverrideValue.store(c.ringApproachSpeed);speedOverrideActive.store(true);}
        if(!wasComplete && ring.result.completed)
        {
            ring.zebraBlockUntil=now+std::chrono::milliseconds(c.ringZebraBlockAfterExitMs);
            ring.marker.reset();ring.markerResult={};ring.probe.reset();
        }
        if(ring.result.changed)
        {
            static const char* names[]={"普通巡线","初见环岛","入环","环内巡线","出环","出环直行"};
            const auto &e=ring.result.evidence;
            std::cout<<ANSI_RED<<"【环岛模式】state="<<ring.result.stage<<" "<<names[ring.result.stage]
                     <<" finder=MOORE_8 左下/中/上y="<<e.down.y<<"/"<<e.mid.y<<"/"<<e.up.y
                     <<" 右出口y="<<e.rightOut.y<<" 双丢行="<<e.bothMissingRows
                     <<" complete="<<ring.result.completed<<ANSI_RESET<<std::endl;
        }
    }

    static void startGarageFromSecondCrossing(const Config &c, const Clock::time_point now)
    {
        if(!c.garageAutoEnable) return;
        garage = GarageStateData{};
        garage.stage = GarageStage::ADVANCE;
        garage.stageAt = now;
        garageStageAtomic.store((int)garage.stage);
        std::cout << "[NewTrack][Garage] second crossing confirmed -> ADVANCE" << std::endl;
    }

    static bool updateCrossing(const ForkEvidence &fork, const Config &c)
    {
        crossing.justDetected = false;
        if(!c.crossingEnable || lapCounter.load() >= 2) return false;

        const auto now = frameNow();
        const long long sinceLap=std::chrono::duration_cast<std::chrono::milliseconds>(now-lapEpoch).count();
        const bool timeReady = sinceLap >= c.crossingStartupBlockMs;
        const bool startupReady = startupCrossingSynced || lapCounter.load()>0 || !c.startupCrossingSyncEnable;
        const bool ringCompleted = ring.controller.completed();
        // 环岛完成状态只做参考，默认不再作为计圈硬条件。
        // 这样即使某一圈环岛漏检，只要18秒后真正识别到车库倒Y，仍然可以正常计圈。
        const bool ringGateReady = !c.crossingRequireRingComplete || ringCompleted;
        const bool sequenceReady = ring.stage==RingStage::NORMAL && !ring.probe.controlsSteering() &&
            startupReady && timeReady && ringGateReady && (!c.crossingRequireZebra || zebra.seenThisLap);
        const long long nowMs=std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
        const bool forkAllowed=sequenceReady && now>=crossing.cooldownUntil &&
            TrackPolicy::afterZebraResume(nowMs,zebra.stopEndMs,c.crossingAfterZebraDelayMs) && !zebra.sequence.farArmed();
        const bool nearConfirmed=crossing.farNear.update(fork.visible,fork.nearVisible,nowMs,forkAllowed,
                                                       c.crossingFarConfirmFrames,c.crossingFarHoldMs);
        const auto decision=crossing.confirmation.update(forkAllowed,
                                                        nearConfirmed,nowMs,zebra.stopEndMs,
                                                        c.crossingAfterZebraDelayMs,c.crossingOpenConfirmFrames);
        crossing.armed=decision.ready;crossing.openConfirm=decision.confirmed;
        if(!decision.ready) return false;
        if(c.debugPrint)
        {
            static Clock::time_point lastForkPrint=Clock::time_point{};
            if(lastForkPrint==Clock::time_point{} || std::chrono::duration_cast<std::chrono::milliseconds>(now-lastForkPrint).count()>=700)
            {
                lastForkPrint=now;
                std::cout << "【倒Y交叉口观察】分叉行=" << fork.splitRows << "/" << c.crossingForkMinSplitRows
                          << " 近区分叉行=" << fork.nearSplitRows << "/" << c.crossingNearMinSplitRows
                          << " 最近分叉y=" << fork.nearestSplitY
                          << " 主干行=" << fork.stemRows << "/" << c.crossingForkMinStemRows
                          << " 最大分支数=" << fork.maxBranches
                          << " 最大分支间距=" << fork.maxSeparation
                          << " 环岛状态=" << (ringCompleted?"已完成":"未完成")
                          << (c.crossingRequireRingComplete?"(硬条件)":"(仅参考)")
                          << " 本圈时间=" << sinceLap << "ms/" << c.crossingStartupBlockMs << "ms"
                          << " 视频确认=" << crossing.openConfirm << "/" << c.crossingOpenConfirmFrames
                          << std::endl;
            }
        }
        if(!decision.detected) return false;

        crossing.justDetected = true;
        crossing.openConfirm = 0;
        crossing.cooldownUntil = now + std::chrono::milliseconds(c.crossingCooldownMs);
        crossing.armed = false;crossing.farNear.reset();

        const int nextLap = std::min(2, lapCounter.load()+1);
        lapCounter.store(nextLap);
        coneDetector.reset();coneSequence.reset();conePulseUntilNs.store(0);coneSequenceActive.store(false);coneInputStop.store(false);coneGuideStop.store(false);
        lapEpoch = now; // 下一圈的可选环岛屏蔽、人行横道12秒从这里重新计时（实际值仍以参数文件为准）。
        startupCrossingSynced = true;

        zebra = ZebraState{};

        if(nextLap < 2)
        {
            // 第一圈完成：继续第二圈，解除上一圈环岛完成锁存。
            ring = RingState{};
            ringStageAtomic.store((int)RingStage::NORMAL);
            speedOverrideActive.store(false);
        }
        else
        {
            // 第二圈完成：不再开放第三圈环岛。
            ring.stage = RingStage::NORMAL;
            ringStageAtomic.store(0);
            startGarageFromSecondCrossing(c, now);
            if(c.finishStopEnable && !c.garageAutoEnable)
            {
                finishStopPending=true;
                finishStopped=false;
                finishStopAt=now+std::chrono::milliseconds(c.finishStopDelayMs);
                std::cout << ANSI_RED
                          << "!!!【第二圈停车倒计时】倒Y交叉口已确认，继续前进"
                          << c.finishStopDelayMs << "毫秒后自动把左右电机目标置0并保持停车；程序不会自动退出!!!"
                          << ANSI_RESET << std::endl;
            }
        }

        std::cout << ANSI_RED;
        if(nextLap==1)
            std::cout << "!!!【检测到车库右侧交叉口】第一圈完成：直行不入库；第二圈计时已清零，"
                      << c.ringStartupBlockMs/1000.0 << "秒后开放环岛门控，"
                      << c.zebraStartupBlockMs/1000.0 << "秒后开放人行横道检测!!!";
        else
        {
            std::cout << "!!!【检测到车库右侧倒Y交叉口】第二圈完成";
            if(c.finishStopEnable && !c.garageAutoEnable)
                std::cout << "：继续前进" << c.finishStopDelayMs << "毫秒后自动停车!!!";
            else
                std::cout << "：已到倒车入库触发点!!!";
        }
        std::cout << ANSI_RESET << std::endl;
        return true;
    }

    static void updateGarage(const BlackLineResult &black, const Config &c,
                             int imageHeight, int imageWidth, double centerPratio,
                             float &error)
    {
        if(!c.garageAutoEnable)
        {
            if(garage.stage!=GarageStage::DONE) garage.stage=GarageStage::IDLE;
            garageStageAtomic.store((int)garage.stage);
            return;
        }
        if(garage.stage==GarageStage::IDLE || garage.stage==GarageStage::ARMED)
        {
            garageStageAtomic.store((int)garage.stage);
            return;
        }

        const auto now=frameNow();
        const long long e=std::chrono::duration_cast<std::chrono::milliseconds>(now-garage.stageAt).count();
        motorStopForced.store(false);
        if(garage.stage==GarageStage::ADVANCE)
        {
            speedOverrideValue.store(c.garageSpeed); speedOverrideActive.store(true);
            if(e>=c.garageForwardMs){garage.stage=GarageStage::TURN_RIGHT;garage.stageAt=now;}
        }
        else if(garage.stage==GarageStage::TURN_RIGHT)
        {
            speedOverrideValue.store(c.garageTurnSpeed); speedOverrideActive.store(true);
            error=(float)c.garageRightTurnError;
            if(e>=c.garageRightTurnMs){garage.stage=GarageStage::REVERSE_SEARCH;garage.stageAt=now;garage.blackConfirm=0;}
        }
        else if(garage.stage==GarageStage::REVERSE_SEARCH)
        {
            speedOverrideValue.store(-c.garageReverseSpeed);speedOverrideActive.store(true);
            if(black.reliable)
            {
                garage.blackConfirm++;
                const double blackForwardError = centerToError(black.weightedX,imageHeight,imageWidth,centerPratio);
                error=(float)(-blackForwardError*c.garageReverseBlackGain);
            }
            else garage.blackConfirm=0;
            if(garage.blackConfirm>=c.garageBlackConfirmFrames || e>=c.garageReverseSearchMs){garage.stage=GarageStage::REVERSE;garage.stageAt=now;}
        }
        else if(garage.stage==GarageStage::REVERSE)
        {
            speedOverrideValue.store(-c.garageReverseSpeed);speedOverrideActive.store(true);
            if(black.reliable)
            {
                const double blackForwardError = centerToError(black.weightedX,imageHeight,imageWidth,centerPratio);
                error=(float)(-blackForwardError*c.garageReverseBlackGain);
            }
            if(e>=c.garageReverseMs){garage.stage=GarageStage::STRAIGHTEN;garage.stageAt=now;}
        }
        else if(garage.stage==GarageStage::STRAIGHTEN)
        {
            speedOverrideValue.store(-c.garageReverseSpeed);speedOverrideActive.store(true);
            error=(float)c.garageStraightenError;
            if(e>=c.garageStraightenMs)
            {
                garage.stage=GarageStage::DONE; garage.stageAt=now;
                motorStopForced.store(true); speedOverrideActive.store(false);
                std::cout<<"[NewTrack][Garage] DONE motor stop"<<std::endl;
            }
        }
        else if(garage.stage==GarageStage::DONE)
        {
            motorStopForced.store(true); speedOverrideActive.store(false);
        }
        garageStageAtomic.store((int)garage.stage);
    }

}

std::string diagnosticCsvHeader()
{
    return "valid,ring_stage,ring_complete,probe_phase,probe_dx,probe_rows,probe_border,probe_width_delta,probe_width_rows,marker_blocks,marker_armed,left_down_y,left_mid_y,left_up_y,right_out_y,source,target_x,steer_error,speed_command,stop_command,center_black_used,black_rows,cone_side,cone_active,cone_detected,cone_held,cone_blocked,cone_x,cone_y,cone_w,cone_h,cone_changed_rows,zebra_far,zebra_seen,zebra_triggered,zebra_stopping,fork_far,near_fork_rows,crossing_confirmed,lap,lap_ms,garage_stage,left_by_y,right_by_y,center_by_y,black_by_y,binary_threshold,binary_white_percent,binary_quality,ring_gate_open,low_camera_candidate,low_camera_confirmed,virtual_down_x,virtual_down_y,right_entry_x,right_entry_y,left_down_x,left_mid_x,left_up_x,right_out_x,raw_left_by_y,raw_right_by_y,ring_observation_paused,cone_confirmed,cone_confirmation_count,cone_blue_pixels,cone_components,cone_rejected_size,cone_rejected_road,cone_reason,cone_phase,cone_remaining_ms,cone_sequence_used,cone_tuning_valid,stop_reason,lane_valid,cone_route_complete,cone_entry_reason,cone_visual_reason,cone_near_seen,cone_gray_base_threshold,cone_control_mode,cone_pulse_remaining_ms";
}

// ============================================================================
// 【修正6 / A1】统一的诊断 CSV 生成。
//
// 目的：有效帧和无效帧都走同一份字段布局，保证
//       FrameResult / CSV / 四宫格三者来自同一份最终状态。
//
// A1 约定：原有 62 个字段的顺序【完全不变】，只在末尾追加 ring_observation_paused。
// 无效帧：二值化/门控/点/原始边界照常写入；只有目标与转向（target_x/steer_error/
//         speed_command）明确留空，表示"本帧没有可信目标"，而不是伪造 0。
// ============================================================================
namespace
{
    static std::string coneControlSource() {
        if(coneSequence.needsStop()) return "CONE_FAULT";
        if(!coneSequence.inRoute()) return ConeSequence::name(coneSequence.phase());
        return std::string(coneSequence.forcing()?"CONE_PULSE_":coneGuideStop.load()?"CONE_GUIDE_STOP_":"CONE_GUIDE_")+
            ConeSequence::name(coneSequence.phase());
    }
    struct CsvRow
    {
        bool valid=false,laneValid=false;
        int ringStage=0, garageStage=0;
        bool ringComplete=false;
        int probePhase=0, probeDx=0, probeRows=0, probeBorder=0, probeWidthDelta=0, probeWidthRows=0;
        int markerBlocks=0; bool markerArmed=false;
        int leftDownY=-1, leftMidY=-1, leftUpY=-1, rightOutY=-1;
        std::string source="NONE", coneSide="NONE";
        double targetX=0.0, speedCommand=0.0; float steerError=0.0f;
        bool stopCommand=false, centerBlackUsed=false; int blackRows=0,coneGrayThreshold=0;
        bool coneActive=false, coneDetected=false, coneHeld=false, coneBlocked=false;
        int coneX=0, coneY=0, coneW=0, coneH=0, coneChangedRows=0;
        bool zebraFar=false, zebraSeen=false, zebraTriggered=false, zebraStopping=false;
        bool forkFar=false; int nearForkRows=0; bool crossingConfirmed=false;
        int lap=0, lapMs=0;
        const std::vector<int> *leftByY=nullptr,*rightByY=nullptr,*centerByY=nullptr,*blackByY=nullptr;
        int binaryThreshold=-1; double binaryWhitePercent=0.0; bool binaryQuality=false;
        bool ringGateOpen=false, lowCameraCandidate=false, lowCameraConfirmed=false;
        int virtualDownX=-1, virtualDownY=-1;
        int rightEntryX=-1, rightEntryY=-1, leftDownX=-1, leftMidX=-1, leftUpX=-1, rightOutX=-1;
        const std::vector<int> *rawLeftByY=nullptr,*rawRightByY=nullptr;
        bool ringObservationPaused=false;   // 【A1】末尾追加
        ConeBoundary::Result cone;bool coneTuningValid=true;
    };

    inline void csvAppendArray(std::ostringstream &csv, const std::vector<int> *a)
    {
        csv<<',';
        if(!a) return;
        for(std::size_t i=0;i<a->size();++i){if(i)csv<<';';csv<<(*a)[i];}
    }

    std::string buildDiagnosticCsv(const CsvRow &r)
    {
        std::ostringstream csv;
        csv<<std::setprecision(17); // 与原实现一致：保留最终控制浮点值的精确快照
        csv<<(r.valid?1:0)<<','<<r.ringStage<<','<<r.ringComplete<<','<<r.probePhase<<','
           <<r.probeDx<<','<<r.probeRows<<','<<r.probeBorder<<','
           <<r.probeWidthDelta<<','<<r.probeWidthRows<<','
           <<r.markerBlocks<<','<<r.markerArmed<<','
           <<r.leftDownY<<','<<r.leftMidY<<','<<r.leftUpY<<','<<r.rightOutY<<','
           <<r.source<<',';
        // 【修正6】无效帧的目标/转向明确留空（字段仍占位，列数保持一致）
        if(r.valid) csv<<r.targetX<<','<<r.steerError<<','<<r.speedCommand;
        else        csv<<",,";
        csv<<','<<r.stopCommand<<','<<r.centerBlackUsed<<','<<r.blackRows<<','
           <<r.coneSide<<','<<r.coneActive<<','<<r.coneDetected<<','<<r.coneHeld<<','<<r.coneBlocked<<','
           <<r.coneX<<','<<r.coneY<<','<<r.coneW<<','<<r.coneH<<','<<r.coneChangedRows<<','
           <<r.zebraFar<<','<<r.zebraSeen<<','<<r.zebraTriggered<<','<<r.zebraStopping<<','
           <<r.forkFar<<','<<r.nearForkRows<<','<<r.crossingConfirmed<<','<<r.lap<<','<<r.lapMs<<','
           <<r.garageStage;
        csvAppendArray(csv,r.leftByY);csvAppendArray(csv,r.rightByY);
        csvAppendArray(csv,r.centerByY);csvAppendArray(csv,r.blackByY);
        csv<<','<<r.binaryThreshold<<','<<r.binaryWhitePercent<<','<<r.binaryQuality<<','<<r.ringGateOpen<<','
           <<r.lowCameraCandidate<<','<<r.lowCameraConfirmed<<','<<r.virtualDownX<<','<<r.virtualDownY<<','
           <<r.rightEntryX<<','<<r.rightEntryY<<','<<r.leftDownX<<','<<r.leftMidX<<','<<r.leftUpX<<','<<r.rightOutX;
        csvAppendArray(csv,r.rawLeftByY);csvAppendArray(csv,r.rawRightByY);
        csv<<','<<r.ringObservationPaused;
        csv<<','<<r.cone.confirmed<<','<<r.cone.confirmations<<','<<r.cone.bluePixels<<','<<r.cone.components
           <<','<<r.cone.rejectedSize<<','<<r.cone.rejectedRoad<<','<<r.cone.reason
           <<','<<ConeSequence::name(coneSequence.phase())<<','<<coneSequence.remainingMs()
           <<','<<coneSequence.used()<<','<<r.coneTuningValid;
        const char* stopReason=!r.stopCommand?"NONE":gMotorStop?"OPERATOR_STOP":!r.coneTuningValid?"CONE_TUNING_INVALID":
            coneSequence.needsStop()?"CONE_VISUAL_TIMEOUT":coneStopForced.load()?"CONE_CORRIDOR_BLOCKED":coneGuideStop.load()?"CONE_GUIDANCE_UNAVAILABLE":
            coneInputStop.load()?"CONE_INPUT_OR_PAUSE":lostLaneStopForced.load()?"LANE_LOST":
            ringSideStopForced.load()?"RING_SIDE_STOP":ringEntryStopForced.load()?"RING_ENTRY_STOP":
            motorStopForced.load()?"TASK_STOP":"ZEBRA_OR_STALE_CAMERA";
        csv<<','<<stopReason<<','<<r.laneValid<<','<<coneSequence.routeComplete()<<','<<coneSequence.entryReason()<<','<<coneSequence.reason()<<','<<coneSequence.nearSeen()<<','<<r.coneGrayThreshold<<','<<(coneSequence.forcing()?"PULSE":coneSequence.guiding()?(coneGuideStop.load()?"GUIDE_STOP":"GUIDE"):"NONE")<<','<<coneSequence.pulseRemainingMs();   // 【A1】末尾追加，前面顺序不动
        return csv.str();
    }
}

bool enabled(){return cfg().enable!=0;}

static FrameResult processFrameImpl(const cv::Mat &frameBlock, bool buildDebugFrame)
{
    FrameResult out; const Config c=cfg();
    if(!c.enable) return out;
    const long long coneNowMs=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow().time_since_epoch()).count();
    coneLastFrameNs.store(coneNowMs*1000000);
    const bool coneSequenceEnabled=c.coneSequenceSettings.enabled;
    if(coneLaunchAtMs<0) coneLaunchAtMs=coneNowMs;
    if(frameBlock.empty())
    {
        coneDetector.reset();
        coneSequence.update(coneNowMs,lapCounter.load(),coneSequenceEnabled,0,false,c.coneSequenceSettings,{});
        coneSequenceActive.store(coneSequence.active());
        coneGuideStop.store(false);coneInputStop.store(coneSequenceEnabled);
        if(processStarted)
        {
            const auto now=frameNow();
            const long long ms=std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count();
            const long long delta=ring.lastFrameMs>0?std::max(0LL,ms-ring.lastFrameMs):0;
            if(c.coneSequenceSettings.enabled && coneSequence.routeComplete()) ring.routePausedMs+=delta;
            const bool physicalPause=finishStopped || coneStopForced.load() || ms*1000000<zebraStopUntilNs.load();
            if(ring.taskAtMs>0) ring.taskAtMs+=delta;
            if(ring.entryAssistUntilMs>0) ring.entryAssistUntilMs+=delta;
            if(physicalPause || ring.physicalPausePreviously) lapEpoch+=std::chrono::milliseconds(delta);
            ring.lastFrameMs=ms;ring.pausedPreviously=true;ring.physicalPausePreviously=physicalPause;ring.gateOpen=false;
            auto settings=c.ringSettings;settings.entryValidationRequired=c.ringProbeEnable!=0;
            ring.controller.update({}, {},0,ms,false,c.workMode==1 && c.ringEnable,settings,{}, {},false,true);
            ring.probe.update(false,false,false,ring.controller.stage()>=2,ring.controller.completed(),
                c.workMode==1 && c.ringEnable && c.ringProbeEnable,true,{},0,ms,c.probeSettings);
            ring.result.evidence={};out.ringStage=ring.stage;out.garageStage=garage.stage;
        }
        return out;
    }
    cv::Mat frame=cropSafe(frameBlock,c.topCropRatio,c.bottomCropRatio,c.leftCropRatio,c.rightCropRatio);
    const auto binary=Binarization::process(frame,c.binarySettings);
    cv::Mat rawMask=binary.mask;
    out.binaryThreshold=binary.threshold;out.binaryWhitePercent=binary.whitePercent;out.binaryQuality=binary.quality;
    if(rawMask.empty()) return processFrameImpl(cv::Mat(),buildDebugFrame);

    cv::Mat boundaryMask=rawMask.clone();
    int k=c.boundaryCloseKernel; if(k%2==0) ++k; k=std::max(1,k);
    if(k>1){cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(k,1));cv::morphologyEx(boundaryMask,boundaryMask,cv::MORPH_CLOSE,kernel);}

    const int laneMethod=c.workMode==1?4:c.laneMethod;
    LaneLines lane=extractLane(boundaryMask,laneMethod,RuntimeConfig::getCenterPratio(),c);
    if(!processStarted && lane.valid && binary.quality)
    {
        processStarted=true;
        processStart=frameNow();
        lapEpoch=processStart;
        if(coneSequenceEnabled)
            std::cout << "[ConeRoute] BOUNDED_PULSE_GRAY_PASS entry=" << c.coneSequenceSettings.entryTimeMs
                      << "-" << c.coneSequenceSettings.entryEarlyMs << "+" << c.coneSequenceSettings.entryLateMs
                      << "ms from first control frame after START; L/R/L/R, bounded pulse then visual guide/pass, wait=0" << std::endl;
        if(!coneSequenceEnabled) std::cout << "【发车】先寻找起点倒Y交叉口；若未识别，约" << c.startupCrossingFallbackMs/1000.0
                  << "秒后自动按已通过起点交叉口处理。同步后：" << c.ringStartupBlockMs/1000.0
                  << "秒开放环岛门控，" << c.zebraStartupBlockMs/1000.0
                  << "秒开放人行横道，" << c.crossingStartupBlockMs/1000.0
                  << "秒开放下一次车库交叉口计圈。" << std::endl;
    }

    if(c.workMode==1 && c.ringEnable) extractRingEvidence(rawMask,lane);
    const LaneLines observationLane=lane; // Detectors use ordinary road bounds before circle repair.
    ConeBoundary::Result blueResult;
    LaneLines coneLane=lane;
    bool blueAvoidActive=false;
    if(!coneSequenceEnabled && ring.stage==RingStage::NORMAL && !ring.markerResult.armed && !ring.probe.controlsSteering() && !zebra.seenThisLap)
        blueAvoidActive=applyBlueObstacleToLane(frame,coneLane,c,blueResult);
    else {coneDetector.reset();blueResult.reason=coneSequenceEnabled?"GRAY_SHAPE":c.blueConeEnable?"TASK_GATE":"DISABLED";}
    coneStopForced.store(blueResult.blocked);
    GrayCone::Result grayCone;
    if(coneSequenceEnabled) {
        const auto& settings=coneSequence.used()?coneSequence.settings():c.coneSequenceSettings;
        grayCone=GrayCone::detect(frame,settings);
        grayCone.observation.roadValid=lane.valid && binary.quality && lane.bilateralRows>=rawMask.rows/3;
        grayCone.observation.roadCenter=lane.aggregateCenter*100.0/rawMask.cols;
        blueResult.detected=!grayCone.observation.candidates.empty();
        blueResult.components=grayCone.components;blueResult.rejectedSize=grayCone.rejected;
        blueResult.reason=grayCone.observation.valid?"GRAY_SHAPE":"GRAY_FRAME_INVALID";
        if(blueResult.detected) {
            const auto& candidate=grayCone.observation.candidates.front();
            blueResult.box=cv::Rect(int(candidate.x*rawMask.cols/100),int(candidate.y*rawMask.rows/100),
                int(candidate.w*rawMask.cols/100),int(candidate.h*rawMask.rows/100));
            blueResult.side=candidate.center()<45?ConeBoundary::Side::LEFT:candidate.center()>55?ConeBoundary::Side::RIGHT:ConeBoundary::Side::CENTER;
        }
    }
    const long long routeOrigin=lapCounter.load()==0?coneLaunchAtMs:
        std::chrono::duration_cast<std::chrono::milliseconds>(lapEpoch.time_since_epoch()).count();
    const long long routeMs=coneNowMs-routeOrigin;
    const bool externallyStopped=gMotorStop || finishStopped || motorStopForced.load() ||
        coneStopForced.load() || ringSideStopForced.load() || ringEntryStopForced.load() ||
        coneNowMs*1000000<zebraStopUntilNs.load();
    const bool sequenceAllowed=c.coneTuningValid && !externallyStopped && (!coneSequenceEnabled || grayCone.observation.valid) &&
        !(coneSequence.routeComplete() && !lane.valid);
    const auto previousConePhase=coneSequence.phase();
    const int previousConeMode=coneSequence.forcing()?1:coneSequence.guiding()?(coneGuideStop.load()?3:2):0;
    coneSequence.update(coneNowMs,lapCounter.load(),coneSequenceEnabled,routeMs,sequenceAllowed,c.coneSequenceSettings,grayCone.observation);
    if(coneSequence.inRoute()) {
        lostLaneStopForced.store(false);lostLaneStopActive=false;lostLaneCount=0;lostLaneRecover=0;
    }
    GrayCone::Guidance coneGuidance;
    if(coneSequence.guiding())
        coneGuidance=GrayCone::guide(grayCone.observation,coneSequence.currentTarget(),coneSequence.phase(),
            observationLane.left,observationLane.right,rawMask.cols,c.coneSettings.marginPx80,c.coneSettings.minCorridorPx80);
    coneGuideStop.store(coneSequence.guiding() && !coneGuidance.valid);
    if(coneSequenceEnabled && coneGuidance.blocked) {blueResult.blocked=true;coneStopForced.store(true);}
    coneSequence.reportDriveAllowed(sequenceAllowed && !coneGuideStop.load() && !coneSequence.needsStop());
    conePulseUntilNs.store(coneSequence.forcing()?(coneNowMs+coneSequence.pulseRemainingMs())*1000000:0);
    coneSequenceActive.store(coneSequence.active());
    coneInputStop.store(coneSequenceEnabled && (!c.coneTuningValid || coneSequence.needsStop() || coneGuideStop.load() ||
        (coneSequence.inRoute() && !sequenceAllowed) || (coneSequence.routeComplete() && !lane.valid)));
    const int currentConeMode=coneSequence.forcing()?1:coneSequence.guiding()?(coneGuideStop.load()?3:2):0;
    if(previousConePhase!=coneSequence.phase() || previousConeMode!=currentConeMode)
        std::cout<<"[ConeRoute] phase="<<ConeSequence::name(coneSequence.phase())<<" control="<<coneControlSource()
                 <<" entry="<<coneSequence.entryReason()<<" reason="<<coneSequence.reason()
                 <<" pulse_remaining_ms="<<coneSequence.pulseRemainingMs()<<" route_ms="<<routeMs<<std::endl;
    const bool coneControls=blueResult.confirmed || coneSequence.inRoute();
    const bool ringRouteBlocked=coneSequenceEnabled && !coneSequence.routeComplete();
    updateRing(rawMask,lane,rawMask.cols,c,(!coneSequenceEnabled && (blueResult.detected || blueResult.held)) || coneSequence.inRoute(),binary.quality,frame);
    // 【环岛右侧多模板触发】在原生 80x56 灰度图上匹配 f033~f039 那 7 张模板。
    // 触发后进入 完全断电停车 -> 强制左转 的固定序列，并全程把速度压到 ringSideSpeed。
    {
        cv::Mat grey80;
        if(frame.channels()==1) grey80=frame;
        else if(frame.channels()==3) cv::cvtColor(frame,grey80,cv::COLOR_BGR2GRAY);
        const long long sideMs=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow().time_since_epoch()).count();
        const long long sideLapMs=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow()-lapEpoch).count();
        if(!grey80.empty()) ringSideUpdate(grey80,c,sideLapMs,sideMs,!coneControls && !ringRouteBlocked);
    }
    // 【环岛入口定位·虚线段判据】用运行时原始二值图（和 diagnostics 的 RAW BINARY 同源）。
    {
        const long long cueMs=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow().time_since_epoch()).count();
        const long long cueLapMs=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow()-lapEpoch).count();
        ringEntryCueUpdate(rawMask,c,cueLapMs,cueMs,!coneControls && !ringRouteBlocked);
    }
    // 【修正6】lowCameraCandidate / lowCameraConfirmed / virtualDown / ringGateOpen
    // 不再在这里写进 out。原因见交接文档：
    //   updateCrossing() 在本行之后还可能复位 ring，
    //   若此处先赋值，就会出现 FrameResult 用的是复位前的值、
    //   而 CSV 读的是复位后的值 —— 两者不一致。
    // 现在统一挪到函数末尾的"最终状态汇总"处，与 CSV / 四宫格同源。
    if(ring.stage==RingStage::NORMAL && !ring.probe.controlsSteering() && blueAvoidActive && !coneSequence.inRoute()) lane=coneLane;
    else if(ring.stage!=RingStage::NORMAL || ring.probe.controlsSteering()) blueAvoidActive=false;
    if(!lane.valid)
    {
        // 【修正6】无效 lane 分支：不再只写 "0,stage + 空格"。
        // 仍然给出本帧可用的二值化 / 门控 / 点 / 原始边界信息；
        // 只有目标与转向（target_x / steer_error / speed_command）明确留空。
        // 字段布局与有效帧完全一致 —— 两边都调同一个 buildDiagnosticCsv()。
        out.ringStage=ring.stage;out.garageStage=garage.stage;
        out.valid=false;

        CsvRow row;row.cone=blueResult;row.coneTuningValid=c.coneTuningValid;row.coneGrayThreshold=grayCone.threshold;
        const bool routeSteering=coneSequence.forcing();
        out.valid=routeSteering;out.steerError=routeSteering && !forceMotorStop() && !gMotorStop?(float)coneSequence.error():0.0f;
        row.valid=routeSteering;row.laneValid=false;
        row.stopCommand=forceMotorStop() || gMotorStop;
        if(routeSteering) {
            speedOverrideValue.store(coneSequence.speed());speedOverrideActive.store(true);
            row.targetX=-1;row.steerError=out.steerError;row.speedCommand=row.stopCommand?0:coneSequence.speed();
        }
        row.ringStage=static_cast<int>(ring.stage);
        row.garageStage=static_cast<int>(garage.stage);
        row.ringComplete=ring.result.completed;
        row.probePhase=ring.probe.phase();
        row.probeDx=ring.probeResult.deltaX;
        row.probeRows=ring.probeResult.sampleRows;
        row.probeBorder=ring.probeResult.clippedRows;
        row.probeWidthDelta=ring.probeResult.deltaWidth;
        row.probeWidthRows=ring.probeResult.widthRows;
        row.markerBlocks=ring.markerEvidence.blocks;
        row.markerArmed=ring.markerResult.armed;
        row.source=(coneSequence.inRoute() || coneSequence.needsStop())?coneControlSource():"INVALID_LANE";
        row.coneSide=ConeBoundary::sideName(blueResult.side);
        row.coneX=blueResult.box.x;row.coneY=blueResult.box.y;
        row.coneW=blueResult.box.width;row.coneH=blueResult.box.height;
        row.coneActive=blueAvoidActive;
        row.coneDetected=blueResult.detected;
        row.coneHeld=blueResult.held;
        row.coneBlocked=blueResult.blocked;
        row.leftDownY=ring.result.evidence.down.y;
        row.leftMidY=ring.result.evidence.mid.y;
        row.leftUpY=ring.result.evidence.up.y;
        row.rightOutY=ring.result.evidence.rightOut.y;
        row.binaryThreshold=binary.threshold;
        row.binaryWhitePercent=binary.whitePercent;
        row.binaryQuality=binary.quality;
        row.ringGateOpen=ring.gateOpen;
        row.lowCameraCandidate=ring.result.evidence.lowCameraEntry;
        row.lowCameraConfirmed=ring.result.lowCameraEntryConfirmed;
        row.virtualDownX=ring.result.evidence.virtualDown.x;
        row.virtualDownY=ring.result.evidence.virtualDown.y;
        row.rightEntryX=ring.result.evidence.rightEntry.x;
        row.rightEntryY=ring.result.evidence.rightEntry.y;
        row.leftDownX=ring.result.evidence.down.x;
        row.leftMidX=ring.result.evidence.mid.x;
        row.leftUpX=ring.result.evidence.up.x;
        row.rightOutX=ring.result.evidence.rightOut.x;
        row.leftByY=&lane.left;row.rightByY=&lane.right;row.centerByY=&lane.center;
        row.rawLeftByY=&lane.rawLeft;row.rawRightByY=&lane.rawRight;
        row.lap=lapCounter.load();
        row.ringObservationPaused=ring.observationPaused;

        // 同步到 FrameResult，保证四宫格与 CSV 用的是同一份状态
        out.lowCameraCandidate=row.lowCameraCandidate;
        out.lowCameraConfirmed=row.lowCameraConfirmed;
        out.virtualDownX=row.virtualDownX;out.virtualDownY=row.virtualDownY;
        out.ringGateOpen=row.ringGateOpen;
        out.binaryThreshold=row.binaryThreshold;
        out.binaryWhitePercent=row.binaryWhitePercent;
        out.binaryQuality=row.binaryQuality;
        out.markerBlocks=row.markerBlocks;out.markerArmed=row.markerArmed;
        out.ringObservationPaused=row.ringObservationPaused;

        if(buildDebugFrame)
        {
            DiagnosticView::Snapshot snapshot;snapshot.left=lane.left;snapshot.right=lane.right;snapshot.center=lane.center;
            snapshot.blueMask=blueResult.blueMask;snapshot.coneBox=blueResult.box;snapshot.coneHeld=blueResult.held;
            snapshot.lines={routeSteering?std::string("CONE PULSE: ")+ConeSequence::name(coneSequence.phase())+"; no visual lane":coneSequence.guiding()?"CONE GUIDE STOP: road/vision unavailable; steering correction=0":"INVALID LANE: steering result=0; task observations were not updated",
                "ring="+std::to_string(static_cast<int>(ring.stage))+" marker="+std::to_string(ring.markerResult.armed),
                "cone="+std::string(blueResult.reason)+" count="+std::to_string(blueResult.confirmations)+" blue="+std::to_string(blueResult.bluePixels),
                "This is the current camera frame, not a cached successful result."};
            out.debugFrame=DiagnosticView::render(frame,rawMask,boundaryMask,snapshot);
            out.diagnosticCsv=buildDiagnosticCsv(row);
        }
        return out;
    }
    cv::Mat blackInput=rawMask;
    if(blueResult.confirmed && blueResult.box.area()>0) {
        blackInput=rawMask.clone();blackInput(blueResult.box).setTo(255);
    }
    BlackLineResult black=detectCenterBlack(blackInput,lane,c);
    if(coneControls) black.reliable=false;

    double targetX=lane.aggregateCenter;
    bool partialBlackApplied=false;
    const bool ringActive=ring.result.active || ring.probe.controlsSteering();
    if(ringActive)
    {
        targetX=lane.aggregateCenter; // Only the repaired geometric center commands steering.
    }
    else if(blueAvoidActive || coneSequence.inRoute())
    {
        // 蓝锥属于任务区，优先级高于普通中心黑线。
        targetX=coneSequence.guiding() && coneGuidance.valid?coneGuidance.target:lane.aggregateCenter;
        speedOverrideValue.store(coneSequence.inRoute()?coneSequence.speed():c.blueConeSpeed);
        speedOverrideActive.store(true);
    }
    else if(c.centerBlackEnable && black.reliable)
    {
        targetX=black.weightedX;
        out.centerBlackReliable=true;
    }
    else
    {
        // 普通转弯单边界兜底：只剩右边界 -> 目标再向左给少量偏置；只剩左边界则反向。
        // 如果此时还能看到少量中心黑线，则优先朝这段黑线收敛，但使用混合而不是瞬间跳过去。
        const double scale=rawMask.cols/160.0;
        if(lane.rightOnlyRows>lane.leftOnlyRows && lane.rightOnlyRows>=3)
            targetX-=c.singleEdgeBiasPx160*scale;
        else if(lane.leftOnlyRows>lane.rightOnlyRows && lane.leftOnlyRows>=3)
            targetX+=c.singleEdgeBiasPx160*scale;

        const bool partialBlack = c.partialBlackAssistEnable && lane.singleEdgeUsed &&
            black.validRows>=c.partialBlackMinValidRows &&
            black.nearRows>=c.partialBlackMinNearRows;
        if(partialBlack)
        {
            partialBlackApplied=true;
            const double a=c.partialBlackBlendPercent/100.0;
            targetX=targetX*(1.0-a)+black.weightedX*a;
        }
    }

    // 【转向目标限速】参考 cc 版：中心黑线不可靠时，限制 targetX 每帧最多变化
    // steerTargetMaxStepPx 像素，把"一帧跳几十像素 -> error 跳 ±200 度"的抖动掐掉。
    // 注意：黑线可靠时直接跟线（不抹平真弯道），环岛/蓝锥阶段也不限速。
    if(!black.reliable && c.steerTargetMaxStepPx>0 && lastTargetX>=0 &&
       !ringActive && !blueAvoidActive && !coneSequence.inRoute())
    {
        targetX=clampd(targetX,lastTargetX-c.steerTargetMaxStepPx,
                                lastTargetX+c.steerTargetMaxStepPx);
    }
    lastTargetX=targetX;

    float error=(float)centerToError(targetX,rawMask.rows,rawMask.cols,RuntimeConfig::getCenterPratio());
    // 仅第一次发车出库：短时右偏。正误差在这辆旧车代码里对应右转。
    const auto startupNow=frameNow();
    const long long startupElapsed=std::chrono::duration_cast<std::chrono::milliseconds>(startupNow-processStart).count();
    const bool startupRightActive=c.startupRightEnable && lapCounter.load()==0 &&
        startupElapsed>=0 && startupElapsed<c.startupRightDurationMs && !ringActive;
    if(startupRightActive)
    {
        error=(float)c.startupRightError; // Fixed small departure command; do not amplify raw lane error.
        speedOverrideValue.store(c.startupRightSpeed);
        speedOverrideActive.store(true);
    }

    // 【丢线停车】参考 cc 版：中心黑线和白路边界都没有时，连续 lostLaneStopFrames 帧
    // 后强制停车（防止丢线时车乱冲），恢复 lostLaneRecoverFrames 帧才解除。
    // 只在普通巡线段生效：环岛/蓝锥阶段有自己的转向和速度控制，不能在这里踩刹车。
    {
        const bool laneLost = !black.reliable &&
            lane.leftOnlyRows==0 && lane.rightOnlyRows==0 && lane.bilateralRows<3;
        if(c.lostLaneStopFrames>0 && !ringActive && !blueAvoidActive && !coneSequence.inRoute())
        {
            if(laneLost) { if(lostLaneCount<10000) ++lostLaneCount; lostLaneRecover=0; }
            else { lostLaneCount=0; if(lostLaneStopActive) ++lostLaneRecover; }
            if(lostLaneCount>=c.lostLaneStopFrames) lostLaneStopActive=true;
            if(lostLaneStopActive && lostLaneRecover>=std::max(1,c.lostLaneRecoverFrames))
                lostLaneStopActive=false;
        }
        else
        {
            lostLaneCount=0; lostLaneRecover=0; lostLaneStopActive=false;
        }
        lostLaneStopForced.store(lostLaneStopActive);
    }

    if(ring.probe.phase()==2)
        error=(float)clampd(error-c.probeSettings.leftError,-45.0,-c.probeSettings.leftError);
    else if(ring.probe.phase()==3)
        error=(float)c.probeSettings.rightError;
    else if(ring.stage==RingStage::ENTRY && c.ringProbeEnable && ring.entryAssistUntilMs>0)
    {
        const long long ms=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow().time_since_epoch()).count();
        if(ms<ring.entryAssistUntilMs)
            error=(float)clampd(error-c.probeSettings.leftError,-45.0,-c.probeSettings.leftError);
    }
    // 【强制左转】进环后的一段固定左打窗口：直接覆盖 error 与速度，
    // 优先级高于上面的探针/入环辅助。用于解决"刚进环就被出环判据拉出去"。
    // force_left_ms=0 时完全不生效（默认）。
    if(ring.result.forceLeft)
    {
        error=(float)ring.result.forceLeftError;
        speedOverrideValue.store(ring.result.forceLeftSpeed);
        speedOverrideActive.store(true);
    }
    // 【环岛右侧多模板】固定序列的覆盖。优先级最高 —— 认出环岛后由它接管：
    //   phase1 STOP : 速度0 + 完全断电停车（ringSide.stopForced 会进 forceMotorStop）
    //   phase2 LEFT : error 固定 -60度，速度压到 ringSideSpeed(2)
    if(c.ringSideEnable)
    {
        if(ringSide.phase==1)
        {
            speedOverrideValue.store(0.0);
            speedOverrideActive.store(true);
        }
        else if(ringSide.phase==2)
        {
            error=(float)c.ringSideLeftError;
            speedOverrideValue.store(c.ringSideSpeed);
            speedOverrideActive.store(true);
        }
    }
    const auto forkEv=crossingForkEvidence(rawMask,observationLane,c);

    // 发车起点倒Y交叉口同步：它只用于确定“第一圈计时起点”，绝不算作完成一圈。
    // 若视觉看到倒Y宽口则立即同步；若漏检，约2.8秒后按车辆必然已驶过起点交叉口自动同步。
    if(c.startupCrossingSyncEnable && !startupCrossingSynced && lapCounter.load()==0)
    {
        const auto syncNow=frameNow();
        const long long fromStart=std::chrono::duration_cast<std::chrono::milliseconds>(syncNow-processStart).count();
        if(fromStart>=c.startupCrossingMinDetectMs && forkEv.visible) ++startupCrossingConfirm;
        else if(!forkEv.visible) startupCrossingConfirm=0;

        if(startupCrossingConfirm>=c.startupCrossingConfirmFrames)
        {
            startupCrossingSynced=true; startupCrossingConfirm=0; lapEpoch=syncNow;
            std::cout << ANSI_RED
                      << "!!!【检测到起点倒Y交叉口】已通过出库交叉口，进入第一圈；本圈计时清零。"
                      << c.ringStartupBlockMs/1000.0 << "秒后开放环岛门控，"
                      << c.zebraStartupBlockMs/1000.0 << "秒后开放人行横道，"
                      << c.crossingStartupBlockMs/1000.0 << "秒后开放车库交叉口计圈!!!"
                      << ANSI_RESET << std::endl;
        }
        else if(fromStart>=c.startupCrossingFallbackMs)
        {
            startupCrossingSynced=true; startupCrossingConfirm=0; lapEpoch=syncNow;
            std::cout << ANSI_RED
                      << "!!!【起点交叉口自动确认】未稳定识别倒Y交叉口，但发车已超过"
                      << c.startupCrossingFallbackMs << "毫秒，按已通过起点交叉口处理；进入第一圈并重新计时!!!"
                      << ANSI_RESET << std::endl;
        }
    }

    // 赛道固定顺序：锥桶宽道 -> 环岛 -> S弯 -> 人行横道 -> 车库右侧交叉口。
    // 交叉口不再依赖人行横道必须识别成功，而使用本圈独立时间门控，避免人行横道漏检导致无法计圈。
    out.zebraTriggered=zebraUpdate(rawMask,observationLane,c,ring.stage);
    if(out.zebraTriggered)
    {
        const auto until=frameNow()+std::chrono::milliseconds(c.zebraStopMs);
        zebra.stopEndMs=std::chrono::duration_cast<std::chrono::milliseconds>(until.time_since_epoch()).count();
        if(finishStopPending) finishStopAt=until+std::chrono::milliseconds(c.crossingAfterZebraDelayMs+c.finishStopDelayMs);
        zebraStopUntilNs.store(std::chrono::duration_cast<std::chrono::nanoseconds>(until.time_since_epoch()).count());
        std::cout << ANSI_RED << "!!!【人行横道停车】原地停车=" << c.zebraStopMs << "毫秒，不执行倒退!!!" << ANSI_RESET << std::endl;
    }
    out.crossingDetected=updateCrossing(forkEv,c);
    out.crossingArmed=crossing.armed;

    updateGarage(black, c, rawMask.rows, rawMask.cols,
                 RuntimeConfig::getCenterPratio(), error);

    // Later warning modules may also set a speed override. Preserve the ring slowdown ceiling.
    if(ring.markerResult.armed || ring.result.active || ring.probe.controlsSteering())
    {
        const double limit=ring.result.active?c.ringSpeed:c.ringApproachSpeed;
        if(!speedOverrideActive.load() || speedOverrideValue.load()>limit)
        {
            speedOverrideValue.store(limit);speedOverrideActive.store(true);
        }
    }

    // Resolve the cone ceiling AFTER warning/garage modules: none can raise a forced-turn speed.
    // POST is a ceiling, so a normal target below postSpeed must remain below it.
    if(coneSequence.active()) {
        const double requested=speedOverrideActive.load()?speedOverrideValue.load():RuntimeConfig::getMotorAimSpeed();
        speedOverrideValue.store(std::min(requested,coneSequence.speed()));speedOverrideActive.store(true);
    }
    // 第二圈倒Y交叉口后自动停车：这里只停驱动并把转向命令归零，程序继续运行。
    // 这不能替代车上的舵机电源开关；停车后仍应先关舵机开关，再 Ctrl+C。
    const long long stopCheckMs=std::chrono::duration_cast<std::chrono::milliseconds>(frameNow().time_since_epoch()).count();
    if(finishStopPending && frameNow()>=finishStopAt &&
       TrackPolicy::afterZebraResume(stopCheckMs,zebra.stopEndMs,c.crossingAfterZebraDelayMs))
    {
        finishStopPending=false;
        finishStopped=true;
        motorStopForced.store(true);
        speedOverrideActive.store(false);
        std::cout << ANSI_RED
                  << "!!!【第二圈结束自动停车】已停车并将转向命令归零。请先关闭车上的舵机电源开关，再按 Ctrl+C 退出程序!!!"
                  << ANSI_RESET << std::endl;
    }
    if(coneSequence.forcing()) error=(float)coneSequence.error();
    if(coneSequenceEnabled && (coneInputStop.load() || coneSequence.needsStop())) error=0.0f;
    if(finishStopped)
    {
        motorStopForced.store(true);
        speedOverrideActive.store(false);
        error=0.0f;
    }

    const auto nowForStatus=frameNow();
    out.lapElapsedMs=(int)std::max<long long>(0,std::chrono::duration_cast<std::chrono::milliseconds>(nowForStatus-lapEpoch).count());
    out.valid=true;out.steerError=error;out.leftDownY=ring.result.evidence.down.y;out.leftMidY=ring.result.evidence.mid.y;out.leftUpY=ring.result.evidence.up.y;out.rightOutY=ring.result.evidence.rightOut.y;out.ringCompleted=ring.result.completed;out.markerBlocks=ring.markerEvidence.blocks;out.markerArmed=ring.markerResult.armed;out.ringStage=ring.stage;out.garageStage=garage.stage;
    // 【修正6】这些字段原先在 updateRing() 之后立刻写进 out，
    // 但 updateCrossing()（更靠后）仍可能复位 ring，
    // 于是出现"FrameResult=复位前 / CSV=复位后"的不一致。
    // 现在统一挪到这个最终状态汇总点，与 CSV / 四宫格同源。
    out.lowCameraCandidate=ring.result.evidence.lowCameraEntry;
    out.lowCameraConfirmed=ring.result.lowCameraEntryConfirmed;
    out.virtualDownX=ring.result.evidence.virtualDown.x;
    out.virtualDownY=ring.result.evidence.virtualDown.y;
    out.ringGateOpen=ring.gateOpen;
    out.ringObservationPaused=ring.observationPaused;
    out.binaryThreshold=binary.threshold;
    out.binaryWhitePercent=binary.whitePercent;
    out.binaryQuality=binary.quality;

    if(c.debugPrint)
    {
        static Clock::time_point lastStatus = Clock::time_point{};
        const auto now = frameNow();
        if(lastStatus == Clock::time_point{} ||
           std::chrono::duration_cast<std::chrono::milliseconds>(now-lastStatus).count() >= 1000)
        {
            lastStatus = now;
            std::ostringstream status;
            status << "[NewTrack][STATUS] lane=" << (laneMethod==3?"M3":"725")
                   << " source=" << ((coneSequence.inRoute() || coneSequence.needsStop())?coneControlSource():ringActive?"RING":(startupRightActive?"START_RIGHT":(blueAvoidActive?"BLUE_CONE":((c.centerBlackEnable && black.reliable)?"CENTER_BLACK":"BOUNDARY"))))
                   << " error=" << std::fixed << std::setprecision(1) << error
                   << " blackRows=" << black.validRows << "/" << black.nearRows
                   << " edgeRows=" << lane.bilateralRows << "/L" << lane.leftOnlyRows << "/R" << lane.rightOnlyRows
                   << " target_speed=" << (speedOverrideActive.load()?speedOverrideValue.load():RuntimeConfig::getMotorAimSpeed())
                   << " marker=" << ring.markerEvidence.blocks << "/" << ring.markerResult.armed
                   << " zebra_wait=" << (!TrackPolicy::afterZebraResume(stopCheckMs,zebra.stopEndMs,c.crossingAfterZebraDelayMs))
                   << " fork_far=" << crossing.farNear.armed()
                   << " near_fork=" << forkEv.nearSplitRows
                   << " fork_near_y=" << c.crossingNearYPercent
                   << " probe=" << ring.probe.phase() << " probe_dx=" << ring.probeResult.deltaX
                   << " probe_rows=" << ring.probeResult.sampleRows
                   << " probe_width_delta=" << ring.probeResult.deltaWidth
                   << " probe_border=" << ring.probeResult.clippedRows
                   << " tmpl=" << ring.templateScore << "/" << ring.templateConfirm
                   << " side=" << ringSide.score << "/" << ringSide.hits << "/p" << ringSide.phase
                   << "/T" << ringSide.bestIdx << "/n" << ringSide.triggerCount
                   << " cue=" << ringEntryCue.dashBlocks << "/" << ringEntryCue.armedCount
                   << "/p" << ringEntryCue.phase
                   << "/" << ringEntryCue.bigW << "x" << ringEntryCue.bigH
                   << " ring_finder=MOORE_8"
                   << " binary_T=" << binary.threshold << " binary_white=" << binary.whitePercent
                   << " binary_quality=" << binary.quality
                   << " low_camera=" << out.lowCameraCandidate << "/" << out.lowCameraConfirmed
                   << " ring=" << (int)ring.stage
                   << " corners_y=" << ring.result.evidence.down.y << "/" << ring.result.evidence.mid.y << "/" << ring.result.evidence.up.y
                   << " right_out_y=" << ring.result.evidence.rightOut.y
                   << " breaks=" << ring.result.evidence.leftBreaks
                   << " right_ok=" << ring.result.evidence.rightContinuous
                   << " both_missing=" << ring.result.evidence.bothMissingRows
                   << " complete=" << ring.result.completed
                   << " lap=" << lapCounter.load()
                   << " lap_ms=" << out.lapElapsedMs
                   << " cross=" << (crossing.armed?1:0) << "/" << crossing.openConfirm
                   << " garage=" << (int)garage.stage;
            std::cout << status.str() << std::endl;
        }
    }

    if(buildDebugFrame)
    {
        const bool stop=forceMotorStop() || gMotorStop;
        const double speedCommand=stop?0.0:(speedOverrideActive.load()?speedOverrideValue.load():RuntimeConfig::getMotorAimSpeed());
        const bool blackUsed=!coneSequence.inRoute() && !ringActive && !startupRightActive && !finishStopped &&
            (out.centerBlackReliable || partialBlackApplied);
        const std::string source=(coneSequence.inRoute() || coneSequence.needsStop())?coneControlSource():garage.stage!=GarageStage::IDLE?"GARAGE":(ringActive?"RING":(startupRightActive?"START_RIGHT":(blueAvoidActive?"CONE":(out.centerBlackReliable?"CENTER_BLACK":(partialBlackApplied?"PARTIAL_BLACK":"BOUNDARY")))));
        const std::string coneSide=ConeBoundary::sideName(blueResult.side);
        const auto& e=ring.result.evidence;
        // 【修正6 / A1】有效帧也走统一的 buildDiagnosticCsv()，
        // 与无效帧共用同一份字段布局，保证 FrameResult / CSV / 四宫格来自同一份最终状态。
        CsvRow row;row.cone=blueResult;row.coneTuningValid=c.coneTuningValid;row.coneGrayThreshold=grayCone.threshold;
        row.valid=true;row.laneValid=true;
        row.ringStage=static_cast<int>(ring.stage);
        row.garageStage=static_cast<int>(garage.stage);
        row.ringComplete=ring.result.completed;
        row.probePhase=ring.probe.phase();
        row.probeDx=ring.probeResult.deltaX;
        row.probeRows=ring.probeResult.sampleRows;
        row.probeBorder=ring.probeResult.clippedRows;
        row.probeWidthDelta=ring.probeResult.deltaWidth;
        row.probeWidthRows=ring.probeResult.widthRows;
        row.markerBlocks=ring.markerEvidence.blocks;
        row.markerArmed=ring.markerResult.armed;
        row.leftDownY=e.down.y;row.leftMidY=e.mid.y;row.leftUpY=e.up.y;row.rightOutY=e.rightOut.y;
        row.source=source;row.coneSide=coneSide;
        row.targetX=targetX;row.steerError=error;row.speedCommand=speedCommand;
        row.stopCommand=stop;row.centerBlackUsed=blackUsed;row.blackRows=black.validRows;
        row.coneActive=blueAvoidActive;row.coneDetected=blueResult.detected;
        row.coneHeld=blueResult.held;row.coneBlocked=blueResult.blocked;
        row.coneX=blueResult.box.x;row.coneY=blueResult.box.y;
        row.coneW=blueResult.box.width;row.coneH=blueResult.box.height;
        row.coneChangedRows=blueResult.changedRows;
        row.zebraFar=zebra.sequence.farArmed();row.zebraSeen=zebra.seenThisLap;
        row.zebraTriggered=out.zebraTriggered;
        row.zebraStopping=(stopCheckMs*1000000<zebraStopUntilNs.load());
        row.forkFar=crossing.farNear.armed();row.nearForkRows=forkEv.nearSplitRows;
        row.crossingConfirmed=out.crossingDetected;
        row.lap=lapCounter.load();row.lapMs=out.lapElapsedMs;
        row.leftByY=&lane.left;row.rightByY=&lane.right;row.centerByY=&lane.center;row.blackByY=&black.x;
        row.binaryThreshold=binary.threshold;row.binaryWhitePercent=binary.whitePercent;row.binaryQuality=binary.quality;
        row.ringGateOpen=ring.gateOpen;
        row.lowCameraCandidate=e.lowCameraEntry;
        row.lowCameraConfirmed=ring.result.lowCameraEntryConfirmed;
        row.virtualDownX=e.virtualDown.x;row.virtualDownY=e.virtualDown.y;
        row.rightEntryX=e.rightEntry.x;row.rightEntryY=e.rightEntry.y;
        row.leftDownX=e.down.x;row.leftMidX=e.mid.x;row.leftUpX=e.up.x;row.rightOutX=e.rightOut.x;
        row.rawLeftByY=&lane.rawLeft;row.rawRightByY=&lane.rawRight;
        row.ringObservationPaused=ring.observationPaused;
        out.diagnosticCsv=buildDiagnosticCsv(row);
        DiagnosticView::Snapshot snap;snap.left=lane.left;snap.right=lane.right;snap.center=lane.center;snap.black=black.x;
        snap.corners={e.down,e.mid,e.up,e.rightOut};snap.coneBox=blueResult.box;snap.coneHeld=blueResult.held;snap.blueMask=blueResult.blueMask;
        snap.virtualDown=e.virtualDown;
        snap.valid=true;snap.targetX=targetX;snap.blackUsed=blackUsed;snap.forkNearPercent=c.crossingNearYPercent;
        std::ostringstream line1,line2,line3,line4;
        line1<<"source="<<source<<" target_x="<<std::fixed<<std::setprecision(1)<<targetX<<" steer_error="<<error
             <<" SPEED_COMMAND="<<speedCommand<<" STOP="<<stop<<" BLACK_USED="<<snap.blackUsed;
        line2<<"ring="<<static_cast<int>(ring.stage)<<" complete="<<ring.result.completed<<" marker="<<ring.markerEvidence.blocks
             <<'/'<<ring.markerResult.armed<<" probe="<<ring.probe.phase()<<" dx="<<ring.probeResult.deltaX
             <<" dw="<<ring.probeResult.deltaWidth<<" rows="<<ring.probeResult.sampleRows
             <<" y="<<e.down.y<<'/'<<e.mid.y<<'/'<<e.up.y<<" out="<<e.rightOut.y;
        line3<<"cone="<<coneSide<<" ACTIVE="<<blueAvoidActive<<" held="<<blueResult.held<<" blocked="<<blueResult.blocked
             <<" confirm="<<blueResult.confirmations<<"/"<<c.coneSettings.confirmFrames<<" "<<blueResult.reason
             <<" "<<ConeSequence::name(coneSequence.phase())<<":"<<coneSequence.remainingMs()<<"ms"<<" "<<coneSequence.entryReason()<<"/"<<coneSequence.reason()
             <<" zebra_far="<<zebra.sequence.farArmed()<<" zebra_seen="<<zebra.seenThisLap<<" zebra_stop="<<(stopCheckMs*1000000<zebraStopUntilNs.load());
        line4<<"fork_far="<<crossing.farNear.armed()<<" near_rows="<<forkEv.nearSplitRows<<" near_y="<<c.crossingNearYPercent
             <<"% lap="<<lapCounter.load()<<" lap_ms="<<out.lapElapsedMs<<" garage="<<static_cast<int>(garage.stage)
             <<" T="<<binary.threshold<<" quality="<<binary.quality<<" low="<<e.lowCameraEntry
             <<'/'<<ring.result.lowCameraEntryConfirmed<<" virtual=orange";
        snap.lines={line1.str(),line2.str(),line3.str(),line4.str()};
        out.debugFrame=DiagnosticView::render(frame,rawMask,boundaryMask,snap);
    }
    return out;
}


FrameResult processFrame(const cv::Mat &frameBlock, bool buildDebugFrame)
{
    const FrameClockGuard guard(Clock::now());
    return processFrameImpl(frameBlock,buildDebugFrame);
}
FrameResult processFrameAt(const cv::Mat &frameBlock, bool buildDebugFrame, long long timestampMs)
{
    const FrameClockGuard guard(Clock::time_point{}+std::chrono::milliseconds(timestampMs));
    return processFrameImpl(frameBlock,buildDebugFrame);
}

bool speedOverride(double &targetSpeed)
{
    if(!enabled()) return false;
    if(motorStopForced.load() || coneStopForced.load() || lostLaneStopForced.load() ||
       ringSideStopForced.load() || ringEntryStopForced.load())
    {targetSpeed=0;return true;}
    if(speedOverrideActive.load()){targetSpeed=speedOverrideValue.load();return true;}
    return false;
}

bool forceMotorStop()
{
    const Config c=cfg();
    if(!c.enable) return false;
    if(c.coneSequenceSettings.enabled && !c.coneTuningValid) return true;
    if(motorStopForced.load() || coneStopForced.load() || lostLaneStopForced.load() ||
        ringSideStopForced.load() || ringEntryStopForced.load() || coneInputStop.load()) return true;
    if(coneSequenceActive.load()) {
        const auto ns=std::chrono::duration_cast<std::chrono::nanoseconds>(frameNow().time_since_epoch()).count();
        const auto last=coneLastFrameNs.load();
        const auto pulseUntil=conePulseUntilNs.load();
        if(pulseUntil>0 && ns>=pulseUntil) return true;
        if(last<0 || ns-last>150000000LL) return true;
    }
    const long long until=zebraStopUntilNs.load();
    if(until<=0) return false;
    const long long now=std::chrono::duration_cast<std::chrono::nanoseconds>(frameNow().time_since_epoch()).count();
    if(now<until) return true;
    zebraStopUntilNs.store(0);
    return false;
}
int lapCount(){return lapCounter.load();}
RingStage ringStage(){return (RingStage)ringStageAtomic.load();}
GarageStage garageStage(){return (GarageStage)garageStageAtomic.load();}

void reset()
{
    coneDetector.reset();coneSequence.reset();conePulseUntilNs.store(0);coneLaunchAtMs=-1;coneInputStop.store(false);coneGuideStop.store(false);coneSequenceActive.store(false);coneLastFrameNs.store(-1);
    coneStopForced.store(false);
    ring=RingState{};zebra=ZebraState{};crossing=CrossingState{};garage=GarageStateData{};lapCounter.store(0);ringStageAtomic.store(0);garageStageAtomic.store(0);speedOverrideActive.store(false);motorStopForced.store(false);lostLaneStopForced.store(false);zebraStopUntilNs.store(0);processStart=frameNow();processStarted=false;lapEpoch=processStart;startupCrossingSynced=false;startupCrossingConfirm=0;finishStopPending=false;finishStopped=false;finishStopAt=Clock::time_point{}; laneWidthProfile.clear(); laneWidthMedian=0.0; laneWidthProfileHeight=0;
    // 【新增】限速器 / 丢线停车 / 中线平滑 的静态状态也一起清零。
    lastTargetX=-1.0; lostLaneCount=0; lostLaneRecover=0; lostLaneStopActive=false;
    // 【环岛右侧多模板】状态与模板缓存一起清零。
    ringSide=RingSideState{};
    ringSideTmpls.clear(); ringSideTmplKey.clear();
    ringEntryCue=RingEntryCueState{};
    // 【修正1】必须清掉配置热加载的时间缓存。
    // 否则：本进程先前用真实时钟加载过一次 -> lastConfigLoad 是大值；
    //       reset() 后立刻用较小时间戳回放 -> 时间差为负 -> 配置永久不更新。
    // 下一帧 loadConfigIfNeeded() 会立刻重新读一次 image_params.txt。
    lastConfigLoad=Clock::time_point{};
}
}
