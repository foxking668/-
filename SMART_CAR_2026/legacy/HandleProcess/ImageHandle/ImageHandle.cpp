#include "ImageHandle.hpp"

using namespace ImageProcess;
using namespace Contral;
using namespace Other;

/*
 * 全局赛道修线器。
 */
FixLine fixLine;

/*
 * 全局红绿灯检测器。
 */
TrafficLightDetector trafficLightDetector;

/*
 * 蓝色桶锥检测器。
 * 当前版本会把检测到的蓝色桶锥转换成虚拟左右边线，
 * 再重新写入 leftLine/rightLine/centerLine，让它真正参与后面的巡线误差计算。
 */
BlueObstacleDetector blueObstacleDetector;

namespace
{
    void applyBlueConeEdge(const BlueObstacleResult &blueResult,
                           int *leftLine,
                           int *rightLine,
                           int *centerLine,
                           int imageHeight,
                           int imageWidth)
    {
        if (!blueResult.use_edge_line ||
            (blueResult.type != BlueObstacleType::CONE_PAIR &&
             blueResult.type != BlueObstacleType::CONE_WARN &&
             blueResult.type != BlueObstacleType::BOARD_FALLBACK) ||
            centerLine == nullptr ||
            blueResult.edge_line.empty() ||
            imageHeight <= 0 ||
            imageWidth <= 0)
        {
            return;
        }

        /*
         * 蓝色桶锥使用更宽的 10% 裁剪图检测，虚拟边线映射回巡线裁剪图后，
         * 可能落在巡线图外侧。这个版本允许蓝色桶锥的虚拟边线越界，
         * 但只把它作为数学边界参与 centerLine 计算，不拿越界 x 去访问图像像素。
         *
         * 注意：真正写入 leftLine/rightLine 时仍然要求 x 在图像内部；
         * 如果 x 越界，只更新 centerLine，避免后续画点或访问像素出问题。
         */
        const int yStart = std::max(0, static_cast<int>(imageHeight * 0.20));
        const int yEnd = std::min(imageHeight - 1, static_cast<int>(imageHeight * 0.92));
        const int minRoadWidth = std::max(8, static_cast<int>(imageWidth * 0.18));
        const int minCenterShift = std::max(4, static_cast<int>(imageWidth * 0.05));

        int changedRows = 0;

        for (int y = yStart; y <= yEnd; y++)
        {
            if (y < 0 || y >= static_cast<int>(blueResult.edge_line.size()))
            {
                continue;
            }

            int blueX = blueResult.edge_line[y];
            if (blueX == -1)
            {
                continue;
            }

            int currentCenter = centerLine[y];
            if (currentCenter < 0 || currentCenter >= imageWidth)
            {
                int l = (leftLine != nullptr) ? leftLine[y] : -1;
                int r = (rightLine != nullptr) ? rightLine[y] : -1;
                if (l >= 0 && r >= 0 && r > l)
                {
                    currentCenter = (l + r) / 2;
                }
                else
                {
                    currentCenter = imageWidth / 2;
                }
            }

            if (blueResult.side == BlueObstacleSide::LEFT)
            {
                int roadRight = imageWidth - 1;
                if (rightLine != nullptr && rightLine[y] >= 0)
                {
                    roadRight = rightLine[y];
                }

                // 当蓝色虚拟左线在巡线图外侧时仍允许参与计算。
                // 为了避免方向反了，最终 centerLine 必须向右侧避让。
                int candidateCenter = (blueX + roadRight) / 2;
                if (candidateCenter <= currentCenter)
                {
                    candidateCenter = currentCenter + minCenterShift;
                }
                candidateCenter = std::max(0, std::min(imageWidth - 1, candidateCenter));

                if (leftLine != nullptr && blueX >= 0 && blueX < imageWidth && blueX < roadRight - minRoadWidth)
                {
                    leftLine[y] = blueX;
                }

                centerLine[y] = candidateCenter;
                changedRows++;
            }
            else if (blueResult.side == BlueObstacleSide::RIGHT)
            {
                int roadLeft = 0;
                if (leftLine != nullptr && leftLine[y] >= 0)
                {
                    roadLeft = leftLine[y];
                }

                // 当蓝色虚拟右线在巡线图外侧时仍允许参与计算。
                // 为了避免方向反了，最终 centerLine 必须向左侧避让。
                int candidateCenter = (roadLeft + blueX) / 2;
                if (candidateCenter >= currentCenter)
                {
                    candidateCenter = currentCenter - minCenterShift;
                }
                candidateCenter = std::max(0, std::min(imageWidth - 1, candidateCenter));

                if (rightLine != nullptr && blueX >= 0 && blueX < imageWidth && blueX > roadLeft + minRoadWidth)
                {
                    rightLine[y] = blueX;
                }

                centerLine[y] = candidateCenter;
                changedRows++;
            }
        }

        /* quiet: virtual-line rows are intentionally not printed */
    }



    std::vector<int> remapLineToBlueFrame(const int *sourceLine,
                                          int imageHeight,
                                          int blueImageWidth,
                                          int lineToBlueOffset)
    {
        std::vector<int> mapped(imageHeight, -1);
        if (sourceLine == nullptr || imageHeight <= 0 || blueImageWidth <= 0)
        {
            return mapped;
        }

        for (int y = 0; y < imageHeight; y++)
        {
            if (sourceLine[y] < 0)
            {
                continue;
            }
            const int x = sourceLine[y] + lineToBlueOffset;
            if (x >= 0 && x < blueImageWidth)
            {
                mapped[y] = x;
            }
        }
        return mapped;
    }

    std::vector<int> buildDirectionalVirtualLineInLineFrame(BlueObstacleSide side,
                                                            int imageHeight,
                                                            int imageWidth)
    {
        std::vector<int> line(imageHeight, -1);
        if (imageHeight <= 0 || imageWidth <= 0 || side == BlueObstacleSide::UNKNOWN)
        {
            return line;
        }

        /*
         * 这个函数只给蓝色挡板兜底使用：挡板已经在巡线图里比较靠前，
         * 这里仍然生成图像内部的虚拟线，方向更稳。
         *
         * 蓝色桶锥不走这里。桶锥用宽图生成的真实 edge_line 映射回来，
         * 允许虚拟边线落在巡线图外侧，以获得提前预判避障。
         */
        const double ratio = (side == BlueObstacleSide::LEFT) ? 0.42 : 0.58;
        const int x = std::max(0, std::min(imageWidth - 1, static_cast<int>(imageWidth * ratio)));
        for (int y = 0; y < imageHeight; y++)
        {
            line[y] = x;
        }
        return line;
    }


    const char *blueSideToStringLocal(BlueObstacleSide side)
    {
        if (side == BlueObstacleSide::LEFT)
        {
            return "LEFT";
        }
        if (side == BlueObstacleSide::RIGHT)
        {
            return "RIGHT";
        }
        return "UNKNOWN";
    }

    const char *blueAvoidDirectionString(BlueObstacleSide obstacleSide)
    {
        if (obstacleSide == BlueObstacleSide::LEFT)
        {
            return "RIGHT";
        }
        if (obstacleSide == BlueObstacleSide::RIGHT)
        {
            return "LEFT";
        }
        return "UNKNOWN";
    }

    BlueObstacleSide oppositeBlueSide(BlueObstacleSide side)
    {
        if (side == BlueObstacleSide::LEFT)
        {
            return BlueObstacleSide::RIGHT;
        }
        if (side == BlueObstacleSide::RIGHT)
        {
            return BlueObstacleSide::LEFT;
        }
        return BlueObstacleSide::UNKNOWN;
    }

    BlueObstacleResult buildBoardTurnBackResult(BlueObstacleSide originalBoardSide,
                                                int imageHeight,
                                                int imageWidth)
    {
        BlueObstacleResult result;
        BlueObstacleSide counterVirtualSide = oppositeBlueSide(originalBoardSide);
        if (counterVirtualSide == BlueObstacleSide::UNKNOWN || imageHeight <= 0 || imageWidth <= 0)
        {
            return result;
        }

        /*
         * 蓝色挡板二段接弯逻辑：
         *   原始挡板在右侧 -> 第一段向左躲；1 秒后需要向右接弯。
         *   想让车向右，数学上相当于放一条“左侧虚拟边线”，让 centerLine 右移。
         *
         *   原始挡板在左侧 -> 第一段向右躲；1 秒后需要向左接弯。
         *   想让车向左，数学上相当于放一条“右侧虚拟边线”，让 centerLine 左移。
         */
        result.type = BlueObstacleType::BOARD_FALLBACK;
        result.side = counterVirtualSide;
        result.edge_line = buildDirectionalVirtualLineInLineFrame(counterVirtualSide, imageHeight, imageWidth);
        result.use_edge_line = true;
        result.return_slow_mode = true;
        result.virtual_line_hold = true;
        return result;
    }

    BlueObstacleResult remapBlueResultToLineFrame(const BlueObstacleResult &blueResult,
                                                  int lineImageWidth,
                                                  int lineImageHeight,
                                                  int lineToBlueOffset)
    {
        BlueObstacleResult mapped = blueResult;
        if (lineImageWidth <= 0 || lineImageHeight <= 0)
        {
            return mapped;
        }

        for (auto &center : mapped.centers)
        {
            center.x -= static_cast<float>(lineToBlueOffset);
        }

        if (!mapped.use_edge_line ||
            (mapped.type != BlueObstacleType::CONE_PAIR &&
             mapped.type != BlueObstacleType::CONE_WARN &&
             mapped.type != BlueObstacleType::BOARD_FALLBACK) ||
            mapped.side == BlueObstacleSide::UNKNOWN)
        {
            mapped.edge_line.clear();
            mapped.use_edge_line = false;
            return mapped;
        }

        if (mapped.type == BlueObstacleType::CONE_PAIR || mapped.type == BlueObstacleType::CONE_WARN)
        {
            /*
             * 蓝色桶锥使用宽视野 5% 裁剪图检测。这里保留宽图生成的 edge_line，
             * 只做坐标平移，不再强制夹到巡线图内部。
             *
             * 这样当桶锥还在巡线裁剪图外侧时，虚拟边线仍然可以作为数学边界
             * 和另一侧真实边线计算新的 centerLine，实现提前预判避障。
             */
            std::vector<int> mappedLine(lineImageHeight, -1);
            const int copyHeight = std::min(lineImageHeight, static_cast<int>(mapped.edge_line.size()));
            for (int y = 0; y < copyHeight; y++)
            {
                if (mapped.edge_line[y] == -1)
                {
                    continue;
                }
                mappedLine[y] = mapped.edge_line[y] - lineToBlueOffset;
            }
            mapped.edge_line = mappedLine;
        }
        else
        {
            /*
             * 蓝色挡板兜底不需要外侧预判线。挡板只负责给出方向，
             * 在巡线图内部生成稳定的虚拟边线。
             */
            mapped.edge_line = buildDirectionalVirtualLineInLineFrame(mapped.side, lineImageHeight, lineImageWidth);
        }

        return mapped;
    }

    void drawBlueObstacleDebug(cv::Mat &frame, const BlueObstacleResult &blueResult)
    {
        if (frame.empty())
        {
            return;
        }

        for (const auto &center : blueResult.centers)
        {
            cv::circle(frame, center, 3, cv::Scalar(255, 255, 0), -1);
        }

        if (blueResult.edge_line.empty())
        {
            return;
        }

        cv::Point lastPoint(-1, -1);
        for (int y = 0; y < frame.rows; y++)
        {
            int x = blueResult.edge_line[y];
            if (x < 0)
            {
                continue;
            }

            cv::Point point(x, y);
            if (lastPoint.x >= 0)
            {
                cv::line(frame, lastPoint, point, cv::Scalar(255, 255, 0), 1);
            }
            lastPoint = point;
        }
    }

    /*
     * 蓝色桶锥/挡板间接红灯分支状态。
     *
     * 当 BlueObstacleDetector 判断“蓝色挡板区域离开到阈值”后，
     * 这里不立刻停车，而是继续运行 blue_curve_run_after_leave_ms 毫秒，
     * 然后强制进入等待绿灯状态，相当于另一个红灯触发分支。
     */
    bool blueCurveBranchRunning = false;
    std::chrono::steady_clock::time_point blueCurveBranchStartTime = std::chrono::steady_clock::now();
    const char *blueCurveBranchReason = "none";

    /*
     * 人行横道停车恢复后的右偏修正状态。
     * Zebra 触发时先记录时间，等待 zebra_after_reverse_stop_ms + pid_recover_delay_ms
     * 再进入短时间强制右偏。这样右偏动作发生在小车重新起步之后，而不是停车期间。
     */
    bool zebraPostRightScheduled = false;
    bool zebraPostRightPrinted = false;
    std::chrono::steady_clock::time_point zebraPostRightTriggerTime = std::chrono::steady_clock::now();

    void updateBlueCurveStopBranch(const BlueObstacleResult &blueResult)
    {
        if (blueResult.curve_stop_branch_request && !blueCurveBranchRunning)
        {
            if (gMotorStop)
            {
/* quiet: blue branch ignored because already stopped */
            }
            else
            {
                blueCurveBranchRunning = true;
                blueCurveBranchStartTime = std::chrono::steady_clock::now();
                blueCurveBranchReason = blueResult.curve_stop_reason;

/* quiet: blue curve branch countdown started */
            }
        }

        if (!blueCurveBranchRunning)
        {
            return;
        }

        const int runAfterLeaveMs = RuntimeConfig::getBlueCurveRunAfterLeaveMs();
        const auto now = std::chrono::steady_clock::now();
        const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   now - blueCurveBranchStartTime)
                                   .count();

        if (elapsedMs >= runAfterLeaveMs)
        {
            blueCurveBranchRunning = false;

            /*
             * 优先级高：不需要真的看到红灯，直接进入等待绿灯状态。
             * 电机线程看到 gMotorStop=true 后会 PWM=0 停车；
             * 之后只有识别到绿灯，TrafficLightDetector 才会返回 GREEN_GO，
             * ImageHandle.cpp 再把 gMotorStop=false。
             */
            gMotorStop = true;
            trafficLightDetector.forceWaitGreen("blue_curve_branch");

/* wait green message is printed by TrafficLightDetector::forceWaitGreen */
        }
    }
}

/*
 * 斑马线检测回调。
 *
 * 注意：
 * Zebra.cpp 里已经会打印 Zebra，避免终端重复刷两次，
 * 这里不再额外打印 Zebra。
 */
void zebraHandle(bool isExist)
{
    if (isExist)
    {
        /*
         * 人行横道触发后，先进行语音播报。
         *
         * 命令来自 wonderEcho 示例工程：
         *     wonderEchoSend(0xFF, 0x11);
         *
         * 注意：Zebra.cpp 已经有冷却时间，冷却期内不会再次进入这里，
         * 所以语音不会因为同一个人行横道连续重复播报。
         */
        WonderEchoSpeakZebra();

        /*
         * 人行横道触发后，不再直接 gMotorStop 停 3 秒。
         * 改为通知电机线程执行倒退逻辑：
         * - 倒退速度 zebra_reverse_speed 从 ./car_params/image_params.txt 读取；
         * - 倒退时间 zebra_reverse_time_ms 从配置文件读取；
         * - 电机线程只使用右编码器反馈，左轮反馈跟随右编码器。
         */
        gMotorReverseRequest = true;

        /*
         * 记录 Zebra 触发时间。后续图像线程会在停车保持和 PID 恢复延迟结束后，
         * 根据配置执行一小段强制向右修正。
         */
        zebraPostRightScheduled = true;
        zebraPostRightPrinted = false;
        zebraPostRightTriggerTime = std::chrono::steady_clock::now();

/* quiet: zebra safe-stop request set */
    }
}

float getImageHandle(bool *result, void **imageDate, int *height, int *width)
{
    /*
     * 每帧尝试刷新运行时参数。
     * 配置文件路径：
     * ./car_params/image_params.txt
     *
     * 你可以在小车终端当前运行目录下直接改这个文件，
     * 不需要重新编译。
     */
    RuntimeConfig::reloadIfNeeded();

    Camera *camera = (Camera *)gCamera;
    if (camera == nullptr)
    {
        if (result != nullptr)
        {
            *result = false;
        }
        return 0;
    }

    cv::Mat frameBlock = camera->getFrame(true);

    if (frameBlock.empty())
    {
        if (result != nullptr)
        {
            *result = false;
        }
        return 0;
    }

    CutFrame cutFrame;

    /*
     * 人行横道后动态巡线裁剪状态。
     * 计时事件由 MotorHandle 在“实际恢复巡线速度”时置位，因此停车、反转、锁止
     * 和 PID 恢复等待都不会消耗这个窗口。此处绝不检测蓝色、不改变舵机方向。
     */
    static std::chrono::steady_clock::time_point zebraLineCropResumeTime =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(30000);
    static bool previousLineCropExpanded = false;
    if (gZebraResumeEvent)
    {
        zebraLineCropResumeTime = std::chrono::steady_clock::now();
        previousLineCropExpanded = false;
        gBlueBoardLineCropFinishedEvent = false;
        gZebraResumeEvent = false;
    }

    /*
     * 红绿灯专用裁剪图。
     *
     * 这部分只给红绿灯检测使用，不影响巡线。
     * 顶部裁剪使用 0.05，比巡线原来的 0.2 更早看到红绿灯。
     *
     * 如果红绿灯还是检测晚：把 0.05 改成 0.0。
     * 如果红绿灯误识别多：把 0.05 改成 0.1。
     */
    double trafficCutTopRatio = RuntimeConfig::getTrafficCutTopRatio();
    double trafficCutLeftRatio = RuntimeConfig::getTrafficCutLeftRatio();
    double trafficCutRightRatio = RuntimeConfig::getTrafficCutRightRatio();

    /*
     * CutFrame 的百分比函数把 0 当作非法值并返回空 Mat。
     * 红绿灯需要允许 top=0.00 才能看到最上方，因此这里使用本地安全裁剪。
     */
    auto cropTopSafe = [](const cv::Mat &input, double ratio) -> cv::Mat
    {
        if (input.empty())
        {
            return cv::Mat();
        }
        const int y = std::max(0, std::min(input.rows - 1,
            static_cast<int>(input.rows * std::max(0.0, std::min(0.95, ratio)))));
        return input(cv::Rect(0, y, input.cols, input.rows - y)).clone();
    };
    auto cropLeftSafe = [](const cv::Mat &input, double ratio) -> cv::Mat
    {
        if (input.empty())
        {
            return cv::Mat();
        }
        const int x = std::max(0, std::min(input.cols - 1,
            static_cast<int>(input.cols * std::max(0.0, std::min(0.95, ratio)))));
        return input(cv::Rect(x, 0, input.cols - x, input.rows)).clone();
    };
    auto cropRightSafe = [](const cv::Mat &input, double ratio) -> cv::Mat
    {
        if (input.empty())
        {
            return cv::Mat();
        }
        const int keep = std::max(1, std::min(input.cols,
            input.cols - static_cast<int>(input.cols * std::max(0.0, std::min(0.95, ratio)))));
        return input(cv::Rect(0, 0, keep, input.rows)).clone();
    };

    Mat trafficHeadFrameTop = cropTopSafe(frameBlock, trafficCutTopRatio);
    Mat trafficFrameLeft = cropLeftSafe(trafficHeadFrameTop, trafficCutLeftRatio);
    Mat trafficFrame = cropRightSafe(trafficFrameLeft, trafficCutRightRatio);

    /*
     * 红绿灯实时参数。
     * 这些参数来自 ./car_params/image_params.txt，运行时约 500ms 自动刷新。
     */
    TrafficLightConfig trafficConfig = trafficLightDetector.getConfig();
    trafficConfig.roi_x_ratio_start = RuntimeConfig::getTrafficRoiXStart();
    trafficConfig.roi_x_ratio_end = RuntimeConfig::getTrafficRoiXEnd();
    trafficConfig.roi_y_ratio_start = RuntimeConfig::getTrafficRoiYStart();
    trafficConfig.roi_y_ratio_end = RuntimeConfig::getTrafficRoiYEnd();
    trafficConfig.dual_roi_enable = RuntimeConfig::getTrafficDualRoiEnable();
    trafficConfig.lower_roi_y_ratio_start = RuntimeConfig::getTrafficLowerRoiYStart();
    trafficConfig.lower_roi_y_ratio_end = RuntimeConfig::getTrafficLowerRoiYEnd();
    trafficConfig.red_confirm_frames = RuntimeConfig::getTrafficRedConfirmFrames();

    trafficConfig.red1_h_min = RuntimeConfig::getRed1HMin();
    trafficConfig.red1_h_max = RuntimeConfig::getRed1HMax();
    trafficConfig.red2_h_min = RuntimeConfig::getRed2HMin();
    trafficConfig.red2_h_max = RuntimeConfig::getRed2HMax();
    trafficConfig.red_s_min = RuntimeConfig::getRedSMin();
    trafficConfig.red_v_min = RuntimeConfig::getRedVMin();
    trafficConfig.red_circle_min = RuntimeConfig::getRedCircleMin();
    trafficConfig.red_area_min = RuntimeConfig::getRedAreaMin();
    trafficConfig.red_area_max_ratio = RuntimeConfig::getRedAreaMaxRatio();
    trafficConfig.red_aspect_min = RuntimeConfig::getRedAspectMin();
    trafficConfig.red_aspect_max = RuntimeConfig::getRedAspectMax();

    trafficConfig.green_h_min = RuntimeConfig::getGreenHMin();
    trafficConfig.green_h_max = RuntimeConfig::getGreenHMax();
    trafficConfig.green_s_min = RuntimeConfig::getGreenSMin();
    trafficConfig.green_v_min = RuntimeConfig::getGreenVMin();
    trafficConfig.green_pixel_min_ratio = RuntimeConfig::getGreenPixelMinRatio();
    trafficConfig.green_pixel_min_absolute = RuntimeConfig::getGreenPixelMinAbsolute();

    trafficConfig.red_ignore_after_green_ms = RuntimeConfig::getRedIgnoreAfterGreenMs();
    trafficConfig.morph_kernel_size = RuntimeConfig::getTrafficMorphKernelSize();

    trafficLightDetector.setConfig(trafficConfig);

    /*
     * 红灯总开关。
     * 旧版没有显式开关，因此“到底有没有启动”很难确认；
     * 现在 traffic_red_enable=1 才运行红灯状态机，调试输出会同步显示实际状态。
     */
    const bool trafficRedEnable = (RuntimeConfig::getTrafficRedEnable() != 0);

    /*
     * 红绿灯状态机。
     * 红灯：gMotorStop = true
     * 绿灯：gMotorStop = false
     */
    TrafficLightAction trafficLightAction = TrafficLightAction::NONE;
    if (trafficRedEnable)
    {
        trafficLightAction = trafficLightDetector.update(trafficFrame);

        if (trafficLightAction == TrafficLightAction::RED_STOP)
        {
            gMotorStop = true;
        }
        else if (trafficLightAction == TrafficLightAction::GREEN_GO)
        {
            gMotorStop = false;
        }
    }

    /*
     * 红灯诊断：不改变任何识别阈值与停车逻辑，只把“模块是否运行、
     * 红色像素是否进入 ROI、被哪一层轮廓过滤”定期打印出来。
     */
    static std::chrono::steady_clock::time_point lastTrafficDebugTime =
        std::chrono::steady_clock::now() - std::chrono::milliseconds(10000);
    const int trafficDebugEnable = RuntimeConfig::getTrafficRedDebugEnable();
    const int trafficDebugIntervalMs = RuntimeConfig::getTrafficRedDebugIntervalMs();
    const auto trafficDebugNow = std::chrono::steady_clock::now();
    const long long trafficDebugElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        trafficDebugNow - lastTrafficDebugTime).count();

    if (trafficDebugEnable != 0 && trafficDebugElapsedMs >= trafficDebugIntervalMs)
    {
        lastTrafficDebugTime = trafficDebugNow;

        auto stateName = [](TrafficLightState state) -> const char *
        {
            if (state == TrafficLightState::IDLE)
            {
                return "IDLE";
            }
            if (state == TrafficLightState::WAIT_GREEN)
            {
                return "WAIT_GREEN";
            }
            if (state == TrafficLightState::COOLDOWN)
            {
                return "COOLDOWN";
            }
            return "UNKNOWN";
        };

        const TrafficLightDebugInfo trafficDebug = trafficLightDetector.getLastDebugInfo();
        std::cout << "[TrafficDebug] running=" << (trafficRedEnable ? 1 : 0)
                  << " state=" << stateName(trafficLightDetector.getState())
                  << " traffic_img=" << trafficFrame.cols << "x" << trafficFrame.rows
                  << " crop(t/l/r)=" << trafficCutTopRatio << "/"
                  << trafficCutLeftRatio << "/" << trafficCutRightRatio
                  << " dual_roi=" << trafficConfig.dual_roi_enable
                  << " rois=" << trafficDebug.roi_count
                  << " red_px=" << trafficDebug.red_pixels
                  << " contours=" << trafficDebug.contour_count
                  << " pass(area/aspect/circle)=" << trafficDebug.area_pass_count
                  << "/" << trafficDebug.aspect_pass_count
                  << "/" << trafficDebug.circle_pass_count
                  << " best(area/circle)=" << trafficDebug.best_area
                  << "/" << trafficDebug.best_circle
                  << std::endl;
    }

    /*
     * 巡线专用裁剪图。
     * 正常使用 normal_top_crop_ratio（默认 0.20）；人行横道实际重新起步后，
     * 在 delay + duration 窗口内改用 post_zebra_top_crop_ratio（默认 0.04），
     * 窗口结束立刻恢复正常值。左右裁剪仍保持原来的 0.24，巡线算法本身不改。
     */
    const auto lineCropNow = std::chrono::steady_clock::now();
    const long long zebraResumeElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
        lineCropNow - zebraLineCropResumeTime).count();
    const int lineCropDelayMs = RuntimeConfig::getBlueBoardLineExpandDelayAfterZebraResumeMs();
    const int lineCropDurationMs = RuntimeConfig::getBlueBoardLineExpandDurationMs();
    const bool lineCropExpandEnable = RuntimeConfig::getBlueBoardLineExpandEnable() != 0;
    const bool lineCropExpanded = lineCropExpandEnable && lineCropDurationMs > 0 &&
        zebraResumeElapsedMs >= lineCropDelayMs &&
        zebraResumeElapsedMs < static_cast<long long>(lineCropDelayMs) + lineCropDurationMs;

    /*
     * 只在“上一帧正在动态裁剪、本帧已经恢复正常裁剪”的转换点通知电机线程。
     * 因而后置低速严格发生在动态裁剪结束之后，而不是与裁剪同时开始。
     */
    const long long lineCropEndMs = static_cast<long long>(lineCropDelayMs) + lineCropDurationMs;
    if (previousLineCropExpanded && !lineCropExpanded && lineCropExpandEnable &&
        lineCropDurationMs > 0 && zebraResumeElapsedMs >= lineCropEndMs)
    {
        gBlueBoardLineCropFinishedEvent = true;
        std::cout << "[BlueBoardLineCropEnd] delay_ms=" << lineCropDelayMs
                  << " crop_duration_ms=" << lineCropDurationMs
                  << " elapsed_ms=" << zebraResumeElapsedMs
                  << std::endl;
    }
    previousLineCropExpanded = lineCropExpanded;

    const double normalTopCropRatio = RuntimeConfig::getBlueBoardLineNormalTopCropRatio();
    const double postZebraTopCropRatio = RuntimeConfig::getBlueBoardLinePostZebraTopCropRatio();
    const double activeTopCropRatio = lineCropExpanded ? postZebraTopCropRatio : normalTopCropRatio;
    const int activeTopCropPixels = std::max(0, std::min(frameBlock.rows - 1,
        static_cast<int>(frameBlock.rows * activeTopCropRatio)));
    /* cutFrameUpByPercent 不接受 0.00，所以这里使用按高度裁剪，0 像素是合法的。 */
    Mat cutHeadFrameTop = cutFrame.cutFrameUpByHeight(frameBlock, activeTopCropPixels);

    const double lineCropLeftRatio = 0.24;
    Mat cutFrameLeft = cutFrame.cutFrameLeftByPercent(cutHeadFrameTop, static_cast<float>(lineCropLeftRatio));
    Mat frame = cutFrame.cutFrameRightByPercent(cutFrameLeft, static_cast<float>(lineCropLeftRatio));

    Mat frameCopy = frame.clone();

    gImageProcessAttribute.origin_image.data = frame.data;
    gImageProcessAttribute.origin_image.width = frame.cols;
    gImageProcessAttribute.origin_image.height = frame.rows;
    gImageProcessAttribute.origin_image.channel = frame.channels();

    HandleImage handleImage;
    handleImage.submitOriginFrame(frame);
    cv::Mat frameBin = handleImage.getGreyAndWhiteBinaryFromOriginalFrame();

    HandleImage handleImageZebra;
    handleImageZebra.submitOriginFrame(frame);
    cv::Mat binaryFrameZebra = handleImageZebra.getWhiteBinaryFromOriginalFrame();

    FindLine findLine;
    findLine.submitCountorFrame(frameBin);

    bool isFindContours = findLine.handleContoursInTopFromCountorFrameSimple();

    if (isFindContours)
    {
        findLine.findLineContoursSimple();

        vector<Point> leftContour = findLine.getLeftLineContours();
        vector<Point> rightContour = findLine.getRightLineContours();

        int *leftLine = findLine.getLeftLine();
        int *centerLine = findLine.getCenterLine();
        int *rightLine = findLine.getRightLine();

        Point leftStartPos = findLine.getLeftStartPos();
        Point rightStartPos = findLine.getRightStartPos();
        Point leftEndPos = findLine.getLeftEndPos();
        Point rightEndPos = findLine.getRightEndPos();

        fixLine.submitZebraCallback(zebraHandle);
        fixLine.submitZebraImage(binaryFrameZebra);
        fixLine.submitLeftStartPos(leftStartPos);
        fixLine.submitRightStartPos(rightStartPos);
        fixLine.submitLeftEndPos(leftEndPos);
        fixLine.submitRightEndPos(rightEndPos);

        fixLine.image_width = frame.cols;
        fixLine.image_height = frame.rows;

        fixLine.submitCenterLine(centerLine);
        fixLine.submitLeftLine(leftLine);
        fixLine.submitLeftLineContours(leftContour);
        fixLine.submitRightLine(rightLine);
        fixLine.submitRightLineContours(rightContour);

        fixLine.analyzeLineSimple();
        fixLine.fixLineSimple();

        /*
         * 这里没有蓝色桶锥、蓝色 HSV 挡板检测，也没有蓝挡板强制打方向。
         * 蓝色挡板能否避开完全取决于上面的动态巡线裁剪 + 原有 FindLine/FixLine/CalcError。
         */

        /*
         * 前方黑块兜底避障逻辑。
         *
         * 当前版本不再依赖蓝色桶锥/蓝色挡板 HSV 识别。原因是实车中蓝色目标
         * 不一定稳定打印 [BlueCone]/[BlueBoard]，但会在巡线二值图的赛道内部
         * 表现为“前方突然出现一大块黑色区域”。
         *
         * 状态机：
         *   IDLE：正常巡线，连续检测到前方黑块后触发；
         *   PRIMARY_AVOID：第一段强制向黑块相反方向躲避；
         *   COUNTER_TURN：第二段强制向第一段相反方向接弯；
         *   SLOW_HOLD：动作结束后继续低速保持，再恢复正常巡线。
         *
         * 方向约定：
         *   右侧黑块 -> 第一段向左躲，第二段向右接弯；
         *   左侧黑块 -> 第一段向右躲，第二段向左接弯。
         */
        enum class FrontBlackStage
        {
            IDLE,
            PRIMARY_AVOID,
            COUNTER_TURN,
            SLOW_HOLD
        };

        static FrontBlackStage frontBlackStage = FrontBlackStage::IDLE;
        static BlueObstacleSide frontBlackSide = BlueObstacleSide::UNKNOWN;
        static int frontBlackConfirmCount = 0;
        static bool frontBlackCounterPrinted = false;
        static std::chrono::steady_clock::time_point frontBlackStageStartTime =
            std::chrono::steady_clock::now();
        static std::chrono::steady_clock::time_point frontBlackHoldUntil =
            std::chrono::steady_clock::now() - std::chrono::milliseconds(1000);

        const auto frontBlackNow = std::chrono::steady_clock::now();
        const bool frontBlackEnable = false; // 当前版本暂停前方黑块兜底避障，避免干扰红灯/人行横道急停测试
        const int FRONT_BLACK_PRIMARY_MS = RuntimeConfig::getFrontBlackPrimaryAvoidMs();
        const double FRONT_BLACK_PRIMARY_ERROR = RuntimeConfig::getFrontBlackPrimaryAvoidError();
        const int FRONT_BLACK_COUNTER_MS = RuntimeConfig::getFrontBlackCounterTurnMs();
        const double FRONT_BLACK_COUNTER_ERROR = RuntimeConfig::getFrontBlackCounterTurnError();
        const int FRONT_BLACK_HOLD_MS = RuntimeConfig::getFrontBlackSlowHoldAfterMs();

        auto detectFrontBlackSide = [&]() -> BlueObstacleSide
        {
            if (!frontBlackEnable || frameBin.empty() || leftLine == nullptr || rightLine == nullptr)
            {
                return BlueObstacleSide::UNKNOWN;
            }

            double yStartRatio = RuntimeConfig::getFrontBlackRoiYStart();
            double yEndRatio = RuntimeConfig::getFrontBlackRoiYEnd();
            if (yEndRatio <= yStartRatio)
            {
                yEndRatio = yStartRatio + 0.05;
            }

            const int yStart = std::max(0, std::min(frame.rows - 1, static_cast<int>(frame.rows * yStartRatio)));
            const int yEnd = std::max(yStart + 1, std::min(frame.rows - 1, static_cast<int>(frame.rows * yEndRatio)));

            long long leftBlack = 0;
            long long rightBlack = 0;
            long long totalArea = 0;

            for (int y = yStart; y <= yEnd; y++)
            {
                int l = leftLine[y];
                int r = rightLine[y];
                if (l < 0 || r < 0)
                {
                    continue;
                }
                if (l > r)
                {
                    std::swap(l, r);
                }
                l = std::max(0, std::min(frame.cols - 1, l));
                r = std::max(0, std::min(frame.cols - 1, r));
                if (r - l < std::max(12, frame.cols / 12))
                {
                    continue;
                }

                int c = (centerLine != nullptr && centerLine[y] >= 0) ? centerLine[y] : (l + r) / 2;
                c = std::max(l + 1, std::min(r - 1, c));

                const uchar *rowPtr = frameBin.ptr<uchar>(y);
                for (int x = l; x <= r; x++)
                {
                    const bool isBlack = rowPtr[x] == gBinBlackPointValue;
                    if (!isBlack)
                    {
                        continue;
                    }

                    if (x < c)
                    {
                        leftBlack++;
                    }
                    else
                    {
                        rightBlack++;
                    }
                }
                totalArea += static_cast<long long>(r - l + 1);
            }

            if (totalArea <= 0)
            {
                return BlueObstacleSide::UNKNOWN;
            }

            const long long totalBlack = leftBlack + rightBlack;
            const double blackRatio = static_cast<double>(totalBlack) / static_cast<double>(totalArea);
            if (blackRatio < RuntimeConfig::getFrontBlackAreaRatioMin())
            {
                return BlueObstacleSide::UNKNOWN;
            }

            const double sideRatioMin = RuntimeConfig::getFrontBlackSideRatioMin();
            const long long maxSide = std::max(leftBlack, rightBlack);
            const long long minSide = std::max(1LL, std::min(leftBlack, rightBlack));
            if (static_cast<double>(maxSide) / static_cast<double>(minSide) < sideRatioMin)
            {
                return BlueObstacleSide::UNKNOWN;
            }

            return (rightBlack > leftBlack) ? BlueObstacleSide::RIGHT : BlueObstacleSide::LEFT;
        };

        if (frontBlackStage == FrontBlackStage::IDLE)
        {
            const BlueObstacleSide detectedSide = detectFrontBlackSide();
            if (detectedSide != BlueObstacleSide::UNKNOWN && !gMotorStop)
            {
                frontBlackConfirmCount++;
                if (frontBlackConfirmCount >= RuntimeConfig::getFrontBlackConfirmFrames())
                {
                    frontBlackStage = FrontBlackStage::PRIMARY_AVOID;
                    frontBlackSide = detectedSide;
                    frontBlackStageStartTime = frontBlackNow;
                    frontBlackCounterPrinted = false;
                    frontBlackConfirmCount = 0;
                    frontBlackHoldUntil = frontBlackNow + std::chrono::milliseconds(
                                                          FRONT_BLACK_PRIMARY_MS +
                                                          FRONT_BLACK_COUNTER_MS +
                                                          FRONT_BLACK_HOLD_MS);

                    std::cout << "[FrontBlack] side=" << blueSideToStringLocal(frontBlackSide)
                              << " avoid=" << blueAvoidDirectionString(frontBlackSide)
                              << " primary_ms=" << FRONT_BLACK_PRIMARY_MS
                              << " counter_ms=" << FRONT_BLACK_COUNTER_MS
                              << " slow_speed=" << RuntimeConfig::getFrontBlackAimSpeed()
                              << std::endl;
                }
            }
            else
            {
                frontBlackConfirmCount = 0;
            }
        }

        bool frontBlackForceTurn = false;
        float frontBlackForceError = 0.0f;
        bool frontBlackSlow = (frontBlackNow < frontBlackHoldUntil);

        if (frontBlackStage == FrontBlackStage::PRIMARY_AVOID && frontBlackSide != BlueObstacleSide::UNKNOWN)
        {
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       frontBlackNow - frontBlackStageStartTime)
                                       .count();
            if (elapsedMs < FRONT_BLACK_PRIMARY_MS)
            {
                frontBlackSlow = true;
                frontBlackForceTurn = true;
                // 右侧黑块：第一段向左；左侧黑块：第一段向右。
                frontBlackForceError = static_cast<float>(
                    (frontBlackSide == BlueObstacleSide::RIGHT)
                        ? -FRONT_BLACK_PRIMARY_ERROR
                        : FRONT_BLACK_PRIMARY_ERROR);
            }
            else
            {
                frontBlackStage = FrontBlackStage::COUNTER_TURN;
                frontBlackStageStartTime = frontBlackNow;
            }
        }

        if (frontBlackStage == FrontBlackStage::COUNTER_TURN && frontBlackSide != BlueObstacleSide::UNKNOWN)
        {
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       frontBlackNow - frontBlackStageStartTime)
                                       .count();
            if (elapsedMs < FRONT_BLACK_COUNTER_MS)
            {
                frontBlackSlow = true;
                frontBlackForceTurn = true;
                // 第二段反向接弯：右侧黑块 -> 向右；左侧黑块 -> 向左。
                frontBlackForceError = static_cast<float>(
                    (frontBlackSide == BlueObstacleSide::RIGHT)
                        ? FRONT_BLACK_COUNTER_ERROR
                        : -FRONT_BLACK_COUNTER_ERROR);

                if (!frontBlackCounterPrinted)
                {
                    std::cout << "[FrontBlackTurnBack] block=" << blueSideToStringLocal(frontBlackSide)
                              << " first_avoid=" << blueAvoidDirectionString(frontBlackSide)
                              << " turn=" << blueAvoidDirectionString(oppositeBlueSide(frontBlackSide))
                              << " force_error=" << frontBlackForceError
                              << " slow_speed=" << RuntimeConfig::getFrontBlackAimSpeed()
                              << " slow_hold_ms=" << FRONT_BLACK_HOLD_MS
                              << std::endl;
                    frontBlackCounterPrinted = true;
                }
            }
            else
            {
                frontBlackStage = FrontBlackStage::SLOW_HOLD;
                frontBlackStageStartTime = frontBlackNow;
                frontBlackHoldUntil = frontBlackNow + std::chrono::milliseconds(FRONT_BLACK_HOLD_MS);
                frontBlackCounterPrinted = false;
            }
        }

        if (frontBlackStage == FrontBlackStage::SLOW_HOLD)
        {
            frontBlackSlow = (frontBlackNow < frontBlackHoldUntil);
            if (!frontBlackSlow)
            {
                frontBlackStage = FrontBlackStage::IDLE;
                frontBlackSide = BlueObstacleSide::UNKNOWN;
                frontBlackConfirmCount = 0;
            }
        }

        bool zebraPostRightForceTurn = false;
        float zebraPostRightForceError = 0.0f;
        if (RuntimeConfig::getZebraPostRightEnable() != 0 && zebraPostRightScheduled)
        {
            const int waitMs = RuntimeConfig::getZebraAfterReverseStopMs() +
                               RuntimeConfig::getPidRecoverDelayMs() +
                               RuntimeConfig::getZebraPostRightDelayMs();
            const int rightMs = RuntimeConfig::getZebraPostRightMs();
            const double rightError = RuntimeConfig::getZebraPostRightError();
            const auto zebraElapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           frontBlackNow - zebraPostRightTriggerTime)
                                           .count();

            if (rightMs <= 0 || rightError <= 0.0)
            {
                zebraPostRightScheduled = false;
                zebraPostRightPrinted = false;
            }
            else if (zebraElapsedMs >= waitMs && zebraElapsedMs < waitMs + rightMs)
            {
                /*
                 * 只在非停车状态下生效。若此时前方黑块强制避障也在运行，
                 * 前方黑块优先级更高，避免两个强制方向互相打架。
                 */
                if (!gMotorStop && !frontBlackForceTurn)
                {
                    zebraPostRightForceTurn = true;
                    zebraPostRightForceError = static_cast<float>(rightError); // 正值 = 向右

                    if (!zebraPostRightPrinted)
                    {
                        std::cout << "[ZebraPostRight] force=RIGHT"
                                  << " error=" << zebraPostRightForceError
                                  << " duration_ms=" << rightMs
                                  << std::endl;
                        zebraPostRightPrinted = true;
                    }
                }
            }
            else if (zebraElapsedMs >= waitMs + rightMs)
            {
                zebraPostRightScheduled = false;
                zebraPostRightPrinted = false;
            }
        }

        // 本版本不运行蓝色避障状态机；这里只保留原有前方黑块兜底的限速状态。
        gBlueReturnSlowMode = frontBlackSlow;

        CalcError errorCalc;
        errorCalc.image_width = frame.cols;
        errorCalc.image_height = frame.rows;
        errorCalc.submitCenterLine(centerLine);

        float errorValue = 0;

        if (imageDate != nullptr)
        {
            if (leftLine != nullptr)
            {
                for (size_t i = 0; i < gImageProcessAttribute.origin_image.height; i++)
                {
                    if (leftLine[i] < 0)
                    {
                        continue;
                    }

                    Point point = Point(leftLine[i], i);
                    cv::circle(frameCopy, point, 2, Scalar(0, 255, 0), -1);
                }
            }

            if (centerLine != nullptr)
            {
                for (size_t i = 0; i < gImageProcessAttribute.origin_image.height; i++)
                {
                    if (centerLine[i] < 0)
                    {
                        continue;
                    }

                    Point point = Point(centerLine[i], i);
                    cv::circle(frameCopy, point, 2, Scalar(0, 0, 255), -1);
                }
            }

            if (rightLine != nullptr)
            {
                for (size_t i = 0; i < gImageProcessAttribute.origin_image.height; i++)
                {
                    if (rightLine[i] < 0)
                    {
                        continue;
                    }

                    Point point = Point(rightLine[i], i);
                    cv::circle(frameCopy, point, 2, Scalar(255, 0, 0), -1);
                }
            }

            if (*imageDate == nullptr)
            {
                *imageDate = new unsigned char[frameCopy.total() * frameCopy.elemSize()];
            }

            memcpy(*imageDate, frameCopy.data, frameCopy.total() * frameCopy.elemSize());

            if (height != nullptr)
            {
                *height = frameCopy.rows;
            }

            if (width != nullptr)
            {
                *width = frameCopy.cols;
            }
        }

        /*
         * 原来这里写死 0.41：
         *     errorCalc.calcErrorAngleByCenterSimple(..., 0.41)
         *
         * 现在改成从 ./car_params/image_params.txt 读取：
         *     center_pratio=0.41
         */
        double centerPratio = RuntimeConfig::getCenterPratio();

        errorValue = errorCalc.calcErrorAngleByCenterSimple(
            errorCalc.image_height * 3 / 4,
            errorCalc.image_height / 4,
            4,
            centerPratio);

        if (frontBlackForceTurn)
        {
            /*
             * 前方黑块兜底避障的强制打方向优先级高于普通巡线误差。
             * 第一段负责绕开黑块，第二段负责反向接弯，结束后恢复正常巡线。
             */
            errorValue = frontBlackForceError;
        }
        else if (zebraPostRightForceTurn)
        {
            /*
             * 人行横道停车恢复后的轻微右偏修正。优先级高于普通巡线，
             * 但低于前方黑块兜底强制避障。
             */
            errorValue = zebraPostRightForceError;
        }

        if (result != nullptr)
        {
            *result = true;
        }

        return errorValue;
    }
    else
    {
        gBlueReturnSlowMode = false;
        gZebraSlowMode = false;

        if (result != nullptr)
        {
            *result = false;
        }

        return 0;
    }
}

void threadRunImageHandle(void)
{
    void **imageDataBlock = nullptr;
    unsigned char *imageData = nullptr;

    int height = 0;
    int width = 0;

    bool success = false;
    bool lastGlobalImageInfoIsShow = false;

    auto fpsStartTime = std::chrono::high_resolution_clock::now();
    float frameCount = 0;

    Steer *steer = (Steer *)gSteer;

    while (!gIsClose)
    {
        if (gImageStop)
        {
            gHandleImageError = 0;

            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(1000));
            continue;
        }

        if (lastGlobalImageInfoIsShow != gImageInfoIsShow)
        {
            if (gImageInfoIsShow)
            {
                fpsStartTime = std::chrono::high_resolution_clock::now();
                frameCount = 0;
                imageDataBlock = (void **)&imageData;
            }
            else
            {
                imageDataBlock = nullptr;
                success = false;
                height = 0;
                width = 0;

                if (imageData != nullptr)
                {
                    delete[] imageData;
                }

                imageData = nullptr;
                gImageInfoDate.data = nullptr;
            }

            lastGlobalImageInfoIsShow = gImageInfoIsShow;
        }

        gHandleImageError = getImageHandle(&success, imageDataBlock, &height, &width);

        if (steer != nullptr)
        {
            steer->setAngle(90.0f + gHandleImageError);
        }

        if (!success)
        {
            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
            continue;
        }

        if (gImageInfoIsShow)
        {
            if (success)
            {
                gImageInfoDate.data = imageData;
                gImageInfoDate.success = success;
                gImageInfoDate.height = height;
                gImageInfoDate.width = width;
                gImageInfoDate.errorValue = gHandleImageError;
            }

            frameCount++;
            auto currentTime = std::chrono::high_resolution_clock::now();
            auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(
                                   currentTime - fpsStartTime)
                                   .count();

            if (elapsedTime >= 1000)
            {
                gImageInfoDate.fps = (frameCount * 1000) / elapsedTime;
                frameCount = 0;
                fpsStartTime = currentTime;
            }
        }

        std::this_thread::yield();
    }

    if (imageData != nullptr)
    {
        delete[] imageData;
    }
}
