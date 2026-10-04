#include "../FixLine.hpp"

#ifdef FIXLINE_INCLUDE_ELEMENTS_ZEBRA

using namespace Other;

namespace ImageProcess
{
    namespace
    {
        std::chrono::steady_clock::time_point gLastZebraTriggerTime =
            std::chrono::steady_clock::now() - std::chrono::milliseconds(10000);
        std::chrono::steady_clock::time_point gLastZebraSlowTime =
            std::chrono::steady_clock::now() - std::chrono::milliseconds(10000);

        bool gFarZebraSeen = false;
        bool gLastSlowPrint = false;

        /*
         * 人行横道远区高度交替计数。
         * 只有真正完成一次 Zebra 最终确认并触发停车时才加 1：
         *   count=0 -> 第 1 次用第一高度；
         *   count=1 -> 第 2 次用第二高度；
         *   count=2 -> 第 3 次回到第一高度。
         */
        int gZebraTriggerCount = 0;
        int gLastFarCycleEnable = -1;
        double gLastFarCenterYUsed = 0.25;

        void syncZebraFarCycleMode()
        {
            const int enable = RuntimeConfig::getZebraRegionFarCycleEnable();
            if (gLastFarCycleEnable != enable)
            {
                /* 用户关闭/重新开启循环时，从第 1 次重新开始计数。 */
                gZebraTriggerCount = 0;
                gLastFarCycleEnable = enable;
                std::cout << "[Zebra] far-cycle reset enable=" << enable << std::endl;
            }
        }

        double getActiveZebraFarCenterY()
        {
            syncZebraFarCycleMode();
            const bool useSecond = (RuntimeConfig::getZebraRegionFarCycleEnable() != 0) &&
                                   ((gZebraTriggerCount % 2) == 1);
            gLastFarCenterYUsed = useSecond
                                      ? RuntimeConfig::getZebraRegionFarSecondCenterYRatio()
                                      : RuntimeConfig::getZebraFarYRatio();
            return gLastFarCenterYUsed;
        }

        bool isZebraInCooldown()
        {
            const int ignoreMs = RuntimeConfig::getZebraIgnoreAfterDetectMs();
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now - gLastZebraTriggerTime)
                                       .count();
            return elapsedMs < ignoreMs;
        }

        int zebraCooldownLeftMs()
        {
            const int ignoreMs = RuntimeConfig::getZebraIgnoreAfterDetectMs();
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                       now - gLastZebraTriggerTime)
                                       .count();
            int left = ignoreMs - static_cast<int>(elapsedMs);
            return left > 0 ? left : 0;
        }

        void clearZebraSlow()
        {
            gZebraSlowMode = false;
            gFarZebraSeen = false;
            gLastSlowPrint = false;
        }

        struct RowZebraResult
        {
            int y = 0;
            int whiteRuns = 0;
            int jumps = 0;
            int minGap = 0;
            int maxGap = 0;
            bool gapUniform = false;
            bool hit = false;
        };
    }

    void FixLine::submitZebraCallback(std::function<void(bool)> callback)
    {
        this->zebra_callback = callback;
    }

    void ImageProcess::FixLine::submitZebraImage(Mat zebraImage)
    {
        this->zebra_image = zebraImage;
    }

    bool ImageProcess::FixLine::checkZebraSimple()
    {
        if (gMotorStop || isZebraInCooldown())
        {
            clearZebraSlow();
            return false;
        }

        const int width = this->image_width;
        const int height = this->image_height;

        if (this->left_line == nullptr || this->right_line == nullptr || this->zebra_image.empty() ||
            width <= 0 || height <= 0)
        {
            clearZebraSlow();
            return false;
        }

        auto checkRow = [&](double yRatio) -> RowZebraResult
        {
            RowZebraResult result;
            int y = std::max(0, std::min(height - 1, static_cast<int>(height * yRatio)));
            result.y = y;

            int leftX = this->left_line[y];
            int rightX = this->right_line[y];
            if (leftX < 0 || rightX < 0)
            {
                return result;
            }
            if (leftX > rightX)
            {
                std::swap(leftX, rightX);
            }

            const int roadWidth = rightX - leftX;
            if (roadWidth < 20)
            {
                return result;
            }

            const int margin = std::max(2, static_cast<int>(roadWidth * 0.05));
            leftX = std::max(0, leftX + margin);
            rightX = std::min(width - 1, rightX - margin);
            if (rightX <= leftX)
            {
                return result;
            }

            const uchar *rowPtr = this->zebra_image.ptr<uchar>(y);
            const int minWhiteRunWidth = 3;
            std::vector<std::pair<int, int>> whiteRuns;
            bool inWhite = false;
            int startX = leftX;

            for (int x = leftX; x <= rightX; x++)
            {
                const bool isWhite = rowPtr[x] == gBinWhilePointValue;
                if (isWhite && !inWhite)
                {
                    startX = x;
                    inWhite = true;
                }
                else if (!isWhite && inWhite)
                {
                    int endX = x - 1;
                    if (endX - startX + 1 >= minWhiteRunWidth)
                    {
                        whiteRuns.push_back({startX, endX});
                    }
                    inWhite = false;
                }
            }
            if (inWhite)
            {
                int endX = rightX;
                if (endX - startX + 1 >= minWhiteRunWidth)
                {
                    whiteRuns.push_back({startX, endX});
                }
            }

            result.whiteRuns = static_cast<int>(whiteRuns.size());
            result.jumps = result.whiteRuns * 2;

            std::vector<int> gaps;
            for (size_t i = 1; i < whiteRuns.size(); i++)
            {
                int gap = whiteRuns[i].first - whiteRuns[i - 1].second - 1;
                if (gap >= 2)
                {
                    gaps.push_back(gap);
                }
            }

            if (!gaps.empty())
            {
                result.minGap = *std::min_element(gaps.begin(), gaps.end());
                result.maxGap = *std::max_element(gaps.begin(), gaps.end());
                const double gapRatioMax = RuntimeConfig::getZebraGapRatioMax();
                result.gapUniform = (result.minGap > 0 && result.maxGap <= result.minGap * gapRatioMax);
            }

            const int jumpMin = RuntimeConfig::getZebraJumpMinCount();
            result.hit = (result.jumps >= jumpMin) &&
                         (result.whiteRuns >= std::max(2, (jumpMin + 1) / 2)) &&
                         result.gapUniform;
            return result;
        };

        /*
         * 区域型确认：在目标高度上下的小带状区域内取多条扫描线。
         * 这样某一条线正好压在黑缝/白条边缘时，不会让整个人行横道漏掉。
         * 仍然只统计左右赛道线之间的白色跳变，不会对整张图盲扫。
         */
        struct RegionZebraResult
        {
            int hitRows = 0;
            int sampleRows = 0;
            int roadPixels = 0;
            int whitePixels = 0;
            int qualifiedComponents = 0;
            double whiteAreaRatio = 0.0;
            RowZebraResult best;
            bool hit = false;
        };

        auto checkRegion = [&](double centerYRatio) -> RegionZebraResult
        {
            RegionZebraResult region;
            const int sampleRows = std::max(3, RuntimeConfig::getZebraRegionSampleRows());
            const int minHitRows = std::max(1, RuntimeConfig::getZebraRegionMinHitRows());
            const double halfHeight = RuntimeConfig::getZebraRegionHalfHeightRatio();

            /*
             * 第一层：多条横向扫描线必须都有“重复白条 + 间隔均匀”。
             * 第二、三层不再只看点：把整个带状区域内、左右赛道线之间的白色像素
             * 收集成 mask，再计算总白色覆盖率和连通白条的面积/高度。
             */
            region.sampleRows = sampleRows;
            for (int i = 0; i < sampleRows; ++i)
            {
                const double alpha = (sampleRows <= 1)
                                         ? 0.0
                                         : (static_cast<double>(i) / static_cast<double>(sampleRows - 1) * 2.0 - 1.0);
                const double ratio = std::max(0.0, std::min(1.0, centerYRatio + alpha * halfHeight));
                RowZebraResult one = checkRow(ratio);

                if (one.hit)
                {
                    region.hitRows++;
                }

                if (one.whiteRuns > region.best.whiteRuns ||
                    (one.whiteRuns == region.best.whiteRuns && one.jumps > region.best.jumps))
                {
                    region.best = one;
                }
            }

            const int yCenter = std::max(0, std::min(height - 1, static_cast<int>(height * centerYRatio)));
            const int halfPixels = std::max(1, static_cast<int>(height * halfHeight));
            const int yStart = std::max(0, yCenter - halfPixels);
            const int yEnd = std::min(height - 1, yCenter + halfPixels);
            /* 只对当前带状区域做连通域，避免把整幅图的零散白点都送进连通域计算。 */
            cv::Mat regionWhiteMask = cv::Mat::zeros(yEnd - yStart + 1, width, CV_8UC1);

            for (int y = yStart; y <= yEnd; ++y)
            {
                int leftX = this->left_line[y];
                int rightX = this->right_line[y];
                if (leftX < 0 || rightX < 0)
                {
                    continue;
                }
                if (leftX > rightX)
                {
                    std::swap(leftX, rightX);
                }
                const int roadWidth = rightX - leftX;
                if (roadWidth < 20)
                {
                    continue;
                }
                const int margin = std::max(2, static_cast<int>(roadWidth * 0.05));
                leftX = std::max(0, leftX + margin);
                rightX = std::min(width - 1, rightX - margin);
                if (rightX <= leftX)
                {
                    continue;
                }

                const uchar *src = this->zebra_image.ptr<uchar>(y);
                uchar *dst = regionWhiteMask.ptr<uchar>(y - yStart);
                for (int x = leftX; x <= rightX; ++x)
                {
                    region.roadPixels++;
                    if (src[x] == gBinWhilePointValue)
                    {
                        dst[x] = gBinWhilePointValue;
                        region.whitePixels++;
                    }
                }
            }

            if (region.roadPixels > 0)
            {
                region.whiteAreaRatio = static_cast<double>(region.whitePixels) /
                                        static_cast<double>(region.roadPixels);
            }

            const bool rowEvidence = (region.hitRows >= minHitRows);
            const bool areaEvidence = region.whiteAreaRatio >= RuntimeConfig::getZebraRegionWhiteAreaRatioMin() &&
                                      region.whiteAreaRatio <= RuntimeConfig::getZebraRegionWhiteAreaRatioMax();

            /*
             * 前两关不通过时无需计算连通域，既减少零散噪声影响，也避免把全图噪声
             * 反复做耗时统计；只有确实像斑马线的区域才继续检查“白条面积/高度”。
             */
            if (rowEvidence && areaEvidence)
            {
                cv::Mat labels, stats, centroids;
                const int componentCount = cv::connectedComponentsWithStats(
                    regionWhiteMask, labels, stats, centroids, 8, CV_32S);
                const int bandHeight = std::max(1, yEnd - yStart + 1);
                const int minArea = std::max(3, static_cast<int>(
                    region.roadPixels * RuntimeConfig::getZebraComponentMinAreaRatio()));
                const int minHeight = std::max(2, static_cast<int>(
                    bandHeight * RuntimeConfig::getZebraComponentMinHeightRatio()));

                for (int label = 1; label < componentCount; ++label)
                {
                    const int area = stats.at<int>(label, cv::CC_STAT_AREA);
                    const int componentHeight = stats.at<int>(label, cv::CC_STAT_HEIGHT);
                    if (area >= minArea && componentHeight >= minHeight)
                    {
                        region.qualifiedComponents++;
                    }
                }
            }
            const bool stripeEvidence = region.qualifiedComponents >= RuntimeConfig::getZebraComponentMinCount();
            region.hit = rowEvidence && areaEvidence && stripeEvidence;
            return region;
        };

        /* 远区中心按“第 1/3/... 次、 第 2/4/... 次”实时交替。 */
        const double activeFarCenterY = getActiveZebraFarCenterY();
        RegionZebraResult far = checkRegion(activeFarCenterY);
        RegionZebraResult near = checkRegion(RuntimeConfig::getZebraNearYRatio());

        const auto now = std::chrono::steady_clock::now();
        const int slowHoldMs = RuntimeConfig::getZebraSlowHoldMs();
        const bool farSlowEnable = (RuntimeConfig::getZebraFarSlowEnable() != 0);
        const bool slowHoldActive = std::chrono::duration_cast<std::chrono::milliseconds>(
                                        now - gLastZebraSlowTime)
                                        .count() < slowHoldMs;

        if (far.hit)
        {
            gFarZebraSeen = true;
            gLastZebraSlowTime = now;
            gZebraSlowMode = farSlowEnable;

            if (farSlowEnable && !gLastSlowPrint)
            {
                /* quiet: far zebra pre-detect only changes speed */
                gLastSlowPrint = true;
            }
        }
        else if (slowHoldActive)
        {
            // 远处命中后的保持期继续作为“二次确认窗口”；
            // 只有 zebra_far_slow_enable=1 时才真正降低电机速度。
            gZebraSlowMode = farSlowEnable;
        }
        else
        {
            gZebraSlowMode = false;
            gLastSlowPrint = false;
            gFarZebraSeen = false;
        }

        const bool stopCandidate = near.hit && (far.hit || gFarZebraSeen || slowHoldActive);
        if (stopCandidate)
        {
            this->zebra_state = 1;
            gZebraSlowMode = false;
            gFarZebraSeen = false;

/* quiet: final zebra message is printed in handleZebraSimple() */
            return true;
        }

        this->zebra_state = 0;
        return false;
    }

    FixLineState ImageProcess::FixLine::handleZebraSimple()
    {
        if (this->zebra_state == 1)
        {
            gLastZebraTriggerTime = std::chrono::steady_clock::now();
            clearZebraSlow();

            const int detectedIndex = gZebraTriggerCount + 1;
            const int cycleEnable = RuntimeConfig::getZebraRegionFarCycleEnable();
            std::cout << "[Zebra] DETECT index=" << detectedIndex
                      << " far_center_y=" << gLastFarCenterYUsed
                      << " cycle=" << cycleEnable
                      << std::endl;

            /* 只在真正进入停车回调时增加计数，避免远区预检或冷却期误计数。 */
            gZebraTriggerCount++;

            if (this->zebra_callback)
            {
                this->zebra_callback(true);
            }

            this->zebra_state = 0;
            return FIX_LINE_OK;
        }

        if (zebraCooldownLeftMs() > 0)
        {
            clearZebraSlow();
        }

        return FIX_LINE_OK;
    }
}
#endif
