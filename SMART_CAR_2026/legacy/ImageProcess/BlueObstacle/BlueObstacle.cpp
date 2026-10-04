#include "BlueObstacle.hpp"

using namespace Other;

namespace ImageProcess
{
    BlueObstacleDetector::BlueObstacleDetector()
        : last_side_(BlueObstacleSide::UNKNOWN),
          lost_frames_(0),
          held_cone_area_(0.0),
          hold_frames_left_(0),
          held_board_side_(BlueObstacleSide::UNKNOWN),
          held_board_center_(0.0f, 0.0f),
          held_board_score_(0.0),
          board_hold_frames_left_(0)
    {
    }

    cv::Mat BlueObstacleDetector::buildBlueMask(const cv::Mat &bgrFrame) const
    {
        cv::Mat hsv;
        cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(hsv, cv::Scalar(90, 60, 45), cv::Scalar(135, 255, 255), mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        return mask;
    }

    cv::Mat BlueObstacleDetector::buildBlackMask(const cv::Mat &bgrFrame) const
    {
        cv::Mat hsv;
        cv::cvtColor(bgrFrame, hsv, cv::COLOR_BGR2HSV);

        cv::Mat mask;
        cv::inRange(hsv,
                    cv::Scalar(0, 0, 0),
                    cv::Scalar(179, RuntimeConfig::getBlueBoardBlackSMax(), RuntimeConfig::getBlueBoardBlackVMax()),
                    mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        return mask;
    }

    cv::Mat BlueObstacleDetector::buildConeDetectMask(const cv::Size &frameSize,
                                                      const int *leftLine,
                                                      const int *rightLine) const
    {
        cv::Mat roiMask(frameSize.height, frameSize.width, CV_8UC1, cv::Scalar(0));

        const int width = frameSize.width;
        const int height = frameSize.height;
        if (width <= 0 || height <= 0 || leftLine == nullptr || rightLine == nullptr)
        {
            return roiMask;
        }

        /*
         * 蓝色桶锥只看赛道内部的蓝色 HSV 掩膜白块。
         *
         * 这里不分析普通巡线二值图里的所有白色，只分析蓝色 HSV mask 中的白色块；
         * 因此红绿灯、人行横道、黄色/白色方块不会进入桶锥候选。
         *
         * 为了不把靠边的桶锥裁掉，赛道内部不再额外收缩太多，只要 leftLine/rightLine
         * 可靠，就从左线到右线之间直接取 ROI；没有可靠边线的行直接跳过。
         */
        const int topRejectY = static_cast<int>(height * 0.02);
        const int bottomRejectY = static_cast<int>(height * 0.90);
        const int innerMargin = 0;

        for (int y = 0; y < height; y++)
        {
            if (y < topRejectY || y >= bottomRejectY)
            {
                continue;
            }

            if (leftLine[y] < 0 || rightLine[y] < 0 || rightLine[y] <= leftLine[y])
            {
                continue;
            }

            int x1 = std::max(0, leftLine[y] + innerMargin);
            int x2 = std::min(width - 1, rightLine[y] - innerMargin);

            if (x2 - x1 < width * 0.10)
            {
                continue;
            }

            cv::line(roiMask, cv::Point(x1, y), cv::Point(x2, y), cv::Scalar(255), 1);
        }

        return roiMask;
    }

    bool BlueObstacleDetector::isConeCandidate(const BlobInfo &blob,
                                               const cv::Size &frameSize) const
    {
        const double frameArea = static_cast<double>(frameSize.width * frameSize.height);
        const double minArea = std::max(RuntimeConfig::getBlueConeMinAreaAbsolute(),
                                        frameArea * RuntimeConfig::getBlueConeMinAreaRatio());

        /*
         * 蓝色桶锥候选：接受赛道内部的蓝色白块。
         * 后续再根据两个块/单块预警/上方挡板进行分类。
         */
        const double maxArea = std::max(18.0, frameArea * 0.045);

        if (blob.area < minArea || blob.area > maxArea)
        {
            return false;
        }

        if (blob.box.width < 2 || blob.box.height < 2)
        {
            return false;
        }

        if (blob.box.width > frameSize.width * 0.32 ||
            blob.box.height > frameSize.height * 0.40)
        {
            return false;
        }

        if (blob.box.y < frameSize.height * 0.02 ||
            blob.box.y + blob.box.height > frameSize.height * 0.92)
        {
            return false;
        }

        const double aspect = static_cast<double>(blob.box.height) / std::max(1, blob.box.width);
        if (aspect < 0.25 || aspect > 4.00)
        {
            return false;
        }

        const double rectArea = static_cast<double>(blob.box.width * blob.box.height);
        const double fillRatio = rectArea > 1.0 ? blob.area / rectArea : 0.0;
        if (fillRatio < 0.12)
        {
            return false;
        }

        return true;
    }

    std::vector<BlueObstacleDetector::BlobInfo> BlueObstacleDetector::findBlueBlobs(
        const cv::Mat &mask,
        const cv::Size &frameSize) const
    {
        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        std::vector<BlobInfo> blobs;
        for (const auto &contour : contours)
        {
            double area = cv::contourArea(contour);
            cv::Rect box = cv::boundingRect(contour);
            if (box.width < 2 || box.height < 2)
            {
                continue;
            }

            cv::Moments moments = cv::moments(contour);
            cv::Point2f center(
                static_cast<float>(box.x + box.width * 0.5),
                static_cast<float>(box.y + box.height * 0.5));

            if (moments.m00 > 0.0)
            {
                center.x = static_cast<float>(moments.m10 / moments.m00);
                center.y = static_cast<float>(moments.m01 / moments.m00);
            }

            BlobInfo blob;
            blob.box = box;
            blob.center = center;
            blob.area = area;

            if (isConeCandidate(blob, frameSize))
            {
                blobs.push_back(blob);
            }
        }

        std::sort(blobs.begin(), blobs.end(), [](const BlobInfo &a, const BlobInfo &b)
                  { return a.center.y < b.center.y; });

        return blobs;
    }

    float BlueObstacleDetector::getReferenceCenterX(const int *centerLine,
                                                    int y,
                                                    int imageWidth) const
    {
        if (centerLine == nullptr)
        {
            return imageWidth * 0.5f;
        }

        y = std::max(0, y);
        if (centerLine[y] >= 0 && centerLine[y] < imageWidth)
        {
            return static_cast<float>(centerLine[y]);
        }

        return imageWidth * 0.5f;
    }

    bool BlueObstacleDetector::detectConePair(const std::vector<BlobInfo> &blobs,
                                              const cv::Size &frameSize,
                                              const int *centerLine,
                                              std::vector<cv::Point2f> *centers,
                                              double *coneArea,
                                              BlueObstacleSide *side) const
    {
        if (blobs.size() < 2 || centers == nullptr || coneArea == nullptr || side == nullptr)
        {
            return false;
        }

        /*
         * 只接受“两个竖向有间隔的小蓝块”。
         *
         * dx 不能太大：排除两侧蓝色边线/蓝色背景拼成一组。
         * dy 不能太小：两个白色蓝块必须有可见间隔。
         * dy 不能太大：距离过远的两个蓝块不是同一个桶锥，直接舍掉。
         */
        const float maxXDistance = std::max(5.0f, frameSize.width * 0.18f);
        const float minYDistance = std::max(3.0f, frameSize.height * 0.020f);
        const float maxYDistance = std::max(minYDistance + 1.0f, frameSize.height * 0.34f);
        const float minCenterY = frameSize.height * 0.04f;
        const float maxCenterY = frameSize.height * 0.86f;

        double bestScore = -1.0;
        cv::Point2f bestA;
        cv::Point2f bestB;
        double bestArea = 0.0;
        BlueObstacleSide bestSide = BlueObstacleSide::UNKNOWN;

        for (size_t i = 0; i < blobs.size(); i++)
        {
            for (size_t j = i + 1; j < blobs.size(); j++)
            {
                const BlobInfo &a = blobs[i];
                const BlobInfo &b = blobs[j];

                if (a.center.y < minCenterY || b.center.y < minCenterY ||
                    a.center.y > maxCenterY || b.center.y > maxCenterY)
                {
                    continue;
                }

                const float dx = std::abs(a.center.x - b.center.x);
                const float dy = std::abs(a.center.y - b.center.y);
                if (dx > maxXDistance || dy < minYDistance || dy > maxYDistance)
                {
                    continue;
                }

                /* 两个小蓝块需要在同一纵向附近，不能横向隔得太远。 */
                const float meanWidth = static_cast<float>(a.box.width + b.box.width) * 0.5f;
                if (dx > std::max(meanWidth * 4.0f, frameSize.width * 0.14f))
                {
                    continue;
                }

                const double areaMax = std::max(a.area, b.area);
                const double areaMin = std::min(a.area, b.area);
                const double areaRatio = areaMax > 1.0 ? areaMin / areaMax : 0.0;
                /*
                 * 桶锥必须是两个有间隔的蓝色/白色掩膜块。
                 * 由于真实摄像头下桶锥会被格线、数字、反光切碎，面积比例不能太死；
                 * 但只看到单个蓝块时一律不算桶锥，防止蓝色边线/背景误判。
                 */
                if (areaRatio < 0.25)
                {
                    continue;
                }

                const int verticalGap = std::max(0,
                                                 std::max(a.box.y, b.box.y) -
                                                     std::min(a.box.y + a.box.height, b.box.y + b.box.height));
                const int minVerticalGap = std::max(1, static_cast<int>(frameSize.height * 0.006));
                const int maxVerticalGap = std::max(minVerticalGap + 1, static_cast<int>(frameSize.height * 0.26));
                if (verticalGap < minVerticalGap || verticalGap > maxVerticalGap)
                {
                    continue;
                }

                const int ay = std::max(0, std::min(frameSize.height - 1, static_cast<int>(a.center.y + 0.5f)));
                const int by = std::max(0, std::min(frameSize.height - 1, static_cast<int>(b.center.y + 0.5f)));
                const bool aLeft = a.center.x < getReferenceCenterX(centerLine, ay, frameSize.width);
                const bool bLeft = b.center.x < getReferenceCenterX(centerLine, by, frameSize.width);

                /* 两个桶锥必须位于中心线同一侧，否则不把它当成一组竖向桶锥。 */
                if (aLeft != bLeft)
                {
                    continue;
                }

                BlueObstacleSide candidateSide = aLeft ? BlueObstacleSide::LEFT : BlueObstacleSide::RIGHT;
                const double pairArea = a.area + b.area;
                const double score = pairArea + dy * 2.0 - dx * 3.0 + areaRatio * 20.0;

                if (score > bestScore)
                {
                    bestScore = score;
                    bestA = a.center;
                    bestB = b.center;
                    bestArea = pairArea;
                    bestSide = candidateSide;
                }
            }
        }

        if (bestScore < 0.0 || bestSide == BlueObstacleSide::UNKNOWN)
        {
            return false;
        }

        centers->clear();
        if (bestA.y < bestB.y)
        {
            centers->push_back(bestA);
            centers->push_back(bestB);
        }
        else
        {
            centers->push_back(bestB);
            centers->push_back(bestA);
        }

        *coneArea = bestArea;
        *side = bestSide;
        return true;
    }

    bool BlueObstacleDetector::detectConeWarn(const std::vector<BlobInfo> &blobs,
                                              const cv::Size &frameSize,
                                              const int *centerLine,
                                              cv::Point2f *center,
                                              double *coneArea,
                                              BlueObstacleSide *side) const
    {
        if (blobs.empty() || center == nullptr || coneArea == nullptr || side == nullptr)
        {
            return false;
        }

        double bestScore = -1.0;
        BlobInfo best;
        BlueObstacleSide bestSide = BlueObstacleSide::UNKNOWN;

        for (const BlobInfo &blob : blobs)
        {
            const int cy = std::max(0, std::min(frameSize.height - 1, static_cast<int>(blob.center.y + 0.5f)));
            if (blob.center.y < frameSize.height * 0.08f || blob.center.y > frameSize.height * 0.88f)
            {
                continue;
            }

            const float refCenter = getReferenceCenterX(centerLine, cy, frameSize.width);
            BlueObstacleSide candidateSide = BlueObstacleSide::UNKNOWN;
            if (blob.center.x < refCenter - frameSize.width * 0.035f)
            {
                candidateSide = BlueObstacleSide::LEFT;
            }
            else if (blob.center.x > refCenter + frameSize.width * 0.035f)
            {
                candidateSide = BlueObstacleSide::RIGHT;
            }
            else
            {
                continue;
            }

            /*
             * 单块只作为预警：面积不能太大，不能是挡板/背景大蓝区。
             * 预警只减速并轻微躲避，不当作正式双桶锥。
             */
            const double frameArea = static_cast<double>(frameSize.width * frameSize.height);
            if (blob.area > frameArea * 0.028)
            {
                continue;
            }

            const double score = blob.area + blob.box.height * 3.0 - blob.box.width * 1.0;
            if (score > bestScore)
            {
                bestScore = score;
                best = blob;
                bestSide = candidateSide;
            }
        }

        if (bestScore < 0.0 || bestSide == BlueObstacleSide::UNKNOWN)
        {
            return false;
        }

        *center = best.center;
        *coneArea = best.area;
        *side = bestSide;
        return true;
    }

    std::vector<int> BlueObstacleDetector::buildWarnEdgeLine(const cv::Point2f &center,
                                                            BlueObstacleSide side,
                                                            int imageHeight,
                                                            int imageWidth) const
    {
        std::vector<int> line(imageHeight, -1);
        if (imageHeight <= 0 || imageWidth <= 0 || side == BlueObstacleSide::UNKNOWN)
        {
            return line;
        }

        /*
         * 单块预警只做轻微避让：比正式双块桶锥的 safeMargin 小。
         */
        const int warnMargin = std::max(4, static_cast<int>(imageWidth * 0.08));
        int x = static_cast<int>(center.x + 0.5f);
        if (side == BlueObstacleSide::LEFT)
        {
            x += warnMargin;
        }
        else if (side == BlueObstacleSide::RIGHT)
        {
            x -= warnMargin;
        }

        /* 预警线允许映射后落在巡线图外侧，用于数学中线计算。 */
        for (int y = 0; y < imageHeight; y++)
        {
            line[y] = x;
        }
        return line;
    }

    std::vector<int> BlueObstacleDetector::buildEdgeLine(const std::vector<cv::Point2f> &centers,
                                                         BlueObstacleSide side,
                                                         int imageHeight,
                                                         int imageWidth) const
    {
        std::vector<int> line(imageHeight, -1);
        if (centers.size() < 2 || imageHeight <= 0 || imageWidth <= 0)
        {
            return line;
        }

        cv::Point2f top = centers[0];
        cv::Point2f bottom = centers[1];
        if (top.y > bottom.y)
        {
            std::swap(top, bottom);
        }

        const float dy = bottom.y - top.y;
        /*
         * 安全外扩量：蓝色桶锥本身在赛道内部，不能直接把桶锥中心当边线。
         * 左桶锥把虚拟左线向右推，右桶锥把虚拟右线向左推，
         * 这样按原巡线逻辑重新算出的中线会明显避开桶锥。
         */
        const int safeMargin = std::max(5, static_cast<int>(imageWidth * 0.16));

        for (int y = 0; y < imageHeight; y++)
        {
            float x = top.x;
            if (std::abs(dy) > 1.0f)
            {
                const float ratio = (static_cast<float>(y) - top.y) / dy;
                x = top.x + (bottom.x - top.x) * ratio;
            }

            if (side == BlueObstacleSide::LEFT)
            {
                x += safeMargin;
            }
            else if (side == BlueObstacleSide::RIGHT)
            {
                x -= safeMargin;
            }

            /* 正式桶锥虚拟线允许后续映射到巡线图外侧，只参与数学中线计算。 */
            int xi = static_cast<int>(x + 0.5f);
            line[y] = xi;
        }

        return line;
    }

    bool BlueObstacleDetector::detectBoardFallback(const cv::Mat &blueMask,
                                                   const cv::Mat &blackMask,
                                                   const cv::Size &frameSize,
                                                   const int *leftLine,
                                                   const int *rightLine,
                                                   const int *centerLine,
                                                   BlueObstacleSide *side,
                                                   double *score,
                                                   cv::Point2f *center) const
    {
        if (side == nullptr || score == nullptr || center == nullptr ||
            blueMask.empty() || frameSize.width <= 0 || frameSize.height <= 0 ||
            leftLine == nullptr || rightLine == nullptr)
        {
            return false;
        }

        *side = BlueObstacleSide::UNKNOWN;
        *score = 0.0;
        *center = cv::Point2f(0.0f, 0.0f);

        const int width = frameSize.width;
        const int height = frameSize.height;
        const int roiEndY = std::max(3, std::min(height, static_cast<int>(height * RuntimeConfig::getBlueBoardRoiYRatioEnd())));

        /*
         * 蓝色挡板兜底也只使用“巡线裁剪图 frame”里的赛道内部上方。
         *
         * 重要修正：
         * 1. 不再统计整幅图左/右半边；
         * 2. 不再把普通黑区直接当挡板，黑区太容易把背景、阴影、赛道边缘算进去；
         * 3. 必须存在一个位于赛道内部的“大蓝色连通块”，才认为是蓝色挡板。
         *
         * 这样左右两侧蓝色背景不会触发 [BlueBoard]。
         */
        cv::Mat boardRoiMask(height, width, CV_8UC1, cv::Scalar(0));
        double roadArea = 0.0;
        const int margin = std::max(3, static_cast<int>(width * 0.025));

        for (int y = 0; y < roiEndY; y++)
        {
            if (leftLine[y] < 0 || rightLine[y] < 0 || rightLine[y] <= leftLine[y])
            {
                continue;
            }

            const int x1 = std::max(0, leftLine[y] + margin);
            const int x2 = std::min(width - 1, rightLine[y] - margin);
            if (x2 <= x1 || x2 - x1 < width * 0.18)
            {
                continue;
            }

            cv::line(boardRoiMask, cv::Point(x1, y), cv::Point(x2, y), cv::Scalar(255), 1);
            roadArea += static_cast<double>(x2 - x1 + 1);
        }

        if (roadArea <= 10.0)
        {
            return false;
        }

        cv::Mat boardMask;
        cv::bitwise_and(blueMask, boardRoiMask, boardMask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(boardMask, boardMask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(boardMask, boardMask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(boardMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        const double minArea = std::max(60.0, roadArea * RuntimeConfig::getBlueBoardBlackMinAreaRatio());
        double bestArea = 0.0;
        cv::Rect bestBox;
        cv::Point2f bestCenter(0.0f, 0.0f);

        for (const auto &contour : contours)
        {
            const double area = cv::contourArea(contour);
            if (area < minArea)
            {
                continue;
            }

            const cv::Rect box = cv::boundingRect(contour);
            if (box.width < width * 0.12 || box.height < height * 0.06)
            {
                continue;
            }

            const double aspect = static_cast<double>(box.width) / std::max(1, box.height);
            if (aspect < 0.80 || aspect > 8.00)
            {
                continue;
            }

            cv::Moments moments = cv::moments(contour);
            cv::Point2f c(static_cast<float>(box.x + box.width * 0.5f),
                          static_cast<float>(box.y + box.height * 0.5f));
            if (moments.m00 > 1.0)
            {
                c.x = static_cast<float>(moments.m10 / moments.m00);
                c.y = static_cast<float>(moments.m01 / moments.m00);
            }

            if (area > bestArea)
            {
                bestArea = area;
                bestBox = box;
                bestCenter = c;
            }
        }

        if (bestArea <= 1.0)
        {
            return false;
        }

        const int cy = std::max(0, std::min(height - 1, static_cast<int>(bestCenter.y + 0.5f)));
        const float refCenter = getReferenceCenterX(centerLine, cy, width);

        BlueObstacleSide boardSide = BlueObstacleSide::UNKNOWN;
        if (bestCenter.x < refCenter - width * 0.04f)
        {
            boardSide = BlueObstacleSide::LEFT;
        }
        else if (bestCenter.x > refCenter + width * 0.04f)
        {
            boardSide = BlueObstacleSide::RIGHT;
        }
        else
        {
            return false;
        }

        *side = boardSide;
        *score = bestArea / std::max(1.0, roadArea);
        *center = bestCenter;
        return true;
    }

    std::vector<int> BlueObstacleDetector::buildBoardEdgeLine(BlueObstacleSide side,
                                                              int imageHeight,
                                                              int imageWidth) const
    {
        std::vector<int> line(imageHeight, -1);
        if (imageHeight <= 0 || imageWidth <= 0 || side == BlueObstacleSide::UNKNOWN)
        {
            return line;
        }

        /*
         * 挡板兜底虚拟线：
         * 右侧挡板 -> 把虚拟右边线压到画面偏中位置，中线向左，车向左躲；
         * 左侧挡板 -> 把虚拟左边线压到画面偏中位置，中线向右，车向右躲。
         */
        const int x = (side == BlueObstacleSide::RIGHT)
                          ? std::max(0, std::min(imageWidth - 1, static_cast<int>(imageWidth * 0.58)))
                          : std::max(0, std::min(imageWidth - 1, static_cast<int>(imageWidth * 0.42)));

        for (int y = 0; y < imageHeight; y++)
        {
            line[y] = x;
        }

        return line;
    }

    const char *BlueObstacleDetector::sideToString(BlueObstacleSide side) const
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

    BlueObstacleResult BlueObstacleDetector::update(const cv::Mat &bgrFrame,
                                                    const int *leftLine,
                                                    const int *rightLine,
                                                    const int *centerLine)
    {
        BlueObstacleResult result;

        if (bgrFrame.empty())
        {
            last_side_ = BlueObstacleSide::UNKNOWN;
            lost_frames_ = 0;
            hold_frames_left_ = 0;
            held_edge_line_.clear();
            held_centers_.clear();
            held_cone_area_ = 0.0;
            held_board_side_ = BlueObstacleSide::UNKNOWN;
            held_board_edge_line_.clear();
            held_board_center_ = cv::Point2f(0.0f, 0.0f);
            held_board_score_ = 0.0;
            board_hold_frames_left_ = 0;
            return result;
        }

        cv::Mat blueMask = buildBlueMask(bgrFrame);
        cv::Mat blackMask = buildBlackMask(bgrFrame);
        cv::Mat coneRoiMask = buildConeDetectMask(bgrFrame.size(), leftLine, rightLine);
        cv::Mat blueConeMask;
        cv::bitwise_and(blueMask, coneRoiMask, blueConeMask);

        std::vector<BlobInfo> blobs = findBlueBlobs(blueConeMask, bgrFrame.size());

        std::vector<cv::Point2f> centers;
        double coneArea = 0.0;
        BlueObstacleSide side = BlueObstacleSide::UNKNOWN;

        if (!detectConePair(blobs, bgrFrame.size(), centerLine, &centers, &coneArea, &side))
        {
            /*
             * 丢帧保持：上一帧/前几帧已经确认过蓝色桶锥时，继续使用缓存的虚拟边线。
             * 这样蓝色桶锥不会因为单帧 HSV 抖动而“偏一下又马上回正”。
             */
            if (hold_frames_left_ > 0 &&
                last_side_ != BlueObstacleSide::UNKNOWN &&
                !held_edge_line_.empty())
            {
                hold_frames_left_--;
                lost_frames_++;

                result.type = BlueObstacleType::CONE_PAIR;
                result.side = last_side_;
                result.centers = held_centers_;
                result.cone_area = held_cone_area_;
                result.cone_area_ratio = 1.0;
                result.edge_line = held_edge_line_;
                result.use_edge_line = true;
                result.return_slow_mode = true;
                result.virtual_line_hold = true;

                return result;
            }

            BlueObstacleSide boardSide = BlueObstacleSide::UNKNOWN;
            double boardScore = 0.0;
            cv::Point2f boardCenter;

            if (detectBoardFallback(blueMask, blackMask, bgrFrame.size(), leftLine, rightLine, centerLine, &boardSide, &boardScore, &boardCenter))
            {
                result.type = BlueObstacleType::BOARD_FALLBACK;
                result.side = boardSide;
                result.centers.clear();
                result.centers.push_back(boardCenter);
                result.cone_area = boardScore;
                result.board_black_area = boardScore;
                result.edge_line = buildBoardEdgeLine(boardSide, bgrFrame.rows, bgrFrame.cols);
                result.use_edge_line = true;
                result.return_slow_mode = true;
                result.virtual_line_hold = false;

                const bool shouldPrintBoard = (boardSide != held_board_side_) || (board_hold_frames_left_ <= 0);
                held_board_side_ = boardSide;
                held_board_center_ = boardCenter;
                held_board_score_ = boardScore;
                held_board_edge_line_ = result.edge_line;
                board_hold_frames_left_ = 10;

                if (shouldPrintBoard)
                {
                    std::cout << "[BlueBoard] side=" << sideToString(boardSide)
                              << " avoid=" << (boardSide == BlueObstacleSide::RIGHT ? "LEFT" : "RIGHT")
                              << std::endl;
                }

                return result;
            }

            if (board_hold_frames_left_ > 0 &&
                held_board_side_ != BlueObstacleSide::UNKNOWN &&
                !held_board_edge_line_.empty())
            {
                board_hold_frames_left_--;

                result.type = BlueObstacleType::BOARD_FALLBACK;
                result.side = held_board_side_;
                result.centers.clear();
                result.centers.push_back(held_board_center_);
                result.cone_area = held_board_score_;
                result.board_black_area = held_board_score_;
                result.edge_line = held_board_edge_line_;
                result.use_edge_line = true;
                result.return_slow_mode = true;
                result.virtual_line_hold = true;

                return result;
            }

            /*
             * 单蓝块预警：
             * 没有双块桶锥，也没有上方大块挡板时，如果在中线一侧看到一个合理蓝色块，
             * 不正式判定桶锥，只进入低速并轻微避让。
             */
            cv::Point2f warnCenter;
            double warnArea = 0.0;
            BlueObstacleSide warnSide = BlueObstacleSide::UNKNOWN;
            if (detectConeWarn(blobs, bgrFrame.size(), centerLine, &warnCenter, &warnArea, &warnSide))
            {
                result.type = BlueObstacleType::CONE_WARN;
                result.side = warnSide;
                result.centers.clear();
                result.centers.push_back(warnCenter);
                result.cone_area = warnArea;
                result.cone_area_ratio = 1.0;
                result.edge_line = buildWarnEdgeLine(warnCenter, warnSide, bgrFrame.rows, bgrFrame.cols);
                result.use_edge_line = true;
                result.return_slow_mode = true;
                result.virtual_line_hold = false;

                static BlueObstacleSide lastWarnPrintSide = BlueObstacleSide::UNKNOWN;
                static int warnPrintCooldown = 0;
                if (warnSide != lastWarnPrintSide || warnPrintCooldown <= 0)
                {
                    std::cout << "[BlueConeWarn] side=" << sideToString(warnSide)
                              << " avoid=" << (warnSide == BlueObstacleSide::LEFT ? "RIGHT" : "LEFT")
                              << std::endl;
                    lastWarnPrintSide = warnSide;
                    warnPrintCooldown = 12;
                }
                else
                {
                    warnPrintCooldown--;
                }

                return result;
            }

            if (last_side_ != BlueObstacleSide::UNKNOWN)
            {
                lost_frames_++;
                if (lost_frames_ >= RuntimeConfig::getBlueLostFramesTrigger())
                {
/* quiet: blue cone lost */
                    last_side_ = BlueObstacleSide::UNKNOWN;
                    lost_frames_ = 0;
                    hold_frames_left_ = 0;
                    held_edge_line_.clear();
                    held_centers_.clear();
                    held_cone_area_ = 0.0;
                }
            }
            return result;
        }

        result.type = BlueObstacleType::CONE_PAIR;
        result.side = side;
        result.centers = centers;
        result.cone_area = coneArea;
        result.cone_area_ratio = 1.0;
        result.edge_line = buildEdgeLine(centers, side, bgrFrame.rows, bgrFrame.cols);
        result.use_edge_line = true;
        result.return_slow_mode = true;
        result.virtual_line_hold = false;

        /* 桶锥识别优先级高于挡板兜底，识别到桶锥后清掉挡板保持。 */
        held_board_side_ = BlueObstacleSide::UNKNOWN;
        held_board_edge_line_.clear();
        held_board_score_ = 0.0;
        board_hold_frames_left_ = 0;

        /*
         * 检测到可靠桶锥后刷新虚拟边线缓存。
         * 约 12 帧保持，实际时间取决于摄像头帧率，足够覆盖短暂丢帧又不会一直干扰正常巡线。
         */
        held_edge_line_ = result.edge_line;
        held_centers_ = centers;
        held_cone_area_ = coneArea;
        hold_frames_left_ = 12;

        if (side != last_side_)
        {
            std::cout << "[BlueCone] side=" << sideToString(side)
                      << " avoid=" << (side == BlueObstacleSide::LEFT ? "RIGHT" : "LEFT")
                      << std::endl;
        }

        last_side_ = side;
        lost_frames_ = 0;
        return result;
    }
}
