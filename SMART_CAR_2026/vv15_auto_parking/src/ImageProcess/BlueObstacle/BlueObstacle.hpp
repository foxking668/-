#pragma once

#include "headfile.hpp"

namespace ImageProcess
{
    enum class BlueObstacleType
    {
        NONE = 0,
        CONE_PAIR,
        CONE_WARN,
        BOARD_FALLBACK
    };

    enum class BlueObstacleSide
    {
        UNKNOWN = 0,
        LEFT,
        RIGHT
    };

    struct BlueObstacleResult
    {
        BlueObstacleType type = BlueObstacleType::NONE;
        BlueObstacleSide side = BlueObstacleSide::UNKNOWN;
        std::vector<cv::Point2f> centers;
        std::vector<int> edge_line;
        float steer_bias = 0.0f;
        bool use_edge_line = false;
        double cone_area = 0.0;
        double cone_area_ratio = 1.0;

        /* 检测到蓝色桶锥虚拟边线时，电机进入蓝色低速。 */
        bool return_slow_mode = false;

        /* true 表示本帧没有重新检测到蓝色桶锥，但仍在使用上一帧保持的虚拟边线。 */
        bool virtual_line_hold = false;

        /* 保留字段，兼容 ImageHandle.cpp 里的旧接口；新桶锥逻辑不触发停车分支。 */
        bool curve_stop_branch_request = false;
        const char *curve_stop_reason = "none";

        /* 保留调试字段，当前蓝色桶锥重构版不使用。 */
        double board_black_area = 0.0;
        double board_black_max_area = 0.0;
        double board_black_ratio = 1.0;
        int board_black_decrease_count = 0;
    };

    class BlueObstacleDetector
    {
    public:
        BlueObstacleDetector();

        /*
         * 输入建议使用蓝色专用裁剪后的彩色图，而不是巡线黑白二值图。
         * 本模块内部用 HSV 单独提取蓝色，蓝色目标在 mask 中表现为白色。
         * 检测优先级：
         *   1. 图像上方大块连续蓝色白区 -> 蓝色挡板兜底；
         *   2. 中线一侧两个有间隔的蓝色白块 -> 蓝色桶锥；
         *   3. 中线一侧只有一个蓝色白块 -> 预警，减速并轻微躲避。
         */
        BlueObstacleResult update(const cv::Mat &bgrFrame,
                                  const int *leftLine,
                                  const int *rightLine,
                                  const int *centerLine);

    private:
        struct BlobInfo
        {
            cv::Rect box;
            cv::Point2f center;
            double area = 0.0;
        };

        cv::Mat buildBlueMask(const cv::Mat &bgrFrame) const;
        cv::Mat buildBlackMask(const cv::Mat &bgrFrame) const;
        cv::Mat buildConeDetectMask(const cv::Size &frameSize,
                                    const int *leftLine,
                                    const int *rightLine) const;
        std::vector<BlobInfo> findBlueBlobs(const cv::Mat &mask,
                                            const cv::Size &frameSize) const;

        bool isConeCandidate(const BlobInfo &blob,
                             const cv::Size &frameSize) const;

        bool detectConePair(const std::vector<BlobInfo> &blobs,
                            const cv::Size &frameSize,
                            const int *centerLine,
                            std::vector<cv::Point2f> *centers,
                            double *coneArea,
                            BlueObstacleSide *side) const;

        bool detectConeWarn(const std::vector<BlobInfo> &blobs,
                            const cv::Size &frameSize,
                            const int *centerLine,
                            cv::Point2f *center,
                            double *coneArea,
                            BlueObstacleSide *side) const;

        std::vector<int> buildWarnEdgeLine(const cv::Point2f &center,
                                           BlueObstacleSide side,
                                           int imageHeight,
                                           int imageWidth) const;

        std::vector<int> buildEdgeLine(const std::vector<cv::Point2f> &centers,
                                       BlueObstacleSide side,
                                       int imageHeight,
                                       int imageWidth) const;

        bool detectBoardFallback(const cv::Mat &blueMask,
                                 const cv::Mat &blackMask,
                                 const cv::Size &frameSize,
                                 const int *leftLine,
                                 const int *rightLine,
                                 const int *centerLine,
                                 BlueObstacleSide *side,
                                 double *score,
                                 cv::Point2f *center) const;

        std::vector<int> buildBoardEdgeLine(BlueObstacleSide side,
                                            int imageHeight,
                                            int imageWidth) const;

        float getReferenceCenterX(const int *centerLine, int y, int imageWidth) const;
        const char *sideToString(BlueObstacleSide side) const;

    private:
        BlueObstacleSide last_side_;
        int lost_frames_;

        /*
         * 虚拟边线保持缓存。
         * 蓝色桶锥检测会受曝光、遮挡和运动模糊影响，如果一帧识别不到就立刻恢复正常巡线，
         * 小车会表现为“舵机只轻轻动一下”。这里缓存上一条可靠虚拟边线，短时间丢帧仍继续参与中线计算。
         */
        std::vector<int> held_edge_line_;
        std::vector<cv::Point2f> held_centers_;
        double held_cone_area_;
        int hold_frames_left_;

        /*
         * 蓝色挡板/上方黑区兜底保持。
         * 当桶锥因视野或转弯识别不到时，如果上方一侧出现大蓝挡板或大黑区，
         * 使用这个结果临时生成虚拟边线，让小车先减速并向挡板反方向躲。
         */
        BlueObstacleSide held_board_side_;
        std::vector<int> held_board_edge_line_;
        cv::Point2f held_board_center_;
        double held_board_score_;
        int board_hold_frames_left_;
    };
}
