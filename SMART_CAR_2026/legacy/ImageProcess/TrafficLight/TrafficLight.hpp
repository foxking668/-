#pragma once

#include <opencv2/opencv.hpp>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <vector>

namespace ImageProcess
{
    enum class TrafficLightAction
    {
        NONE = 0,
        RED_STOP,
        GREEN_GO
    };

    enum class TrafficLightState
    {
        IDLE = 0,
        WAIT_GREEN,
        COOLDOWN
    };

    struct TrafficLightConfig
    {
        /*
         * 红灯 HSV 阈值。
         *
         * 现在把 S/V 下限提高，减少暗红噪声、反光、赛道杂色误判。
         *
         * 如果红灯识别不到：
         *     red_s_min 可以从 60 降到 50；
         *     red_v_min 可以从 55 降到 45。
         *
         * 如果还是误判：
         *     red_s_min 提到 70；
         *     red_v_min 提到 65。
         */
        int red1_h_min = 0;
        int red1_h_max = 12;
        int red2_h_min = 165;
        int red2_h_max = 179;
        int red_s_min = 60;
        int red_v_min = 55;

        /*
         * 绿灯 HSV 阈值。
         *
         * 原来 green_pixel_min_absolute = 10 太低，
         * 你日志里 green_pixels=10、15 都触发了绿灯，明显太敏感。
         */
        int green_h_min = 40;
        int green_h_max = 90;
        int green_s_min = 55;
        int green_v_min = 50;

        /*
         * ROI 检测区域。
         *
         * 之前是整张 trafficFrame 都检测，容易把赛道、边缘、背景里的颜色误判。
         * 现在只检测中间偏上的区域。
         *
         * 如果红绿灯在画面更靠边：
         *     roi_x_ratio_start 改成 0.05
         *     roi_x_ratio_end 改成 0.95
         *
         * 如果红绿灯在画面更靠下：
         *     roi_y_ratio_end 改成 0.85 或 1.00
         */
        double roi_x_ratio_start = 0.10;
        double roi_x_ratio_end = 0.90;
        double roi_y_ratio_start = 0.20;
        double roi_y_ratio_end = 0.56;

        /*
         * 双 ROI：主 ROI 用于左侧上部，lower ROI 用于弯道起始时常出现的左下红灯。
         * 两个 ROI 都排除画面最顶部背景，红灯需连续多帧确认才停车。
         */
        int dual_roi_enable = 1;
        double lower_roi_y_ratio_start = 0.58;
        double lower_roi_y_ratio_end = 1.00;
        int red_confirm_frames = 2;

        /*
         * 红灯轮廓过滤参数。
         *
         * 你日志里的误判面积是 16.5、20、25，
         * 所以 red_area_min 必须明显大于这些值。
         *
         * 如果真实红灯比较远、比较小，先试 45；
         * 如果还误判，再升到 80。
         */
        double red_circle_min = 0.55;
        double red_area_min = 60.0;
        double red_area_max_ratio = 0.12;

        /*
         * 宽高比限制。
         *
         * 真正圆形斜视后会变椭圆，但不会特别扁长。
         * 原来 0.20 ~ 4.00 太宽，很多杂色块都能过。
         */
        double red_aspect_min = 0.45;
        double red_aspect_max = 2.20;

        /*
         * 绿灯像素数量阈值。
         *
         * 原来：
         *     green_pixel_min_absolute = 10
         *
         * 太低了，日志里 green_pixels=10 就触发。
         * 现在提高到 80。
         *
         * 如果绿灯识别不到：
         *     green_pixel_min_absolute 降到 50；
         *     green_pixel_min_ratio 降到 0.002。
         */
        double green_pixel_min_ratio = 0.003;
        int green_pixel_min_absolute = 80;

        /*
         * 绿灯放行后，5 秒内忽略红灯。
         */
        int red_ignore_after_green_ms = 5000;

        /*
         * 形态学滤波核大小。
         *
         * 3 比较均衡。
         * 如果噪声很多，可以改成 5。
         * 如果真实红灯很小，改成 1。
         */
        int morph_kernel_size = 3;
    };

    /*
     * 调试信息仅用于终端输出，不参与红灯判定。
     * 它能直接说明“红灯模块是否在运行、红色像素有没有进入 ROI、
     * 又是被面积/宽高比/圆度哪一层过滤掉”。
     */
    struct TrafficLightDebugInfo
    {
        int state = 0;
        int frame_width = 0;
        int frame_height = 0;
        int roi_count = 0;
        int red_pixels = 0;
        int contour_count = 0;
        int area_pass_count = 0;
        int aspect_pass_count = 0;
        int circle_pass_count = 0;
        int green_pixels = 0;
        double best_circle = 0.0;
        double best_area = 0.0;
        bool red_found = false;
    };

    class TrafficLightDetector
    {
    public:
        TrafficLightDetector();

        void setConfig(const TrafficLightConfig &config);
        TrafficLightConfig getConfig() const;

        TrafficLightAction update(const cv::Mat &bgrFrame);

        /*
         * 强制进入等待绿灯状态。
         * 用于蓝色挡板/弯道分支：不需要真实检测到红灯，
         * 只要到达配置的停靠点，就等价于“红灯停车”，直到绿灯放行。
         */
        void forceWaitGreen(const char *reason = "manual");

        TrafficLightState getState() const;
        TrafficLightDebugInfo getLastDebugInfo() const;

    private:
        TrafficLightConfig config_;
        TrafficLightState state_;
        std::chrono::steady_clock::time_point cooldown_start_time_;
        int red_confirm_count_;
        TrafficLightDebugInfo last_debug_info_;

        std::vector<cv::Rect> buildRois(const cv::Mat &frame) const;
        cv::Mat preprocessMask(const cv::Mat &mask) const;

        bool detectRedByContour(const cv::Mat &bgrFrame,
                                double *bestCircle = nullptr,
                                double *bestArea = nullptr,
                                TrafficLightDebugInfo *debugInfo = nullptr) const;

        bool detectGreenFast(const cv::Mat &bgrFrame,
                             int *greenPixels = nullptr) const;

        static double absDouble(double value);
    };
}