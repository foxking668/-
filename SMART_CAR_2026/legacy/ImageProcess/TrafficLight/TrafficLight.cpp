#include "TrafficLight.hpp"

namespace ImageProcess
{
    TrafficLightDetector::TrafficLightDetector()
        : state_(TrafficLightState::IDLE),
          cooldown_start_time_(std::chrono::steady_clock::now()),
          red_confirm_count_(0)
    {
    }

    void TrafficLightDetector::setConfig(const TrafficLightConfig &config)
    {
        config_ = config;
    }

    TrafficLightConfig TrafficLightDetector::getConfig() const
    {
        return config_;
    }

    TrafficLightState TrafficLightDetector::getState() const
    {
        return state_;
    }

    TrafficLightDebugInfo TrafficLightDetector::getLastDebugInfo() const
    {
        return last_debug_info_;
    }

    void TrafficLightDetector::forceWaitGreen(const char *reason)
    {
        if (state_ != TrafficLightState::WAIT_GREEN)
        {
            state_ = TrafficLightState::WAIT_GREEN;
            std::cout << "[Traffic] WAIT_GREEN source="
                      << (reason == nullptr ? "manual" : reason)
                      << std::endl;
        }
    }

    double TrafficLightDetector::absDouble(double value)
    {
        return value >= 0.0 ? value : -value;
    }

    std::vector<cv::Rect> TrafficLightDetector::buildRois(const cv::Mat &frame) const
    {
        std::vector<cv::Rect> rois;
        if (frame.empty())
        {
            return rois;
        }

        const int width = frame.cols;
        const int height = frame.rows;

        auto makeRoi = [&](double yStartRatio, double yEndRatio) -> cv::Rect
        {
            int x0 = static_cast<int>(width * config_.roi_x_ratio_start);
            int x1 = static_cast<int>(width * config_.roi_x_ratio_end);
            int y0 = static_cast<int>(height * yStartRatio);
            int y1 = static_cast<int>(height * yEndRatio);

            x0 = std::max(0, std::min(x0, width - 1));
            x1 = std::max(x0 + 1, std::min(x1, width));
            y0 = std::max(0, std::min(y0, height - 1));
            y1 = std::max(y0 + 1, std::min(y1, height));
            return cv::Rect(x0, y0, x1 - x0, y1 - y0);
        };

        rois.push_back(makeRoi(config_.roi_y_ratio_start, config_.roi_y_ratio_end));
        if (config_.dual_roi_enable != 0 &&
            config_.lower_roi_y_ratio_end > config_.lower_roi_y_ratio_start + 0.02)
        {
            rois.push_back(makeRoi(config_.lower_roi_y_ratio_start,
                                   config_.lower_roi_y_ratio_end));
        }
        return rois;
    }

    cv::Mat TrafficLightDetector::preprocessMask(const cv::Mat &mask) const
    {
        cv::Mat result = mask.clone();

        int kernelSize = config_.morph_kernel_size;
        if (kernelSize < 1)
        {
            return result;
        }

        if (kernelSize % 2 == 0)
        {
            kernelSize += 1;
        }

        cv::Mat kernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(kernelSize, kernelSize));

        cv::morphologyEx(result, result, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(result, result, cv::MORPH_CLOSE, kernel);

        return result;
    }

    bool TrafficLightDetector::detectRedByContour(const cv::Mat &bgrFrame,
                                                  double *bestCircle,
                                                  double *bestArea,
                                                  TrafficLightDebugInfo *debugInfo) const
    {
        if (bgrFrame.empty())
        {
            return false;
        }

        const std::vector<cv::Rect> rois = buildRois(bgrFrame);
        if (debugInfo != nullptr)
        {
            debugInfo->roi_count = static_cast<int>(rois.size());
        }

        bool found = false;
        double globalBestCircle = 0.0;
        double globalBestArea = 0.0;

        for (const cv::Rect &roi : rois)
        {
            cv::Mat hsv;
            cv::cvtColor(bgrFrame(roi), hsv, cv::COLOR_BGR2HSV);

            cv::Mat redMask1;
            cv::Mat redMask2;
            cv::Mat redMask;
            cv::inRange(hsv,
                        cv::Scalar(config_.red1_h_min, config_.red_s_min, config_.red_v_min),
                        cv::Scalar(config_.red1_h_max, 255, 255),
                        redMask1);
            cv::inRange(hsv,
                        cv::Scalar(config_.red2_h_min, config_.red_s_min, config_.red_v_min),
                        cv::Scalar(config_.red2_h_max, 255, 255),
                        redMask2);
            cv::bitwise_or(redMask1, redMask2, redMask);
            redMask = preprocessMask(redMask);

            if (debugInfo != nullptr)
            {
                debugInfo->red_pixels += cv::countNonZero(redMask);
            }

            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(redMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
            if (debugInfo != nullptr)
            {
                debugInfo->contour_count += static_cast<int>(contours.size());
            }

            const double roiArea = static_cast<double>(roi.width * roi.height);
            const double maxArea = std::max(config_.red_area_min,
                                            roiArea * config_.red_area_max_ratio);

            for (const auto &contour : contours)
            {
                const double area = cv::contourArea(contour);
                if (area < config_.red_area_min || area > maxArea)
                {
                    continue;
                }
                if (debugInfo != nullptr)
                {
                    debugInfo->area_pass_count++;
                }

                const double perimeter = cv::arcLength(contour, true);
                if (perimeter <= 0.0)
                {
                    continue;
                }

                const double circle = 4.0 * CV_PI * area / (perimeter * perimeter);
                const cv::Rect box = cv::boundingRect(contour);
                if (box.height <= 0)
                {
                    continue;
                }

                const double aspect = static_cast<double>(box.width) / static_cast<double>(box.height);
                if (aspect < config_.red_aspect_min || aspect > config_.red_aspect_max)
                {
                    continue;
                }
                if (debugInfo != nullptr)
                {
                    debugInfo->aspect_pass_count++;
                }

                if (circle > globalBestCircle)
                {
                    globalBestCircle = circle;
                    globalBestArea = area;
                }
                if (circle >= config_.red_circle_min)
                {
                    if (debugInfo != nullptr)
                    {
                        debugInfo->circle_pass_count++;
                    }
                    found = true;
                }
            }
        }

        if (bestCircle != nullptr)
        {
            *bestCircle = globalBestCircle;
        }
        if (bestArea != nullptr)
        {
            *bestArea = globalBestArea;
        }
        return found;
    }

    bool TrafficLightDetector::detectGreenFast(const cv::Mat &bgrFrame,
                                               int *greenPixels) const
    {
        if (bgrFrame.empty())
        {
            return false;
        }

        const std::vector<cv::Rect> rois = buildRois(bgrFrame);
        int totalCount = 0;
        int totalArea = 0;

        for (const cv::Rect &roi : rois)
        {
            cv::Mat hsv;
            cv::cvtColor(bgrFrame(roi), hsv, cv::COLOR_BGR2HSV);

            cv::Mat greenMask;
            cv::inRange(hsv,
                        cv::Scalar(config_.green_h_min, config_.green_s_min, config_.green_v_min),
                        cv::Scalar(config_.green_h_max, 255, 255),
                        greenMask);
            greenMask = preprocessMask(greenMask);
            totalCount += cv::countNonZero(greenMask);
            totalArea += roi.width * roi.height;
        }

        if (greenPixels != nullptr)
        {
            *greenPixels = totalCount;
        }

        const int minPixelsByRatio = static_cast<int>(
            static_cast<double>(totalArea) * config_.green_pixel_min_ratio);
        const int minPixels = std::max(config_.green_pixel_min_absolute, minPixelsByRatio);
        return totalCount >= minPixels;
    }

    TrafficLightAction TrafficLightDetector::update(const cv::Mat &bgrFrame)
    {
        TrafficLightDebugInfo debugInfo;
        debugInfo.state = static_cast<int>(state_);
        if (!bgrFrame.empty())
        {
            debugInfo.frame_width = bgrFrame.cols;
            debugInfo.frame_height = bgrFrame.rows;
        }

        auto saveDebug = [&]()
        {
            debugInfo.state = static_cast<int>(state_);
            last_debug_info_ = debugInfo;
        };

        if (bgrFrame.empty())
        {
            saveDebug();
            return TrafficLightAction::NONE;
        }

        const auto now = std::chrono::steady_clock::now();

        if (state_ == TrafficLightState::COOLDOWN)
        {
            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 now - cooldown_start_time_)
                                 .count();

            if (elapsedMs < config_.red_ignore_after_green_ms)
            {
                saveDebug();
                return TrafficLightAction::NONE;
            }

            state_ = TrafficLightState::IDLE;
        }

        if (state_ == TrafficLightState::WAIT_GREEN)
        {
            int greenPixels = 0;
            const bool greenFound = detectGreenFast(bgrFrame, &greenPixels);
            debugInfo.green_pixels = greenPixels;

            if (greenFound)
            {
                std::cout << "[Traffic] GREEN_GO" << std::endl;

                state_ = TrafficLightState::COOLDOWN;
                cooldown_start_time_ = now;
                saveDebug();
                return TrafficLightAction::GREEN_GO;
            }

            saveDebug();
            return TrafficLightAction::NONE;
        }

        double redCircle = 0.0;
        double redArea = 0.0;

        const bool redFound = detectRedByContour(bgrFrame, &redCircle, &redArea, &debugInfo);
        debugInfo.red_found = redFound;
        debugInfo.best_circle = redCircle;
        debugInfo.best_area = redArea;

        if (redFound)
        {
            red_confirm_count_++;
            const int confirmNeed = std::max(1, config_.red_confirm_frames);
            if (red_confirm_count_ >= confirmNeed)
            {
                std::cout << "[Traffic] RED_WAIT_GREEN confirmed="
                          << red_confirm_count_
                          << " circle=" << redCircle
                          << " area=" << redArea
                          << std::endl;
                red_confirm_count_ = 0;
                state_ = TrafficLightState::WAIT_GREEN;
                saveDebug();
                return TrafficLightAction::RED_STOP;
            }
            saveDebug();
            return TrafficLightAction::NONE;
        }

        red_confirm_count_ = 0;
        saveDebug();
        return TrafficLightAction::NONE;
    }

}