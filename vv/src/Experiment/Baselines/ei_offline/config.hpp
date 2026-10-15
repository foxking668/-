#pragma once

#include <opencv2/core.hpp>

#include <string>

namespace ei
{
struct EvalConfig
{
    int sourceWidth = 0;
    int sourceHeight = 0;
    double topCropRatio = 0.0;
    double leftCropRatio = 0.0;
    double rightCropRatio = 0.0;
    cv::Scalar hsvLower;
    cv::Scalar hsvUpper;
    double rowStartRatio = 0.0;
    double rowEndRatio = 0.0;
    double centerPratio = 0.5;
    double angleGain = 1.0;
    int bottomWidthDivisor = 5;
};

EvalConfig loadConfig(const std::string& path);
} // namespace ei
