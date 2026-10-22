#pragma once

#include "ei_offline/config.hpp"

#include <opencv2/core.hpp>

#include <limits>
#include <string>
#include <vector>

namespace ei
{
struct Preprocessed
{
    cv::Mat resized;
    cv::Mat roi;
    cv::Mat mask;
};

struct MethodResult
{
    std::string name;
    bool valid = false;
    double aggregateCenter = std::numeric_limits<double>::quiet_NaN();
    double angleDeg = std::numeric_limits<double>::quiet_NaN();
    int validRows = 0;
    double latencyMs = 0.0;
    std::vector<double> left;
    std::vector<double> center;
    std::vector<double> right;
    std::string reason;
};

Preprocessed preprocess(const cv::Mat& image, const EvalConfig& config);
MethodResult runM1(const cv::Mat& mask, const EvalConfig& config);
MethodResult runM2(const cv::Mat& mask, const EvalConfig& config);
MethodResult runM3(const cv::Mat& mask, const EvalConfig& config);
MethodResult runM4_725(const cv::Mat& mask, const EvalConfig& config);
std::vector<MethodResult> runAblations(const cv::Mat& mask, const EvalConfig& config);
std::vector<MethodResult> runExtractionControl(const cv::Mat& mask, const EvalConfig& config);
} // namespace ei
