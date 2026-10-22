#include "ei_offline/methods.hpp"

#include <opencv2/imgproc.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <utility>

namespace
{
using Clock = std::chrono::steady_clock;
using Run = std::pair<int, int>;

double elapsedMilliseconds(const Clock::time_point start)
{
    return std::chrono::duration<double, std::milli>(Clock::now() - start).count();
}

void requireMask(const cv::Mat& mask)
{
    if (mask.empty() || mask.type() != CV_8UC1)
    {
        throw std::runtime_error("method input must be a non-empty CV_8UC1 mask");
    }
}

std::pair<int, int> steeringRows(const int height, const ei::EvalConfig& config)
{
    const int a = static_cast<int>(height * config.rowStartRatio);
    const int b = static_cast<int>(height * config.rowEndRatio);
    return {std::min(a, b), std::max(a, b)};
}

bool aggregateCenter(
    const std::vector<double>& center,
    const ei::EvalConfig& config,
    double& aggregate,
    int& count)
{
    const auto rows = steeringRows(static_cast<int>(center.size()), config);
    double sum = 0.0;
    count = 0;
    for (int y = rows.first; y <= rows.second; ++y)
    {
        const double value = center.at(static_cast<std::size_t>(y));
        if (std::isfinite(value) && value >= 0.0)
        {
            sum += value;
            ++count;
        }
    }
    if (count == 0)
    {
        aggregate = std::numeric_limits<double>::quiet_NaN();
        return false;
    }
    aggregate = sum / count;
    return true;
}

double angleFromCenter(
    const double centerX,
    const int height,
    const int width,
    const ei::EvalConfig& config)
{
    const auto rows = steeringRows(height, config);
    const double deltaY = rows.first + (rows.second - rows.first) / 2.0;
    const double radians = std::atan2(
        centerX - width * config.centerPratio,
        height - deltaY);
    return radians * 180.0 / std::acos(-1.0) * config.angleGain;
}

std::vector<Run> contiguousRuns(const cv::Mat& mask, const int y)
{
    std::vector<Run> runs;
    int start = -1;
    for (int x = 0; x < mask.cols; ++x)
    {
        const bool white = mask.at<unsigned char>(y, x) > 0;
        if (white && start < 0)
        {
            start = x;
        }
        if (start >= 0 && (!white || x == mask.cols - 1))
        {
            const int end = white && x == mask.cols - 1 ? x : x - 1;
            runs.emplace_back(start, end);
            start = -1;
        }
    }
    return runs;
}

std::vector<double> trackBoundary(const cv::Mat& mask, const bool leftSide, const int maxStep)
{
    std::vector<std::vector<int>> candidates(static_cast<std::size_t>(mask.rows));
    for (int y = 0; y < mask.rows; ++y)
    {
        for (int x = 0; x < mask.cols; ++x)
        {
            if (mask.at<unsigned char>(y, x) == 0)
            {
                continue;
            }
            const bool isBoundary = leftSide
                ? (x == 0 || mask.at<unsigned char>(y, x - 1) == 0)
                : (x == mask.cols - 1 || mask.at<unsigned char>(y, x + 1) == 0);
            if (isBoundary)
            {
                candidates.at(static_cast<std::size_t>(y)).push_back(x);
            }
        }
    }

    std::vector<double> output(static_cast<std::size_t>(mask.rows), -1.0);
    const std::vector<int>& bottom = candidates.back();
    if (bottom.empty())
    {
        return output;
    }

    int current = leftSide
        ? *std::min_element(bottom.begin(), bottom.end())
        : *std::max_element(bottom.begin(), bottom.end());
    output.back() = current;
    for (int y = mask.rows - 2; y >= 0; --y)
    {
        int best = -1;
        int bestDistance = maxStep + 1;
        for (const int candidate : candidates.at(static_cast<std::size_t>(y)))
        {
            const int distance = std::abs(candidate - current);
            if (distance <= maxStep && distance < bestDistance)
            {
                best = candidate;
                bestDistance = distance;
            }
        }
        if (best >= 0)
        {
            current = best;
            output.at(static_cast<std::size_t>(y)) = current;
        }
    }
    return output;
}
} // namespace

namespace ei
{
MethodResult runM1(const cv::Mat& mask, const EvalConfig& config)
{
    requireMask(mask);
    const auto started = Clock::now();
    MethodResult result;
    result.name = "M1_centroid";

    std::vector<std::vector<cv::Point>> contours;
    cv::Mat copy = mask.clone();
    cv::findContours(copy, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);
    std::stable_sort(contours.begin(), contours.end(), [](const auto& a, const auto& b) {
        return cv::contourArea(a) > cv::contourArea(b);
    });

    const int minimumWidth = mask.rows / config.bottomWidthDivisor;
    const std::vector<cv::Point>* selected = nullptr;
    for (const auto& contour : contours)
    {
        int left = mask.cols;
        int right = -1;
        for (const cv::Point& point : contour)
        {
            if (point.y == mask.rows - 1)
            {
                left = std::min(left, point.x);
                right = std::max(right, point.x);
            }
        }
        if (right >= left && right - left >= minimumWidth)
        {
            selected = &contour;
            break;
        }
    }

    if (selected == nullptr)
    {
        result.reason = "no_bottom_connected_contour";
        result.latencyMs = elapsedMilliseconds(started);
        return result;
    }

    const cv::Moments moments = cv::moments(*selected);
    if (std::abs(moments.m00) < 1e-9)
    {
        result.reason = "zero_contour_area";
        result.latencyMs = elapsedMilliseconds(started);
        return result;
    }

    result.aggregateCenter = moments.m10 / moments.m00;
    result.angleDeg = angleFromCenter(result.aggregateCenter, mask.rows, mask.cols, config);
    result.validRows = mask.rows;
    result.center.assign(static_cast<std::size_t>(mask.rows), result.aggregateCenter);
    result.valid = true;
    result.latencyMs = elapsedMilliseconds(started);
    return result;
}

MethodResult runM2(const cv::Mat& mask, const EvalConfig& config)
{
    requireMask(mask);
    const auto started = Clock::now();
    MethodResult result;
    result.name = "M2_scanline";
    result.left.assign(static_cast<std::size_t>(mask.rows), -1.0);
    result.right.assign(static_cast<std::size_t>(mask.rows), -1.0);
    result.center.assign(
        static_cast<std::size_t>(mask.rows),
        std::numeric_limits<double>::quiet_NaN());

    double previousCenter = std::numeric_limits<double>::quiet_NaN();
    for (int y = mask.rows - 1; y >= 0; --y)
    {
        const std::vector<Run> runs = contiguousRuns(mask, y);
        if (runs.empty())
        {
            continue;
        }

        Run selected = runs.front();
        if (!std::isfinite(previousCenter))
        {
            for (const Run& run : runs)
            {
                if (run.second - run.first > selected.second - selected.first)
                {
                    selected = run;
                }
            }
        }
        else
        {
            bool foundContaining = false;
            for (const Run& run : runs)
            {
                if (run.first <= previousCenter && previousCenter <= run.second)
                {
                    if (!foundContaining || run.second - run.first > selected.second - selected.first)
                    {
                        selected = run;
                    }
                    foundContaining = true;
                }
            }
            if (!foundContaining)
            {
                double bestDistance = std::numeric_limits<double>::infinity();
                for (const Run& run : runs)
                {
                    const double midpoint = (run.first + run.second) / 2.0;
                    const double distance = std::abs(midpoint - previousCenter);
                    if (distance < bestDistance)
                    {
                        selected = run;
                        bestDistance = distance;
                    }
                }
            }
        }

        const double center = (selected.first + selected.second) / 2.0;
        result.left.at(static_cast<std::size_t>(y)) = selected.first;
        result.right.at(static_cast<std::size_t>(y)) = selected.second;
        result.center.at(static_cast<std::size_t>(y)) = center;
        previousCenter = center;
    }

    result.valid = aggregateCenter(result.center, config, result.aggregateCenter, result.validRows);
    if (result.valid)
    {
        result.angleDeg = angleFromCenter(result.aggregateCenter, mask.rows, mask.cols, config);
    }
    else
    {
        result.reason = "no_valid_scan_rows";
    }
    result.latencyMs = elapsedMilliseconds(started);
    return result;
}

MethodResult runM3(const cv::Mat& mask, const EvalConfig& config)
{
    requireMask(mask);
    const auto started = Clock::now();
    MethodResult result;
    result.name = "M3_boundary_track";
    result.reason = "prototype_independent_8neighborhood_like_baseline";
    result.left = trackBoundary(mask, true, 2);
    result.right = trackBoundary(mask, false, 2);
    result.center.assign(
        static_cast<std::size_t>(mask.rows),
        std::numeric_limits<double>::quiet_NaN());

    for (int y = 0; y < mask.rows; ++y)
    {
        const double left = result.left.at(static_cast<std::size_t>(y));
        const double right = result.right.at(static_cast<std::size_t>(y));
        if (left >= 0.0 && right >= 0.0 && left < right)
        {
            result.center.at(static_cast<std::size_t>(y)) = (left + right) / 2.0;
        }
    }

    result.valid = aggregateCenter(result.center, config, result.aggregateCenter, result.validRows);
    if (result.valid)
    {
        result.angleDeg = angleFromCenter(result.aggregateCenter, mask.rows, mask.cols, config);
    }
    result.latencyMs = elapsedMilliseconds(started);
    return result;
}
} // namespace ei
