#pragma once

#include "Baselines/ei_offline/methods.hpp"
#include "RealCarExperiment.hpp"
#include "NaturalMissingVisibility.hpp"
#include <cmath>
#include <stdexcept>

namespace Experiment
{
    inline ei::MethodResult runBaseline(const cv::Mat &mask, int method, double centerPratio)
    {
        ei::EvalConfig config;
        config.rowStartRatio = 0.25;
        config.rowEndRatio = 0.75;
        config.centerPratio = centerPratio;
        config.angleGain = 4.0;
        config.bottomWidthDivisor = 5;
        switch (method)
        {
        case 1: return ei::runM1(mask, config);
        case 2: return ei::runM2(mask, config);
        case 3: return ei::runM3(mask, config);
        default: throw std::invalid_argument("baseline adapter supports only M1/M2/M3");
        }
    }

    inline FrameObservation baselineObservation(const ei::MethodResult &result, int height)
    {
        FrameObservation observation;
        const int first = height / 4;
        const int last = height * 3 / 4;
        observation.evaluatedRows = last - first + 1;
        observation.visibilityState = "UNASSESSED_GLOBAL_CENTROID";
        if (result.left.size() == static_cast<std::size_t>(height) &&
            result.right.size() == static_cast<std::size_t>(height))
        {
            std::vector<int> left(height, -1), right(height, -1);
            for (int y = 0; y < height; ++y)
            {
                left[y] = static_cast<int>(result.left[y]);
                right[y] = static_cast<int>(result.right[y]);
            }
            const auto stats = analyzeBoundaryVisibility(left.data(), right.data(), height, first, last, false);
            observation.rawLeftValidRows = stats.rawLeftValidRows;
            observation.rawRightValidRows = stats.rawRightValidRows;
            observation.bilateralRows = stats.bilateralRows;
            observation.leftMissingRows = stats.leftMissingRows;
            observation.rightMissingRows = stats.rightMissingRows;
            observation.bothMissingRows = stats.bothMissingRows;
            observation.visibilityState = stats.visibilityState;
        }
        for (int y = first; y <= last && y < static_cast<int>(result.center.size()); ++y)
            if (std::isfinite(result.center[y]) && result.center[y] >= 0)
                ++observation.centerValidRows;
        return observation;
    }
}
