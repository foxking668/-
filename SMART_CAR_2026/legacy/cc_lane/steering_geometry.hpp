#pragma once
#include <algorithm>
#include <cmath>
namespace car2026 { namespace capture { namespace cc_detail {
    [[maybe_unused]]
    static double centerToError(double centerX, int h, int w, double centerPratio)
    {
        const double yMid = h * 0.50;
        const double rad = std::atan2(centerX - w * centerPratio, std::max(1.0, h - yMid));
        return rad * 180.0 / std::acos(-1.0) * 4.0;
    }

}}}
