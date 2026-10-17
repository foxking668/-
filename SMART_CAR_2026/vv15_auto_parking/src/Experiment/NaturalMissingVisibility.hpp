#pragma once

#include <algorithm>
#include <string>

namespace Experiment
{
    struct BoundaryVisibilityStats
    {
        int evaluatedRows = 0;
        int rawLeftValidRows = 0;
        int rawRightValidRows = 0;
        int bilateralRows = 0;
        int leftMissingRows = 0;
        int rightMissingRows = 0;
        int bothMissingRows = 0;
        int proxyRows = 0;
        std::string visibilityState = "NO_CONTOUR";
    };

    inline BoundaryVisibilityStats analyzeBoundaryVisibility(
        const int *leftLine,
        const int *rightLine,
        int rowCount,
        int firstRow,
        int lastRow,
        bool edgeProxyEnabled)
    {
        BoundaryVisibilityStats stats;
        if (leftLine == nullptr || rightLine == nullptr || rowCount <= 0)
        {
            return stats;
        }

        const int begin = std::max(0, std::min(firstRow, rowCount - 1));
        const int end = std::max(begin, std::min(lastRow, rowCount - 1));
        for (int row = begin; row <= end; ++row)
        {
            const bool leftValid = leftLine[row] >= 0;
            const bool rightValid = rightLine[row] >= 0;
            ++stats.evaluatedRows;
            stats.rawLeftValidRows += leftValid ? 1 : 0;
            stats.rawRightValidRows += rightValid ? 1 : 0;

            if (leftValid && rightValid)
            {
                ++stats.bilateralRows;
            }
            else if (!leftValid && !rightValid)
            {
                ++stats.bothMissingRows;
            }
            else if (!leftValid)
            {
                ++stats.leftMissingRows;
            }
            else
            {
                ++stats.rightMissingRows;
            }

            if (edgeProxyEnabled && (!leftValid || !rightValid))
            {
                ++stats.proxyRows;
            }
        }

        int dominantRows = stats.bothMissingRows;
        stats.visibilityState = "BOTH_MISSING";
        if (stats.leftMissingRows > dominantRows)
        {
            dominantRows = stats.leftMissingRows;
            stats.visibilityState = "LEFT_MISSING";
        }
        if (stats.rightMissingRows > dominantRows)
        {
            dominantRows = stats.rightMissingRows;
            stats.visibilityState = "RIGHT_MISSING";
        }
        if (stats.bilateralRows > dominantRows)
        {
            stats.visibilityState = "BILATERAL";
        }
        return stats;
    }
}
