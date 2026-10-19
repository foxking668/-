#pragma once

namespace ImageProcess
{
    inline int applyMissingBoundaryPolicy(int &left,
                                          int &right,
                                          int width,
                                          bool edgeProxyEnabled)
    {
        if (!edgeProxyEnabled && (left == -1 || right == -1))
        {
            return -1;
        }
        if (left == -1)
        {
            left = 0;
        }
        if (right == -1)
        {
            right = width - 1;
        }
        return (left + right) / 2;
    }
}
