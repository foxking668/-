#pragma once
#include "Roundabout.hpp"
#include <cstddef>

namespace NewTrack { namespace EightNeighbor {
struct Boundary {
    std::vector<int> left, right;
    std::vector<Roundabout::Point> leftPath, rightPath;
};
// White track=nonzero, black background=0. Stride may exceed width.
// Trace actual adjacent white boundary pixels, preserving horizontal/reverse growth.
Boundary trace(const unsigned char* pixels, int width, int height, std::size_t stride,
               const std::vector<int>& leftHint={}, const std::vector<int>& rightHint={});
} }
