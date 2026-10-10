#pragma once
// Dependency-only replacement for cc's application-wide headfile.hpp.
// No global vehicle objects, race features, actuator or IMU initialization.
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>
#include <functional>
#include <iostream>
#include <list>
#include <numeric>
#include <string>
#include <tuple>
#include <utility>
#include <vector>
#include <sys/types.h>
namespace Other {constexpr unsigned char gBinWhilePointValue=255;}
#include "ImageProcess/HandleImage/HandleImage.hpp"
#include "ImageProcess/FindStartPos/FindStartPos.hpp"
#include "ImageProcess/FindLine/FindLine.hpp"
#include "ImageProcess/FixLine/FixLine.hpp"
