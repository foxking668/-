#pragma once
#include "ZebraTest.hpp"
#include <opencv2/core.hpp>
namespace ZebraTest {
Observation detect(const cv::Mat& gray,const cv::Mat& raw,const std::vector<int>& left,
                   const std::vector<int>& right,bool laneValid,bool quality,long long timeMs,const Settings& s);
}
