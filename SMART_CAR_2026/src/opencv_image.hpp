#pragma once
#include "core.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

namespace car2026 {
inline Image processCameraFrame(const cv::Mat& bgr,const Params& params) {
    if(bgr.empty() || bgr.type()!=CV_8UC3) throw std::runtime_error("Expected a nonempty BGR camera frame");
    cv::Mat small;
    cv::resize(bgr,small,cv::Size(int(params.process_width),int(params.process_height)),0,0,cv::INTER_AREA);
    Image image(small.cols,small.rows);
    for(int y=0;y<small.rows;++y) {
        const auto* row=small.ptr<cv::Vec3b>(y);
        for(int x=0;x<small.cols;++x) image.at(x,y)={row[x][2],row[x][1],row[x][0]};
    }
    return image;
}
} // namespace car2026
