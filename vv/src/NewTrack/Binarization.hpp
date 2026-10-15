#pragma once
#include <opencv2/core.hpp>

namespace NewTrack { namespace Binarization {

struct Settings {
    // 0: original HSV white extraction, 1: road-ROI Otsu (default).
    int mode=1;
    int roiTopPercent=35, roiBottomPercent=95;
    int thresholdOffset=0;
    bool denoise=true;
    // 【二值化前模糊】>1 时在灰度图上先做 GaussianBlur(k,k)，去掉孤立噪点让 Otsu
    // 直方图更干净。参考 cc 版默认 3；1=关闭（保持原行为）。奇数生效。
    int blurKernel=1;
    // 【固定阈值】0=用 ROI-Otsu（原行为）；1~254=完全不用 Otsu，直接按这个灰度阈值二值化。
    // 用来在 Otsu 失效（画面近乎全白/全黑、单峰）时钉死一个阈值兜底。
    int fixedThreshold=0;
};

struct Result {
    cv::Mat mask;                  // CV_8UC1, white road=255, dark region=0.
    int threshold=0;               // Applied grey threshold, or legacy HSV V minimum.
    double whitePercent=0.0;       // Measured in the threshold-estimation ROI.
    double contrast=0.0;           // Absolute grey-mean difference between the classes.
    bool quality=false;            // Both classes present, separated, and not clipped flat.
};

// Accept an already resized/cropped CV_8UC1 or CV_8UC3 frame. No frame history,
// forced black border, or invented corner is introduced by this operation.
Result process(const cv::Mat& frame, const Settings& settings=Settings{});

} }
