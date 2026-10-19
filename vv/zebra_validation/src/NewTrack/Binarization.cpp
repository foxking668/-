#include "Binarization.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace Binarization {
namespace {

cv::Rect roadRoi(const cv::Mat& frame, const Settings& settings) {
    const int topPercent=std::clamp(settings.roiTopPercent,0,99);
    const int bottomPercent=std::clamp(settings.roiBottomPercent,topPercent+1,100);
    const int top=std::clamp(frame.rows*topPercent/100,0,frame.rows-1);
    const int bottom=std::clamp((frame.rows*bottomPercent+99)/100,top+1,frame.rows);
    return {0,top,frame.cols,bottom-top};
}

void removeIsolatedPixels(cv::Mat& mask) {
    // The supplied eight-neighbour example uses 5/2 majority thresholds and
    // modifies its own input during scanning. On an 80-pixel image that erases
    // thin guide lines and makes the result scan-direction dependent. Use the
    // same neighbourhood idea with a simultaneous, isolated-pixel-only update.
    const cv::Mat source=mask.clone();
    for (int y=1;y<mask.rows-1;++y) {
        const auto* previous=source.ptr<unsigned char>(y-1);
        const auto* current=source.ptr<unsigned char>(y);
        const auto* next=source.ptr<unsigned char>(y+1);
        auto* output=mask.ptr<unsigned char>(y);
        for (int x=1;x<mask.cols-1;++x) {
            const unsigned char value=current[x];
            if (previous[x-1]!=value && previous[x]!=value && previous[x+1]!=value &&
                current[x-1]!=value && current[x+1]!=value &&
                next[x-1]!=value && next[x]!=value && next[x+1]!=value)
                output[x]=static_cast<unsigned char>(255-value);
        }
    }
}

void measureQuality(const cv::Mat& grey, const cv::Rect& roi, Result& result) {
    const cv::Mat roiMask=result.mask(roi);
    const int whites=cv::countNonZero(roiMask);
    const int count=roi.width*roi.height;
    result.whitePercent=100.0*whites/count;
    if (whites==0 || whites==count) return;
    cv::Mat blackMask;
    cv::bitwise_not(roiMask,blackMask);
    const double whiteMean=cv::mean(grey(roi),roiMask)[0];
    const double blackMean=cv::mean(grey(roi),blackMask)[0];
    const double wholeMean=cv::mean(grey(roi))[0];
    result.contrast=std::abs(whiteMean-blackMean);
    result.quality=result.whitePercent>=1.0 && result.whitePercent<=99.0 &&
                   result.contrast>=20.0 && wholeMean>3.0 && wholeMean<252.0;
}

}

Result process(const cv::Mat& frame, const Settings& settings) {
    Result result;
    if (frame.empty() || frame.depth()!=CV_8U ||
        (frame.channels()!=1 && frame.channels()!=3)) return result;
    cv::Mat grey;
    if (frame.channels()==1) grey=frame;
    else cv::cvtColor(frame,grey,cv::COLOR_BGR2GRAY);
    // 【二值化前模糊】去掉孤立噪点，让 Otsu 直方图更干净（参考 cc 版）。
    if (settings.blurKernel>1) {
        const int k=settings.blurKernel|1;
        cv::GaussianBlur(grey,grey,cv::Size(k,k),0);
    }
    const cv::Rect roi=roadRoi(frame,settings);
    if(settings.bandRows>0) {
        result.mask=cv::Mat::zeros(grey.size(),CV_8UC1);
        result.threshold=-2; // Per-row thresholds; never report a fictitious global threshold.
        const int half=std::max(1,settings.bandRows/2);
        cv::Mat temporary;
        for(int y=0;y<grey.rows;++y) {
            const int a=std::max(0,y-half),b=std::min(grey.rows,y+half+1);
            const int estimated=static_cast<int>(cv::threshold(grey.rowRange(a,b),temporary,0,255,cv::THRESH_BINARY|cv::THRESH_OTSU));
            const int threshold=std::clamp(estimated+std::clamp(settings.thresholdOffset,-40,40),8,254);
            cv::threshold(grey.row(y),result.mask.row(y),threshold,255,cv::THRESH_BINARY);
        }
    } else if (settings.mode==0) {
        result.threshold=46;
        if (frame.channels()==1) {
            // In a replicated grey BGR image S is zero and V equals grey.
            cv::inRange(grey,cv::Scalar(46),cv::Scalar(255),result.mask);
        } else {
            cv::Mat hsv;
            cv::cvtColor(frame,hsv,cv::COLOR_BGR2HSV);
            cv::inRange(hsv,cv::Scalar(0,0,46),cv::Scalar(180,48,255),result.mask);
        }
    } else if (settings.fixedThreshold>0) {
        // 【固定阈值】完全不用 Otsu，直接钉死。offset 仍然生效。
        result.threshold=std::clamp(settings.fixedThreshold+
                                   std::clamp(settings.thresholdOffset,-255,255),0,255);
        cv::threshold(grey,result.mask,result.threshold,255,cv::THRESH_BINARY);
    } else {
        // OpenCV implements the same histogram/class-variance Otsu criterion
        // as the reference, without its uint8 loop/empty-class division bugs.
        cv::Mat estimatedMask;
        const double estimated=cv::threshold(grey(roi),estimatedMask,0,255,
                                             cv::THRESH_BINARY|cv::THRESH_OTSU);
        // Bound the offset before addition so arbitrary settings cannot overflow.
        result.threshold=std::clamp(static_cast<int>(estimated)+
                                   std::clamp(settings.thresholdOffset,-255,255),0,255);
        cv::threshold(grey,result.mask,result.threshold,255,cv::THRESH_BINARY);
    }
    if (settings.denoise) removeIsolatedPixels(result.mask);
    measureQuality(grey,roi,result);
    return result;
}

} }
