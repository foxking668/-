#include "NewTrack/Binarization.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>

using namespace NewTrack::Binarization;

int main() {
    Settings settings;
    // Empty/unsupported frames and constant dark, grey, and clipped images
    // must never supply permission to commit to a corner or a ring entrance.
    assert(process(cv::Mat(),settings).mask.empty());
    assert(process(cv::Mat(60,80,CV_16UC1),settings).mask.empty());
    for (int value: {0,90,255}) {
        const auto result=process(cv::Mat(60,80,CV_8UC1,cv::Scalar(value)),settings);
        assert(!result.mask.empty() && !result.quality);
    }
    // In a low-exposure scene HSV V>=46 admits both classes as white. Otsu
    // recovers the dark island and keeps the dim road without a fixed V floor.
    cv::Mat dim(60,80,CV_8UC1,cv::Scalar(45));
    cv::rectangle(dim,{0,0,20,60},cv::Scalar(18),-1);
    auto result=process(dim,settings);
    assert(result.quality && result.threshold==18);
    assert(result.mask.at<unsigned char>(40,5)==0);
    assert(result.mask.at<unsigned char>(40,40)==255);
    Settings old=settings;old.mode=0;
    assert(!process(dim,old).quality);
    // The forest/sky above the road ROI cannot move the road threshold. Match
    // both grey and BGR inputs, and avoid dependence on contiguous ROI storage.
    cv::Mat scene(60,80,CV_8UC1,cv::Scalar(110));
    cv::rectangle(scene,{0,21,20,39},cv::Scalar(35),-1);
    const auto clean=process(scene,settings);
    scene.rowRange(0,21).setTo(255);
    const auto brightBackground=process(scene,settings);
    assert(clean.threshold==brightBackground.threshold && brightBackground.quality);
    cv::Mat bgr;cv::cvtColor(scene,bgr,cv::COLOR_GRAY2BGR);
    assert(cv::countNonZero(process(bgr,settings).mask!=brightBackground.mask)==0);
    cv::Mat storage(60,100,CV_8UC1,cv::Scalar(0));
    scene.copyTo(storage(cv::Rect(10,0,80,60)));
    assert(process(storage(cv::Rect(10,0,80,60)),settings).threshold==clean.threshold);
    // Low-contrast splits can look binary but must not qualify as reliable.
    cv::Mat flat(60,80,CV_8UC1,cv::Scalar(100));
    flat.colRange(0,40).setTo(107);
    assert(!process(flat,settings).quality);
    // Isolated speckles disappear. A single-pixel connected guide line and a
    // multi-pixel opening survive; no artificial black border is added.
    cv::Mat details(60,80,CV_8UC1,cv::Scalar(180));
    details.colRange(0,20).setTo(25);
    cv::line(details,{40,23},{40,53},cv::Scalar(25),1);
    cv::rectangle(details,{45,28,10,15},cv::Scalar(25),-1);
    cv::rectangle(details,{45,33,6,4},cv::Scalar(180),-1);
    details.at<unsigned char>(40,30)=25;
    details.at<unsigned char>(40,10)=180;
    result=process(details,settings);
    assert(result.mask.at<unsigned char>(40,30)==255);
    assert(result.mask.at<unsigned char>(40,10)==0);
    for (int y=23;y<=53;++y) assert(result.mask.at<unsigned char>(y,40)==0);
    for (int y=33;y<37;++y)
        for (int x=45;x<51;++x) assert(result.mask.at<unsigned char>(y,x)==255);
    assert(result.mask.at<unsigned char>(0,79)==255);
    // Synchronous denoise is symmetric under reflection, unlike an in-place
    // majority scan which can consume a thin edge in one direction.
    cv::Mat reflected;cv::flip(details,reflected,1);
    cv::Mat reflectedResult;cv::flip(process(reflected,settings).mask,reflectedResult,1);
    assert(cv::countNonZero(result.mask!=reflectedResult)==0);
    // Explicit legacy mode remains exactly comparable when denoise is disabled.
    old.denoise=false;
    cv::Mat hsv,expected;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(0,0,46),cv::Scalar(180,48,255),expected);
    assert(cv::countNonZero(process(bgr,old).mask!=expected)==0);
    // Pathological parameter values are clamped safely, including INT_MAX.
    settings.roiTopPercent=200;settings.roiBottomPercent=-100;
    settings.thresholdOffset=2147483647;
    result=process(scene,settings);
    assert(result.threshold==255 && !result.quality);
    assert(!process(cv::Mat(1,1,CV_8UC1,cv::Scalar(100)),Settings{}).quality);
    std::cout<<"PASS binarization: ROI Otsu, dim road, quality, speckles, thin lines, openings, legacy comparison\n";
}
