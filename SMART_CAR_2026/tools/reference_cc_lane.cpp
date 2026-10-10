#include "reference_cc_lane.hpp"
#include "headfile.hpp"
using namespace ImageProcess;
namespace car2026 { namespace capture {
namespace {
#include "lane_core.inc"
}
void ReferenceCcLane::reset() {
    laneWidthProfile.clear();laneWidthMedian=0;laneWidthProfileHeight=0;
}
CcLaneObservation ReferenceCcLane::analyze(const cv::Mat& frameBlock) {
    CcLaneObservation out;
    if(frameBlock.empty()) return out;
    if(frameBlock.type()!=CV_8UC3) throw std::runtime_error("CC lane requires 8-bit BGR camera frame");
    cv::Mat cameraFrame;
    // Camera::getFrame(true), define.hpp CAMERA_OPENCV_WIDTH/HEIGHT.
    cv::resize(frameBlock,cameraFrame,cv::Size(80,60),0,0,cv::INTER_NEAREST);
    const Config c;
    cv::Mat frame=cropSafe(cameraFrame,0.05,0.02,0.00,0.00);
    HandleImage hi;hi.submitOriginFrame(frame);
    cv::Mat rawMask=hi.getGreyAndWhiteBinaryFromOriginalFrame();
    if(rawMask.empty()) return out;
    cv::Mat boundaryMask=rawMask.clone();
    cv::Mat kernel=cv::getStructuringElement(cv::MORPH_RECT,cv::Size(11,1));
    cv::morphologyEx(boundaryMask,boundaryMask,cv::MORPH_CLOSE,kernel);
    const LaneLines lane=extractLane(boundaryMask,c);
    if(!lane.valid) return out;
    const BlackLineResult black=detectCenterBlack(rawMask,lane,c);
    double targetX=lane.aggregateCenter;
    if(c.centerBlackEnable && black.reliable) {
        targetX=black.weightedX;out.centerBlackReliable=true;
    } else {
        const double scale=rawMask.cols/160.0;
        if(lane.rightOnlyRows>lane.leftOnlyRows && lane.rightOnlyRows>=3)
            targetX-=c.singleEdgeBiasPx160*scale;
        else if(lane.leftOnlyRows>lane.rightOnlyRows && lane.leftOnlyRows>=3)
            targetX+=c.singleEdgeBiasPx160*scale;
        const bool partialBlack=c.partialBlackAssistEnable && lane.singleEdgeUsed &&
            black.validRows>=c.partialBlackMinValidRows && black.nearRows>=c.partialBlackMinNearRows;
        if(partialBlack) {
            const double a=c.partialBlackBlendPercent/100.0;
            targetX=targetX*(1.0-a)+black.weightedX*a;
        }
    }
    // Same float conversion and center_pratio=0.42 as the supplied image_params.txt.
    out.error=(float)centerToError(targetX,rawMask.rows,rawMask.cols,0.42);
    out.rawTarget=targetX;out.singleEdgeUsed=lane.singleEdgeUsed;
    out.bilateralRows=lane.bilateralRows;out.leftOnlyRows=lane.leftOnlyRows;out.rightOnlyRows=lane.rightOnlyRows;
    out.line.reliable=std::isfinite(out.error);
    out.line.rows=black.validRows;out.line.nearRows=black.nearRows;
    // Preserve the existing CSV's normalized 160-pixel coordinate convention.
    out.line.target=targetX*160.0/rawMask.cols;
    return out;
}
}}
