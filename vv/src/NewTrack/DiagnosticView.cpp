#include "DiagnosticView.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace DiagnosticView {
cv::Mat render(const cv::Mat& bgr,const cv::Mat& raw,const cv::Mat& closed,const Snapshot& s) {
    cv::Mat canvas(canvasSize(),CV_8UC3,cv::Scalar(14,14,14));
    if(bgr.empty() || raw.empty() || closed.empty()) return canvas;
    cv::Mat component,processed,labels,stats,centroids;
    const int n=cv::connectedComponentsWithStats(closed,labels,stats,centroids,8);
    int biggest=0,area=0;
    for(int i=1;i<n;++i) if(stats.at<int>(i,cv::CC_STAT_AREA)>area) {area=stats.at<int>(i,cv::CC_STAT_AREA);biggest=i;}
    cv::Mat maxWhite=cv::Mat::zeros(raw.size(),CV_8UC1);
    if(biggest) cv::compare(labels,biggest,maxWhite,cv::CMP_EQ);
    cv::cvtColor(maxWhite,component,cv::COLOR_GRAY2BGR);
    // Steering uses the final arrays on the complete closed input, NOT this display-only max component.
    cv::cvtColor(closed,processed,cv::COLOR_GRAY2BGR);
    auto trace=[&](const std::vector<int>& xs,cv::Scalar colour) {
        cv::Point last(-1,-1);
        for(int y=0;y<processed.rows && y<static_cast<int>(xs.size());++y) {
            const int x=xs[y];
            if(x<0 || x>=processed.cols) {last={-1,-1};continue;}
            if(last.y>=0 && y-last.y<=2) cv::line(processed,last,{x,y},colour,1);
            processed.at<cv::Vec3b>(y,x)=cv::Vec3b(colour[0],colour[1],colour[2]);last={x,y};
        }
    };
    trace(s.black,s.blackUsed?cv::Scalar(0,0,0):cv::Scalar(0,180,180));
    trace(s.left,{0,255,0});trace(s.right,{255,110,0});trace(s.center,{255,255,0});
    for(auto p:s.corners) if(p.valid() && p.x<raw.cols && p.y<raw.rows) cv::circle(processed,{p.x,p.y},1,{255,0,255},-1);
    if(s.virtualDown.valid() && s.virtualDown.x<raw.cols && s.virtualDown.y<raw.rows)
        cv::circle(processed,{s.virtualDown.x,s.virtualDown.y},2,{0,165,255},1);
    if(s.coneBox.area()>0) cv::rectangle(processed,s.coneBox,s.coneHeld?cv::Scalar(0,165,255):cv::Scalar(0,0,255),1);
    // Target marker is the actual pre-error targetX; error also shows startup/probe/garage corrections.
    if(s.valid && s.targetX>=0 && s.targetX<raw.cols) cv::drawMarker(processed,
        {static_cast<int>(std::lround(s.targetX)),raw.rows*3/4},{255,0,255},cv::MARKER_CROSS,5,1);
    auto panel=[&](const cv::Mat& src,int col,int row,const char* title) {
        const int px=col*480,py=24+row*340;
        cv::putText(canvas,title,{px+8,py+17},cv::FONT_HERSHEY_SIMPLEX,.45,{0,220,255},1,cv::LINE_AA);
        const double scale=std::min(480.0/src.cols,316.0/src.rows);
        const cv::Size size(std::max(1,static_cast<int>(src.cols*scale)),std::max(1,static_cast<int>(src.rows*scale)));
        cv::Mat enlarged;cv::resize(src,enlarged,size,0,0,cv::INTER_NEAREST);
        enlarged.copyTo(canvas(cv::Rect(px+(480-size.width)/2,py+24+(316-size.height)/2,size.width,size.height)));
    };
    cv::putText(canvas,"VV: ONE FRAME / LIVE CONTROL SNAPSHOT",{8,17},cv::FONT_HERSHEY_SIMPLEX,.48,{240,240,240},1);
    cv::Mat cameraPreview;
    if(bgr.channels()==1) cv::cvtColor(bgr,cameraPreview,cv::COLOR_GRAY2BGR);else cameraPreview=bgr;
    panel(cameraPreview,0,0,"CAMERA CROP (gray / color source)");
    // No diagnostic guides are drawn into RAW: these pixels are exactly the detector input.
    cv::Mat untouchedRaw;cv::cvtColor(raw,untouchedRaw,cv::COLOR_GRAY2BGR);
    panel(untouchedRaw,1,0,"RAW BINARY (black line retained)");
    panel(component,0,1,"MAX WHITE 8C (closed / display only)");
    panel(processed,1,1,"FINAL: L green / R blue / CENTER cyan");
    if(!s.blueMask.empty()) {
        cv::Mat preview;cv::resize(s.blueMask,preview,{120,84},0,0,cv::INTER_NEAREST);
        cv::cvtColor(preview,preview,cv::COLOR_GRAY2BGR);
        preview.copyTo(canvas(cv::Rect(350,615,120,84)));
        cv::putText(canvas,"BLUE MASK",{350,612},cv::FONT_HERSHEY_SIMPLEX,.35,{0,220,255},1);
    }
    int y=721;
    for(const auto& line:s.lines) {
        if(y>791) break;
        cv::putText(canvas,line,{8,y},cv::FONT_HERSHEY_SIMPLEX,.43,{240,240,240},1,cv::LINE_AA);y+=18;
    }
    return canvas;
}
} }
