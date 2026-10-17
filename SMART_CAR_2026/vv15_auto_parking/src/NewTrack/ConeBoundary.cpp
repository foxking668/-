#include "ConeBoundary.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace ConeBoundary {
Result Detector::update(const cv::Mat& bgr, std::vector<int>& left, std::vector<int>& right,
                        std::vector<int>& center, long long nowMs, const Settings& s) {
    Result out;
    const int w=bgr.cols,h=bgr.rows;
    if(bgr.empty() || bgr.type()!=CV_8UC3 || w<12 || h<12 ||
       left.size()!=static_cast<std::size_t>(h) || right.size()!=left.size() || center.size()!=left.size()) {
        reset();return out;
    }
    if(w!=width_ || h!=height_) {reset();width_=w;height_=h;}
    cv::Mat hsv, mask;
    cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(s.hMin,s.sMin,s.vMin),cv::Scalar(s.hMax,255,255),mask);
    // Close tiny vertical gaps caused by reflection/white stripes; do not split a solid blue cone.
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{1,3}));
    out.blueMask=mask;
    cv::Mat labels,stats,centroids;
    const int count=cv::connectedComponentsWithStats(mask,labels,stats,centroids,8);
    const int minArea=std::max(3,static_cast<int>(std::lround(s.minArea80x60*w*h/4800.0)));
    cv::Rect best; Side bestSide=Side::NONE;
    int bestBottom=-1;
    for(int id=1;id<count;++id) {
        const int area=stats.at<int>(id,cv::CC_STAT_AREA);
        const cv::Rect box(stats.at<int>(id,cv::CC_STAT_LEFT),stats.at<int>(id,cv::CC_STAT_TOP),
                           stats.at<int>(id,cv::CC_STAT_WIDTH),stats.at<int>(id,cv::CC_STAT_HEIGHT));
        const int bottom=box.y+box.height-1;
        const double aspect=box.height/static_cast<double>(std::max(1,box.width));
        if(area<minArea || area>w*h*s.maxAreaPercent/100 || box.width<2 || box.height<3 ||
           box.width>w*0.24 || box.height>h*0.55 || aspect<0.65 || aspect>4.5 ||
           area<box.area()*0.25 || bottom<h*s.nearYPercent/100) continue;
        // Reject background/boards before applying the road ROI (ROI clipping can turn a board into a tiny blob).
        const int x=static_cast<int>(std::lround(centroids.at<double>(id,0)));
        const int y=std::min(h-1,std::max(0,bottom));
        if(left[y]<0 || right[y]<=left[y] || x<=left[y] || x>=right[y]) continue;
        int interior=0;
        for(int yy=box.y;yy<=bottom;++yy) {
            if(left[yy]<0 || right[yy]<=left[yy]) continue;
            const auto *row=labels.ptr<int>(yy);
            for(int xx=box.x;xx<box.x+box.width;++xx)
                if(row[xx]==id && xx>left[yy] && xx<right[yy]) ++interior;
        }
        if(interior*100<area*70) continue;
        const int ref=center[y]>=0?center[y]:(left[y]+right[y])/2;
        const int dead=std::max(1,w/40);
        const Side side=x<ref-dead?Side::LEFT:(x>ref+dead?Side::RIGHT:Side::NONE);
        if(side==Side::NONE) continue;
        if(bottom>bestBottom) {bestBottom=bottom;best=box;bestSide=side;}
    }
    if(bestSide!=Side::NONE) {
        out.detected=true;out.side=bestSide;out.box=best;
        if(heldSide_!=Side::NONE && heldSide_!=bestSide) heldSide_=Side::NONE;
        const int tolerance=std::max(3,w/8);
        const bool same=bestSide==pending_ && std::abs(best.x-pendingBox_.x)<=tolerance &&
                        std::abs(best.y-pendingBox_.y)<=std::max(3,h/5);
        confirmations_=same?confirmations_+1:1;pending_=bestSide;pendingBox_=best;
        if(confirmations_>=std::max(1,s.confirmFrames)) {
            heldSide_=bestSide;heldBox_=best;lastSeenMs_=nowMs;
        } else return out; // A new opposite cone never inherits the previous cone's boundary.
    } else {
        pending_=Side::NONE;confirmations_=0;
        if(heldSide_==Side::NONE || nowMs<lastSeenMs_ || nowMs-lastSeenMs_>s.holdMs) {
            heldSide_=Side::NONE;return out;
        }
        out.held=true;out.side=heldSide_;out.box=heldBox_;
    }
    const int margin=std::max(1,static_cast<int>(std::lround(s.marginPx80*w/80.0)));
    const int clearance=std::max(3,static_cast<int>(std::lround(s.minCorridorPx80*w/80.0)));
    const int anchorY=std::min(h-1,out.box.y+out.box.height-1);
    const int startY=std::max(0,out.box.y-h/8);
    const int anchorX=out.side==Side::LEFT?out.box.x+out.box.width-1+margin:out.box.x-margin;
    // Check the entire affected corridor before mutating any row. Never clamp a cone edge through the cone.
    std::vector<int> proposed(h,-1);
    for(int y=startY;y<h;++y) {
        if(left[y]<0 || right[y]<=left[y]) continue;
        const int original=out.side==Side::LEFT?left[y]:right[y];
        const double a=y>=anchorY?1.0:(y-startY)/static_cast<double>(std::max(1,anchorY-startY));
        int x=static_cast<int>(std::lround(original*(1-a)+anchorX*a));
        x=out.side==Side::LEFT?std::max(left[y],x):std::min(right[y],x);
        if(x<0 || x>=w || (out.side==Side::LEFT?right[y]-x:x-left[y])<clearance) {
            out.blocked=true;heldSide_=Side::NONE;return out;
        }
        proposed[y]=x;
    }
    for(int y=startY;y<h;++y) if(proposed[y]>=0) {
        int& edge=out.side==Side::LEFT?left[y]:right[y];
        if(edge!=proposed[y]) ++out.changedRows;
        edge=proposed[y];center[y]=(left[y]+right[y])/2;
    }
    out.active=out.changedRows>0;
    return out;
}
} }
