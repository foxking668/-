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
        reset();out.reason="INVALID_INPUT";return out;
    }
    if(w!=width_ || h!=height_) {reset();width_=w;height_=h;}
    cv::Mat hsv,mask;cv::cvtColor(bgr,hsv,cv::COLOR_BGR2HSV);
    cv::inRange(hsv,cv::Scalar(s.hMin,s.sMin,s.vMin),cv::Scalar(s.hMax,255,255),mask);
    cv::morphologyEx(mask,mask,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{1,3}));
    out.blueMask=mask;out.bluePixels=cv::countNonZero(mask);
    cv::Mat labels,stats,centroids;
    const int count=cv::connectedComponentsWithStats(mask,labels,stats,centroids,8);
    out.components=count-1;out.reason=out.bluePixels?"SIZE_OR_POSITION":"NO_BLUE";
    const int minArea=std::max(3,int(std::lround(s.minArea80x60*w*h/4800.0)));
    cv::Rect best;Side bestSide=Side::NONE;int bestBottom=-1;bool bestRoad=false;
    for(int id=1;id<count;++id) {
        const int area=stats.at<int>(id,cv::CC_STAT_AREA);
        const cv::Rect box(stats.at<int>(id,cv::CC_STAT_LEFT),stats.at<int>(id,cv::CC_STAT_TOP),
                           stats.at<int>(id,cv::CC_STAT_WIDTH),stats.at<int>(id,cv::CC_STAT_HEIGHT));
        const int bottom=box.y+box.height-1;
        const double aspect=box.height/double(std::max(1,box.width));
        const bool large=area>w*h*s.maxAreaPercent/100 || box.width>w*.24 || box.height>h*.55;
        bool nearShape=false;
        if(large && bottom>=h/2 && box.height>=6) {
            int upper=0,lower=0,rows=std::max(1,box.height/3);
            for(int yy=box.y;yy<=bottom;++yy) for(int xx=box.x;xx<box.x+box.width;++xx)
                if(labels.at<int>(yy,xx)==id) {
                    if(yy<box.y+rows) ++upper;
                    if(yy>bottom-rows) ++lower;
                }
            nearShape=upper*100<lower*85; // Compact taper distinguishes a growing cone from a solid board.
        }
        if(area<minArea || box.width<2 || box.height<3 || aspect<.65 || aspect>4.5 ||
           area<box.area()*.25 || bottom<h*s.nearYPercent/100 ||
           (large && (!nearShape || area>w*h*s.nearMaxAreaPercent/100 ||
            box.width>w*s.nearMaxWidthPercent/100.0 || box.height>h*s.nearMaxHeightPercent/100.0))) {
            ++out.rejectedSize;continue;
        }
        const int x=int(std::lround(centroids.at<double>(id,0)));
        int interior=0,known=0;
        for(int yy=box.y;yy<=bottom;++yy) {
            if(left[yy]<0 || right[yy]<=left[yy]) continue;
            for(int xx=box.x;xx<box.x+box.width;++xx) if(labels.at<int>(yy,xx)==id) {
                ++known;if(xx>left[yy] && xx<right[yy]) ++interior;
            }
        }
        const bool road=left[bottom]>=0 && right[bottom]>left[bottom] && known*100>=area*70;
        if(road && (x<=left[bottom] || x>=right[bottom] || interior*100<known*70)) {
            ++out.rejectedRoad;out.reason="OUTSIDE_ROAD";continue;
        }
        const int ref=road?(center[bottom]>=0?center[bottom]:(left[bottom]+right[bottom])/2):w/2;
        const int dead=std::max(1,w/40);
        const Side side=x<ref-dead?Side::LEFT:x>ref+dead?Side::RIGHT:Side::CENTER;
        if(bottom>bestBottom) {bestBottom=bottom;best=box;bestSide=side;bestRoad=road;}
    }
    if(bestSide!=Side::NONE) {
        out.detected=true;out.side=bestSide;out.box=best;out.roadValid=bestRoad;
        const bool compatible=bestSide==pending_ || bestSide==Side::CENTER || pending_==Side::CENTER;
        const bool same=compatible && nowMs>pendingMs_ && nowMs-pendingMs_<=s.maxConfirmGapMs &&
            std::abs(best.x-pendingBox_.x)<=std::max(3,w/8) &&
            std::abs(best.y-pendingBox_.y)<=std::max(3,h/5) &&
            best.width<=pendingBox_.width*2+2 && pendingBox_.width<=best.width*2+2;
        if(heldSide_!=Side::NONE && heldSide_!=bestSide &&
           heldSide_!=Side::CENTER && bestSide!=Side::CENTER) heldSide_=Side::NONE;
        confirmations_=same?std::min(confirmations_+1,1000):1;
        pending_=bestSide;pendingBox_=best;pendingMs_=nowMs;out.confirmations=confirmations_;
        if(confirmations_<std::max(1,s.confirmFrames)) {out.reason="CONFIRMING";return out;}
        heldSide_=bestSide;heldBox_=best;lastSeenMs_=nowMs;out.confirmed=true;
    } else {
        pending_=Side::NONE;confirmations_=0;
        if(heldSide_==Side::NONE || nowMs<lastSeenMs_ || nowMs-lastSeenMs_>s.holdMs) {
            heldSide_=Side::NONE;return out;
        }
        out.held=true;out.confirmed=true;out.side=heldSide_;out.box=heldBox_;
        const int bottom=out.box.y+out.box.height-1;
        out.roadValid=left[bottom]>=0 && right[bottom]>left[bottom];
    }
    if(!out.roadValid) {out.reason="ROAD_UNKNOWN";return out;}
    const int margin=std::max(1,int(std::lround(s.marginPx80*w/80.0)));
    const int clearance=std::max(3,int(std::lround(s.minCorridorPx80*w/80.0)));
    const int anchorY=out.box.y+out.box.height-1,startY=std::max(0,out.box.y-h/8);
    // A CENTER cone uses the approved left-first route only if a left corridor fits.
    const bool obstacleLeft=out.side==Side::LEFT;
    const int anchorX=obstacleLeft?out.box.x+out.box.width-1+margin:out.box.x-margin;
    std::vector<int> proposed(h,-1);
    for(int y=startY;y<h;++y) {
        if(left[y]<0 || right[y]<=left[y]) continue;
        const int original=obstacleLeft?left[y]:right[y];
        const double a=y>=anchorY?1.0:(y-startY)/double(std::max(1,anchorY-startY));
        int x=int(std::lround(original*(1-a)+anchorX*a));
        x=obstacleLeft?std::max(left[y],x):std::min(right[y],x);
        if(x<0 || x>=w || (obstacleLeft?right[y]-x:x-left[y])<clearance) {
            out.blocked=true;out.reason="CORRIDOR_BLOCKED";heldSide_=Side::NONE;return out;
        }
        proposed[y]=x;
    }
    for(int y=startY;y<h;++y) if(proposed[y]>=0) {
        int& edge=obstacleLeft?left[y]:right[y];
        if(edge!=proposed[y]) ++out.changedRows;
        edge=proposed[y];center[y]=(left[y]+right[y])/2;
    }
    out.active=out.changedRows>0;out.reason=out.held?"HELD":out.active?"CONFIRMED":"NO_REPAIR_NEEDED";
    return out;
}
} }
