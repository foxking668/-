#include "GrayCone.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace GrayCone {
namespace {
void mergeCandidate(std::vector<ConeSequence::Candidate>& candidates,const ConeSequence::Candidate& next) {
    for(auto& previous:candidates) {
        const double dx=std::max(0.0,std::min(previous.x+previous.w,next.x+next.w)-std::max(previous.x,next.x));
        const double dy=std::max(0.0,std::min(previous.bottom(),next.bottom())-std::max(previous.y,next.y));
        const double intersection=dx*dy;
        if(intersection/std::max(1.0,previous.w*previous.h+next.w*next.h-intersection)>.25 &&
           std::abs(previous.center()-next.center())<8 && std::abs(previous.bottom()-next.bottom())<12) {
            const int previousQuality=previous.acquire?2:previous.nearClipped?1:0;
            const int nextQuality=next.acquire?2:next.nearClipped?1:0;
            if(nextQuality>previousQuality || (nextQuality==previousQuality && next.w*next.h>previous.w*previous.h)) previous=next;
            return;
        }
    }
    candidates.push_back(next);
}
}
Result detect(const cv::Mat& image,const ConeSequence::Settings& s) {
    Result out;if(image.empty() || image.depth()!=CV_8U || image.cols<12 || image.rows<12) return out;
    cv::Mat gray;if(image.channels()==1) gray=image;
    else if(image.channels()==3) cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);else return out;
    double low=0,high=0;cv::minMaxLoc(gray,&low,&high);
    out.observation.valid=high-low>=s.grayMinContrast*2;if(!out.observation.valid) return out;
    cv::Mat unused;const double otsu=cv::threshold(gray,unused,0,255,cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
    const int w=gray.cols,h=gray.rows,minimum=std::max(3,int(std::lround(s.grayMinArea*w*h/4800.0)));
    // Threshold variants/near isolation are candidates in ONE captured frame, never temporal votes.
    for(int offset:{0,20,40}) for(bool isolateNear:{false,true}) {
        const int threshold=std::max(1,int(std::lround(std::min(high-s.grayMinContrast,
            std::max(low+s.grayMinContrast,otsu*std::min(100,s.grayThresholdPercent+offset)/100.0)))));
        if(offset==0) out.threshold=threshold;
        cv::threshold(gray,out.mask,threshold,255,cv::THRESH_BINARY_INV);
        const int cut=std::max(1,int(std::lround(h*.15)));
        if(isolateNear) {
            const int kw=std::max(1,int(std::lround(3*w/80.0)))|1,kh=std::max(1,int(std::lround(5*h/60.0)))|1;
            cv::morphologyEx(out.mask,out.mask,cv::MORPH_OPEN,cv::getStructuringElement(cv::MORPH_RECT,{kw,kh}));
            out.mask.rowRange(0,cut).setTo(0);
        }
        cv::Mat labels,stats,centroids;const int count=cv::connectedComponentsWithStats(out.mask,labels,stats,centroids,8);
        out.components+=count-1;
        for(int id=1;id<count;++id) {
            const cv::Rect box(stats.at<int>(id,0),stats.at<int>(id,1),stats.at<int>(id,2),stats.at<int>(id,3));
            const int area=stats.at<int>(id,cv::CC_STAT_AREA);
            const bool clipped=box.x==0 || box.y==0 || box.br().x==w || box.br().y==h;
            const double aspect=box.height/double(std::max(1,box.width));
            if(area<minimum || box.width<2 || box.height<4 || box.width>w*(isolateNear?.65:.48) ||
               box.height>h*(isolateNear?.95:.85) || aspect<(isolateNear?.3:.8) || aspect>4.5 || area<box.area()*.3) {
                ++out.rejected;continue;
            }
            const int band=std::max(1,box.height/3);int top=0,bottom=0,topRows=0,bottomRows=0,outsideCount=0;
            double tl=0,tr=0,bl=0,br=0,inside=0,around=0;
            for(int y=box.y;y<box.br().y;++y) {
                int left=box.br().x,right=box.x-1;
                for(int x=box.x;x<box.br().x;++x) if(labels.at<int>(y,x)==id) {
                    left=std::min(left,x);right=std::max(right,x);inside+=gray.at<unsigned char>(y,x);
                    if(y<box.y+band) ++top;if(y>=box.br().y-band) ++bottom;
                }
                if(right>=left) {
                    if(y<box.y+band) {tl+=left;tr+=right;++topRows;}
                    if(y>=box.br().y-band) {bl+=left;br+=right;++bottomRows;}
                }
                for(int x=std::max(0,box.x-2);x<std::min(w,box.br().x+2);++x)
                    if(x<box.x || x>=box.br().x) {around+=gray.at<unsigned char>(y,x);++outsideCount;}
            }
            const bool taper=topRows && bottomRows && top*100<bottom*s.grayTaperPercent &&
                tl/topRows-bl/bottomRows>=.5 && br/bottomRows-tr/topRows>=.5;
            const bool roundedBase=topRows && bottomRows && bottom*100<top*85 &&
                bl/bottomRows-tl/topRows>=.5 && tr/topRows-br/bottomRows>=.5;
            const bool contrast=outsideCount && around/outsideCount-inside/area>=s.grayMinContrast;
            const bool isolatedCap=isolateNear && box.y==cut && box.x>0 && box.br().x<w &&
                box.width>=w*.20 && box.height>=h*.15 && area>=w*h*.05 && roundedBase;
            const bool clippedTaper=!isolateNear && clipped && taper && box.br().y>=h*.70;
            const bool continuation=!isolateNear && clipped && box.br().y>=h*.70 && top<bottom;
            if(!contrast || (isolateNear?!isolatedCap:(!taper && !continuation) || (clipped && !continuation))) {
                ++out.rejected;continue;
            }
            mergeCandidate(out.observation.candidates,{box.x*100.0/w,box.y*100.0/h,box.width*100.0/w,
                box.height*100.0/h,!clipped && !isolateNear && taper,isolatedCap || clippedTaper});
        }
    }
    return out;
}
Guidance guide(const ConeSequence::Observation& observation,const ConeSequence::Candidate* target,
    ConeSequence::Phase phase,const std::vector<int>& left,const std::vector<int>& right,int width,
    int marginPx80,int minCorridorPx80) {
    Guidance out;
    if(!observation.valid || !observation.roadValid || !std::isfinite(observation.roadCenter) ||
       observation.roadCenter<0 || observation.roadCenter>100 || width<12 || left.empty() || right.size()!=left.size()) return out;
    out.target=observation.roadCenter*width/100.0;
    if(phase==ConeSequence::Phase::RETURN_RIGHT || observation.candidates.empty()) {out.valid=true;return out;}
    if(!target) return out; // An unconfirmed object cannot be driven through on a guessed corridor.
    const int y=std::max(0,std::min(int(left.size())-1,int(target->bottom()*left.size()/100)-1));
    double l=left[y],r=right[y];if(l<0 || r<=l) return out;
    const double x=target->x*width/100,edge=(target->x+target->w)*width/100;
    if(edge<=l || x>=r) {out.valid=true;return out;}
    const double margin=std::max(1.0,marginPx80*width/80.0);
    if(phase==ConeSequence::Phase::RIGHT) l=std::max(l,edge+margin);else r=std::min(r,x-margin);
    if(r-l<std::max(3.0,minCorridorPx80*width/80.0)) {out.blocked=true;return out;}
    out.target=(l+r)/2;out.valid=true;return out;
}
} }
