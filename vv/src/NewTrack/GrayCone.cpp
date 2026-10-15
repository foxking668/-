#include "GrayCone.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace GrayCone {
Result detect(const cv::Mat& image,const ConeSequence::Settings& s) {
    Result out;if(image.empty() || image.depth()!=CV_8U || image.cols<12 || image.rows<12) return out;
    cv::Mat gray;if(image.channels()==1) gray=image;
    else if(image.channels()==3) cv::cvtColor(image,gray,cv::COLOR_BGR2GRAY);else return out;
    double low=0,high=0;cv::minMaxLoc(gray,&low,&high);
    out.observation.valid=high-low>=s.grayMinContrast*2;
    if(!out.observation.valid) return out;
    cv::Mat unused;const double otsu=cv::threshold(gray,unused,0,255,cv::THRESH_BINARY_INV|cv::THRESH_OTSU);
    const int w=gray.cols,h=gray.rows;
    const int minimum=std::max(3,int(std::lround(s.grayMinArea*w*h/4800.0)));
    // One dark threshold can split a cone under changing exposure. Merge shape candidates
    // across three bounded thresholds; they remain ONE observation, never extra temporal hits.
    for(int offset:{0,20,40}) {
    const int threshold=std::max(1,int(std::lround(std::min(high-s.grayMinContrast,
        std::max(low+s.grayMinContrast,otsu*std::min(100,s.grayThresholdPercent+offset)/100.0)))));
    if(offset==0) out.threshold=threshold;
    cv::threshold(gray,out.mask,threshold,255,cv::THRESH_BINARY_INV);
    cv::Mat labels,stats,centroids;const int count=cv::connectedComponentsWithStats(out.mask,labels,stats,centroids,8);
    out.components+=count-1;
    for(int id=1;id<count;++id) {
        const cv::Rect box(stats.at<int>(id,cv::CC_STAT_LEFT),stats.at<int>(id,cv::CC_STAT_TOP),
            stats.at<int>(id,cv::CC_STAT_WIDTH),stats.at<int>(id,cv::CC_STAT_HEIGHT));
        const int area=stats.at<int>(id,cv::CC_STAT_AREA);const bool clipped=box.x==0 || box.y==0 || box.br().x==w || box.br().y==h;
        const double aspect=box.height/double(std::max(1,box.width));
        if(area<minimum || box.width<2 || box.height<4 || box.width>w*.48 || box.height>h*.85 ||
           aspect<.8 || aspect>4.5 || area<box.area()*.3) {++out.rejected;continue;}
        const int band=std::max(1,box.height/3);int top=0,bottom=0;
        double topLeft=0,topRight=0,bottomLeft=0,bottomRight=0;int topRows=0,bottomRows=0;
        double inside=0,around=0;int outsideCount=0;
        for(int y=box.y;y<box.br().y;++y) {
            int left=box.br().x,right=box.x-1;
            for(int x=box.x;x<box.br().x;++x) if(labels.at<int>(y,x)==id) {
                left=std::min(left,x);right=std::max(right,x);inside+=gray.at<unsigned char>(y,x);
                if(y<box.y+band) ++top;if(y>=box.br().y-band) ++bottom;
            }
            if(right>=left) {
                if(y<box.y+band) {topLeft+=left;topRight+=right;++topRows;}
                if(y>=box.br().y-band) {bottomLeft+=left;bottomRight+=right;++bottomRows;}
            }
            for(int x=std::max(0,box.x-2);x<std::min(w,box.br().x+2);++x)
                if(x<box.x || x>=box.br().x) {around+=gray.at<unsigned char>(y,x);++outsideCount;}
        }
        const bool taper=topRows && bottomRows && top*100<bottom*s.grayTaperPercent &&
            topLeft/topRows-bottomLeft/bottomRows>=.5 && bottomRight/bottomRows-topRight/topRows>=.5;
        const bool contrast=outsideCount && around/outsideCount-inside/area>=s.grayMinContrast;
        // Edge-clipped blobs may continue an existing near track, but can never start a new one.
        const bool continuation=clipped && box.br().y>=h*.70 && top*100<bottom*100;
        if(!contrast || (!taper && !continuation) || (clipped && !continuation)) {++out.rejected;continue;}
        ConeSequence::Candidate candidate{box.x*100.0/w,box.y*100.0/h,box.width*100.0/w,box.height*100.0/h,!clipped && taper};
        bool merged=false;
        for(auto& existing:out.observation.candidates) {
            const double overlapX=std::max(0.0,std::min(existing.x+existing.w,candidate.x+candidate.w)-std::max(existing.x,candidate.x));
            const double overlapY=std::max(0.0,std::min(existing.bottom(),candidate.bottom())-std::max(existing.y,candidate.y));
            const double intersection=overlapX*overlapY;
            if(intersection/std::max(1.0,existing.w*existing.h+candidate.w*candidate.h-intersection)>.25 &&
               std::abs(existing.center()-candidate.center())<8 && std::abs(existing.bottom()-candidate.bottom())<12) {
                if(candidate.acquire && (!existing.acquire || candidate.w*candidate.h>existing.w*existing.h)) existing=candidate;
                merged=true;break;
            }
        }
        if(!merged) out.observation.candidates.push_back(candidate);
    }
    }
    return out;
}
} }
