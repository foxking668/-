#include "TrackGates.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <cmath>

namespace NewTrack { namespace TrackGates {
namespace {
bool validInput(const cv::Mat& b,const std::vector<int>& l,const std::vector<int>& r,
                const std::vector<int>& m) {
    return !b.empty() && b.type()==CV_8UC1 && l.size()==static_cast<std::size_t>(b.rows) &&
           r.size()==l.size() && m.size()==l.size();
}
}

MarkerEvidence detectMarker(const cv::Mat& raw,const std::vector<int>& left,
                            const std::vector<int>& right,const std::vector<int>& center,
                            const MarkerSettings& s) {
    MarkerEvidence out;
    if(!validInput(raw,left,right,center)) return out;
    cv::Mat m=cv::Mat::zeros(raw.size(),CV_8UC1);
    for(int y=0;y<raw.rows;++y) {
        int l=left[y],r=right[y];if(l<0 || r>=raw.cols || r-l<12) continue;
        l=std::max(0,l+3);r=std::min(raw.cols-1,r-3);
        for(int x=l;x<=r;++x) if(raw.at<unsigned char>(y,x)==0) m.at<unsigned char>(y,x)=255;
    }
    cv::Mat labels,stats,centroids;
    const int n=cv::connectedComponentsWithStats(m,labels,stats,centroids,8,CV_32S);
    std::vector<cv::Point> centers;
    const int tol=std::max(3,static_cast<int>(std::lround(s.centerTolerancePx160*raw.cols/160.0)));
    for(int i=1;i<n;++i) {
        const int area=stats.at<int>(i,cv::CC_STAT_AREA),bw=stats.at<int>(i,cv::CC_STAT_WIDTH);
        const int bh=stats.at<int>(i,cv::CC_STAT_HEIGHT);
        if(area<3 || area>static_cast<int>(raw.total()/8) || bh>raw.rows/4) continue;
        const int cy=static_cast<int>(std::lround(centroids.at<double>(i,1)));
        const int cx=static_cast<int>(std::lround(centroids.at<double>(i,0)));
        if(cy<0 || cy>=raw.rows) continue;
        const int l=left[cy],r=right[cy];if(l<0 || r>=raw.cols || r<=l) continue;
        if(bw*100>(r-l)*s.maxWidthPercent) continue;
        const int target=center[cy]>=0?center[cy]:(l+r)/2;
        if(std::abs(cx-target)>tol) continue;
        centers.emplace_back(cx,cy);
    }
    out.minSpan=std::max(4,static_cast<int>(std::lround(s.minSpanRows82*raw.rows/82.0)));
    if(!centers.empty()) {
        std::sort(centers.begin(),centers.end(),[](auto a,auto b){return a.y<b.y;});
        out.blocks=static_cast<int>(centers.size());out.span=centers.back().y-centers.front().y;
        int lo=centers.front().x,hi=lo;
        std::vector<cv::Point2f> pts;
        for(auto p:centers) {lo=std::min(lo,p.x);hi=std::max(hi,p.x);pts.emplace_back(p.x,p.y);}
        out.spread=hi-lo;
        if(pts.size()>=2) {
            cv::Vec4f line;cv::fitLine(pts,line,cv::DIST_L2,0,0.01,0.01);
            out.angleDeg=std::atan2(std::abs(double(line[1])),std::abs(double(line[0]))+1e-6)*180/std::acos(-1.0);
        }
    }
    out.blocksPass=out.blocks>=s.minBlocks;out.spanPass=out.span>=out.minSpan;
    out.ratioPass=out.span*100>=std::max(1,out.spread)*s.verticalRatioPercent;
    out.anglePass=out.angleDeg>=s.minAngleDeg;
    out.passed=out.blocksPass&&out.spanPass&&out.ratioPass&&out.anglePass;
    return out;
}

ForkEvidence detectFork(const cv::Mat& raw,const std::vector<int>& left,
                       const std::vector<int>& right,const std::vector<int>& center,const ForkSettings& s) {
    ForkEvidence e;if(!validInput(raw,left,right,center)) return e;
    const int minSep=std::max(3,static_cast<int>(std::lround(s.minSeparationPx160*raw.cols/160.0)));
    int firstSplitY=-1,lastStemY=-1;
    for(int y=raw.rows*18/100;y<raw.rows-1;++y) {
        int l=left[y],r=right[y];if(l<0||r>=raw.cols||r<=l+14) continue;
        const int roadW=r-l,inset=std::max(3,roadW/10);l+=inset;r-=inset;
        if(r<=l+8) continue;
        std::vector<int> mids;
        for(int x=l;x<=r;) {
            if(raw.at<unsigned char>(y,x)!=0) {++x;continue;}
            const int a=x;while(x<=r&&raw.at<unsigned char>(y,x)==0) ++x;
            const int b=x-1,run=b-a+1;
            if(run>=1&&run<=std::max(3,roadW/5)) mids.push_back((a+b)/2);
        }
        e.maxBranches=std::max(e.maxBranches,static_cast<int>(mids.size()));
        if(mids.size()==2) {
            const int sep=std::abs(mids[1]-mids[0]);e.maxSeparation=std::max(e.maxSeparation,sep);
            if(sep>=minSep) {
                ++e.splitRows;if(firstSplitY<0) firstSplitY=y;e.nearestSplitY=y;
                if(y>=raw.rows*s.nearYPercent/100) ++e.nearSplitRows;
            }
        } else if(mids.size()==1) {
            const int mid=center[y]>=0?center[y]:(left[y]+right[y])/2;
            if(std::abs(mids[0]-mid)<=std::max(4,roadW/4)) {++e.stemRows;lastStemY=y;}
        }
    }
    e.visible=e.splitRows>=s.minSplitRows && e.stemRows>=s.minStemRows &&
              e.maxSeparation>=minSep && lastStemY>e.nearestSplitY;
    e.nearVisible=e.visible&&e.nearSplitRows>=s.minNearSplitRows;
    return e;
}

ZebraEvidence detectZebraNear(const cv::Mat& raw,const std::vector<int>& left,
                             const std::vector<int>& right,const ZebraSettings& s) {
    ZebraEvidence e;
    if(raw.empty() || raw.type()!=CV_8UC1 || left.size()!=static_cast<std::size_t>(raw.rows) || right.size()!=left.size()) return e;
    cv::Mat internal=cv::Mat::zeros(raw.size(),CV_8UC1);
    for(int y=0;y<raw.rows;++y) {
        const int a=left[y],b=right[y];if(a<0 || b>=raw.cols || b<=a+12) continue;
        const int inset=std::max(2,(b-a)/20);
        const int lo=std::max(a+inset,raw.cols*s.cropLeftPercent/100);
        const int hi=std::min(b-inset,raw.cols*(100-s.cropRightPercent)/100-1);
        for(int x=lo;x<=hi;++x) if(raw.at<unsigned char>(y,x)==0) internal.at<unsigned char>(y,x)=255;
    }
    cv::Mat labels;
    cv::connectedComponents(internal,labels,8,CV_32S);
    long long black=0,area=0,stripeBlack=0,stripeArea=0;
    int consecutive=0,bandRows=0,whiteGap=2;
    const int y0=std::max(0,raw.rows*s.yStartPercent/100);
    const int y1=std::min(raw.rows-1,raw.rows*s.yEndPercent/100);
    for(int y=y0;y<=y1;++y) {
        int l=left[y],r=right[y];
        if(l<0 || r>=raw.cols || r<=l+12) {consecutive=0;bandRows=0;whiteGap=0;continue;}
        const int margin=std::max(2,(r-l)/20);
        l=std::max(l+margin,raw.cols*s.cropLeftPercent/100);
        r=std::min(r-margin,raw.cols*(100-s.cropRightPercent)/100-1);
        if(r<=l+8) {consecutive=0;bandRows=0;whiteGap=0;continue;}
        const auto* row=raw.ptr<unsigned char>(y);
        int count=0,transitions=0,stripes=0;
        std::vector<int> stripeLabels;
        const int minRun=std::max(3,(r-l+1)/14);
        for(int x=l;x<=r;++x) {
            count+=row[x]==0;
            if(x>l && (row[x]==0)!=(row[x-1]==0)) ++transitions;
        }
        for(int x=l;x<=r;) {
            if(row[x]!=0) {++x;continue;}
            const int a=x;while(x<=r && row[x]==0) ++x;
            const int run=x-a;
            // Thin Y branches and the single center line are not zebra stripes.
            if(a>l && x<=r && run>=minRun && run*100<=(r-l+1)*35) {
                const int label=labels.at<int>(y,a);
                if(label>0 && std::find(stripeLabels.begin(),stripeLabels.end(),label)==stripeLabels.end())
                    stripeLabels.push_back(label);
            }
        }
        // Connected Y arms form one black component, even when two thick runs appear in a row.
        stripes=static_cast<int>(stripeLabels.size());
        black+=count;area+=r-l+1;
        e.maxTransitions=std::max(e.maxTransitions,transitions);e.maxStripes=std::max(e.maxStripes,stripes);
        const int pct=count*100/(r-l+1);
        const bool stripeRow=transitions>=s.minTransitions && stripes>=s.minStripes &&
                             pct>=s.minBlackPercent && pct<=s.maxBlackPercent;
        if(stripeRow) {
            ++consecutive;e.maxConsec=std::max(e.maxConsec,consecutive);
            stripeBlack+=count;stripeArea+=r-l+1;
        } else consecutive=0;
        // Also recognize two broad transverse bars separated by actual white rows.
        if(pct>=55) {
            if(bandRows==0 && whiteGap>=1) ++e.transverseBands;
            ++bandRows;whiteGap=0;
        } else if(pct<=20) {bandRows=0;++whiteGap;}
        else {bandRows=0;whiteGap=0;}
    }
    if(area) e.blackPct=static_cast<int>(black*100/area);
    if(stripeArea) e.stripeBlackPct=static_cast<int>(stripeBlack*100/stripeArea);
    e.visible=e.maxConsec>=s.minRows || e.transverseBands>=2;
    e.strong=(e.maxConsec>=std::max(3,s.minRows) && e.stripeBlackPct>=15) || e.transverseBands>=2;
    return e;
}
} }
