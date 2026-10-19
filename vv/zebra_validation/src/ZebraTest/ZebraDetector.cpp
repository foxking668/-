#include "ZebraDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <numeric>
#include <map>

namespace ZebraTest {
namespace {
struct Run {int a,b;};
BandEvidence band(const cv::Mat& gray,const cv::Mat& raw,const std::vector<int>& left,
                  const std::vector<int>& right,double top,double bottom,const Settings& s) {
    BandEvidence e;cv::Mat mask=cv::Mat::zeros(raw.size(),CV_8UC1);
    const int y0=std::clamp(int(top*raw.rows),0,raw.rows-1),y1=std::clamp(int(bottom*raw.rows),y0,raw.rows-1);
    long long black=0,area=0;double grayBlack=0,grayWhite=0;long long nb=0,nw=0;
    struct Sample {int y,count;double middle,width;};
    std::vector<Sample> samples;
    int longest=0,consecutive=0;
    for(int y=y0;y<=y1;++y) {
        int l=left[y],r=right[y];
        if(l<0||r>=raw.cols||r-l<raw.cols*.22) {consecutive=0;continue;}
        const int margin=std::max(1,int((r-l)*s.road_margin));l+=margin;r-=margin;
        if(r-l<10){consecutive=0;continue;}
        ++e.valid_rows;const int w=r-l+1;std::vector<Run> runs;
        int rowBlack=0;
        for(int x=l;x<=r;++x) {
            const bool b=raw.at<unsigned char>(y,x)==0;
            rowBlack+=b;black+=b;++area;
            if(b){grayBlack+=gray.at<unsigned char>(y,x);++nb;mask.at<unsigned char>(y,x)=255;}
            else {grayWhite+=gray.at<unsigned char>(y,x);++nw;}
        }
        for(int x=l;x<=r;) {
            if(raw.at<unsigned char>(y,x)!=0){++x;continue;}
            const int a=x;while(x<=r&&raw.at<unsigned char>(y,x)==0)++x;
            const int b=x-1,len=b-a+1;
            if(a>l&&b<r&&len>=std::max(2,int(std::lround(w*s.run_min)))&&len<=w*s.run_max)runs.push_back({a,b});
        }
        // Select consecutive groups; a single thin guide line must not split a valid group.
        bool hit=false;int bestCount=0;std::size_t bestA=0,bestB=0;
        for(std::size_t a=0;a<runs.size();++a)for(std::size_t b=a+s.min_stripes-1;b<runs.size();++b) {
            int minW=raw.cols,maxW=0,minG=raw.cols,maxG=0;
            for(std::size_t i=a;i<=b;++i) {
                const int width=runs[i].b-runs[i].a+1;minW=std::min(minW,width);maxW=std::max(maxW,width);
                if(i>a){const int gap=runs[i].a-runs[i-1].b-1;minG=std::min(minG,gap);maxG=std::max(maxG,gap);}
            }
            if(minG<std::max(1,int(std::lround(w*s.gap_min)))||maxG>minG*s.gap_ratio_max||maxW>minW*s.width_ratio_max)continue;
            hit=true;if(int(b-a+1)>bestCount){bestCount=int(b-a+1);bestA=a;bestB=b;}
        }
        const double ratio=double(rowBlack)/w;
        hit=hit&&ratio>=s.black_min&&ratio<=s.black_max;
        e.max_stripes=std::max(e.max_stripes,bestCount);
        if(hit) {
            ++e.hit_rows;++consecutive;longest=std::max(longest,consecutive);
            e.nearest_y=double(y)/raw.rows;
            samples.push_back({y,bestCount,(runs[bestA].a+runs[bestB].b)*.5,double(runs[bestB].b-runs[bestA].a)});
            for(std::size_t i=bestA;i<=bestB;++i)e.segments.push_back({y,runs[i].a,runs[i].b});
        }else consecutive=0;
    }
    e.valid=e.valid_rows>=std::max(3,int((y1-y0+1)*.25));
    e.black_ratio=area?double(black)/area:0;
    e.contrast=nb&&nw?grayWhite/nw-grayBlack/nb:0;
    // Evaluate each supported group size separately. Perspective can expose fewer bars
    // on one row; neither a single maximum-count row nor a changing group is reliable.
    std::map<int,int> counts;for(auto row:samples)++counts[row.count];
    bool geometryPass=false;int geometryRows=0,bestBadRows=0;
    for(auto group:counts) {
        if(group.second<s.min_rows)continue;
        double my=0,mx=0,mw=0;
        for(auto row:samples)if(row.count==group.first){my+=row.y;mx+=row.middle;mw+=row.width;}
        my/=group.second;mx/=group.second;mw/=group.second;
        double variance=0,covariance=0,widthCov=0;
        for(auto row:samples)if(row.count==group.first){variance+=(row.y-my)*(row.y-my);covariance+=(row.y-my)*(row.middle-mx);widthCov+=(row.y-my)*(row.width-mw);}
        const double slope=variance>0?std::abs(covariance/variance):0,spread=variance>0?std::abs(widthCov/variance):0;
        if(slope<=s.max_slope&&spread<=s.max_direction_spread) {
            geometryPass=true;
            if(group.second>geometryRows){geometryRows=group.second;e.alignment=slope;e.direction_spread=spread;}
        } else if(!geometryPass&&group.second>bestBadRows){bestBadRows=group.second;e.alignment=slope;e.direction_spread=spread;}
    }
    cv::Mat labels,stats,centers;
    if(e.hit_rows>=s.min_rows) {
        const int n=cv::connectedComponentsWithStats(mask,labels,stats,centers,8,CV_32S);
        for(int i=1;i<n;++i)if(stats.at<int>(i,cv::CC_STAT_AREA)>=3&&stats.at<int>(i,cv::CC_STAT_HEIGHT)>=2)++e.components;
    }
    const bool shape=longest>=s.min_rows&&geometryPass;
    e.hit=e.valid&&shape&&e.contrast>=s.contrast_min;
    e.strong=e.hit&&longest>=s.strong_rows;
    e.reason=!e.valid?"ROAD_COVERAGE_LOW":e.max_stripes<s.min_stripes?"STRIPES_LOW":
        longest<s.min_rows?"ROWS_LOW":e.alignment>s.max_slope?"GROUP_ALIGNMENT":e.direction_spread>s.max_direction_spread?"BRANCH_DIVERGENCE":
        e.contrast<s.contrast_min?"CONTRAST_LOW":!geometryPass?"GROUP_SUPPORT_LOW":e.hit?"GROUP_OK":"AREA_OR_GAPS";
    return e;
}
}
Observation detect(const cv::Mat& gray,const cv::Mat& raw,const std::vector<int>& left,
                   const std::vector<int>& right,bool laneValid,bool quality,long long timeMs,const Settings& s) {
    Observation o;o.time_ms=timeMs;o.lane_valid=laneValid;
    o.input_valid=!raw.empty()&&raw.type()==CV_8UC1&&gray.type()==CV_8UC1&&gray.size()==raw.size()&&
        left.size()==std::size_t(raw.rows)&&right.size()==left.size()&&quality;
    if(!o.input_valid)return o;
    o.far=band(gray,raw,left,right,s.far_top,s.far_bottom,s);
    o.near=band(gray,raw,left,right,s.near_top,s.near_bottom,s);
    return o;
}
}
