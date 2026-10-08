#pragma once
#include "core.hpp"

namespace car2026 {
// Normalized image features only: neither metres nor measured vehicle yaw.
struct StraightImageFeatures {
    double lateral=0,heading=0,maximumResidual=0;
    int supportedRows=0,excludedRows=0;
};
namespace straight_image_detail {
struct LineFit {
    double sumY=0,sumX=0,sumYY=0,sumYX=0;int count=0;
    void add(double row,double x,int weight=1) {
        sumY+=weight*row;sumX+=weight*x;sumYY+=weight*row*row;sumYX+=weight*row*x;count+=weight;
    }
    bool solve(double& slope,double& intercept) const {
        const double denominator=count*sumYY-sumY*sumY;
        if(count<2 || denominator<=1e-12) return false;
        slope=(count*sumYX-sumY*sumX)/denominator;
        intercept=(sumX-slope*sumY)/count;
        return std::isfinite(slope) && std::isfinite(intercept);
    }
};
}
inline bool imageBandAvailable(const Path& path,double ratio) {
    const int h=int(path.x.size()),center=int(h*ratio),radius=std::max(2,h/30);
    int available=0,total=0;
    for(int y=std::max(0,center-radius);y<std::min(h,center+radius+1);++y) {
        const double x=path.x[y];++total;
        if(!std::isfinite(x) || x>1 || x< -1) return false;
        if(x>=0) ++available;
    }
    return total && available*2>=total;
}
// Shared by reference-hold observation and forward straight following.
// Shape acceptance is independent of image centering and slope. It does not
// establish semantic main-line identity at a junction.
inline bool fitStraightImageFeatures(const Path& path,StraightImageFeatures& features) {
    features=StraightImageFeatures{};
    if(path.x.size()<24 || path.x.size()>1080) return false;
    constexpr double top=.40,bottom=.90,maxResidual=.008,maxIsolatedResidual=.012;
    constexpr int maxIsolatedPoints=2,minimumPointsPerExclusion=40;
    const int h=int(path.x.size());
    straight_image_detail::LineFit fit;int total=0;
    for(int y=int(h*top);y<=int(h*bottom);++y) {
        ++total;const double x=path.x[y];
        if(!std::isfinite(x) || x>1 || x< -1) return false;
        if(x<0) continue;
        fit.add(double(y)/h,x);
    }
    if(fit.count*5<total*4) return false;
    double slope=0,intercept=0;
    if(!fit.solve(slope,intercept)) return false;
    const int allowance=std::min(maxIsolatedPoints,fit.count/minimumPointsPerExclusion);
    int excluded[maxIsolatedPoints]={-1,-1},excludedCount=0;
    for(int y=int(h*top);y<=int(h*bottom);++y) {
        if(path.x[y]<0) continue;
        const double residual=std::abs(path.x[y]-(intercept+slope*double(y)/h));
        if(residual>maxIsolatedResidual) return false;
        if(residual>maxResidual) {
            if(excludedCount>=allowance) return false;
            excluded[excludedCount++]=y;
        }
    }
    if(excludedCount) {
        for(int n=0;n<excludedCount;++n) fit.add(double(excluded[n])/h,path.x[excluded[n]],-1);
        if(fit.count*5<total*4 || !fit.solve(slope,intercept)) return false;
    }
    double maximum=0;
    for(int y=int(h*top);y<=int(h*bottom);++y) {
        if(path.x[y]<0) continue;
        const bool omitted=y==excluded[0] || y==excluded[1];
        const double residual=std::abs(path.x[y]-(intercept+slope*double(y)/h));
        if(residual>(omitted ? maxIsolatedResidual : maxResidual)) return false;
        maximum=std::max(maximum,residual);
    }
    features.lateral=intercept+slope*.84-.5;
    features.heading=slope*(.45-.84);
    features.maximumResidual=maximum;features.supportedRows=fit.count;features.excludedRows=excludedCount;
    return std::isfinite(features.lateral) && std::isfinite(features.heading);
}
} // namespace car2026
