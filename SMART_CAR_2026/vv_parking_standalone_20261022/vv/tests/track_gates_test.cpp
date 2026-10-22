#include "NewTrack/TrackGates.hpp"
#include <opencv2/imgproc.hpp>
#include <cassert>
#include <iostream>

using namespace NewTrack::TrackGates;
int main() {
    const int h=56,w=80;
    std::vector<int> l(h,5),r(h,75),m(h,40);
    cv::Mat bin(h,w,CV_8UC1,cv::Scalar(255));
    for(int y:{15,25,35}) cv::rectangle(bin,{39,y,3,3},cv::Scalar(0),-1);
    auto e=detectMarker(bin,l,r,m,MarkerSettings{});
    assert(e.passed&&e.blocks==3&&e.angleDeg>89&&e.minSpan==5);
    cv::Mat one=bin.clone();one.rowRange(34,39).setTo(255);
    assert(!detectMarker(one,l,r,m,MarkerSettings{}).passed);
    cv::Mat sideways(h,w,CV_8UC1,cv::Scalar(255));
    for(int x:{33,40,47}) cv::rectangle(sideways,{x,20,3,3},cv::Scalar(0),-1);
    auto horizontal=detectMarker(sideways,l,r,m,MarkerSettings{});
    assert(horizontal.blocks==3&&!horizontal.anglePass&&!horizontal.passed);
    cv::Mat offset(h,w,CV_8UC1,cv::Scalar(255));
    for(int y:{15,25,35}) cv::rectangle(offset,{15,y,3,3},cv::Scalar(0),-1);
    assert(detectMarker(offset,l,r,m,MarkerSettings{}).blocks==0);
    cv::Mat tilted(h,w,CV_8UC1,cv::Scalar(255));
    for(int i=0;i<3;++i) cv::rectangle(tilted,{35+i*4,15+i*8,3,3},cv::Scalar(0),-1);
    assert(detectMarker(tilted,l,r,m,MarkerSettings{}).passed);
    cv::Mat zebra(h,w,CV_8UC1,cv::Scalar(255));
    for(int y:{15,25,35}) cv::rectangle(zebra,{10,y,60,3},cv::Scalar(0),-1);
    assert(!detectMarker(zebra,l,r,m,MarkerSettings{}).passed);

    auto fork=[&](int splitY,int count=2) {
        cv::Mat out(h,w,CV_8UC1,cv::Scalar(255));
        for(int y=splitY;y<splitY+4;++y) {
            out.at<unsigned char>(y,32)=0;out.at<unsigned char>(y,48)=0;
            if(count==3) out.at<unsigned char>(y,40)=0;
        }
        for(int y=45;y<51;++y) out.at<unsigned char>(y,40)=0;
        return out;
    };
    ForkSettings legacy;legacy.nearYPercent=60;
    auto far=detectFork(fork(18),l,r,m,legacy);
    assert(far.visible&&!far.nearVisible&&far.nearSplitRows==0);
    auto near=detectFork(fork(36),l,r,m,legacy);
    assert(near.visible&&near.nearVisible&&near.nearSplitRows==4);
    assert(!detectFork(fork(36,3),l,r,m,ForkSettings{}).visible);
    assert(!detectFork(zebra,l,r,m,ForkSettings{}).visible);
    cv::Mat branches= fork(36);branches.rowRange(45,51).setTo(255);
    assert(!detectFork(branches,l,r,m,ForkSettings{}).visible);
    assert(!detectMarker({},l,r,m,MarkerSettings{}).passed);
    assert(!detectFork(bin,{},r,m,ForkSettings{}).visible);
    // Bottom 30% is y>=70%: a former 60%-ROI candidate remains observation only.
    assert(!detectFork(fork(34),l,r,m,ForkSettings{}).nearVisible);
    assert(detectFork(fork(40),l,r,m,ForkSettings{}).nearVisible);
    ZebraSettings zs;
    cv::Mat two(h,w,CV_8UC1,cv::Scalar(255));
    // Two broad stripes in just a few near rows: global ROI area can be small.
    for(int x:{25,48}) cv::rectangle(two,{x,39,7,4},cv::Scalar(0),-1);
    const auto ze=detectZebraNear(two,l,r,zs);
    assert(ze.visible&&ze.strong&&ze.maxStripes==2&&ze.maxConsec==4);
    assert(ze.stripeBlackPct>ze.blackPct);
    assert(!detectZebraNear(fork(40),l,r,zs).visible);
    cv::Mat centered(h,w,CV_8UC1,cv::Scalar(255));
    cv::rectangle(centered,{38,21,3,31},cv::Scalar(0),-1);
    assert(!detectZebraNear(centered,l,r,zs).visible);
    cv::Mat thickY(h,w,CV_8UC1,cv::Scalar(255));
    cv::line(thickY,{26,23},{40,45},cv::Scalar(0),5);
    cv::line(thickY,{54,23},{40,45},cv::Scalar(0),5);
    cv::line(thickY,{40,45},{40,53},cv::Scalar(0),5);
    assert(!detectZebraNear(thickY,l,r,zs).visible); // Thick connected Y arms are also not stripes.
    cv::Mat bars(h,w,CV_8UC1,cv::Scalar(255));
    for(int y:{32,40}) cv::rectangle(bars,{10,y,60,3},cv::Scalar(0),-1);
    const auto transverse=detectZebraNear(bars,l,r,zs);
    assert(transverse.visible&&transverse.strong&&transverse.transverseBands==2);
    cv::Mat solid(h,w,CV_8UC1,cv::Scalar(0));
    assert(!detectZebraNear(solid,l,r,zs).visible); // One solid dark patch is not two bars.
    std::cout<<"PASS marker filters: 3 blocks, angle, span, center, zebra rejection; far/near Y geometry.\n";
}
