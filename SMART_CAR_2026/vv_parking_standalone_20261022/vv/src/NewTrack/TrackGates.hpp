#pragma once

#include <opencv2/core.hpp>
#include <vector>

namespace NewTrack { namespace TrackGates {
struct MarkerSettings {
    int minBlocks=3, centerTolerancePx160=18, minSpanRows82=8;
    int verticalRatioPercent=20, maxWidthPercent=45;
    double minAngleDeg=22.0;
};
struct MarkerEvidence {
    int blocks=0,span=0,spread=0,minSpan=0;
    double angleDeg=0;
    bool blocksPass=false,spanPass=false,ratioPass=false,anglePass=false,passed=false;
};
MarkerEvidence detectMarker(const cv::Mat& binary,const std::vector<int>& left,
                            const std::vector<int>& right,const std::vector<int>& center,
                            const MarkerSettings& settings);

struct ForkSettings {
    int minSplitRows=2,minStemRows=3,minSeparationPx160=12;
    int nearYPercent=70,minNearSplitRows=2;
};
struct ForkEvidence {
    bool visible=false,nearVisible=false;
    int splitRows=0,stemRows=0,maxBranches=0,maxSeparation=0;
    int nearSplitRows=0,nearestSplitY=-1;
};
ForkEvidence detectFork(const cv::Mat& binary,const std::vector<int>& left,
                       const std::vector<int>& right,const std::vector<int>& center,
                       const ForkSettings& settings);
struct ZebraSettings {
    int yStartPercent=38,yEndPercent=92,cropLeftPercent=8,cropRightPercent=8;
    int minTransitions=3,minStripes=2,minRows=2,minBlackPercent=5,maxBlackPercent=65;
};
struct ZebraEvidence {
    int maxConsec=0,maxTransitions=0,maxStripes=0,blackPct=0,stripeBlackPct=0,transverseBands=0;
    bool visible=false,strong=false;
};
ZebraEvidence detectZebraNear(const cv::Mat& binary,const std::vector<int>& left,
                             const std::vector<int>& right,const ZebraSettings& settings);
} }
