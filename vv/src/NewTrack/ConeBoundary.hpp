#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace NewTrack { namespace ConeBoundary {
enum class Side { NONE=0, LEFT=1, RIGHT=2, CENTER=3 };
inline const char* sideName(Side side) {return side==Side::LEFT?"LEFT":side==Side::RIGHT?"RIGHT":side==Side::CENTER?"CENTER":"NONE";}
struct Settings {
    int hMin=90, hMax=135, sMin=60, vMin=45;
    int minArea80x60=5, maxAreaPercent=8, marginPx80=4, minCorridorPx80=12;
    int confirmFrames=2, holdMs=160, nearYPercent=30, maxConfirmGapMs=100;
    int nearMaxAreaPercent=16, nearMaxWidthPercent=38, nearMaxHeightPercent=80;
};
struct Result {
    bool detected=false, confirmed=false, roadValid=false, active=false, held=false, blocked=false;
    int bluePixels=0, components=0, rejectedSize=0, rejectedRoad=0, confirmations=0;
    const char* reason="NO_BLUE";
    Side side=Side::NONE;
    cv::Rect box;
    int changedRows=0;
    cv::Mat blueMask;
};
// Detect on this frame's colour image and UNMODIFIED lane. A board is never a cone fallback.
class Detector {
    Side pending_=Side::NONE;
    int confirmations_=0, width_=0, height_=0;
    cv::Rect pendingBox_, heldBox_;
    Side heldSide_=Side::NONE;
    long long lastSeenMs_=0, pendingMs_=0;
public:
    void reset() { *this=Detector{}; }
    Result update(const cv::Mat& bgr, std::vector<int>& left, std::vector<int>& right,
                  std::vector<int>& center, long long nowMs, const Settings& settings);
};
} }
