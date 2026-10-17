#pragma once
#include <opencv2/core.hpp>
#include <vector>

namespace NewTrack { namespace ConeBoundary {
enum class Side { NONE=0, LEFT=1, RIGHT=2 };
struct Settings {
    int hMin=90, hMax=135, sMin=60, vMin=45;
    int minArea80x60=5, maxAreaPercent=8, marginPx80=4, minCorridorPx80=12;
    int confirmFrames=2, holdMs=160, nearYPercent=30;
};
struct Result {
    bool detected=false, active=false, held=false, blocked=false;
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
    long long lastSeenMs_=0;
public:
    void reset() { *this=Detector{}; }
    Result update(const cv::Mat& bgr, std::vector<int>& left, std::vector<int>& right,
                  std::vector<int>& center, long long nowMs, const Settings& settings);
};
} }
