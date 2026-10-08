#pragma once
#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace car2026 {
constexpr double pi = 3.14159265358979323846;
inline double clamp(double x, double lo, double hi) { return std::max(lo, std::min(hi, x)); }
inline double wrapDegrees(double x) {
    x = std::fmod(x + 180.0, 360.0); if (x < 0) x += 360.0; return x - 180.0;
}

// All distances are metres, speeds m/s, time seconds and steering degrees.
// These defaults are conservative starting values, not measurements of this car.
#define CAR_PARAMETERS(X) \
 X(wheel_circumference_m, 0.20, 0.05, 1.0) \
 X(wheelbase_m, 0.18, 0.08, 0.60) \
 X(steer_to_wheel_ratio, 1.0, 0.05, 3.0) \
 X(steer_offset_deg, 0.0, -30, 30) \
 X(steer_sign, 1, -1, 1) \
 X(encoder_left_sign, 1, -1, 1) \
 X(encoder_right_sign, 1, -1, 1) \
 X(imu_yaw_sign, 1, -1, 1) \
 X(encoder_ppr, 1024, 1, 20000) \
 X(cruise_speed, 0.25, 0.02, 1.5) \
 X(cone_speed, 0.12, 0.02, 0.5) \
 X(ring_speed, 0.10, 0.02, 0.5) \
 X(cross_speed, 0.08, 0.02, 0.3) \
 X(parking_speed, 0.06, 0.01, 0.2) \
 X(max_steer_deg, 22, 5, 35) \
 X(steer_rate_deg_s, 100, 10, 300) \
 X(lateral_gain, 26, 0, 100) \
 X(heading_gain, 12, 0, 100) \
 X(straight_lateral_gain, 8, 0, 100) \
 X(straight_heading_gain, 12, 0, 100) \
 X(straight_filter_s, 0.15, 0, 1) \
 X(straight_deadband_image, 0.005, 0, 0.05) \
 X(straight_max_command, 5, 0.1, 5) \
 X(straight_command_rate_s, 3, 0.1, 30) \
 X(curvature_slow_gain, 2.0, 0, 10) \
 X(pid_kp, 64, 0, 5000) \
 X(pid_ki, 32, 0, 5000) \
 X(pid_kd, 48, 0, 5000) \
 X(pwm_limit, 12000, 100, 50000) \
 X(accel_m_s2, 0.25, 0.05, 2.0) \
 X(frame_timeout_s, 0.4, 0.1, 2.0) \
 X(max_encoder_rps, 15, 1, 100) \
 X(stall_timeout_s, 2.0, 0.5, 10) \
 X(black_v_max, 85, 10, 180) \
 X(black_s_max, 100, 10, 255) \
 X(white_v_min, 140, 50, 245) \
 X(white_s_max, 85, 10, 200) \
 X(blue_h_min, 90, 0, 179) \
 X(blue_h_max, 135, 0, 179) \
 X(blue_s_min, 90, 20, 255) \
 X(blue_v_min, 35, 5, 255) \
 X(crop_top_ratio, 0.28, 0, 0.65) \
 X(line_width_min_ratio, 0.004, 0.001, 0.05) \
 X(line_width_max_ratio, 0.085, 0.01, 0.20) \
 X(parking_line_near_width_ratio, 0.15, 0.02, 0.25) \
 X(line_search_ratio, 0.16, 0.03, 0.4) \
 X(line_min_confidence, 0.35, 0.1, 0.9) \
 X(line_local_contrast_min, 15, 5, 80) \
 X(line_hold_s, 0.30, 0, 1.0) \
 X(confirm_frames, 3, 1, 20) \
 X(clear_frames, 5, 2, 30) \
 X(cone_min_area_ratio, 0.0008, 0.0001, 0.02) \
 X(cone_max_area_ratio, 0.07, 0.005, 0.3) \
 X(cone_clearance_ratio, 0.12, 0.03, 0.3) \
 X(cone_min_travel_m, 0.5, 0.1, 5) \
 X(cone_timeout_s, 35, 5, 120) \
 X(ring_side, -1, -1, 1) \
 X(ring_entry_bias_deg, 12, 2, 25) \
 X(ring_entry_distance_m, 0.25, 0.05, 1.5) \
 X(ring_min_yaw_deg, 340, 270, 370) \
 X(ring_max_yaw_deg, 430, 370, 540) \
 X(ring_min_travel_m, 0.8, 0.2, 10) \
 X(ring_exit_min_travel_m, 0.15, 0.05, 1.0) \
 X(ring_timeout_s, 60, 10, 180) \
 X(cross_near_y_ratio, 0.70, 0.4, 0.95) \
 X(cross_stop_y_ratio, 0.78, 0.5, 0.98) \
 X(cross_stop_s, 3.3, 3, 10) \
 X(cross_pass_m, 0.32, 0.1, 2.0) \
 X(cross_approach_max_m, 0.8, 0.2, 2.0) \
 X(cross_wait_timeout_s, 120, 10, 300) \
 X(stop_every_cross, 0, 0, 1) \
 X(bar_roi_top, 0.08, 0, 0.7) \
 X(bar_roi_bottom, 0.55, 0.1, 0.9) \
 X(bar_span_ratio, 0.45, 0.1, 0.95) \
 X(bar_saturation_min, 65, 20, 200) \
 X(bar_pixel_ratio, 0.07, 0.01, 0.5) \
 X(garage_depart_m, 0.35, 0.1, 2.0) \
 X(garage_min_after_cross_m, 0.08, 0, 2) \
 X(garage_align_distance_m, 0.12, 0.03, 1) \
 X(garage_advance_m, 0.25, 0.05, 1.0) \
 X(garage_reverse_m, 0.35, 0.1, 1.2) \
 X(garage_align_tolerance, 0.08, 0.01, 0.3) \
 X(garage_outward_yaw_delta_deg, -30, -90, 90) \
 X(parking_heading_gain, 0.8, 0.05, 5) \
 X(parking_timeout_s, 25, 5, 120) \
 X(stage_timeout_s, 60, 5, 180) \
 X(camera_index, 0, 0, 20) \
 X(camera_width, 1280, 320, 1920) \
 X(camera_height, 720, 240, 1080) \
 X(process_width, 320, 160, 640) \
 X(process_height, 240, 120, 480) \
 X(motion_calibrated, 0, 0, 1) \
 X(require_imu, 1, 0, 1)

struct Params {
#define FIELD(n,d,lo,hi) double n = d;
    CAR_PARAMETERS(FIELD)
#undef FIELD
    std::string imu_device;
    std::string imu_format = "wit11"; // wit11 binary angles, or explicitly supplied ascii yaw
    int imu_baud = 115200;
    void validate() const;
    static Params load(const std::string& filename);
};

struct Pixel { uint8_t r=0, g=0, b=0; };
struct Image {
    int width=0, height=0;
    std::vector<Pixel> pixels;
    Image() = default;
    Image(int w, int h, Pixel color={}) : width(w), height(h), pixels(w*h,color) {}
    Pixel& at(int x,int y) { return pixels.at(y*width+x); }
    const Pixel& at(int x,int y) const { return pixels.at(y*width+x); }
};
struct Path {
    std::vector<double> x; // Normalized x for each full-frame row; -1 means missing.
    double confidence=0, lateral=0, heading=0, curvature=0;
    bool ambiguous=false; // Competing guide segments; not a measured pose.
    bool discontinuous=false; // Abrupt cross-row jump; cannot be trusted for parking.
    bool valid() const { return confidence > 0 && !x.empty(); }
};
struct Blob { int x=0,y=0,w=0,h=0,area=0; double bottomX=0,bottomY=0; };
struct Observation {
    bool frameValid=false, stripe=false, bar=false, barObservable=false;
    bool garage=false, ringCandidate=false;
    double stripeY=0, garageError=0;
    Path blackPath, roadPath;
    std::vector<Blob> cones;
    std::vector<Blob> ringIslands; // Excluded from cone count, retained as approach obstacles.
};
struct Telemetry {
    double time=0, distance=0, speed=0, yaw=0;
    bool yawValid=false, yawMeasured=false, encoderValid=true;
};
enum class Stage {
    Depart, ToCones, Cones, ToRing, RingEntry, RingLap, RingExit,
    ToCross, CrossApproach, CrossWait, CrossPass, ToGarage,
    GarageAlign, GarageAdvance, GarageReverse, Complete, Fault
};
const char* stageName(Stage stage);
struct Command {
    double speed=0, steer=0;
    bool speak=false;
    Stage stage=Stage::Depart;
    int completedLaps=0;
    std::string reason;
};

class Vision {
public:
    explicit Vision(Params p) : params_(std::move(p)) {}
    Observation analyze(const Image& image, Stage stage);
    Path avoidCones(const Observation& o, const Image& image);
    void resetTracking();
private:
    Params params_;
    Path previousBlack_;
    Path pendingBlack_;
    int blackReacquireHits_=0;
    bool coneSideValid_=false, passRight_=false;
    double lastConeX_=0,lastConeY_=0;
    std::vector<uint8_t> black_, white_, blue_, guide_, brightness_;
    Path trackBlack(int w,int h, Stage stage);
    Path trackBlackFrom(int w,int h, Stage stage,const Path& prior,double startX);
    Path trackRoad(int w,int h, Stage stage);
    std::vector<Blob> blueBlobs(int w,int h) const;
};
void measurePath(Path& path);

class Mission {
public:
    explicit Mission(Params p) : params_(std::move(p)) {}
    Command update(const Observation& observation, const Telemetry& telemetry, const Path& avoidance);
    Stage stage() const { return stage_; }
    int completedLaps() const { return completedLaps_; }
    const std::string& fault() const { return fault_; }
    void fail(const std::string& reason) { fault_=reason; stage_=Stage::Fault; }
private:
    Params params_;
    Stage stage_=Stage::Depart;
    int completedLaps_=0, hits_=0, clears_=0;
    bool initialized_=false, stopStarted_=false, spoke_=false;
    double stageTime_=0, stageDistance_=0, ringYaw_=0, stopTime_=0;
    double parkingYaw_=0, lastPathTime_=-1e9, lastSteer_=0, lastTime_=0, lastDistance_=0;
    bool crossStopRequired_=false;
    Path heldPath_;
    std::string fault_;
    void enter(Stage next, const Telemetry& t);
    bool confirmed(bool condition);
    bool clearConfirmed(bool condition);
    double follow(const Path& path, double& speed, const Telemetry& t);
};

// Unwrap measured yaw and reject impossible jumps rather than adding a false revolution.
class YawTracker {
public:
    bool ingest(double wrappedYaw,double time);
    bool fresh(double time) const { return initialized_ && !lost_ && time-lastTime_>=0 && time-lastTime_<0.35; }
    double value() const { return accumulated_; }
private:
    bool initialized_=false, lost_=false;
    double previous_=0, accumulated_=0, lastTime_=0;
};
} // namespace car2026
