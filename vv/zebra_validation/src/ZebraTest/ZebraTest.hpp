#pragma once
#include <cmath>
#include <string>
#include <vector>

namespace ZebraTest {
// Speeds use the original encoder RPS convention; PWM uses the VV17 driver units.
struct Settings {
    double far_top=.15, far_bottom=.55, near_top=.30, near_bottom=.92;
    double road_margin=.035, run_min=.025, run_max=.28, gap_min=.018;
    double gap_ratio_max=3.0, width_ratio_max=3.0;
    double black_min=.10, black_max=.70, contrast_min=20, max_slope=1.4, max_direction_spread=.8;
    int min_stripes=3, min_rows=2, strong_rows=3, far_frames=2, near_frames=2, fallback_frames=3;
    double trigger_y=.48, approach_speed=7, stationary_rps=.4;
    int candidate_timeout_ms=1500, reverse_pwm=12000, reverse_max_ms=800, coast_ms=50;
    int reverse_min_ms=50, stationary_ms=200, settle_timeout_ms=4000, hold_ms=3000;
    int rearm_min_ms=1500, absent_ms=400, input_timeout_ms=350, speed_timeout_ms=200;
    int resume_valid_frames=3, video_fps=10, video_queue=4, log_queue=2048, control_log_ms=50;
    int flush_ms=200, minimum_free_mb=256, storage_check_ms=1000;
    bool voice_enable=true;
    int illumination_band_rows=9;
    int lane_loss_timeout_ms=1500;
    int startup_timeout_ms=5000;
};
Settings loadSettings(const std::string& path);
void validate(const Settings& s);

struct StripeSegment {int y,a,b;};
struct BandEvidence {
    bool valid=false, hit=false, strong=false;
    int valid_rows=0, hit_rows=0, max_stripes=0, components=0;
    double black_ratio=0, contrast=0, nearest_y=-1, alignment=0, direction_spread=0;
    std::string reason="NO_INPUT";
    std::vector<StripeSegment> segments;
};
struct Observation {
    bool input_valid=false, lane_valid=false;
    long long time_ms=-1;
    BandEvidence far, near;
};
enum class Phase { Waiting, Cruise, Approach, LanePause, Coast, Brake, Settle, Hold, Depart, Fault };
const char* phaseName(Phase p);
struct MotorInput {
    long long time_ms=0;
    double left_rps=0,right_rps=0;
    bool encoder_valid=false, external_stop=false;
};
struct Command {
    bool override_pwm=true;
    int left_pwm=0,right_pwm=0;
    bool speed_limit=false;
    double max_speed=0;
};
// One owner (Runtime mutex) serializes image and control observations.
class Controller {
public:
    explicit Controller(const Settings& settings):s_(settings) {}
    void observe(const Observation& o);
    Command step(const MotorInput& in);
    void acknowledge(long long now,bool success);
    void fault(const std::string& reason);
    Phase phase() const {return phase_;}
    unsigned count() const {return count_;}
    int farFrames() const {return far_count_;}
    int nearFrames() const {return near_count_;}
    int strongFrames() const {return strong_count_;}
    long long holdRemaining(long long now) const;
    const std::string& reason() const {return reason_;}
    const Observation& observation() const {return obs_;}
private:
    Settings s_;
    Observation obs_;
    Phase phase_=Phase::Waiting;
    Phase resume_phase_=Phase::Cruise;
    std::string reason_="WAITING_VALID_IMAGE";
    long long phase_at_=-1,output_at_=-1,still_at_=-1,last_candidate_=-1,absent_at_=-1;
    int far_count_=0,near_count_=0,strong_count_=0,valid_count_=0;
    unsigned count_=0;
    bool output_ok_=false,depart_moved_=false;
    void enter(Phase p,long long now,const std::string& reason);
};
}
