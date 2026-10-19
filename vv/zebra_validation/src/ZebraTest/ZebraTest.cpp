#include "ZebraTest.hpp"
#include <algorithm>
#include <fstream>
#include <map>
#include <set>
#include <stdexcept>

namespace ZebraTest {
static std::string trim(std::string s) {
    const auto a=s.find_first_not_of(" \r\n\t");
    return a==std::string::npos?"":s.substr(a,s.find_last_not_of(" \r\n\t")-a+1);
}
Settings loadSettings(const std::string& path) {
    Settings s;std::ifstream f(path);if(!f) throw std::runtime_error("CONFIG_OPEN: "+path);
    std::map<std::string,double*> doubles;
    std::map<std::string,int*> ints;
#define D(k) doubles[#k]=&s.k
#define I(k) ints[#k]=&s.k
    D(far_top);D(far_bottom);D(near_top);D(near_bottom);D(road_margin);D(run_min);D(run_max);D(gap_min);
    D(gap_ratio_max);D(width_ratio_max);D(black_min);D(black_max);D(contrast_min);D(max_slope);D(max_direction_spread);
    D(trigger_y);D(approach_speed);D(stationary_rps);
    I(min_stripes);I(min_rows);I(strong_rows);I(far_frames);I(near_frames);I(fallback_frames);
    I(candidate_timeout_ms);I(reverse_pwm);I(reverse_max_ms);I(coast_ms);I(reverse_min_ms);
    I(stationary_ms);I(settle_timeout_ms);I(hold_ms);I(rearm_min_ms);I(absent_ms);
    I(input_timeout_ms);I(speed_timeout_ms);I(resume_valid_frames);I(video_fps);I(video_queue);
    I(log_queue);I(control_log_ms);I(flush_ms);I(minimum_free_mb);I(storage_check_ms);
    I(illumination_band_rows);
    I(lane_loss_timeout_ms);
    I(startup_timeout_ms);
#undef D
#undef I
    std::set<std::string> seen;std::string line;int line_no=0;
    while(std::getline(f,line)) {
        ++line_no;line=trim(line.substr(0,line.find('#')));if(line.empty()) continue;
        const auto eq=line.find('=');
        if(eq==std::string::npos) throw std::runtime_error("CONFIG_SYNTAX line "+std::to_string(line_no));
        const auto k=trim(line.substr(0,eq)),v=trim(line.substr(eq+1));
        if(!seen.insert(k).second) throw std::runtime_error("CONFIG_DUPLICATE: "+k);
        std::size_t used=0;double value=std::stod(v,&used);
        if(used!=v.size() || !std::isfinite(value)) throw std::runtime_error("CONFIG_NUMBER: "+k);
        if(doubles.count(k)) *doubles[k]=value;
        else if(ints.count(k)) {
            if(value!=std::floor(value)||value<0||value>10000000) throw std::runtime_error("CONFIG_INTEGER: "+k);
            *ints[k]=static_cast<int>(value);
        } else if(k=="voice_enable" && (value==0||value==1)) s.voice_enable=value==1;
        else throw std::runtime_error("CONFIG_UNKNOWN: "+k);
    }
    if(!f.eof()) throw std::runtime_error("CONFIG_READ: "+path);
    validate(s);return s;
}
void validate(const Settings& s) {
    auto require=[](bool ok,const char* msg){if(!ok)throw std::runtime_error(msg);};
    require(s.far_top>=0&&s.far_top<s.far_bottom&&s.far_bottom<=1,"CONFIG_FAR_ROI");
    require(s.near_top>=0&&s.near_top<s.near_bottom&&s.near_bottom<=1,"CONFIG_NEAR_ROI");
    require(s.trigger_y>=s.near_top&&s.trigger_y<=s.near_bottom,"CONFIG_TRIGGER_Y");
    require(s.road_margin>=0&&s.road_margin<.25&&s.run_min>0&&s.run_min<s.run_max&&s.run_max<.5,"CONFIG_RUNS");
    require(s.gap_min>0&&s.gap_min<.25&&s.gap_ratio_max>=1&&s.width_ratio_max>=1,"CONFIG_GAPS");
    require(s.black_min>0&&s.black_min<s.black_max&&s.black_max<1&&s.contrast_min>=0&&s.contrast_min<=255,"CONFIG_AREA");
    require(s.max_slope>=0&&s.max_slope<=3,"CONFIG_SLOPE");
    require(s.max_direction_spread>=0&&s.max_direction_spread<=3,"CONFIG_DIRECTION_SPREAD");
    require(s.min_stripes>=3&&s.min_stripes<=10&&s.min_rows>=2&&s.min_rows<=20&&s.strong_rows>=s.min_rows&&s.strong_rows<=30,"CONFIG_STRIPES");
    require(s.far_frames>=1&&s.far_frames<=15&&s.near_frames>=1&&s.near_frames<=15&&s.fallback_frames>=s.near_frames&&s.fallback_frames<=20,"CONFIG_CONFIRM");
    require(s.approach_speed>0&&s.approach_speed<=20&&s.stationary_rps>0&&s.stationary_rps<=2,"CONFIG_SPEED");
    require(s.reverse_pwm>=100&&s.reverse_pwm<=50000&&s.reverse_min_ms>=1&&s.reverse_min_ms<=s.reverse_max_ms&&s.reverse_max_ms<=2000,"CONFIG_REVERSE");
    require(s.coast_ms>=10&&s.coast_ms<=500&&s.stationary_ms>=100&&s.stationary_ms<=1500,"CONFIG_STATIONARY");
    require(s.hold_ms>=3000&&s.hold_ms<=30000&&s.settle_timeout_ms>=s.stationary_ms&&s.settle_timeout_ms<=15000,"CONFIG_HOLD");
    require(s.input_timeout_ms>=100&&s.input_timeout_ms<=1500&&s.speed_timeout_ms>=50&&s.speed_timeout_ms<=1000,"CONFIG_WATCHDOG");
    require(s.candidate_timeout_ms>=200&&s.candidate_timeout_ms<=10000&&s.rearm_min_ms>=500&&s.rearm_min_ms<=30000&&s.absent_ms>=100&&s.absent_ms<=5000,"CONFIG_REARM");
    require(s.resume_valid_frames>=2&&s.resume_valid_frames<=30,"CONFIG_RESUME");
    require(s.video_fps>=1&&s.video_fps<=30&&s.video_queue>=1&&s.video_queue<=16,"CONFIG_VIDEO");
    require(s.log_queue>=64&&s.log_queue<=16384&&s.control_log_ms>=20&&s.control_log_ms<=1000&&s.flush_ms>=20&&s.flush_ms<=1000,"CONFIG_LOG");
    require(s.minimum_free_mb>=32&&s.storage_check_ms>=200&&s.storage_check_ms<=10000,"CONFIG_STORAGE");
    require(s.illumination_band_rows==0||(s.illumination_band_rows>=3&&s.illumination_band_rows<=25&&s.illumination_band_rows%2==1),"CONFIG_ILLUMINATION_BAND");
    require(s.lane_loss_timeout_ms>=300&&s.lane_loss_timeout_ms<=10000,"CONFIG_LANE_TIMEOUT");
    require(s.startup_timeout_ms>=1000&&s.startup_timeout_ms<=30000,"CONFIG_STARTUP_TIMEOUT");
}
const char* phaseName(Phase p) {
    switch(p) {case Phase::Waiting:return "WAITING";case Phase::Cruise:return "CRUISE";
    case Phase::Approach:return "APPROACH";case Phase::Coast:return "COAST";case Phase::Brake:return "RIGHT_BRAKE";
    case Phase::LanePause:return "LANE_PAUSE";
    case Phase::Settle:return "SETTLE";case Phase::Hold:return "HOLD_3S";case Phase::Depart:return "DEPART";case Phase::Fault:return "FAULT";}
    return "UNKNOWN";
}
void Controller::enter(Phase p,long long now,const std::string& reason) {
    phase_=p;phase_at_=now;output_at_=still_at_=-1;output_ok_=false;reason_=reason;
    if(p==Phase::Depart)depart_moved_=false;
}
void Controller::fault(const std::string& reason) {
    if(phase_==Phase::Fault)return;
    phase_=Phase::Fault;reason_=reason;output_ok_=false;
}
void Controller::observe(const Observation& o) {
    if(obs_.time_ms>=0&&o.time_ms<=obs_.time_ms) {fault("IMAGE_TIMESTAMP_NOT_INCREASING");return;}
    const bool gap=obs_.time_ms>=0&&o.time_ms-obs_.time_ms>s_.input_timeout_ms;
    obs_=o;
    const bool valid=o.input_valid&&o.lane_valid;
    if(!valid||gap) {valid_count_=far_count_=near_count_=strong_count_=0;absent_at_=-1;return;}
    valid_count_=std::min(valid_count_+1,s_.resume_valid_frames);
    if(phase_==Phase::Waiting&&valid_count_>=s_.resume_valid_frames)enter(Phase::Cruise,o.time_ms,"VALID_LANE_READY");
    if(phase_==Phase::Cruise||phase_==Phase::Approach) {
        far_count_=o.far.hit?std::min(far_count_+1,s_.far_frames):0;
        if(o.far.hit||o.near.hit)last_candidate_=o.time_ms;
        if(phase_==Phase::Cruise&&far_count_>=s_.far_frames)enter(Phase::Approach,o.time_ms,"FAR_GROUP_CONFIRMED");
        const bool at_stop=o.near.hit&&o.near.nearest_y>=s_.trigger_y;
        near_count_=at_stop&&phase_==Phase::Approach?std::min(near_count_+1,s_.near_frames):0;
        strong_count_=at_stop&&o.near.strong?std::min(strong_count_+1,s_.fallback_frames):0;
        if(near_count_>=s_.near_frames||strong_count_>=s_.fallback_frames) {
            ++count_;enter(Phase::Coast,o.time_ms,"ZEBRA_CONFIRMED");
        } else if(phase_==Phase::Approach&&last_candidate_>=0&&o.time_ms-last_candidate_>=s_.candidate_timeout_ms) {
            far_count_=near_count_=strong_count_=0;enter(Phase::Cruise,o.time_ms,"CANDIDATE_EXPIRED");
        }
    } else if(phase_==Phase::Depart) {
        // Insufficient road coverage is UNKNOWN, never evidence of absence.
        if(!o.far.valid||!o.near.valid||!depart_moved_) {absent_at_=-1;return;}
        if(o.far.hit||o.near.hit)absent_at_=-1;
        else if(absent_at_<0)absent_at_=o.time_ms;
        if(absent_at_>=0&&o.time_ms-absent_at_>=s_.absent_ms&&o.time_ms-phase_at_>=s_.rearm_min_ms) {
            far_count_=near_count_=strong_count_=0;enter(Phase::Cruise,o.time_ms,"LEFT_ZEBRA_REARMED");
        }
    }
}
Command Controller::step(const MotorInput& in) {
    Command out;
    if(phase_==Phase::Fault)return out;
    if(in.external_stop) {fault("EXTERNAL_STOP");return out;}
    if(!in.encoder_valid||!std::isfinite(in.left_rps)||!std::isfinite(in.right_rps)) {fault("ENCODER_INVALID");return out;}
    const bool fresh=obs_.time_ms>=0&&in.time_ms>=obs_.time_ms&&in.time_ms-obs_.time_ms<=s_.input_timeout_ms;
    const bool valid=fresh&&obs_.input_valid&&obs_.lane_valid;
    if(!fresh&&phase_!=Phase::Waiting) {fault("CAMERA_STALE");return out;}
    if(phase_==Phase::Waiting) {
        if(phase_at_<0)phase_at_=in.time_ms;
        if(in.time_ms-phase_at_>=s_.startup_timeout_ms){fault("STARTUP_IMAGE_TIMEOUT");return out;}
    }
    if(!valid) {
        valid_count_=0;absent_at_=-1;still_at_=-1;
        if(phase_==Phase::Cruise||phase_==Phase::Approach||phase_==Phase::Depart) {
            resume_phase_=phase_==Phase::Approach?Phase::Cruise:phase_;
            enter(Phase::LanePause,in.time_ms,"LANE_OR_IMAGE_INVALID");
        } else if(phase_==Phase::LanePause&&in.time_ms-phase_at_>=s_.lane_loss_timeout_ms)fault("LANE_LOSS_TIMEOUT");
        else if(phase_==Phase::Brake||phase_==Phase::Coast||phase_==Phase::Hold)enter(Phase::Settle,in.time_ms,"STOP_INPUT_INVALID_RECONFIRM");
        return out;
    }
    if(phase_==Phase::LanePause) {
        if(valid_count_<s_.resume_valid_frames)return out;
        enter(resume_phase_,in.time_ms,"LANE_RECOVERED");
    }
    const bool still=std::abs(in.left_rps)<=s_.stationary_rps&&std::abs(in.right_rps)<=s_.stationary_rps;
    if(phase_==Phase::Depart&&!still)depart_moved_=true;
    if(phase_==Phase::Coast&&output_at_>=0&&in.time_ms-output_at_>=s_.coast_ms)enter(Phase::Brake,in.time_ms,"COAST_COMPLETE");
    if(phase_==Phase::Brake) {
        if(output_at_>=0) {
            const auto elapsed=in.time_ms-output_at_;
            if(elapsed>=s_.reverse_min_ms&&still)enter(Phase::Settle,in.time_ms,"BRAKE_SPEED_ZERO");
            else if(elapsed>=s_.reverse_max_ms)enter(Phase::Settle,in.time_ms,"BRAKE_MAX_REACHED");
        }
        if(phase_==Phase::Brake) {out.right_pwm=-s_.reverse_pwm;return out;}
    }
    if(phase_==Phase::Settle&&output_ok_) {
        if(still) {if(still_at_<0)still_at_=in.time_ms;}
        else still_at_=-1;
        if(still_at_>=0&&in.time_ms-still_at_>=s_.stationary_ms)enter(Phase::Hold,in.time_ms,"STATIONARY_CONFIRMED");
        else if(in.time_ms-phase_at_>=s_.settle_timeout_ms)fault("SETTLE_TIMEOUT");
    }
    if(phase_==Phase::Hold) {
        if(!still) {still_at_=-1;enter(Phase::Settle,in.time_ms,"MOVED_DURING_HOLD");}
        else if(in.time_ms-phase_at_>=s_.hold_ms&&valid_count_>=s_.resume_valid_frames) {
            enter(Phase::Depart,in.time_ms,"HOLD_COMPLETE_RESUME");absent_at_=-1;
        }
    }
    if(phase_==Phase::Cruise||phase_==Phase::Approach||phase_==Phase::Depart) {
        out.override_pwm=false;
        out.speed_limit=phase_==Phase::Approach;out.max_speed=s_.approach_speed;
    }
    return out;
}
void Controller::acknowledge(long long now,bool success) {
    if(!success){fault("MOTOR_WRITE_FAILED");return;}
    output_ok_=true;if(output_at_<0)output_at_=now;
}
long long Controller::holdRemaining(long long now) const {
    return phase_==Phase::Hold?std::max(0LL,static_cast<long long>(s_.hold_ms)-(now-phase_at_)):s_.hold_ms;
}
}
