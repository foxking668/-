#pragma once

// Hardware-free, monotonic-clock parking sequence. The motor thread owns tick/ack.
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <optional>
#include <ostream>
#include <stdexcept>
#include <string>
#include <utility>

namespace AutoParking {
inline constexpr const char* version = "VV15_AUTO_PARKING_20261017.2";
inline constexpr std::array<const char*,6> names = {
    "前移巡线", "第一倒弯", "分支直退", "第二倒弯", "库内直退", "停止确认"};
inline constexpr std::array<const char*,6> files = {
    "01_advance.csv", "02_first_bend.csv", "03_branch.csv",
    "04_second_bend.csv", "05_straight_reverse.csv", "06_stopped.csv"};

struct Stage {
    double steer=0, settle=.5, hold=0, seconds=0, correction=0, correctionSeconds=0;
    int pwm=0;
};
struct Config {
    bool enabled=true, rightOnly=true, manual=false; // Missing start_mode preserves legacy race entry.
    int zebraCount=2;
    double lineLost=.35, lineAcquire=3;
    std::array<Stage,6> stages{};
    static std::string trim(const std::string& s) {
        const auto a=s.find_first_not_of(" \t\r\n");
        return a==std::string::npos ? "" : s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);
    }
    static double number(const std::string& s) {
        std::size_t n=0; const double v=std::stod(s,&n);
        if(n!=s.size() || !std::isfinite(v)) throw std::runtime_error("Invalid finite number: "+s);
        return v;
    }
    static int integer(double v,int lo,int hi) {
        if(v<lo || v>hi || std::floor(v)!=v) throw std::runtime_error("Invalid integer parameter");
        return static_cast<int>(v);
    }
    void validate() const {
        if(manual && !enabled) throw std::runtime_error("start_mode=manual requires enabled=1");
        if(zebraCount<1 || zebraCount>20 || !std::isfinite(lineLost) || lineLost<.05 || lineLost>.5 ||
           !std::isfinite(lineAcquire) || lineAcquire<.5 || lineAcquire>10)
            throw std::runtime_error("Invalid session trigger/line timeout");
        for(int i=0;i<6;++i) {
            const auto& s=stages[i];
            for(double t:{s.settle,s.hold,s.seconds,s.correctionSeconds})
                if(!std::isfinite(t) || t<0 || t>120) throw std::runtime_error("Invalid stage time");
            if(s.settle<.5 || s.settle>5 || !std::isfinite(s.steer) || std::abs(s.steer)>90 ||
               !std::isfinite(s.correction) || std::abs(s.correction)>90 || s.pwm<0 || s.pwm>50000 ||
               (s.pwm>0 && s.pwm<100)) throw std::runtime_error("Invalid steering/PWM/settle in stage_"+std::to_string(i+1));
            if(i<5 && s.seconds>0 && s.pwm==0) throw std::runtime_error("Enabled stage requires motor_pwm_ns>0");
            if(s.correctionSeconds>0) {
                const int sign=i==1?1:-1;
                if((i!=1 && i!=3) || s.hold<=0 || s.steer*sign>=0 || s.correction*sign<=0)
                    throw std::runtime_error("Correction must oppose the primary turn in stage_2/stage_4");
            }
            if(i<5 && s.seconds>0 && s.hold+s.correctionSeconds>s.seconds+1e-9)
                throw std::runtime_error("stage_"+std::to_string(i+1)+" motor_run_time_s must cover hold_time_s + correction_hold_time_s");
            if(i!=1 && i!=3 && (s.correction!=0 || s.correctionSeconds!=0))
                throw std::runtime_error("Only stages 2 and 4 support corrections");
        }
        const auto& a=stages[0]; const auto& end=stages[5];
        if(enabled && (a.seconds<=0 || stages[1].seconds<=0))
            throw std::runtime_error("Automatic parking requires positive stage_1 and stage_2 motor times");
        if(a.steer!=0 || a.hold!=0 || end.steer!=0 || end.seconds!=0 || end.pwm!=0)
            throw std::runtime_error("Stage 1 uses live lane steering; stage 6 must be stopped and centered");
        if(stages[1].seconds>0 && stages[1].steer>=0)
            throw std::runtime_error("stage_2 primary command must be negative (VV left convention)");
        if(stages[3].seconds>0 && stages[3].steer<=0)
            throw std::runtime_error("stage_4 primary command must be positive (VV right convention)");
    }
    static Config read(const std::string& path) {
        std::ifstream in(path); if(!in) throw std::runtime_error("Cannot open parking config: "+path);
        Config c; std::string section,line; std::map<std::string,bool> seen;
        while(std::getline(in,line)) {
            if(line.size()>=3 && static_cast<unsigned char>(line[0])==0xef &&
               static_cast<unsigned char>(line[1])==0xbb && static_cast<unsigned char>(line[2])==0xbf) line.erase(0,3);
            line=trim(line.substr(0,line.find_first_of("#;"))); if(line.empty()) continue;
            if(line.front()=='[' && line.back()==']') {
                section=line.substr(1,line.size()-2);
                if(section!="session" && (section.size()!=7 || section.substr(0,6)!="stage_" || section[6]<'1' || section[6]>'6'))
                    throw std::runtime_error("Unknown section: "+section);
                if(!seen.emplace("["+section+"]",true).second) throw std::runtime_error("Duplicate section: "+section);
                continue;
            }
            const auto eq=line.find('=');
            if(section.empty() || eq==std::string::npos) throw std::runtime_error("Expected [section] and key=value");
            const auto key=trim(line.substr(0,eq)); const auto value=trim(line.substr(eq+1));
            if(!seen.emplace(section+"."+key,true).second) throw std::runtime_error("Duplicate key: "+key);
            if(section=="session" && key=="start_mode") {
                if(value!="manual" && value!="race") throw std::runtime_error("start_mode must be manual or race");
                c.manual=value=="manual";continue;
            }
            const double v=number(value);
            if(section=="session") {
                if(key=="enabled") c.enabled=integer(v,0,1)!=0;
                else if(key=="trigger_zebra_count") c.zebraCount=integer(v,1,20);
                else if(key=="reverse_right_only") c.rightOnly=integer(v,0,1)!=0;
                else if(key=="line_lost_s") c.lineLost=v;
                else if(key=="line_acquire_s") c.lineAcquire=v;
                else throw std::runtime_error("Unknown session key: "+key);
            } else {
                auto& s=c.stages[static_cast<std::size_t>(section[6]-'1')];
                if(key=="steer_command") s.steer=v;
                else if(key=="settle_time_s") s.settle=v;
                else if(key=="hold_time_s") s.hold=v;
                else if(key=="motor_run_time_s") s.seconds=v;
                else if(key=="motor_pwm_ns") s.pwm=integer(v,0,50000);
                else if(key=="correction_steer_command") s.correction=v;
                else if(key=="correction_hold_time_s") s.correctionSeconds=v;
                else throw std::runtime_error("Unknown stage key: "+key);
            }
        }
        for(const auto* k:{"enabled","trigger_zebra_count","reverse_right_only"})
            if(!seen.count(std::string("session.")+k)) throw std::runtime_error(std::string("Missing session.")+k);
        for(int i=1;i<=6;++i) for(const auto* k:{"steer_command","settle_time_s","hold_time_s","motor_run_time_s","motor_pwm_ns"})
            if(!seen.count("stage_"+std::to_string(i)+"."+k)) throw std::runtime_error("Missing stage_"+std::to_string(i)+"."+k);
        c.validate(); return c;
    }
    void write(std::ostream& out) const {
        out.precision(17);
        out<<"[session]\nstart_mode="<<(manual?"manual":"race")<<"\nenabled="<<enabled<<"\ntrigger_zebra_count="<<zebraCount
           <<"\nreverse_right_only="<<rightOnly<<"\nline_lost_s="<<lineLost<<"\nline_acquire_s="<<lineAcquire<<'\n';
        for(int i=0;i<6;++i) {
            const auto& s=stages[i];
            out<<"\n[stage_"<<i+1<<"]\nsteer_command="<<s.steer<<"\nsettle_time_s="<<s.settle
               <<"\nhold_time_s="<<s.hold<<"\nmotor_pwm_ns="<<s.pwm<<"\nmotor_run_time_s="<<s.seconds;
            if(i==1 || i==3) out<<"\ncorrection_steer_command="<<s.correction<<"\ncorrection_hold_time_s="<<s.correctionSeconds;
            out<<'\n';
        }
    }
};

enum class Phase {Cruise, AwaitManualStart, AcquireManualLane, WaitResume, Advance, Preparing, Reverse, StopConfirmation, Done, Fault};
inline const char* phaseName(Phase p) {
    switch(p) {
    case Phase::Cruise:return "正常巡航";case Phase::WaitResume:return "等待斑马线停车结束";
    case Phase::AwaitManualStart:return "等待回车授权";case Phase::AcquireManualLane:return "手动入口等待识线";
    case Phase::Advance:return "阶段1前移巡线";case Phase::Preparing:return "停车设置倒车方向";
    case Phase::Reverse:return "倒车执行";case Phase::StopConfirmation:return "阶段6停止确认";
    case Phase::Done:return "入库完成";default:return "故障停车";
    }
}
struct Lane {bool valid=false; double at=0,error=0;};
struct Output {int left=0,right=0;double steer=0;};
class Controller {
public:
    explicit Controller(Config c):c_(std::move(c)) {c_.validate();if(c_.manual) phase_=Phase::AwaitManualStart;}
    const Config& config() const {return c_;}
    Phase phase() const {return phase_;}
    int stage() const {return stage_;} // public 1..6; zero means cruise/wait
    bool active() const {return phase_!=Phase::Cruise;}
    bool finished() const {return phase_==Phase::Done || phase_==Phase::Fault;}
    bool cameraNeeded() const {return phase_==Phase::Cruise || phase_==Phase::AwaitManualStart ||
        phase_==Phase::AcquireManualLane || phase_==Phase::WaitResume || phase_==Phase::Advance;}
    bool authorizeManualStart(double now) {
        if(phase_!=Phase::AwaitManualStart || !time(now)) return false;
        phase_=Phase::AcquireManualLane;resumeReady_=now;return true;
    }
    const std::string& reason() const {return reason_;}
    double elapsed(double now) const {return started_?std::max(0.0,now-*started_):0;}
    bool motorStarted() const {return started_.has_value();}
    bool onZebra(int count,double now) {
        if(!c_.enabled || active() || count!=c_.zebraCount) return false;
        if(!time(now)) return false;
        phase_=Phase::WaitResume;return true;
    }
    void fault(const std::string& why) {reason_=why;phase_=Phase::Fault;}
    Output tick(double now,bool zebraHeld,const Lane& lane) {
        Output out;
        if(!time(now) || !active() || finished()) return out;
        if(phase_==Phase::AwaitManualStart) return out;
        if(phase_==Phase::WaitResume || phase_==Phase::AcquireManualLane) {
            if(phase_==Phase::WaitResume && zebraHeld) {resumeReady_.reset();return out;}
            if(!resumeReady_) resumeReady_=now;
            if(now-*resumeReady_>=c_.lineAcquire) {fault("LINE_ACQUIRE_TIMEOUT");return out;}
            if(!fresh(lane,now)) return out;
            phase_=Phase::Advance;stage_=1;started_.reset();
        }
        if(phase_==Phase::Advance) {
            // Absolute motor-thread deadline comes before lane validity checks.
            if(started_ && now-*started_>=c_.stages[0].seconds) {prepareNext(2);return out;}
            if(!fresh(lane,now)) {fault("LINE_LOST_OR_CAMERA_STALE");return out;}
            out={c_.stages[0].pwm,c_.stages[0].pwm,lane.error};return out;
        }
        if(phase_==Phase::Preparing) {
            out.steer=c_.stages[stage_-1].steer;
            if(!prepared_ || now-*prepared_<c_.stages[stage_-1].settle) return out;
            phase_=Phase::Reverse;started_.reset();
        }
        if(phase_==Phase::Reverse) {
            const auto& s=c_.stages[stage_-1];const double t=elapsed(now);
            if(started_ && t>=s.seconds) {prepareNext(stage_+1);return {};}
            out.steer=s.steer;
            if(s.hold>0 && t>=s.hold) {
                out.steer=s.correctionSeconds>0 && t<s.hold+s.correctionSeconds?s.correction:0;
            }
            out.right=-s.pwm;out.left=c_.rightOnly?0:-s.pwm;return out;
        }
        if(phase_==Phase::StopConfirmation) {
            if(prepared_ && now-*prepared_>=c_.stages[5].hold) phase_=Phase::Done;
        }
        return out;
    }
    // Called only after both motor writes AND the associated steering write succeed.
    void acknowledge(const Output& out,bool success,double now) {
        if(!time(now)) return;
        if(!success) {fault("ACTUATOR_WRITE_FAILED");return;}
        if(phase_==Phase::Preparing && !prepared_ && out.left==0 && out.right==0 &&
           out.steer==c_.stages[stage_-1].steer) prepared_=now;
        if(phase_==Phase::StopConfirmation && !prepared_) prepared_=now;
        if((phase_==Phase::Advance || phase_==Phase::Reverse) && (out.left!=0 || out.right!=0) && !started_)
            started_=now;
    }
private:
    bool fresh(const Lane& l,double now) const {
        return l.valid && std::isfinite(l.at) && std::isfinite(l.error) && l.at<=now && now-l.at<c_.lineLost;
    }
    bool time(double now) {
        if(!std::isfinite(now) || now<lastTime_) {fault("INVALID_MONOTONIC_TIME");return false;}
        lastTime_=now;return true;
    }
    void prepareNext(int first) {
        started_.reset();prepared_.reset();
        while(first<=5 && c_.stages[first-1].seconds==0) ++first;
        stage_=first;phase_=first==6?Phase::StopConfirmation:Phase::Preparing;
    }
    Config c_;Phase phase_=Phase::Cruise;int stage_=0;double lastTime_=0;
    std::optional<double> started_,prepared_,resumeReady_;std::string reason_;
};
}
