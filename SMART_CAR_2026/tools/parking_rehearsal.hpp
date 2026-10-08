#pragma once
#include "capture_data.hpp"
#include "rehearsal_line.hpp"
#include <optional>
namespace car2026 { namespace capture {
constexpr const char* rehearsalVersion="2026-10-08.4";
constexpr const char* stageNames[]={"前移","第一倒弯","分支直退","第二倒弯","库内直退","停止确认"};
constexpr const char* stageFiles[]={"01_advance.csv","02_reverse_first.csv","03_reverse_branch.csv","04_reverse_second.csv","05_reverse_straight.csv","06_stop_confirmation.csv"};
struct BendCorrection {double command=0,holdSeconds=0;};
struct StageTuning {
    double command=0,settleSeconds=.5,holdSeconds=0;
    BendCorrection correction{};bool correctionOnly=false;
    double motorLeft=0,motorRight=0,motorSeconds=0;
    LineFollowTuning line{};
    bool powered() const {return motorSeconds>0 && (motorLeft>0 || motorRight>0);}
};
inline bool isBendStage(int stageIndex) {return stageIndex==1 || stageIndex==3;}
inline const char* rehearsalSegmentName(int stageIndex,unsigned segment) {
    if(stageIndex==1) return segment==1 ? "RIGHT_CORRECTION" : "FIRST_LEFT_BEND";
    if(stageIndex==3) return segment==1 ? "LEFT_CORRECTION" : "SECOND_RIGHT_BEND";
    return "PRIMARY";
}
inline const char* rehearsalSegmentTitle(int stageIndex,unsigned segment) {
    if(stageIndex==1) return segment==1 ? "2B右打修正" : "2A左打倒弯";
    if(stageIndex==3) return segment==1 ? "4B左打修正" : "4A右打倒弯";
    return "本段";
}
struct ParkingTuning {
    bool single=true;int stage=2;std::array<StageTuning,6> stages{};std::string source;
    // Zero means uncalibrated: report counts/s without inventing cm/s.
    double speedLeftCmPerCount=0,speedRightCmPerCount=0;
    void validate(double limit=15,double motorLimit=12000) const {
        if(stage<1 || stage>6) throw std::runtime_error("stage must be 1..6");
        for(double scale:{speedLeftCmPerCount,speedRightCmPerCount})
            if(!std::isfinite(scale) || scale<0 || scale>10)
                throw std::runtime_error("speed_*_cm_per_count must be 0..10 (0=uncalibrated)");
        for(const auto& item:stages)
            if(!std::isfinite(item.command) || std::abs(item.command)>std::min(15.,limit) ||
               !std::isfinite(item.settleSeconds) || item.settleSeconds<.5 || item.settleSeconds>5 ||
               !std::isfinite(item.holdSeconds) || item.holdSeconds<0 || item.holdSeconds>120)
                throw std::runtime_error("steer_command exceeds limit, settle_time_s outside 0.5..5, or hold_time_s outside 0..120");
        for(int index:{1,3}) {
            const auto& item=stages[size_t(index)];const auto& correction=item.correction;
            const double direction=index==1 ? 1. : -1.;
            const auto label="stage_"+std::to_string(index+1);
            if(!std::isfinite(correction.command) || correction.command*direction<0 || std::abs(correction.command)>std::min(15.,limit) ||
               !std::isfinite(correction.holdSeconds) || correction.holdSeconds<0 || correction.holdSeconds>120)
                throw std::runtime_error(label+" correction has wrong direction, exceeds limit, or hold time outside 0..120");
            if(correction.holdSeconds>0 && (correction.command*direction<=0 || (!item.correctionOnly && (item.command*direction>=0 || item.holdSeconds<=0))))
                throw std::runtime_error(label+" enabled correction requires opposite first command and positive hold times");
            if(item.correctionOnly && (!single || stage!=index+1 || correction.holdSeconds<=0))
                throw std::runtime_error(label+" correction-only requires single mode, matching stage and enabled correction");
        }
        for(int index:{0,2,4,5}) {
            const auto& item=stages[size_t(index)];
            if(item.correction.command!=0 || item.correction.holdSeconds!=0 || item.correctionOnly)
                throw std::runtime_error("Correction parameters are only supported in stages 2 and 4");
        }
        for(size_t i=0;i<stages.size();++i) {
            const auto& item=stages[i];
            item.line.validate(item.line.enabled ? limit : 15);
            if(item.line.enabled && (i!=0 || !single || item.command!=0 || item.holdSeconds!=0 || item.motorLeft<=0 || item.motorRight<=0))
                throw std::runtime_error("Line following requires single stage_1, steer_command=0, hold_time_s=0 and positive motor amplitudes (run time 0 may disable motors)");
            for(double command:{item.motorLeft,item.motorRight})
                if(!std::isfinite(command) || command<0 || command>std::min(12000.,motorLimit))
                    throw std::runtime_error("motor commands must be 0..min(12000,pwm_limit)");
            if(!std::isfinite(item.motorSeconds) || item.motorSeconds<0 || item.motorSeconds>120)
                throw std::runtime_error("motor_run_time_s must be 0..120 (0=disabled)");
            if(i==5 && (item.motorLeft!=0 || item.motorRight!=0 || item.motorSeconds!=0))
                throw std::runtime_error("stage_6 motor settings must all be zero");
            if(item.powered() && !single) throw std::runtime_error("Powered tuning requires mode=single");
        }
    }
    static ParkingTuning parse(const std::string& text) {
        if(text.size()>16384) throw std::runtime_error("Tuning file too large");
        ParkingTuning result;result.source=text;
        std::istringstream input(text);std::string line,section;std::set<std::string> sections,keys;unsigned lineNumber=0;
        while(std::getline(input,line)) {
            if(++lineNumber==1 && line.compare(0,3,"\xef\xbb\xbf")==0) line.erase(0,3);
            line=trimmed(line.substr(0,line.find('#')));if(line.empty()) continue;
            if(line.front()=='[' && line.back()==']') {
                section=line.substr(1,line.size()-2);
                if(section!="session" && (section.size()!=7 || section.substr(0,6)!="stage_" || section[6]<'1' || section[6]>'6'))
                    throw std::runtime_error("Unknown tuning section: "+section);
                if(!sections.insert(section).second) throw std::runtime_error("Duplicate tuning section: "+section);
                continue;
            }
            const auto equal=line.find('=');
            if(equal==std::string::npos || section.empty()) throw std::runtime_error("Expected [section] and key=value at line "+std::to_string(lineNumber));
            const auto key=trimmed(line.substr(0,equal)),value=trimmed(line.substr(equal+1));
            if(!keys.insert(section+"."+key).second) throw std::runtime_error("Duplicate tuning key: "+key);
            if(section=="session") {
                if(key=="mode") {
                    if(value!="single" && value!="full") throw std::runtime_error("mode must be single or full");
                    result.single=value=="single";
                } else if(key=="stage") {
                    const double number=finiteNumber(value);
                    if(number<1 || number>6 || std::floor(number)!=number) throw std::runtime_error("stage must be 1..6");
                    result.stage=int(number);
                } else if(key=="stage_2_step" || key=="stage_4_step") {
                    const int index=key=="stage_2_step" ? 1 : 3;
                    const auto primary=index==1 ? "first_bend" : "second_bend";
                    const auto correction=index==1 ? "right_correction" : "left_correction";
                    if(value!=primary && value!=correction) throw std::runtime_error(key+" must be "+primary+" or "+correction);
                    result.stages[size_t(index)].correctionOnly=value==correction;
                } else if(key=="speed_left_cm_per_count") result.speedLeftCmPerCount=finiteNumber(value);
                else if(key=="speed_right_cm_per_count") result.speedRightCmPerCount=finiteNumber(value);
                else throw std::runtime_error("Unknown session key: "+key);
            } else {
                const int index=section[6]-'1';auto& item=result.stages[size_t(index)];
                if(key=="steer_command") item.command=finiteNumber(value);
                else if(key=="settle_time_s") item.settleSeconds=finiteNumber(value);
                else if(key=="hold_time_s") item.holdSeconds=finiteNumber(value);
                else if(key=="motor_left_command") item.motorLeft=finiteNumber(value);
                else if(key=="motor_right_command") item.motorRight=finiteNumber(value);
                else if(key=="motor_run_time_s") item.motorSeconds=finiteNumber(value);
                else if(index==0 && key=="line_follow_enable") {
                    if(value!="0" && value!="1") throw std::runtime_error("line_follow_enable must be 0 or 1");
                    item.line.enabled=value=="1";
                }
                else if(index==0 && key=="line_kp") item.line.kp=finiteNumber(value);
                else if(index==0 && key=="line_kd") item.line.kd=finiteNumber(value);
                else if(index==0 && key=="line_preview") item.line.preview=finiteNumber(value);
                else if(index==0 && key=="line_filter_s") item.line.filterSeconds=finiteNumber(value);
                else if(index==0 && key=="line_deadband_px") item.line.deadbandPx=finiteNumber(value);
                else if(index==0 && key=="line_max_command") item.line.maxCommand=finiteNumber(value);
                else if(index==0 && key=="line_slew_per_s") item.line.slewPerSecond=finiteNumber(value);
                else if(index==0 && key=="line_target_x") item.line.targetX=finiteNumber(value);
                else if(index==0 && key=="line_crop_top") item.line.cropTop=finiteNumber(value);
                else if(index==0 && key=="line_crop_bottom") item.line.cropBottom=finiteNumber(value);
                else if(index==0 && key=="line_lost_s") item.line.lostSeconds=finiteNumber(value);
                else if(index==0 && key=="line_acquire_s") item.line.acquireSeconds=finiteNumber(value);
                else if(index==0 && (key=="line_min_contrast" || key=="line_max_width_px")) {
                    const double number=finiteNumber(value);
                    if(number<0 || number>255 || std::floor(number)!=number) throw std::runtime_error("Line contrast/width must be integers");
                    if(key=="line_min_contrast") item.line.contrast=int(number);else item.line.maxWidth=int(number);
                }
                else if(isBendStage(index) && key=="correction_steer_command") item.correction.command=finiteNumber(value);
                else if(isBendStage(index) && key=="correction_hold_time_s") item.correction.holdSeconds=finiteNumber(value);
                else throw std::runtime_error("Unknown stage key: "+key);
            }
        }
        if(!keys.count("session.mode") || !keys.count("session.stage")) throw std::runtime_error("All session keys are required");
        for(int stage=1;stage<=6;++stage)
            for(const auto* key:{"steer_command","settle_time_s","hold_time_s"})
                if(!keys.count("stage_"+std::to_string(stage)+"."+key))
                    throw std::runtime_error("All six stage keys are required");
        for(const auto* sectionName:{"stage_2","stage_4"})
            if(keys.count(std::string(sectionName)+".correction_steer_command")!=keys.count(std::string(sectionName)+".correction_hold_time_s"))
                throw std::runtime_error(std::string(sectionName)+" correction keys are required together");
        for(int stage=1;stage<=6;++stage) {
            const auto prefix="stage_"+std::to_string(stage)+".";
            const auto count=keys.count(prefix+"motor_left_command")+keys.count(prefix+"motor_right_command")+keys.count(prefix+"motor_run_time_s");
            if(count!=0 && count!=3) throw std::runtime_error(prefix+"motor keys are required together");
        }
        result.validate();return result;
    }
    static std::string readSource(const std::string& path) {
        std::ifstream file(path,std::ios::binary);if(!file) throw std::runtime_error("Cannot open tuning file: "+path);
        std::string text;char chunk[1024];
        while(file.read(chunk,sizeof(chunk)) || file.gcount()) {
            text.append(chunk,size_t(file.gcount()));
            if(text.size()>16384) throw std::runtime_error("Tuning file too large");
        }
        if(file.bad()) throw std::runtime_error("Cannot read tuning file");
        return text;
    }
    static ParkingTuning load(const std::string& path) {return parse(readSource(path));}
};
inline void writeTuningSummary(std::ostream& out,const ParkingTuning& tuning,const std::string& path) {
    out<<"VERSION "<<rehearsalVersion<<" | TUNING_FILE "<<path<<'\n'
        <<"选择 mode="<<(tuning.single ? "single" : "full")<<" stage="<<tuning.stage<<'\n';
    out<<"阶段1巡线="<<(tuning.stages[0].line.enabled ? "启用（电机时间为0时仅观察）" : "关闭")<<"；不含交点分支选择。\n";
    for(size_t i=0;i<tuning.stages.size();++i) {
        const auto& item=tuning.stages[i];
        out<<"阶段"<<i+1<<" 电机左="<<item.motorLeft<<" 右="<<item.motorRight<<" 时间="<<item.motorSeconds<<"s"
            <<(item.powered() ? " 启用" : " 禁用")<<'\n';
    }
    for(int index:{1,3}) {
        const auto& item=tuning.stages[size_t(index)];const auto& correction=item.correction;
        out<<"阶段"<<index+1<<' '<<stageNames[index]<<" | 主转弯="<<item.command<<"/"<<item.holdSeconds<<"s"
            <<" | "<<(index==1 ? "右" : "左")<<"修正="<<(correction.holdSeconds>0 ? "启用" : "关闭")
            <<" "<<correction.command<<"/"<<correction.holdSeconds<<"s"
            <<" | 入口="<<(item.correctionOnly ? "只测修正" : "完整弯道")<<'\n';
        if(correction.holdSeconds==0)
            out<<"提示：该弯道不会自动反向修正；旧文件缺少correction字段时默认关闭，请在对应区段补入参数。\n";
    }
}
enum class RehearsalKey {None,Enter,Pause,Continue,Reload,Quit,Invalid};
inline const char* rehearsalKeyName(RehearsalKey key) {
    switch(key) {
        case RehearsalKey::None:return "NONE";
        case RehearsalKey::Enter:return "ENTER";
        case RehearsalKey::Pause:return "PAUSE_RECORDING";
        case RehearsalKey::Continue:return "RESUME_RECORDING";
        case RehearsalKey::Reload:return "PREVIEW_CONFIG";
        case RehearsalKey::Quit:return "QUIT";
        case RehearsalKey::Invalid:return "INVALID";
    }
    return "INVALID";
}
// Queued lines never become multiple permissions. Quit and pause take priority.
inline RehearsalKey parseRehearsalInput(const std::vector<std::string>& lines) {
    bool pause=false;
    for(const auto& raw:lines) {
        const auto line=trimmed(raw);
        if(line=="q" || line=="Q") return RehearsalKey::Quit;
        if(line=="p" || line=="P") pause=true;
    }
    if(pause) return RehearsalKey::Pause;
    if(lines.empty()) return RehearsalKey::None;
    if(lines.size()!=1) return RehearsalKey::Invalid;
    const auto line=trimmed(lines.front());
    if(line.empty()) return RehearsalKey::Enter;
    if(line=="c" || line=="C") return RehearsalKey::Continue;
    if(line=="r" || line=="R") return RehearsalKey::Reload;
    return RehearsalKey::Invalid;
}
class ParkingRehearsal {
public:
    enum class State {AwaitApply,WritePending,Settling,Running};
    explicit ParkingRehearsal(ParkingTuning tuning):tuning_(std::move(tuning)),stage_(tuning_.stage-1),nextStage_(stage_) {
        tuning_.validate();segment_=currentTuning().correctionOnly ? 1u : 0u;
    }
    std::optional<double> update(double now,RehearsalKey key) {
        if(finished()) return {};
        if(!std::isfinite(now) || now<lastTime_) {stop("INVALID_TIME");return {};}
        lastTime_=now;
        if(key==RehearsalKey::Quit) {stop("USER_QUIT");return {};}
        if(key==RehearsalKey::Pause) {
            paused_=true;
            if(currentTuning().powered()) {
                motorRestartRequired_=true;state_=State::AwaitApply;nextStage_=stage_;
                pendingCorrection_=automaticCorrectionWrite_=false;
            }
            return {};
        }
        if(pendingCorrection_) {
            if(key==RehearsalKey::Continue) paused_=false;
            pendingCorrection_=false;automaticCorrectionWrite_=true;state_=State::WritePending;return command();
        }
        if(key==RehearsalKey::Continue) paused_=false;
        if(state_==State::Settling) {
            if(now-writeTime_>=tuning_.stages[size_t(stage_)].settleSeconds) state_=State::Running;
            return {}; // One Enter already authorized this trial; settling never needs another.
        }
        if(paused_) return {};
        if(key!=RehearsalKey::Enter) return {};
        if(state_==State::AwaitApply) {
            motorRestartRequired_=false;
            stage_=nextStage_;segment_=currentTuning().correctionOnly ? 1u : 0u;
            correctionComplete_=false;automaticSavedTrial_=automaticCorrectionWrite_=false;
            recordingStarted_=true;++trials_[size_t(stage_)];
            state_=State::WritePending;return command();
        }
        if(state_==State::Running) {
            if(correctionSequenceActive()) return {}; // Timed pair is one authorized maneuver; Q can abort.
            state_=State::AwaitApply;
            nextStage_=tuning_.single ? stage_ : std::min(stage_+1,5);
        }
        return {};
    }
    void acknowledge(bool success,double now) {
        if(state_!=State::WritePending || finished()) throw std::runtime_error("No servo write pending");
        if(!success || !std::isfinite(now) || now<lastTime_) {stop("SERVO_WRITE_FAILED");return;}
        state_=motorRestartRequired_ ? State::AwaitApply :
            (automaticCorrectionWrite_ || (automaticSavedTrial_ && !currentTuning().powered())) ? State::Running : State::Settling;
        writeTime_=lastTime_=now;
    }
    // A stable, valid file save is the user's execution authorization.
    std::optional<double> applySaved(ParkingTuning tuning,double now) {
        if(finished()) return {};
        if(!std::isfinite(now) || now<lastTime_) {stop("INVALID_TIME");return {};}
        tuning.validate();lastTime_=now;
        const bool mainSelectorChanged=tuning.stage!=tuning_.stage || tuning.single!=tuning_.single;
        const int target=mainSelectorChanged ? tuning.stage-1 : stage_;
        const auto& before=tuning_.stages[size_t(target)];const auto& after=tuning.stages[size_t(target)];
        const bool selectorChanged=mainSelectorChanged || before.correctionOnly!=after.correctionOnly;
        const bool primaryChanged=before.command!=after.command || before.holdSeconds!=after.holdSeconds || before.settleSeconds!=after.settleSeconds ||
            before.motorLeft!=after.motorLeft || before.motorRight!=after.motorRight || before.motorSeconds!=after.motorSeconds || !(before.line==after.line);
        const bool correctionChanged=before.correction.command!=after.correction.command || before.correction.holdSeconds!=after.correction.holdSeconds;
        const bool execute=selectorChanged || target!=stage_ || primaryChanged || correctionChanged;
        const bool keepCorrection=target==stage_ && isBendStage(target) && segment_==1 && !selectorChanged && !primaryChanged;
        tuning_=std::move(tuning);++revision_;
        if(paused_ && tuning_.stages[size_t(target)].powered()) motorRestartRequired_=true;
        if(!execute) return {};
        if(keepCorrection && currentTuning().correction.holdSeconds==0) {
            pendingCorrection_=false;stop(stage_==1 ? "RIGHT_CORRECTION_DISABLED" : "LEFT_CORRECTION_DISABLED");return {};
        }
        stage_=nextStage_=target;++trials_[size_t(stage_)];automaticSavedTrial_=true;
        recordingStarted_=true;
        automaticCorrectionWrite_=pendingCorrection_=correctionComplete_=false;
        segment_=(currentTuning().correctionOnly || keepCorrection) ? 1u : 0u;
        state_=State::WritePending;return command();
    }
    void holdCompleted(int stage,unsigned trial,unsigned segment=0) {
        if(finished() || motorRestartRequired_ || state_==State::AwaitApply || stage!=stage_ || trial!=trials_[size_t(stage_)] || segment!=segment_) return;
        if(isBendStage(stage_) && currentTuning().correction.holdSeconds>0 && segment_==0) {
            segment_=1;pendingCorrection_=true;state_=State::WritePending;return;
        }
        if(isBendStage(stage_) && segment_==1) correctionComplete_=true;
        if(automaticSavedTrial_ || tuning_.single) stop("TIMED_TRIAL_COMPLETED");
    }
    bool correctionSequenceActive() const {return isBendStage(stage_) && currentTuning().correction.holdSeconds>0 && !correctionComplete_ && (state_==State::Running || state_==State::WritePending);}
    bool pendingCorrection() const {return pendingCorrection_;}
    bool automaticCorrectionWrite() const {return automaticCorrectionWrite_;}
    unsigned segment() const {return segment_;}
    const char* segmentName() const {return rehearsalSegmentName(stage_,segment_);}
    const char* segmentTitle() const {return rehearsalSegmentTitle(stage_,segment_);}
    const StageTuning& currentTuning() const {return tuning_.stages[size_t(stage_)];}
    double holdSeconds() const {return segment_==1 ? currentTuning().correction.holdSeconds : currentTuning().holdSeconds;}
    bool automaticSavedTrial() const {return automaticSavedTrial_;}
    void stop(const std::string& reason) {if(!finished()) outcome_=reason;}
    bool finished() const {return !outcome_.empty();}
    bool paused() const {return paused_;}
    bool motorRestartRequired() const {return motorRestartRequired_;}
    bool recordingEnabled() const {return recordingStarted_ && !paused_ && !finished();}
    int stage() const {return stage_;}
    int nextStage() const {return nextStage_;}
    unsigned trial() const {return trials_[size_t(stage_)];}
    unsigned revision() const {return revision_;}
    double command() const {return segment_==1 ? currentTuning().correction.command : currentTuning().command;}
    const ParkingTuning& tuning() const {return tuning_;}
    const std::string& outcome() const {return outcome_;}
    State state() const {return state_;}
    const char* stateName() const {
        if(finished()) return "FINISHED";
        if(paused_) return "PAUSED";
        switch(state_) {
            case State::AwaitApply:return "WAIT_SERVO_PERMISSION";
            case State::WritePending:return "SERVO_WRITE_PENDING";
            case State::Settling:return "WAIT_SERVO_SETTLE";
            case State::Running:return "MANUAL_TRIAL";
        }
        return "INVALID";
    }
private:
    ParkingTuning tuning_;bool automaticSavedTrial_=false,automaticCorrectionWrite_=false,pendingCorrection_=false,correctionComplete_=false;
    unsigned segment_=0;
    int stage_,nextStage_;std::array<unsigned,6> trials_{};unsigned revision_=1;
    State state_=State::AwaitApply;double lastTime_=-1,writeTime_=0;
    bool paused_=false,recordingStarted_=false,motorRestartRequired_=false;std::string outcome_;
};
class SavedTuningWatcher {
public:
    SavedTuningWatcher(const ParkingTuning& initial,double limit,double motorLimit=12000):accepted_(initial),observed_(initial.source),handled_(initial.source),limit_(limit),motorLimit_(motorLimit) {}
    std::optional<ParkingTuning> observe(const std::string& source,double now) {
        if(!std::isfinite(now) || now<lastTime_) throw std::runtime_error("Invalid watcher time");
        lastTime_=now;
        if(source!=observed_) {observed_=source;stableSince_=now;return {};}
        if(source==handled_ || now-stableSince_<.3) return {};
        handled_=source; // A rejected save is reported once; the next save can recover.
        auto candidate=ParkingTuning::parse(source);candidate.validate(limit_,motorLimit_);
        bool same=candidate.single==accepted_.single && candidate.stage==accepted_.stage &&
            candidate.speedLeftCmPerCount==accepted_.speedLeftCmPerCount && candidate.speedRightCmPerCount==accepted_.speedRightCmPerCount;
        for(size_t i=0;i<6;++i) same=same && candidate.stages[i].command==accepted_.stages[i].command &&
            candidate.stages[i].holdSeconds==accepted_.stages[i].holdSeconds && candidate.stages[i].settleSeconds==accepted_.stages[i].settleSeconds &&
            candidate.stages[i].correctionOnly==accepted_.stages[i].correctionOnly &&
            candidate.stages[i].correction.command==accepted_.stages[i].correction.command &&
            candidate.stages[i].correction.holdSeconds==accepted_.stages[i].correction.holdSeconds &&
            candidate.stages[i].motorLeft==accepted_.stages[i].motorLeft && candidate.stages[i].motorRight==accepted_.stages[i].motorRight &&
            candidate.stages[i].motorSeconds==accepted_.stages[i].motorSeconds && candidate.stages[i].line==accepted_.stages[i].line;
        accepted_=candidate;
        return same ? std::optional<ParkingTuning>{} : candidate;
    }
private:
    ParkingTuning accepted_;std::string observed_,handled_;double limit_,motorLimit_,stableSince_=0,lastTime_=-1;
};
}}
