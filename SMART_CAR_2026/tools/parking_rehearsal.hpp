#pragma once
#include "capture_data.hpp"
#include <optional>
namespace car2026 { namespace capture {
constexpr const char* rehearsalVersion="2026-10-07.5";
constexpr const char* stageNames[]={"前移","第一倒弯","分支直退","第二倒弯","库内直退","停止确认"};
constexpr const char* stageFiles[]={"01_advance.csv","02_reverse_first.csv","03_reverse_branch.csv","04_reverse_second.csv","05_reverse_straight.csv","06_stop_confirmation.csv"};
struct StageTuning {double command=0,settleSeconds=.5,holdSeconds=0;};
struct FirstBendCorrection {double command=0,holdSeconds=0;};
inline const char* rehearsalSegmentName(int stage,unsigned segment) {
    return stage==1 ? (segment==1 ? "RIGHT_CORRECTION" : "FIRST_LEFT_BEND") : "PRIMARY";
}
struct ParkingTuning {
    bool single=true;int stage=2;std::array<StageTuning,6> stages{};std::string source;
    FirstBendCorrection correction;bool startRightCorrection=false;
    void validate(double limit=15) const {
        if(stage<1 || stage>6) throw std::runtime_error("stage must be 1..6");
        for(const auto& item:stages)
            if(!std::isfinite(item.command) || std::abs(item.command)>std::min(15.,limit) ||
               !std::isfinite(item.settleSeconds) || item.settleSeconds<.5 || item.settleSeconds>5 ||
               !std::isfinite(item.holdSeconds) || item.holdSeconds<0 || item.holdSeconds>120)
                throw std::runtime_error("steer_command exceeds limit, settle_time_s outside 0.5..5, or hold_time_s outside 0..120");
        if(!std::isfinite(correction.command) || correction.command<0 || correction.command>std::min(15.,limit) ||
           !std::isfinite(correction.holdSeconds) || correction.holdSeconds<0 || correction.holdSeconds>120)
            throw std::runtime_error("stage_2 correction command must be 0..limit and hold time 0..120");
        if(correction.holdSeconds>0 && (correction.command<=0 || (!startRightCorrection && (stages[1].command>=0 || stages[1].holdSeconds<=0))))
            throw std::runtime_error("Enabled stage_2 correction requires a left first command, positive first hold time and right correction command");
        if(startRightCorrection && (!single || stage!=2 || correction.holdSeconds<=0))
            throw std::runtime_error("stage_2_step=right_correction requires single mode, stage=2 and enabled correction");
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
                } else if(key=="stage_2_step") {
                    if(value!="first_bend" && value!="right_correction") throw std::runtime_error("stage_2_step must be first_bend or right_correction");
                    result.startRightCorrection=value=="right_correction";
                } else throw std::runtime_error("Unknown session key: "+key);
            } else {
                auto& item=result.stages[size_t(section[6]-'1')];
                if(key=="steer_command") item.command=finiteNumber(value);
                else if(key=="settle_time_s") item.settleSeconds=finiteNumber(value);
                else if(key=="hold_time_s") item.holdSeconds=finiteNumber(value);
                else if(section=="stage_2" && key=="correction_steer_command") result.correction.command=finiteNumber(value);
                else if(section=="stage_2" && key=="correction_hold_time_s") result.correction.holdSeconds=finiteNumber(value);
                else throw std::runtime_error("Unknown stage key: "+key);
            }
        }
        if(!keys.count("session.mode") || !keys.count("session.stage")) throw std::runtime_error("All session keys are required");
        for(int stage=1;stage<=6;++stage)
            for(const auto* key:{"steer_command","settle_time_s","hold_time_s"})
                if(!keys.count("stage_"+std::to_string(stage)+"."+key))
                    throw std::runtime_error("All six stage keys are required");
        if(keys.count("stage_2.correction_steer_command")!=keys.count("stage_2.correction_hold_time_s"))
            throw std::runtime_error("Both stage_2 correction keys are required together");
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
    enum class State {AwaitApply,WritePending,Settling,AwaitStart,Running};
    explicit ParkingRehearsal(ParkingTuning tuning):tuning_(std::move(tuning)),stage_(tuning_.stage-1),nextStage_(stage_) {
        tuning_.validate();segment_=tuning_.startRightCorrection ? 1u : 0u;
    }
    std::optional<double> update(double now,RehearsalKey key) {
        if(finished()) return {};
        if(!std::isfinite(now) || now<lastTime_) {stop("INVALID_TIME");return {};}
        lastTime_=now;
        if(key==RehearsalKey::Quit) {stop("USER_QUIT");return {};}
        if(key==RehearsalKey::Pause) {
            paused_=true;
            if(!correctionSequenceActive()) {state_=State::AwaitApply;nextStage_=stage_;}
            return {};
        }
        if(pendingCorrection_) {
            if(key==RehearsalKey::Continue) paused_=false;
            pendingCorrection_=false;automaticCorrectionWrite_=true;state_=State::WritePending;return command();
        }
        if(paused_) {
            if(key==RehearsalKey::Continue) paused_=false;
            return {};
        }
        if(state_==State::Settling) {
            if(now-writeTime_>=tuning_.stages[size_t(stage_)].settleSeconds) state_=State::AwaitStart;
            return {}; // Discard even Enter at the settling boundary.
        }
        if(key!=RehearsalKey::Enter) return {};
        if(state_==State::AwaitApply) {
            stage_=nextStage_;segment_=(stage_==1 && tuning_.startRightCorrection) ? 1u : 0u;
            correctionComplete_=false;automaticSavedTrial_=automaticCorrectionWrite_=false;
            state_=State::WritePending;return command();
        }
        if(state_==State::AwaitStart) {state_=State::Running;++trials_[size_t(stage_)];}
        else if(state_==State::Running) {
            if(correctionSequenceActive()) return {}; // Timed pair is one authorized maneuver; Q can abort.
            state_=State::AwaitApply;
            nextStage_=tuning_.single ? stage_ : std::min(stage_+1,5);
        }
        return {};
    }
    void acknowledge(bool success,double now) {
        if(state_!=State::WritePending || finished()) throw std::runtime_error("No servo write pending");
        if(!success || !std::isfinite(now) || now<lastTime_) {stop("SERVO_WRITE_FAILED");return;}
        state_=(automaticSavedTrial_ || automaticCorrectionWrite_) ? State::Running : State::Settling;
        writeTime_=lastTime_=now;
    }
    // A stable, valid file save is the user's execution authorization.
    std::optional<double> applySaved(ParkingTuning tuning,double now) {
        if(finished()) return {};
        if(!std::isfinite(now) || now<lastTime_) {stop("INVALID_TIME");return {};}
        tuning.validate();lastTime_=now;
        const bool selectorChanged=tuning.stage!=tuning_.stage || tuning.single!=tuning_.single || tuning.startRightCorrection!=tuning_.startRightCorrection;
        const int target=selectorChanged ? tuning.stage-1 : stage_;
        const auto& before=tuning_.stages[size_t(target)];const auto& after=tuning.stages[size_t(target)];
        const bool primaryChanged=before.command!=after.command || before.holdSeconds!=after.holdSeconds || before.settleSeconds!=after.settleSeconds;
        const bool correctionChanged=target==1 && (tuning.correction.command!=tuning_.correction.command || tuning.correction.holdSeconds!=tuning_.correction.holdSeconds);
        const bool execute=selectorChanged || target!=stage_ || primaryChanged || correctionChanged;
        const bool keepRight=target==stage_ && target==1 && segment_==1 && !selectorChanged && !primaryChanged;
        tuning_=std::move(tuning);++revision_;
        if(!execute) return {};
        if(keepRight && tuning_.correction.holdSeconds==0) {pendingCorrection_=false;stop("RIGHT_CORRECTION_DISABLED");return {};}
        stage_=nextStage_=target;++trials_[size_t(stage_)];automaticSavedTrial_=true;
        automaticCorrectionWrite_=pendingCorrection_=correctionComplete_=false;
        segment_=(stage_==1 && (tuning_.startRightCorrection || keepRight)) ? 1u : 0u;
        state_=State::WritePending;return command();
    }
    void holdCompleted(int stage,unsigned trial,unsigned segment=0) {
        if(finished() || stage!=stage_ || trial!=trials_[size_t(stage_)] || segment!=segment_) return;
        if(stage_==1 && tuning_.correction.holdSeconds>0 && segment_==0) {
            segment_=1;pendingCorrection_=true;state_=State::WritePending;return;
        }
        if(stage_==1 && segment_==1) correctionComplete_=true;
        if(automaticSavedTrial_ || tuning_.single) stop("TIMED_TRIAL_COMPLETED");
    }
    bool correctionSequenceActive() const {return stage_==1 && tuning_.correction.holdSeconds>0 && !correctionComplete_ && (state_==State::Running || state_==State::WritePending);}
    bool pendingCorrection() const {return pendingCorrection_;}
    bool automaticCorrectionWrite() const {return automaticCorrectionWrite_;}
    unsigned segment() const {return segment_;}
    const char* segmentName() const {return rehearsalSegmentName(stage_,segment_);}
    const char* segmentTitle() const {return stage_==1 ? (segment_==1 ? "2B右打修正" : "2A左打倒弯") : "本段";}
    double holdSeconds() const {return (stage_==1 && segment_==1) ? tuning_.correction.holdSeconds : tuning_.stages[size_t(stage_)].holdSeconds;}
    bool automaticSavedTrial() const {return automaticSavedTrial_;}
    void stop(const std::string& reason) {if(!finished()) outcome_=reason;}
    bool finished() const {return !outcome_.empty();}
    bool paused() const {return paused_;}
    int stage() const {return stage_;}
    int nextStage() const {return nextStage_;}
    unsigned trial() const {return trials_[size_t(stage_)];}
    unsigned revision() const {return revision_;}
    double command() const {return (stage_==1 && segment_==1) ? tuning_.correction.command : tuning_.stages[size_t(stage_)].command;}
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
            case State::AwaitStart:return "WAIT_PUSH_PERMISSION";
            case State::Running:return "MANUAL_TRIAL";
        }
        return "INVALID";
    }
private:
    ParkingTuning tuning_;bool automaticSavedTrial_=false,automaticCorrectionWrite_=false,pendingCorrection_=false,correctionComplete_=false;
    unsigned segment_=0;
    int stage_,nextStage_;std::array<unsigned,6> trials_{};unsigned revision_=1;
    State state_=State::AwaitApply;double lastTime_=-1,writeTime_=0;
    bool paused_=false;std::string outcome_;
};
class SavedTuningWatcher {
public:
    SavedTuningWatcher(const ParkingTuning& initial,double limit):accepted_(initial),observed_(initial.source),handled_(initial.source),limit_(limit) {}
    std::optional<ParkingTuning> observe(const std::string& source,double now) {
        if(!std::isfinite(now) || now<lastTime_) throw std::runtime_error("Invalid watcher time");
        lastTime_=now;
        if(source!=observed_) {observed_=source;stableSince_=now;return {};}
        if(source==handled_ || now-stableSince_<.3) return {};
        handled_=source; // A rejected save is reported once; the next save can recover.
        auto candidate=ParkingTuning::parse(source);candidate.validate(limit_);
        bool same=candidate.single==accepted_.single && candidate.stage==accepted_.stage && candidate.startRightCorrection==accepted_.startRightCorrection &&
            candidate.correction.command==accepted_.correction.command && candidate.correction.holdSeconds==accepted_.correction.holdSeconds;
        for(size_t i=0;i<6;++i) same=same && candidate.stages[i].command==accepted_.stages[i].command &&
            candidate.stages[i].holdSeconds==accepted_.stages[i].holdSeconds && candidate.stages[i].settleSeconds==accepted_.stages[i].settleSeconds;
        accepted_=candidate;
        return same ? std::optional<ParkingTuning>{} : candidate;
    }
private:
    ParkingTuning accepted_;std::string observed_,handled_;double limit_,stableSince_=0,lastTime_=-1;
};
}}
