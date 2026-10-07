#pragma once
#include "capture_data.hpp"
#include <optional>
namespace car2026 { namespace capture {
constexpr const char* rehearsalVersion="2026-10-07.1";
constexpr const char* stageNames[]={"前移","第一倒弯","分支直退","第二倒弯","库内直退","停止确认"};
constexpr const char* stageFiles[]={"01_advance.csv","02_reverse_first.csv","03_reverse_branch.csv","04_reverse_second.csv","05_reverse_straight.csv","06_stop_confirmation.csv"};
struct StageTuning {double command=0,settleSeconds=.5,holdSeconds=0;};
struct ParkingTuning {
    bool single=true;int stage=2;std::array<StageTuning,6> stages{};std::string source;
    void validate(double limit=15) const {
        if(stage<1 || stage>6) throw std::runtime_error("stage must be 1..6");
        for(const auto& item:stages)
            if(!std::isfinite(item.command) || std::abs(item.command)>std::min(15.,limit) ||
               !std::isfinite(item.settleSeconds) || item.settleSeconds<.5 || item.settleSeconds>5 ||
               !std::isfinite(item.holdSeconds) || item.holdSeconds<0 || item.holdSeconds>120)
                throw std::runtime_error("steer_command exceeds limit, settle_time_s outside 0.5..5, or hold_time_s outside 0..120");
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
            if(equal==std::string::npos || section.empty()) throw std::runtime_error("Expected [section] and key=value");
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
                } else throw std::runtime_error("Unknown session key: "+key);
            } else {
                auto& item=result.stages[size_t(section[6]-'1')];
                if(key=="steer_command") item.command=finiteNumber(value);
                else if(key=="settle_time_s") item.settleSeconds=finiteNumber(value);
                else if(key=="hold_time_s") item.holdSeconds=finiteNumber(value);
                else throw std::runtime_error("Unknown stage key: "+key);
            }
        }
        if(keys.size()!=20) throw std::runtime_error("All session and six stage keys are required");
        result.validate();return result;
    }
    static ParkingTuning load(const std::string& path) {
        std::ifstream file(path,std::ios::binary);if(!file) throw std::runtime_error("Cannot open tuning file: "+path);
        std::string text;char chunk[1024];
        while(file.read(chunk,sizeof(chunk)) || file.gcount()) {
            text.append(chunk,size_t(file.gcount()));
            if(text.size()>16384) throw std::runtime_error("Tuning file too large");
        }
        if(file.bad()) throw std::runtime_error("Cannot read tuning file");
        return parse(text);
    }
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
    explicit ParkingRehearsal(ParkingTuning tuning):tuning_(std::move(tuning)),stage_(tuning_.stage-1),nextStage_(stage_) {tuning_.validate();}
    std::optional<double> update(double now,RehearsalKey key) {
        if(finished()) return {};
        if(!std::isfinite(now) || now<lastTime_) {stop("INVALID_TIME");return {};}
        lastTime_=now;
        if(key==RehearsalKey::Quit) {stop("USER_QUIT");return {};}
        if(key==RehearsalKey::Pause) {
            paused_=true;state_=State::AwaitApply;nextStage_=stage_;return {};
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
            if(proposal_) {tuning_=*proposal_;proposal_.reset();nextStage_=tuning_.stage-1;++revision_;}
            stage_=nextStage_;state_=State::WritePending;return command();
        }
        if(state_==State::AwaitStart) {state_=State::Running;++trials_[size_t(stage_)];}
        else if(state_==State::Running) {
            state_=State::AwaitApply;
            nextStage_=tuning_.single ? stage_ : std::min(stage_+1,5);
        }
        return {};
    }
    void acknowledge(bool success,double now) {
        if(state_!=State::WritePending || finished()) throw std::runtime_error("No servo write pending");
        if(!success || !std::isfinite(now) || now<lastTime_) {stop("SERVO_WRITE_FAILED");return;}
        state_=State::Settling;writeTime_=lastTime_=now;
    }
    void propose(ParkingTuning tuning) {
        if(!paused_ || finished()) throw std::runtime_error("R requires pause");
        tuning.validate();proposal_=std::move(tuning);
    }
    void stop(const std::string& reason) {if(!finished()) outcome_=reason;}
    bool finished() const {return !outcome_.empty();}
    bool paused() const {return paused_;}
    int stage() const {return stage_;}
    int nextStage() const {return proposal_ ? proposal_->stage-1 : nextStage_;}
    unsigned trial() const {return trials_[size_t(stage_)];}
    unsigned revision() const {return revision_;}
    double command() const {return tuning_.stages[size_t(stage_)].command;}
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
    ParkingTuning tuning_;std::optional<ParkingTuning> proposal_;
    int stage_,nextStage_;std::array<unsigned,6> trials_{};unsigned revision_=1;
    State state_=State::AwaitApply;double lastTime_=-1,writeTime_=0;
    bool paused_=false;std::string outcome_;
};
}}
