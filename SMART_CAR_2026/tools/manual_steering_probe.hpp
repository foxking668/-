#pragma once
#include "visual_observer.hpp"
#include <optional>

namespace car2026 { namespace capture {
// A conservative rest gate, not distance, speed or yaw estimation. Feed the
// existing sampler's two reads together; never read clearing counters twice.
struct EncoderRestStatus { bool fresh=false,stationary=false; double age=0; };
class EncoderRestGate {
public:
    void sample(bool leftValid,double left,bool rightValid,double right,double time) {
        const bool clockOk=std::isfinite(time) && time>=0 && (last_<0 || time>last_);
        const bool gap=last_>=0 && time-last_>maximumAge;
        valid_=clockOk && leftValid && rightValid && std::isfinite(left) && std::isfinite(right);
        if(!clockOk) {valid_=false;zeroSince_=-1;return;}
        last_=time;
        if(!valid_ || left!=0 || right!=0) zeroSince_=-1;
        else if(zeroSince_<0 || gap) zeroSince_=time;
    }
    EncoderRestStatus status(double now) const {
        EncoderRestStatus result;
        if(!std::isfinite(now) || now<last_ || last_<0) return result;
        result.age=now-last_;result.fresh=valid_ && result.age<=maximumAge;
        result.stationary=result.fresh && zeroSince_>=0 && now-zeroSince_>=restInterval;
        return result;
    }
private:
    static constexpr double maximumAge=.25,restInterval=.8;
    double last_=-1,zeroSince_=-1;bool valid_=false;
};

enum class ProbeKey { None,Reference,Positive,Negative,Center,Stop,Conflict };
struct ProbeInput {
    bool quit=false;ProbeKey key=ProbeKey::None;
    void add(const std::string& line) {
        if(line=="Q" || line=="q") {quit=true;return;}
        ProbeKey next=ProbeKey::None;
        if(line=="R" || line=="r") next=ProbeKey::Reference;
        if(line=="+") next=ProbeKey::Positive;
        if(line=="-") next=ProbeKey::Negative;
        if(line=="0") next=ProbeKey::Center;
        if(line=="S" || line=="s") next=ProbeKey::Stop;
        if(next!=ProbeKey::None) key=key==ProbeKey::None ? next : ProbeKey::Conflict;
    }
};
inline const char* probeKeyName(ProbeKey key) {
    switch(key) {
        case ProbeKey::Reference:return "R";case ProbeKey::Positive:return "+";
        case ProbeKey::Negative:return "-";case ProbeKey::Center:return "0";
        case ProbeKey::Stop:return "S";case ProbeKey::Conflict:return "CONFLICT";
        default:return "NONE";
    }
}
struct ProbeAction {
    std::string state,decision;bool resetReference=false;
    std::optional<double> command; // Only explicit keyboard requests/rest recovery, never a suggestion.
};
class ManualSteeringProbe {
public:
    ProbeAction update(ProbeKey key,const EncoderRestStatus& rest,
                       const SteeringObservation& image,double now) {
        ProbeAction result;
        if(!std::isfinite(now) || now<0 || (lastTime_>=0 && now<=lastTime_)) {
            state_=State::Hold;result.decision="INVALID_TIME_STOP_PUSHING";
            result.state=stateName();return result;
        }
        lastTime_=now;
        if(state_==State::Reference && (!rest.fresh || !rest.stationary)) state_=State::Hold;
        if(state_==State::Ready && (!rest.fresh || !usable(image) || image.referenceId!=referenceId_))
            state_=State::Hold;
        if(key==ProbeKey::Stop) state_=State::Hold;
        // Loss is latched; do not chase another line or change steering while
        // the user is still pulling. A fresh rest interval permits zero only.
        if(state_==State::Hold) {
            result.decision="HOLD_STOP_PUSHING";
            if(rest.stationary && rest.fresh) {
                if(hasCommandRequest_) result.command=0;
                state_=State::Unarmed;referenceId_=0;result.resetReference=true;
                result.decision=hasCommandRequest_ ? "REST_ZERO_REARM_REQUIRED" : "REST_NO_OUTPUT_REARM_REQUIRED";
                hasCommandRequest_=false;
            }
            result.state=stateName();return result;
        }
        if(key==ProbeKey::Conflict) result.decision="REJECT_MULTIPLE_KEYS";
        else if(key==ProbeKey::Reference) {
            if(rest.fresh && rest.stationary) {
                state_=State::Reference;referenceId_=0;result.resetReference=true;
                hasCommandRequest_=true;result.command=0;result.decision="CENTER_THEN_BUILD_REFERENCE";
            } else result.decision="REJECT_REFERENCE_NOT_STATIONARY";
        } else if(state_==State::Reference) {
            if(usable(image)) {
                referenceId_=image.referenceId;state_=State::Ready;
                result.decision="PROBE_REFERENCE_READY";
            } else result.decision="WAIT_STATIONARY_STRAIGHT_REFERENCE";
        } else if(key==ProbeKey::Positive || key==ProbeKey::Negative || key==ProbeKey::Center) {
            if(state_!=State::Ready) result.decision="REJECT_NO_MANUAL_REFERENCE";
            else if(!rest.stationary || !rest.fresh) result.decision="REJECT_COMMAND_STOP_PUSHING";
            else {
                result.command=key==ProbeKey::Positive ? 2 : (key==ProbeKey::Negative ? -2 : 0);
                result.decision="PROBE_STEER_READY";
            }
        } else result.decision=state_==State::Ready ? "RECORD_RESPONSE_ONLY" : "WAIT_R_WHILE_STOPPED";
        result.state=stateName();return result;
    }
private:
    enum class State { Unarmed,Reference,Ready,Hold };
    State state_=State::Unarmed;unsigned referenceId_=0;double lastTime_=-1;
    bool hasCommandRequest_=false;
    static bool usable(const SteeringObservation& image) {
        return image.hasReference && image.referenceId>0 && image.hasSuggestion && image.state=="TRACKING" &&
               std::isfinite(image.lateralError) && std::isfinite(image.headingFeatureError) &&
               std::abs(image.lateralError)<=.08 && std::abs(image.headingFeatureError)<=.08;
    }
    const char* stateName() const {
        switch(state_) {
            case State::Reference:return "BUILD_REFERENCE";case State::Ready:return "READY";
            case State::Hold:return "HOLD";default:return "UNARMED";
        }
    }
};
}} // namespace car2026::capture
