#pragma once
#include "visual_observer.hpp"
#include <optional>

namespace car2026 { namespace capture {
// A conservative rest gate, not distance, speed or yaw estimation. Feed the
// existing sampler's two reads together; never read clearing counters twice.
struct EncoderRestStatus {
    bool fresh=false,stationary=false;double age=0;
    std::string reason="NO_SAMPLES";
    double zeroDuration=0,leftDelta=0,rightDelta=0;
    uint64_t movementEpoch=0;
};
// Zero prepares/finishes the reference and needs fresh rest, not an image.
// A nonzero command additionally requires a valid, recent frame.
struct ServoRequestGate {
    bool exitRequested=false,samplerStopped=false;
    double now=0,deadline=45,command=0,frameTime=0,frameTimeout=.4;
    EncoderRestStatus rest;
    std::string issue() const {
        if(exitRequested) return "EXIT_REQUESTED";
        if(samplerStopped) return "SAMPLER_STOPPED";
        if(!std::isfinite(now) || now<0 || !std::isfinite(deadline) || deadline<=0) return "INVALID_WRITE_TIME";
        if(now>=deadline) return "SESSION_DEADLINE";
        if(!std::isfinite(command)) return "INVALID_COMMAND";
        if(!rest.fresh) return "ENCODER_NOT_FRESH";
        if(!rest.stationary) return "NOT_STATIONARY";
        if(command!=0) {
            if(!std::isfinite(frameTime) || frameTime<0 || now<frameTime) return "INVALID_FRAME_TIME";
            if(!std::isfinite(frameTimeout) || frameTimeout<=0) return "INVALID_FRAME_TIMEOUT";
            if(now-frameTime>frameTimeout) return "FRAME_TOO_OLD";
        }
        return {};
    }
};
class EncoderRestGate {
public:
    void sample(bool leftValid,double left,bool rightValid,double right,double time,double readSpan=0) {
        const bool clockOk=std::isfinite(time) && time>=0 && (last_<0 || time>last_);
        const bool gap=last_>=0 && time-last_>maximumAge;
        left_=left;right_=right;
        valid_=clockOk && leftValid && rightValid && std::isfinite(left) && std::isfinite(right) &&
               std::isfinite(readSpan) && readSpan>=0 && readSpan<=maximumAge;
        if(!clockOk) {valid_=false;zeroSince_=-1;sampleReason_=resetReason_="INVALID_SAMPLE_TIME";return;}
        last_=time;
        if(!valid_) {
            zeroSince_=-1;resetReason_="INVALID_SAMPLE";
            if(!std::isfinite(readSpan) || readSpan<0 || readSpan>maximumAge) sampleReason_="ENCODER_READ_SPAN";
            else if(!leftValid || !std::isfinite(left)) sampleReason_="INVALID_LEFT_ENCODER";
            else sampleReason_="INVALID_RIGHT_ENCODER";
        } else if(left!=0 || right!=0) {
            ++movementEpoch_;
            zeroSince_=-1;resetReason_="NONZERO_DELTA";sampleReason_="NONZERO_ENCODER_DELTA";
        } else {
            if(zeroSince_<0 || gap) zeroSince_=time;
            if(gap) resetReason_="SAMPLE_GAP";
            sampleReason_="ZERO_DELTAS";
        }
    }
    EncoderRestStatus status(double now) const {
        EncoderRestStatus result;result.leftDelta=left_;result.rightDelta=right_;result.reason=sampleReason_;
        result.movementEpoch=movementEpoch_;
        if(last_<0) return result;
        if(!std::isfinite(now) || now<last_) {result.reason="INVALID_STATUS_TIME";return result;}
        result.age=now-last_;result.fresh=valid_ && result.age<=maximumAge;
        if(valid_ && !result.fresh) result.reason="STALE_ENCODER_SAMPLE";
        if(result.fresh && zeroSince_>=0) result.zeroDuration=now-zeroSince_;
        result.stationary=result.fresh && zeroSince_>=0 && now-zeroSince_>=restInterval;
        if(result.stationary) result.reason="STATIONARY";
        else if(result.fresh && zeroSince_>=0) result.reason="WAIT_REST_AFTER_"+resetReason_;
        return result;
    }
private:
    static constexpr double maximumAge=.25,restInterval=.8;
    double last_=-1,zeroSince_=-1;bool valid_=false;
    double left_=0,right_=0;
    uint64_t movementEpoch_=0;
    std::string sampleReason_="NO_SAMPLES",resetReason_="STARTUP";
};

enum class ProbeKey { None,Reference,Positive,Negative,Center,Stop,Conflict,Invalid };
struct ProbeInput {
    bool quit=false;ProbeKey key=ProbeKey::None;
    void add(const std::string& line) {
        if(line=="Q" || line=="q") {quit=true;return;}
        if(line.empty() || (line.size()==1 && ((line[0]>='A' && line[0]<='E') ||
                                             (line[0]>='a' && line[0]<='e')))) return;
        ProbeKey next=ProbeKey::None;
        if(line=="R" || line=="r") next=ProbeKey::Reference;
        if(line=="+") next=ProbeKey::Positive;
        if(line=="-") next=ProbeKey::Negative;
        if(line=="0") next=ProbeKey::Center;
        if(line=="S" || line=="s") next=ProbeKey::Stop;
        if(next==ProbeKey::None) {key=ProbeKey::Invalid;return;}
        if(key!=ProbeKey::Invalid) key=key==ProbeKey::None ? next : ProbeKey::Conflict;
    }
};
inline const char* probeKeyName(ProbeKey key) {
    switch(key) {
        case ProbeKey::Reference:return "R";case ProbeKey::Positive:return "+";
        case ProbeKey::Negative:return "-";case ProbeKey::Center:return "0";
        case ProbeKey::Stop:return "S";case ProbeKey::Conflict:return "CONFLICT";
        case ProbeKey::Invalid:return "INVALID";
        default:return "NONE";
    }
}
struct ProbeAction {
    std::string state,decision,reason;bool resetReference=false;
    std::optional<double> command; // Explicit manual/automatic trial requests or rest recovery, never a suggestion.
};
class ManualSteeringProbe {
public:
    ProbeAction update(ProbeKey key,const EncoderRestStatus& rest,
                       const SteeringObservation& image,double now) {
        ProbeAction result;
        if(!std::isfinite(now) || now<0 || (lastTime_>=0 && now<=lastTime_)) {
            hold("INVALID_PROCESSING_TIME");result.decision="INVALID_TIME_STOP_PUSHING";
            result.state=stateName();result.reason=holdReason_;return result;
        }
        lastTime_=now;
        if(state_==State::Reference && (!rest.fresh || !rest.stationary))
            hold(rest.fresh ? "NOT_STATIONARY_DURING_REFERENCE" : "ENCODER_NOT_FRESH_DURING_REFERENCE");
        if(state_==State::Ready) {
            if(!rest.fresh) hold("ENCODER_NOT_FRESH");
            else if(!imageIssue(image).empty()) hold(imageIssue(image));
            else if(image.referenceId!=referenceId_) hold("IMAGE_REFERENCE_CHANGED");
        }
        if(key==ProbeKey::Stop) hold("USER_STOP");
        // Loss is latched; do not chase another line or change steering while
        // the user is still pulling. A fresh rest interval permits zero only.
        if(state_==State::Hold) {
            result.decision="HOLD_STOP_PUSHING";result.reason=holdReason_;
            if(rest.stationary && rest.fresh) {
                if(hasCommandRequest_) result.command=0;
                state_=State::Unarmed;referenceId_=0;result.resetReference=true;
                result.decision=hasCommandRequest_ ? "REST_ZERO_REARM_REQUIRED" : "REST_NO_OUTPUT_REARM_REQUIRED";
                hasCommandRequest_=false;
                holdReason_.clear();
            }
            result.state=stateName();return result;
        }
        if(key==ProbeKey::Invalid) result.decision="REJECT_INVALID_KEY";
        else if(key==ProbeKey::Conflict) result.decision="REJECT_MULTIPLE_KEYS";
        else if(key==ProbeKey::Reference) {
            if(rest.fresh && rest.stationary) {
                state_=State::Reference;referenceId_=0;result.resetReference=true;
                hasCommandRequest_=true;result.command=0;result.decision="CENTER_THEN_BUILD_REFERENCE";
            } else {result.decision="REJECT_REFERENCE_NOT_STATIONARY";result.reason=rest.reason;}
        } else if(state_==State::Reference) {
            if(imageIssue(image).empty()) {
                referenceId_=image.referenceId;state_=State::Ready;
                result.decision="PROBE_REFERENCE_READY";
            } else {result.decision="WAIT_STATIONARY_STRAIGHT_REFERENCE";result.reason=imageIssue(image);}
        } else if(key==ProbeKey::Positive || key==ProbeKey::Negative || key==ProbeKey::Center) {
            if(state_!=State::Ready) {result.decision="REJECT_NO_MANUAL_REFERENCE";result.reason="ENTER_R_WHILE_STOPPED";}
            else if(!rest.stationary || !rest.fresh) {result.decision="REJECT_COMMAND_STOP_PUSHING";result.reason=rest.reason;}
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
    std::string holdReason_;
    void hold(const std::string& reason) {
        if(state_!=State::Hold) holdReason_=reason;
        state_=State::Hold;
    }
    static std::string imageIssue(const SteeringObservation& image) {
        if(image.state!="TRACKING") return "IMAGE_"+image.state;
        if(!image.hasReference || !image.referenceId) return "IMAGE_REFERENCE_UNAVAILABLE";
        if(!image.hasSuggestion) return "IMAGE_FEATURE_UNAVAILABLE";
        if(!std::isfinite(image.lateralError) || !std::isfinite(image.headingFeatureError)) return "INVALID_IMAGE_FEATURE";
        if(std::abs(image.lateralError)>.08 || std::abs(image.headingFeatureError)>.08) return "IMAGE_DEVIATION_LIMIT";
        return {};
    }
    const char* stateName() const {
        switch(state_) {
            case State::Reference:return "BUILD_REFERENCE";case State::Ready:return "READY";
            case State::Hold:return "HOLD";default:return "UNARMED";
        }
    }
};

// One explicit +2 trial. Reuses all ManualSteeringProbe gates. Command writes
// must be acknowledged: a planned write never authorizes the user to pull.
class AutomaticSteeringProbe {
public:
    explicit AutomaticSteeringProbe(double duration=45) : duration_(duration) {
        if(!std::isfinite(duration) || duration<=0 || duration>60)
            throw std::runtime_error("Automatic probe duration must be within 0..60 seconds");
    }
    ProbeAction update(const EncoderRestStatus& rest,const SteeringObservation& image,double now) {
        if(start_<0 && std::isfinite(now) && now>=0) start_=phaseStart_=now;
        if(!std::isfinite(now) || now<0 || (lastTime_>=0 && now<=lastTime_)) stop("INVALID_PROCESSING_TIME");
        if(std::isfinite(now)) lastTime_=now;
        if((phase_==Phase::WaitRest || phase_==Phase::Reference) && now-phaseStart_>15)
            stop(phase_==Phase::WaitRest ? "STARTUP_NOT_STATIONARY" : "STRAIGHT_REFERENCE_TIMEOUT");
        if(phase_==Phase::WaitPull && pending_==Pending::None) {
            if(rest.fresh && rest.movementEpoch!=movementBaseline_) {phase_=Phase::Pull;phaseStart_=now;}
            else if(now-phaseStart_>12) stop("NO_PULL_DETECTED");
        }
        if(phase_==Phase::Pull) {
            if(rest.fresh && rest.stationary) stop("PULL_COMPLETED");
            else if(now-phaseStart_>10) stop("PULL_TIME_LIMIT");
        }
        if(phase_==Phase::SetSteer && duration_-now<25) stop("INSUFFICIENT_TIME_FOR_PULL");
        ProbeKey key=ProbeKey::None;
        if(phase_==Phase::WaitRest && rest.fresh && rest.stationary) key=ProbeKey::Reference;
        if(phase_==Phase::SetSteer) key=ProbeKey::Positive;
        if(phase_==Phase::Stop) key=ProbeKey::Stop;
        // Normal recorder integration acknowledges before the next frame.
        // If a caller delays that acknowledgement, never resend the command.
        if(pending_!=Pending::None) {
            ProbeAction waiting;waiting.state="AWAIT_WRITE";waiting.decision="AUTO_WAIT_WRITE_ACK";return waiting;
        }
        auto action=probe_.update(key,rest,image,now);
        if(phase_==Phase::WaitRest) {
            if(action.command) {phase_=Phase::Reference;pending_=Pending::ReferenceZero;}
            else {action.decision="AUTO_WAIT_STILL";action.reason=rest.reason;}
        } else if(phase_==Phase::Reference && action.decision=="PROBE_REFERENCE_READY") phase_=Phase::SetSteer;
        else if(phase_==Phase::SetSteer) {
            if(action.command && *action.command==2) {phase_=Phase::WaitPull;pending_=Pending::ProbeSteer;}
            else stop(action.reason.empty() ? "PREPARE_COMMAND_REJECTED" : action.reason);
        }
        if(action.state=="HOLD" || action.decision=="REST_ZERO_REARM_REQUIRED")
            stop(action.reason.empty() ? "PROBE_GATE_FAILED" : action.reason);
        if(phase_==Phase::Stop) {
            action.reason=outcome_;
            if(action.command) pending_=Pending::FinishZero;
            else if(!everWritten_) phase_=Phase::Done;
        }
        return action;
    }
    // true means the +2 software write succeeded; only then show the pull cue.
    bool acknowledge(bool written,const EncoderRestStatus& rest,double now,bool imageFresh=true,
                     const std::string& cancellationReason={}) {
        const auto pending=pending_;pending_=Pending::None;
        if(pending==Pending::None) return false;
        if(!written) {
            // No output has occurred. A transient rest loss may wait within the
            // original startup deadline; never retry a cancelled nonzero write.
            if(pending==Pending::ReferenceZero && !everWritten_ && phase_==Phase::Reference &&
               (cancellationReason=="SERVO_CANCELLED_ENCODER_NOT_FRESH" ||
                cancellationReason=="SERVO_CANCELLED_NOT_STATIONARY")) {
                probe_=ManualSteeringProbe{};phase_=Phase::WaitRest;phaseStart_=start_;
                return false;
            }
            stop(cancellationReason.empty() ? "SERVO_REQUEST_CANCELLED" : cancellationReason);return false;
        }
        everWritten_=true;
        if(phase_==Phase::Stop && pending!=Pending::FinishZero) return false;
        if(pending==Pending::ReferenceZero) phaseStart_=now;
        if(pending==Pending::ProbeSteer) {
            if(!rest.fresh || !rest.stationary || !imageFresh || !std::isfinite(now) || now<lastTime_) {
                stop("POST_WRITE_GATE_CHANGED");return false;
            }
            phaseStart_=now;movementBaseline_=rest.movementEpoch;return true;
        }
        if(pending==Pending::FinishZero) phase_=Phase::Done;
        return false;
    }
    void stop(const std::string& reason) {
        if(phase_==Phase::Done) return;
        if(phase_!=Phase::Stop || (outcome_=="PULL_COMPLETED" && reason!="USER_STOP" && reason!="PULL_COMPLETED"))
            outcome_=reason;
        phase_=Phase::Stop;
    }
    bool finished() const {return phase_==Phase::Done;}
    const std::string& outcome() const {return outcome_;}
    const char* phaseName() const {
        switch(phase_) {
            case Phase::Reference:return "BUILD_REFERENCE";case Phase::SetSteer:return "SET_STEER";
            case Phase::WaitPull:return "WAIT_PULL";case Phase::Pull:return "PULLING";
            case Phase::Stop:return "WAIT_STOP";case Phase::Done:return "DONE";
            default:return "WAIT_STILL";
        }
    }
private:
    enum class Phase { WaitRest,Reference,SetSteer,WaitPull,Pull,Stop,Done };
    enum class Pending { None,ReferenceZero,ProbeSteer,FinishZero };
    ManualSteeringProbe probe_;Phase phase_=Phase::WaitRest;Pending pending_=Pending::None;
    double duration_=45,start_=-1,phaseStart_=0,lastTime_=-1;uint64_t movementBaseline_=0;
    bool everWritten_=false;std::string outcome_="INCOMPLETE";
};
}} // namespace car2026::capture
