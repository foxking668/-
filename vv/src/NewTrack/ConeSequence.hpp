#pragma once
#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <map>
#include <stdexcept>
#include <string>
#include <vector>

namespace NewTrack { namespace ConeSequence {
struct Settings {
    bool enabled=false;
    int entryTimeMs=4500,entryEarlyMs=500,entryLateMs=500;
    std::array<int,4> pulseMs{{200,200,200,200}},maxMs{{2500,2500,2500,2500}};
    std::array<double,4> errors{{30,58,30,58}};
    double zoneSpeed=6,postSpeed=7;
    int postMs=5500,entryYPercent=45,passNearYPercent=75,returnCenterTolerance=20;
    int grayThresholdPercent=55,grayMinContrast=12,grayMinArea=5,grayTaperPercent=80;
};
inline bool loadFile(const std::string& path,Settings& out,std::string& error) {
    std::ifstream in(path);if(!in) {error="cannot read "+path;return false;}
    const std::array<std::string,4> stages{{"first_left","second_right","third_left","return_right"}};
    std::map<std::string,std::pair<double,double>> limits={
        {"format_version",{2,2}},
        {"entry_time_ms",{0,60000}},{"entry_early_ms",{0,30000}},{"entry_late_ms",{0,30000}},
        {"cone_zone_speed",{.5,20}},{"wait_duration_ms",{0,0}},
        {"post_duration_ms",{0,30000}},{"post_speed",{.5,20}},
        {"entry_y_percent",{10,90}},{"pass_near_y_percent",{40,95}},
        {"return_center_tolerance_percent",{1,40}},
        {"gray_threshold_percent",{20,100}},{"gray_min_contrast",{5,80}},
        {"gray_min_area_80x60",{3,100}},{"gray_taper_percent",{30,95}}};
    for(const auto& stage:stages) {
        limits[stage+"_duration_ms"]={0,10000};limits[stage+"_timeout_ms"]={100,20000};
        limits[stage+"_error"]={0,90};
    }
    std::map<std::string,double> values;std::string line;int number=0;
    auto trim=[](std::string s) {const auto a=s.find_first_not_of(" \t\r\n");
        return a==std::string::npos?std::string():s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);};
    while(std::getline(in,line)) {
        ++number;if(number==1 && line.compare(0,3,"\xef\xbb\xbf")==0) line.erase(0,3);
        line=trim(line.substr(0,line.find('#')));if(line.empty()) continue;
        const auto eq=line.find('=');if(eq==std::string::npos) {error="missing = at line "+std::to_string(number);return false;}
        const auto key=trim(line.substr(0,eq)),text=trim(line.substr(eq+1));const auto limit=limits.find(key);
        try {
            std::size_t used=0;const double v=std::stod(text,&used);
            const bool integer=key!="cone_zone_speed" && key!="post_speed" && key.find("_error")==std::string::npos;
            if(limit==limits.end() || values.count(key) || used!=text.size() || !std::isfinite(v) ||
               v<limit->second.first || v>limit->second.second || (integer && std::floor(v)!=v)) throw std::runtime_error("invalid value/key");
            values[key]=v;
        } catch(...) {error="invalid/duplicate parameter at line "+std::to_string(number)+": "+key;return false;}
    }
    if(values.size()!=limits.size()) {error="incomplete pulse cone tuning file (format_version=2, 27 parameters required)";return false;}
    Settings next=out;next.entryTimeMs=int(values["entry_time_ms"]);
    next.entryEarlyMs=int(values["entry_early_ms"]);next.entryLateMs=int(values["entry_late_ms"]);
    next.zoneSpeed=values["cone_zone_speed"];next.postMs=int(values["post_duration_ms"]);next.postSpeed=values["post_speed"];
    next.entryYPercent=int(values["entry_y_percent"]);next.passNearYPercent=int(values["pass_near_y_percent"]);
    next.returnCenterTolerance=int(values["return_center_tolerance_percent"]);
    next.grayThresholdPercent=int(values["gray_threshold_percent"]);next.grayMinContrast=int(values["gray_min_contrast"]);
    next.grayMinArea=int(values["gray_min_area_80x60"]);next.grayTaperPercent=int(values["gray_taper_percent"]);
    for(int i=0;i<4;++i) {
        next.pulseMs[i]=int(values[stages[i]+"_duration_ms"]);next.maxMs[i]=int(values[stages[i]+"_timeout_ms"]);
        next.errors[i]=values[stages[i]+"_error"];
        if(next.maxMs[i]<=next.pulseMs[i]) {error="timeout must exceed forced pulse duration: "+stages[i];return false;}
    }
    if(next.entryEarlyMs>next.entryTimeMs || next.passNearYPercent<=next.entryYPercent) {
        error="entry early range exceeds origin, or pass-near row is not below entry row";return false;
    }
    out=next;error.clear();return true;
}

// All candidate coordinates are normalized to 0..100, independent of camera resolution.
struct Candidate {
    double x=0,y=0,w=0,h=0;bool acquire=true,nearClipped=false;
    double bottom() const{return y+h;}double center() const{return x+w/2;}
};
struct Observation {
    bool valid=false,roadValid=false;double roadCenter=50;
    std::vector<Candidate> candidates;
};
enum class Phase {IDLE,LEFT,RIGHT,THIRD_LEFT,RETURN_RIGHT,POST,DONE,FAULT};
inline const char* name(Phase p) {switch(p) {
    case Phase::LEFT:return "LEFT_1";case Phase::RIGHT:return "RIGHT_2";case Phase::THIRD_LEFT:return "LEFT_3";
    case Phase::RETURN_RIGHT:return "RETURN_RIGHT";case Phase::POST:return "POST";case Phase::DONE:return "DONE";
    case Phase::FAULT:return "FAULT";default:return "IDLE";}}
class Controller {
    Settings locked_;int lap_=-1;bool used_=false,previousAllowed_=false,previousDriveAllowed_=false,hasClock_=false,tracking_=false,nearSeen_=false;
    long long elapsed_=0,motionBudgetMs_=0,nearAtMotionMs_=0,lastMs_=0,releaseAtMs_=-1;Phase phase_=Phase::IDLE;
    long long previousMotionLimitMs_=150;
    Candidate target_;int targetHits_=0,missing_=0,roadHits_=0,entryHits_=0;
    Candidate passedTarget_;bool hasPassedTarget_=false;
    double previousRoad_=50;std::string entryReason_="NONE",reason_="WAIT_WINDOW";
    void clearEvidence() {tracking_=false;nearSeen_=false;targetHits_=missing_=roadHits_=entryHits_=0;}
    static bool matches(const Candidate& a,const Candidate& b) {
        return std::abs(a.center()-b.center())<=18 && std::abs(a.bottom()-b.bottom())<=25 &&
            a.w<=b.w*2.5+3 && b.w<=a.w*2.5+3;
    }
    const Candidate* choose(const Observation& observation,bool expectedSide,bool entry) const {
        const Candidate* best=nullptr;double score=1e9;
        for(const auto& c:observation.candidates) {
            if(tracking_) {
                if(!matches(c,target_) || (nearSeen_ && !c.nearClipped && c.bottom()<locked_.passNearYPercent-10)) continue;
            }
            else {
                if(!c.acquire && !c.nearClipped) continue;
                if(entry && !c.acquire) continue;
                if(!entry && hasPassedTarget_ && matches(c,passedTarget_)) continue;
                if(entry && !c.nearClipped && c.bottom()<locked_.entryYPercent) continue;
                if(expectedSide && ((phase_==Phase::RIGHT && c.center()>55) ||
                    ((phase_==Phase::LEFT || phase_==Phase::THIRD_LEFT) && c.center()<45))) continue;
                // Near/clipped acquisition still needs two fresh frames and the expected side.
            }
            const double next=tracking_?std::abs(c.center()-target_.center())+std::abs(c.bottom()-target_.bottom()):100-c.bottom();
            if(next<score) {best=&c;score=next;}
        }
        return best;
    }
    void advance(long long now) {
        if(phase_!=Phase::RETURN_RIGHT && tracking_) {passedTarget_=target_;hasPassedTarget_=true;}
        const int next=int(phase_)+1;phase_=static_cast<Phase>(next);elapsed_=motionBudgetMs_=0;clearEvidence();reason_="SEEK_CURRENT_CONE";
        if(phase_==Phase::RETURN_RIGHT) reason_="SEEK_ROAD";
        if(phase_==Phase::POST) {releaseAtMs_=now;reason_="ROAD_RECOVERED";if(locked_.postMs==0) phase_=Phase::DONE;}
    }
public:
    void reset() {*this=Controller{};}
    void update(long long now,int lap,bool enabled,long long routeMs,bool allowed,const Settings& settings,const Observation& observation) {
        if(lap!=lap_) {reset();lap_=lap;}
        if(!enabled) {phase_=used_?Phase::DONE:Phase::IDLE;previousAllowed_=previousDriveAllowed_=false;hasClock_=false;clearEvidence();return;}
        const bool fresh=hasClock_ && now>lastMs_ && now-lastMs_<=150;
        const bool continuous=fresh && allowed && previousAllowed_ && observation.valid;
        // Motors may run after the preceding permitted frame until the 150ms watchdog.
        // Repeated sparse frames must not freeze the maximum-turn safety budget forever.
        if(inRoute() && hasClock_ && previousDriveAllowed_ && now>lastMs_)
            motionBudgetMs_+=std::min(previousMotionLimitMs_,now-lastMs_);
        if(!continuous) {clearEvidence();if(inRoute()) reason_=allowed?"EVIDENCE_RESET":"PAUSED";}
        if(phase_==Phase::IDLE && !used_) {
            locked_=settings;
            const long long earliest=settings.entryTimeMs-settings.entryEarlyMs,latest=settings.entryTimeMs+settings.entryLateMs;
            if(allowed && observation.valid && routeMs>=earliest && lap<2) {
                const auto candidate=choose(observation,false,true);
                if(candidate) {
                    target_=*candidate;tracking_=true;++entryHits_;
                } else {tracking_=false;entryHits_=0;}
                const bool visual=entryHits_>=2;
                if(visual || routeMs>=latest) {
                    used_=true;phase_=Phase::LEFT;elapsed_=motionBudgetMs_=0;entryReason_=visual?"VISUAL_WINDOW":"TIME_FALLBACK";
                    reason_="SEEK_CURRENT_CONE";targetHits_=candidate?entryHits_:0;
                    nearSeen_=targetHits_>=2 && (target_.nearClipped || target_.bottom()>=locked_.passNearYPercent);
                    nearAtMotionMs_=motionBudgetMs_;
                } else reason_="ENTRY_CONFIRMING";
            } else reason_="WAIT_WINDOW";
        } else if(active() && continuous) {
            elapsed_+=now-lastMs_;
            if(phase_==Phase::POST) {if(elapsed_>=locked_.postMs) {phase_=Phase::DONE;reason_="DONE";}}
            else {
                bool passed=false;
                if(phase_==Phase::RETURN_RIGHT) {
                    const bool centered=observation.roadValid && std::abs(observation.roadCenter-50)<=locked_.returnCenterTolerance;
                    roadHits_=centered && (roadHits_==0 || std::abs(observation.roadCenter-previousRoad_)<=5)?roadHits_+1:centered?1:0;
                    previousRoad_=observation.roadCenter;passed=roadHits_>=2;
                    reason_=passed?"ROAD_STABLE":"SEEK_ROAD";
                } else {
                    const auto candidate=choose(observation,true,false);
                    if(candidate) {
                        target_=*candidate;tracking_=true;missing_=0;++targetHits_;
                        if(targetHits_>=2 && (target_.nearClipped || target_.bottom()>=locked_.passNearYPercent)) {
                            nearSeen_=true;nearAtMotionMs_=motionBudgetMs_;
                        }
                        reason_=nearSeen_?"NEAR_TRACKED":"TRACKING";
                    } else {
                        if(tracking_) {
                            ++missing_;
                            if(!nearSeen_) {targetHits_=0;if(missing_>=2) tracking_=false;}
                        }
                        passed=nearSeen_ && missing_>=2 && motionBudgetMs_>nearAtMotionMs_;
                        reason_=passed?"CONE_PASSED":nearSeen_?"PASS_CONFIRMING":"SEEK_CURRENT_CONE";
                    }
                }
                if(passed) advance(now);
            }
        }
        if(inRoute() && motionBudgetMs_>=locked_.maxMs[int(phase_)-1]) {
            phase_=Phase::FAULT;reason_="VISUAL_STAGE_TIMEOUT";clearEvidence();
        }
        lastMs_=now;hasClock_=true;previousAllowed_=allowed && observation.valid;
        previousDriveAllowed_=previousAllowed_;
        previousMotionLimitMs_=forcing()?std::min(150LL,pulseRemainingMs()):150;
    }
    bool used() const{return used_;}
    bool inRoute() const{return phase_>=Phase::LEFT && phase_<=Phase::RETURN_RIGHT;}
    bool forcing() const{return inRoute() && motionBudgetMs_<locked_.pulseMs[int(phase_)-1];}
    bool guiding() const{return inRoute() && !forcing();}
    bool active() const{return inRoute() || phase_==Phase::POST;}
    void reportDriveAllowed(bool allowed){previousDriveAllowed_=previousDriveAllowed_ && allowed;}
    bool routeComplete() const{return phase_==Phase::POST || phase_==Phase::DONE;}
    bool needsStop() const{return phase_==Phase::FAULT;}
    long long releaseAtMs() const{return releaseAtMs_;}
    Phase phase() const{return phase_;}
    const Settings& settings() const{return locked_;}
    const std::string& entryReason() const{return entryReason_;}
    const std::string& reason() const{return reason_;}
    bool nearSeen() const{return nearSeen_;}
    long long remainingMs() const{return active()?std::max(0LL,phase_==Phase::POST?locked_.postMs-elapsed_:locked_.maxMs[int(phase_)-1]-motionBudgetMs_):0;}
    long long pulseRemainingMs() const{return forcing()?locked_.pulseMs[int(phase_)-1]-motionBudgetMs_:0;}
    const Candidate* currentTarget() const{return tracking_ && targetHits_>=2 && missing_==0?&target_:nullptr;}
    double error() const{return forcing()?(int(phase_)%2?-1:1)*locked_.errors[int(phase_)-1]:0;}
    double speed() const{return inRoute()?locked_.zoneSpeed:locked_.postSpeed;}
};
} }
