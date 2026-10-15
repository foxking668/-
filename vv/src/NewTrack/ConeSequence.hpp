#pragma once
#include <algorithm>
#include <cmath>
#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>

namespace NewTrack { namespace ConeSequence {
struct Settings {
    bool enabled=false;
    int leftMs=500, waitMs=200, rightMs=700, postMs=5500, triggerYPercent=45;
    double leftPratio=.65, leftError=30, rightError=58;
    double leftSpeed=1, waitSpeed=7, rightSpeed=3, postSpeed=7;
};
// Parse transactionally: a malformed/partially saved file never replaces good settings.
inline bool loadFile(const std::string& path, Settings& out, std::string& error) {
    std::ifstream in(path); if(!in) {error="cannot read "+path;return false;}
    const std::map<std::string,std::pair<double,double>> limits={
        {"left_duration_ms",{0,10000}},{"wait_duration_ms",{0,10000}},
        {"right_duration_ms",{0,10000}},{"post_duration_ms",{0,30000}},
        {"left_pratio",{.05,.95}},{"left_min_error",{0,90}},{"right_min_error",{0,90}},
        {"left_speed",{.5,20}},{"wait_speed",{.5,20}},
        {"right_speed",{.5,20}},{"post_speed",{.5,20}},{"trigger_y_percent",{30,85}}};
    std::map<std::string,double> values; std::string line; int number=0;
    auto trim=[](std::string s) {const auto a=s.find_first_not_of(" \t\r\n");
        return a==std::string::npos?std::string():s.substr(a,s.find_last_not_of(" \t\r\n")-a+1);};
    while(std::getline(in,line)) {
        ++number; if(number==1 && line.compare(0,3,"\xef\xbb\xbf")==0) line.erase(0,3);
        line=trim(line.substr(0,line.find('#')));if(line.empty()) continue;
        const auto eq=line.find('=');if(eq==std::string::npos) {error="missing = at line "+std::to_string(number);return false;}
        const auto key=trim(line.substr(0,eq)), text=trim(line.substr(eq+1));
        const auto limit=limits.find(key);
        try {
            std::size_t used=0;const double v=std::stod(text,&used);
            const bool integer=key.find("_ms")!=std::string::npos || key=="trigger_y_percent";
            if(limit==limits.end() || values.count(key) || used!=text.size() || !std::isfinite(v) ||
               v<limit->second.first || v>limit->second.second || (integer && std::floor(v)!=v))
                throw std::runtime_error("invalid value/key");
            values[key]=v;
        } catch(...) {error="invalid/duplicate parameter at line "+std::to_string(number)+": "+key;return false;}
    }
    if(values.size()!=limits.size()) {error="incomplete cone tuning file (12 parameters required)";return false;}
    Settings next=out;
    next.leftMs=int(values["left_duration_ms"]);next.waitMs=int(values["wait_duration_ms"]);
    next.rightMs=int(values["right_duration_ms"]);next.postMs=int(values["post_duration_ms"]);
    next.triggerYPercent=int(values["trigger_y_percent"]);next.leftPratio=values["left_pratio"];
    next.leftError=values["left_min_error"];next.rightError=values["right_min_error"];
    next.leftSpeed=values["left_speed"];next.waitSpeed=values["wait_speed"];
    next.rightSpeed=values["right_speed"];next.postSpeed=values["post_speed"];
    if(next.leftMs+next.rightMs==0) {error="both forced turns are disabled";return false;}
    out=next;error.clear();return true;
}
enum class Phase { IDLE, LEFT, WAIT, RIGHT, POST, DONE };
inline const char* name(Phase p) {switch(p) {
    case Phase::LEFT:return "LEFT";case Phase::WAIT:return "WAIT";case Phase::RIGHT:return "RIGHT";
    case Phase::POST:return "POST";case Phase::DONE:return "DONE";default:return "IDLE";}}
class Controller {
    Settings locked_; int lap_=-1; bool used_=false,previousAllowed_=false,hasClock_=false;
    long long elapsed_=0,lastMs_=0;Phase phase_=Phase::IDLE;
    void refresh() {
        const long long left=locked_.leftMs, wait=left+locked_.waitMs, right=wait+locked_.rightMs;
        phase_=elapsed_<left?Phase::LEFT:elapsed_<wait?Phase::WAIT:
               elapsed_<right?Phase::RIGHT:elapsed_<right+locked_.postMs?Phase::POST:Phase::DONE;
    }
public:
    void reset(){*this=Controller{};}
    void update(long long now,int lap,bool enabled,bool start,bool allowed,const Settings& settings) {
        if(lap!=lap_) {reset();lap_=lap;}
        if(!enabled) {phase_=Phase::IDLE;previousAllowed_=false;hasClock_=false;return;}
        if(start && allowed && !used_) {locked_=settings;used_=true;elapsed_=0;refresh();lastMs_=now;hasClock_=true;}
        else if(hasClock_ && allowed && previousAllowed_ && now>=lastMs_ && now-lastMs_<=150 && active()) {
            elapsed_+=now-lastMs_;refresh();
        }
        lastMs_=now;hasClock_=true;previousAllowed_=allowed;
    }
    bool used() const{return used_;}
    bool steering() const{return phase_==Phase::LEFT || phase_==Phase::WAIT || phase_==Phase::RIGHT;}
    bool active() const{return steering() || phase_==Phase::POST;}
    Phase phase() const{return phase_;}
    const Settings& settings() const{return locked_;}
    long long remainingMs() const {
        long long end=locked_.leftMs;
        if(phase_==Phase::WAIT) end+=locked_.waitMs;
        if(phase_==Phase::RIGHT || phase_==Phase::POST) end+=locked_.waitMs+locked_.rightMs;
        if(phase_==Phase::POST) end+=locked_.postMs;
        return active()?std::max(0LL,end-elapsed_):0;
    }
    double speed() const {return phase_==Phase::LEFT?locked_.leftSpeed:
        phase_==Phase::WAIT?locked_.waitSpeed:phase_==Phase::RIGHT?locked_.rightSpeed:locked_.postSpeed;}
};
} }
