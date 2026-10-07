#pragma once
#include "hardware.hpp"
#include <set>

namespace car2026 {
struct BenchOptions {
    std::string mode="interfaces",hardwareFile="config/hardware.ini",paramsFile="config/competition.ini",wheel="both";
    double duration=15,interval=100,turns=0,rawDuty=0,pulseDuration=300;
    bool help=false;
    static BenchOptions parse(const std::vector<std::string>& args) {
        BenchOptions result;std::set<std::string> seen;
        int modes=0;bool raised=false,rawSpecified=false,wheelSpecified=false,durationSpecified=false,encoderOptions=false;
        auto number=[](const std::string& text) {
            size_t used=0;const double value=std::stod(text,&used);
            if(used!=text.size() || !std::isfinite(value)) throw std::runtime_error("Invalid numeric argument: "+text);
            return value;
        };
        for(size_t i=0;i<args.size();++i) {
            const auto& arg=args[i];
            if(arg=="--help") {result.help=true;return result;}
            if(!seen.insert(arg).second) throw std::runtime_error("Duplicate bench option: "+arg);
            if(arg=="--interfaces" || arg=="--imu" || arg=="--encoders" || arg=="--motor-zero" || arg=="--motor-pulse") {result.mode=arg.substr(2);++modes;continue;}
            if(arg=="--wheels-raised") {raised=true;continue;}
            if(i+1>=args.size()) throw std::runtime_error("Missing value: "+arg);
            const auto& value=args[++i];
            if(arg=="--hardware-config") result.hardwareFile=value;
            else if(arg=="--config") result.paramsFile=value;
            else if(arg=="--wheel") {result.wheel=value;wheelSpecified=true;}
            else if(arg=="--raw-duty") {result.rawDuty=number(value);rawSpecified=true;}
            else if(arg=="--duration-ms") {result.pulseDuration=number(value);durationSpecified=true;}
            else if(arg=="--seconds") {result.duration=number(value);encoderOptions=true;}
            else if(arg=="--interval-ms") {result.interval=number(value);encoderOptions=true;}
            else if(arg=="--turns") {result.turns=number(value);encoderOptions=true;}
            else throw std::runtime_error("Unknown argument: "+arg);
        }
        if(modes>1) throw std::runtime_error("Choose exactly one bench mode");
        if(result.wheel!="left" && result.wheel!="right" && result.wheel!="both") throw std::runtime_error("wheel must be left, right or both");
        if(result.mode!="motor-pulse" && (raised || rawSpecified || durationSpecified)) throw std::runtime_error("Powered options require --motor-pulse");
        if(result.mode!="encoders" && encoderOptions) throw std::runtime_error("Encoder options require --encoders");
        if((result.mode=="interfaces" || result.mode=="imu" || result.mode=="motor-zero") && wheelSpecified) throw std::runtime_error("wheel selection requires an encoder/motor pulse test");
        if(result.duration<1 || result.duration>120 || result.interval<10 || result.interval>1000 || std::floor(result.interval)!=result.interval || result.turns<0 || result.turns>20 || std::floor(result.turns)!=result.turns)
            throw std::runtime_error("Encoder bounds: seconds 1..120, integer interval-ms 10..1000, integer turns 0..20");
        if(result.turns>0 && (!wheelSpecified || result.wheel=="both")) throw std::runtime_error("Declared turns require one selected wheel");
        if(result.mode=="motor-pulse" && (!raised || !rawSpecified || !wheelSpecified || !durationSpecified || std::abs(result.rawDuty)>2000 || std::floor(result.rawDuty)!=result.rawDuty || result.pulseDuration<50 || result.pulseDuration>500 || std::floor(result.pulseDuration)!=result.pulseDuration))
            throw std::runtime_error("Motor test requires --wheels-raised, explicit wheel/raw-duty/duration-ms; integer raw -2000..2000 and duration 50..500ms");
        return result;
    }
};
} // namespace car2026
