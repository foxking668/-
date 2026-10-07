#pragma once
#include "capture_data.hpp"
#include "visual_observer.hpp"
#include <optional>

namespace car2026 { namespace capture {
struct Options {
    std::string configFile="manual_capture.ini",hardwareFile="config/hardware.ini",output="captures";
    std::string vehicleFile="config/calibration_vehicle.ini";
    double duration=180;
    bool help=false,allowPartial=false,check=false,checkOutput=false;
    std::optional<double> steerCommand;
    std::optional<ManualMotion> observeSteering;
    bool steeringProbe=false,autoProbe=false;
    bool isProbe() const {return steeringProbe || autoProbe;}
    static Options parse(const std::vector<std::string>& args) {
        Options result;std::set<std::string> seen;bool vehicleSpecified=false;
        for(size_t i=0;i<args.size();++i) {
            const auto& arg=args[i];
            if(arg=="--help") {result.help=true;return result;}
            if(!seen.insert(arg).second) throw std::runtime_error("Duplicate capture option: "+arg);
            if(arg=="--allow-partial") {result.allowPartial=true;continue;}
            if(arg=="--check-config") {result.check=true;continue;}
            if(arg=="--check-output") {result.checkOutput=true;continue;}
            if(i+1>=args.size()) throw std::runtime_error("Missing option value: "+arg);
            const auto& value=args[++i];
            if(arg=="--config") result.configFile=value;
            else if(arg=="--hardware-config") result.hardwareFile=value;
            else if(arg=="--output") result.output=value;
            else if(arg=="--duration") result.duration=finiteNumber(value);
            else if(arg=="--vehicle-config") {result.vehicleFile=value;vehicleSpecified=true;}
            else if(arg=="--steer-command") result.steerCommand=finiteNumber(value);
            else if(arg=="--observe-steering") result.observeSteering=parseManualMotion(value);
            else if(arg=="--steering-probe" || arg=="--auto-probe") {
                if(value!="reverse") throw std::runtime_error("Steering probe currently requires reverse");
                if(arg=="--auto-probe") result.autoProbe=true;else result.steeringProbe=true;
            }
            else throw std::runtime_error("Unknown option: "+arg);
        }
        if(result.duration<=0 || result.duration>3600)
            throw std::runtime_error("duration must be between 0 and 3600 seconds");
        if(result.check && result.checkOutput) throw std::runtime_error("Choose one check mode");
        if(int(result.steerCommand.has_value())+int(result.observeSteering.has_value())+
           int(result.steeringProbe)+int(result.autoProbe)>1)
            throw std::runtime_error("Observation, fixed steering and steering probe are mutually exclusive");
        if(vehicleSpecified && !result.steerCommand && !result.observeSteering && !result.isProbe())
            throw std::runtime_error("--vehicle-config requires an explicit observation or steering mode");
        if(result.isProbe() && (result.duration>60 || result.checkOutput))
            throw std::runtime_error("Steering probe requires --duration <=60 and forbids --check-output");
        if(result.observeSteering && (result.duration>60 || result.checkOutput))
            throw std::runtime_error("Observation requires --duration <=60 and forbids --check-output");
        if(result.steerCommand) {
            if(std::abs(*result.steerCommand)>15)
                throw std::runtime_error("Manual steering capture command must be within -15..15");
            if(result.duration>60)
                throw std::runtime_error("Explicit --duration <=60 is required for steering capture");
            if(result.checkOutput) throw std::runtime_error("Output-only check cannot include steering options");
        }
        return result;
    }
};
}}
