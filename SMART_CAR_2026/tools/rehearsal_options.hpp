#pragma once
#include <set>
#include <stdexcept>
#include <string>
namespace car2026 { namespace capture {
struct RehearsalOptions {
    std::string capture="manual_capture.ini",hardware="config/calibration_hardware.ini",vehicle="config/calibration_vehicle.ini",
        tuning="config/parking_tuning.ini",output="captures/parking_tuning";
    bool help=false,check=false,partial=false,ccSpeed=false,upgradeCcTuning=false,pwmControl=false,noCamera=false;
    static RehearsalOptions parse(int argc,char** argv) {
        RehearsalOptions result;std::set<std::string> seen;
        for(int i=1;i<argc;++i) {
            const std::string key=argv[i];
            if(!seen.insert(key).second) throw std::runtime_error("Duplicate option: "+key);
            if(key=="--help") result.help=true;
            else if(key=="--check-config") result.check=true;
            else if(key=="--allow-partial") result.partial=true;
            else if(key=="--no-camera") result.noCamera=true;
            else if(key=="--sysfs-pwm-control") {result.pwmControl=true;result.ccSpeed=true;}
            else if(key=="--cc-motor-control") result.ccSpeed=true;
            else if(key=="--upgrade-cc-tuning") result.upgradeCcTuning=true;
            else {
                std::string* target=nullptr;
                if(key=="--config") target=&result.capture;
                else if(key=="--hardware-config") target=&result.hardware;
                else if(key=="--vehicle-config") target=&result.vehicle;
                else if(key=="--tuning-config") target=&result.tuning;
                else if(key=="--output") target=&result.output;
                if(!target || i+1>=argc) throw std::runtime_error("Unknown option or missing value: "+key);
                *target=argv[++i];
                if(target->empty() || target->rfind("--",0)==0) throw std::runtime_error("Missing option value: "+key);
            }
        }
        return result;
    }
};
}}
