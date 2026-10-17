#pragma once
#include "Parking/AutoParking.hpp"
#include "Contral/Steer/SteeringPulse.hpp"

namespace AutoParking {
// Validate every stored command, including disabled corrections, before hardware init.
// This is an electrical duty check, not a mechanical steering-range certification.
inline void validateSteeringCommands(const Config& config,double offset) {
    for(int i=0;i<6;++i) {
        const auto check=[&](double command,const char* key) {
            float duty;
            if(!Contral::SteeringPulse::commandToDuty(command,offset,duty))
                throw std::runtime_error("stage_"+std::to_string(i+1)+" "+key+"="+
                    std::to_string(command)+" with steer_offset_deg="+std::to_string(offset)+
                    " produces invalid servo duty_ns="+std::to_string(duty)+
                    "; require finite 0.."+std::to_string(Contral::SteeringPulse::periodNs));
        };
        check(config.stages[i].steer,"steer_command");
        check(config.stages[i].correction,"correction_steer_command");
    }
}
}
