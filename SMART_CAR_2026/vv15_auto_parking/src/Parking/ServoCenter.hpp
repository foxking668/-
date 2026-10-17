#pragma once
#include "Parking/AutoParking.hpp"
#include "Parking/SteeringOutput.hpp"
namespace AutoParking {
namespace ServoCenter {
// Read the existing setting strictly: no silent clamp, duplicate key or NaN in this mode.
inline double readOffset(const std::string& path) {
    std::ifstream file(path);
    if(!file) throw std::runtime_error("Cannot open "+path);
    std::optional<double> offset;std::string line;
    while(std::getline(file,line)) {
        if(line.compare(0,3,"\xEF\xBB\xBF")==0) line.erase(0,3);
        line=Config::trim(line.substr(0,line.find_first_of("#;")));
        const auto eq=line.find('=');
        if(eq==std::string::npos || Config::trim(line.substr(0,eq))!="steer_offset_deg") continue;
        if(offset) throw std::runtime_error("Duplicate steer_offset_deg");
        const double value=Config::number(Config::trim(line.substr(eq+1)));
        if(value< -150 || value>150) throw std::runtime_error("steer_offset_deg must be -150..150");
        offset=value;
    }
    if(file.bad() || !offset) throw std::runtime_error("Missing/unreadable steer_offset_deg");
    return *offset;
}
template<class Motor,class Steer>
bool apply(Motor* motor,Steer* steer,double offset,double command) {
    if(!stopMotorOutputs(motor)) return false;
    if(!std::isfinite(offset) || offset< -150 || offset>150 ||
       (command!=0 && command!=-5 && command!=5)) return false;
    return writeSteeringCommand(steer,command,offset);
}
}
}
