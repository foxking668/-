#pragma once
#include <array>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iomanip>
#include <map>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace car2026 { namespace capture {
inline std::string trimmed(const std::string& text) {
    const auto begin=text.find_first_not_of(" \t\r\n");
    return begin==std::string::npos ? "" : text.substr(begin,text.find_last_not_of(" \t\r\n")-begin+1);
}
inline double finiteNumber(const std::string& text) {
    const auto value=trimmed(text);size_t used=0;
    const double number=std::stod(value,&used);
    if(used!=value.size() || !std::isfinite(number)) throw std::runtime_error("Invalid number: "+text);
    return number;
}
struct SensorSpec {std::string name,path,format,unit;};
struct Config {
    int width=1280,height=720;double fps=20,sampleMs=20;
    bool factoryEncoderStatusZero=false;
    std::array<SensorSpec,8> sensors{{
        {"encoder_left","","i32le","raw_count"}, {"encoder_right","","i32le","raw_count"},
        {"gray_1","","u8","raw_byte"}, {"gray_2","","u8","raw_byte"},
        {"gray_3","","u8","raw_byte"}, {"gray_4","","u8","raw_byte"},
        {"ultrasonic_front","","text",""}, {"ultrasonic_rear","","text",""}}};
};
inline Config loadConfig(const std::string& filename) {
    std::ifstream file(filename);if(!file) throw std::runtime_error("Cannot open capture config: "+filename);
    Config config;std::set<std::string> seen;std::string line;int lineNumber=0;
    while(std::getline(file,line)) {
        ++lineNumber;
        if(lineNumber==1 && line.compare(0,3,"\xef\xbb\xbf")==0) line.erase(0,3);
        line=trimmed(line.substr(0,line.find('#')));if(line.empty()) continue;
        auto equal=line.find('=');if(equal==std::string::npos) throw std::runtime_error("Missing '=' in capture config");
        auto key=trimmed(line.substr(0,equal)),value=trimmed(line.substr(equal+1));
        if(!seen.insert(key).second) throw std::runtime_error("Duplicate capture key: "+key);
        bool known=true;
        if(key=="camera_width" || key=="camera_height") {
            auto number=finiteNumber(value);
            if(number<16 || number>8192 || std::floor(number)!=number) throw std::runtime_error("Invalid camera dimensions");
            if(key=="camera_width") config.width=int(number);else config.height=int(number);
        } else if(key=="camera_fps") config.fps=finiteNumber(value);
        else if(key=="sample_period_ms") config.sampleMs=finiteNumber(value);
        else if(key=="factory_encoder_status_zero") {
            const auto number=finiteNumber(value);
            if(number!=0 && number!=1) throw std::runtime_error("factory_encoder_status_zero must be 0 or 1");
            config.factoryEncoderStatusZero=number==1;
        }
        else {
            known=false;
            for(auto& sensor:config.sensors) {
                if(key==sensor.name+"_path") {sensor.path=value;known=true;}
                if(key==sensor.name+"_format") {sensor.format=value;known=true;}
                if(key==sensor.name+"_unit") {sensor.unit=value;known=true;}
            }
        }
        if(!known) throw std::runtime_error("Unknown capture key: "+key);
    }
    if(config.fps<1 || config.fps>120 || config.sampleMs<10 || config.sampleMs>1000)
        throw std::runtime_error("Camera FPS/sample interval outside allowed range");
    return config;
}
inline std::vector<std::string> validateSensors(const Config& config,bool allowPartial) {
    std::vector<std::string> missing;std::set<std::string> paths;
    if(config.factoryEncoderStatusZero) {
        for(size_t index=0;index<2;++index)
            if(config.sensors[index].format!="i32le")
                throw std::runtime_error("Factory encoder status-zero mode requires i32le payloads");
    }
    for(const auto& sensor:config.sensors) {
        if(sensor.path.empty()) {missing.push_back(sensor.name);continue;}
        if(sensor.path.rfind("/dev/",0)!=0 && sensor.path.rfind("/sys/",0)!=0)
            throw std::runtime_error("Use a verified /dev or /sys sensor path: "+sensor.name);
        if(sensor.format!="i16le" && sensor.format!="i32le" && sensor.format!="u8" && sensor.format!="text")
            throw std::runtime_error("Unsupported sensor format: "+sensor.name);
        if(sensor.unit.empty()) throw std::runtime_error("Declare actual sensor units: "+sensor.name);
        if(!paths.insert(sensor.path).second) throw std::runtime_error("Duplicate sensor path would consume data twice: "+sensor.path);
    }
    if(!missing.empty() && !allowPartial) {
        std::string message="Unconfigured sensors:";
        for(const auto& name:missing) message+=' '+name;
        throw std::runtime_error(message+". Verify interfaces first; --allow-partial explicitly permits an incomplete capture.");
    }
    return missing;
}
struct Sample {bool valid=false;double value=0;long returned=-1;std::string status,raw;};
inline size_t readBufferSize(const SensorSpec& sensor) {
    if(sensor.format=="i32le") return 4;
    if(sensor.format=="i16le") return 2;
    if(sensor.format=="u8") return 1;
    if(sensor.format=="text") return 256;
    throw std::runtime_error("Unsupported sensor format: "+sensor.name);
}
inline Sample decode(const SensorSpec& sensor,long returned,const std::vector<uint8_t>& bytes) {
    Sample result;result.returned=returned;
    std::ostringstream hex;hex<<std::hex<<std::setfill('0');
    for(auto value:bytes) hex<<std::setw(2)<<unsigned(value);
    result.raw=hex.str();
    if(returned<0) {result.status="read_failed";return result;}
    if(sensor.format!="i16le" && sensor.format!="i32le" && sensor.format!="u8" && sensor.format!="text") {
        result.status="unsupported_format";return result;
    }
    const size_t expected=readBufferSize(sensor);
    if(returned==0 || size_t(returned)>bytes.size() ||
       (sensor.format!="text" && size_t(returned)!=expected)) {
        result.status="invalid_transfer_count";return result;
    }
    try {
        if(sensor.format=="text") {
            if(size_t(returned)==bytes.size()) {result.status="text_too_long";return result;}
            result.value=finiteNumber(std::string(bytes.begin(),bytes.begin()+returned));
        } else if(sensor.format=="i16le") {
            const unsigned raw=unsigned(bytes[0])|(unsigned(bytes[1])<<8);
            result.value=raw>=32768 ? int(raw)-65536 : int(raw);
        } else if(sensor.format=="i32le") {
            const uint32_t raw=uint32_t(bytes[0])|(uint32_t(bytes[1])<<8)|
                (uint32_t(bytes[2])<<16)|(uint32_t(bytes[3])<<24);
            result.value=raw>=0x80000000u ? int64_t(raw)-4294967296LL : int64_t(raw);
        } else if(sensor.format=="u8") result.value=bytes[0];
        else {result.status="unsupported_format";return result;}
        result.valid=true;result.status="ok";
    } catch(const std::exception&) {result.status="invalid_value";}
    return result;
}
inline std::string csvString(const std::string& text) {
    std::string result="\"";
    for(char value:text) {if(value=='"') result+='"';result+=value;}
    return result+'"';
}
// A successful capture loop alone cannot prove that its files contain data.
inline uint64_t verifyTextOutput(std::istream& input,const std::string& firstLine) {
    std::string line;
    if(!std::getline(input,line) || line!=firstLine)
        throw std::runtime_error("Output file is empty or has an unexpected first line");
    uint64_t rows=0;
    while(std::getline(input,line)) {if(!line.empty()) ++rows;}
    if(input.bad() || !input.eof()) throw std::runtime_error("Output file readback failed");
    return rows;
}
}} // namespace car2026::capture
