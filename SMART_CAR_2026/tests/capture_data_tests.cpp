#include "capture_data.hpp"
#include <cstdio>
#include <chrono>
#include <iostream>
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
template<class F> bool rejects(F operation) {try {operation();return false;}catch(const std::exception&) {return true;}}
}
int main() {
    try {
        SensorSpec encoder{"encoder_left","/dev/zf_encoder_1","i16le","raw_count"};
        auto value=decode(encoder,2,{0xff,0xff});check(value.valid && value.value==-1,"signed reverse count");
        check(decode(encoder,2,{0,0x80}).value==-32768,"signed lower bound");
        check(decode(encoder,2,{0xff,0x7f}).value==32767,"signed upper bound");
        check(!decode(encoder,0,{0x34,0x12}).valid,"zero return with changed buffer is not data");
        check(decode(encoder,0,{0x34,0x12}).raw=="3412","preserve zero-return diagnostic bytes");
        check(!decode(encoder,1,{0x34,0xa5}).valid,"short count rejected");
        check(!decode(encoder,2,{0x34}).valid,"buffer bounds checked");
        check(!decode(encoder,-1,{0xa5,0xa5}).valid,"failed read rejected");
        SensorSpec encoder32{"encoder_left","/dev/zf_encoder_1","i32le","raw_count"};
        check(readBufferSize(encoder32)==4,"four-byte encoder transfer size");
        check(decode(encoder32,4,{0xff,0xff,0xff,0xff}).value==-1,"signed 32-bit reverse count");
        check(decode(encoder32,4,{0,0,0,0x80}).value==-2147483648.0,"signed 32-bit lower bound");
        check(decode(encoder32,4,{0xff,0xff,0xff,0x7f}).value==2147483647.0,"signed 32-bit upper bound");
        check(decode(encoder32,4,{0,0,1,0}).value==65536,"high encoder bytes preserved");
        check(!decode(encoder32,2,{0xff,0xff,0xa5,0xa5}).valid,"short 32-bit count rejected");
        check(!decode(encoder32,4,{0xff,0xff}).valid,"32-bit buffer bounds checked");
        check(!decode(encoder32,0,{1,0,0,0}).valid,"32-bit zero return rejected");
        check(Config{}.sensors[0].format=="i32le","shared-project encoder default is 32-bit");
        check(!Config{}.factoryEncoderStatusZero,"generic capture defaults to strict byte-count reads");
        Config factoryConfig;factoryConfig.factoryEncoderStatusZero=true;
        check(validateSensors(factoryConfig,true).size()==8,"factory encoder mode accepts four-byte payloads");
        factoryConfig.sensors[0].format="i16le";
        check(rejects([&]{validateSensors(factoryConfig,true);}),"factory encoder ABI cannot request two bytes");
        Config encoderConfig;encoderConfig.sensors[0]=encoder32;
        check(validateSensors(encoderConfig,true).size()==7,"32-bit sensor format accepted");
        SensorSpec text{"rear","/sys/example","text","cm"};
        check(decode(text,4,{'1','2','.','5',0xa5}).value==12.5,"numeric text decoded");
        check(!decode(text,3,{'n','a','n',0xa5}).valid,"NaN rejected");
        check(!decode(text,3,{'1','2','x',0xa5}).valid,"trailing text rejected");
        check(!decode(text,3,{'1','2','3'}).valid,"full text buffer rejected");
        SensorSpec gpio{"gray","/dev/verified_gpio","u8","raw_byte"};
        check(decode(gpio,1,{0}).valid,"real zero is valid");
        check(decode(gpio,1,{255}).value==255,"GPIO raw byte preserved without polarity guess");
        Config config;
        check(rejects([&]{validateSensors(config,false);}),"missing interfaces cannot masquerade as complete");
        check(validateSensors(config,true).size()==8,"partial has explicit missing names");
        config.sensors[0]=encoder;
        config.sensors[1]=encoder;
        check(rejects([&]{validateSensors(config,true);}),"duplicate delta path rejected");
        config.sensors[1].path="";config.sensors[0].path="/tmp/fake";
        check(rejects([&]{validateSensors(config,true);}),"unverified path rejected");
        check(rejects([&]{finiteNumber("inf");}),"infinite interval rejected");
        check(csvString("error,\"quoted\"")=="\"error,\"\"quoted\"\"\"","CSV escaped");
        std::istringstream emptyOutput;
        check(rejects([&]{verifyTextOutput(emptyOutput,"header");}),"empty output cannot be reported as saved");
        std::istringstream wrongOutput("wrong_header\nrow\n");
        check(rejects([&]{verifyTextOutput(wrongOutput,"header");}),"wrong output header rejected");
        std::istringstream rowsOutput("header\nrow1\nrow2\n");
        check(verifyTextOutput(rowsOutput,"header")==2,"saved row count verified against actual contents");
        std::istringstream markersOutput("header\n");
        check(verifyTextOutput(markersOutput,"header")==0,"header-only marker file is valid");
        std::istringstream badOutput("header\nrow\n");badOutput.setstate(std::ios::badbit);
        check(rejects([&]{verifyTextOutput(badOutput,"header");}),"readback I/O failure rejected");
        const std::string path="capture_config_test_"+std::to_string(
            std::chrono::steady_clock::now().time_since_epoch().count())+".ini";
        {std::ofstream file(path);file<<"camera_width=1280\ncamera_height=720\ncamera_fps=20\nsample_period_ms=20\n";}
        check(loadConfig(path).width==1280,"config parses");
        {std::ofstream file(path);file<<"factory_encoder_status_zero=1\n";}
        check(loadConfig(path).factoryEncoderStatusZero,"explicit factory ABI setting parses");
        {std::ofstream file(path);file<<"factory_encoder_status_zero=0.5\n";}
        check(rejects([&]{loadConfig(path);}),"fractional factory ABI flag rejected");
        {std::ofstream file(path);file<<"factory_encoder_status_zero=2\n";}
        check(rejects([&]{loadConfig(path);}),"out-of-range factory ABI flag rejected");
        {std::ofstream file(path);file<<"camera_fps=20\ncamera_fps=25\n";}
        check(rejects([&]{loadConfig(path);}),"duplicate setting rejected");
        {std::ofstream file(path);file<<"camera_width=1280.5\n";}
        check(rejects([&]{loadConfig(path);}),"fractional dimension rejected");
        {std::ofstream file(path);file<<"sample_period_ms=nan\n";}
        check(rejects([&]{loadConfig(path);}),"nonfinite setting rejected");
        {std::ofstream file(path);file<<"unknown_sensor=1\n";}
        check(rejects([&]{loadConfig(path);}),"misspelled settings rejected");
        std::remove(path.c_str());
        std::cout<<"PASS "<<checks<<" capture-data checks\n";return 0;
    } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
