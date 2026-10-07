#include "hardware_linux.hpp"
#include "servo_output.hpp"
#include <csignal>
#include <iostream>
#include <poll.h>
#include <unistd.h>
using namespace car2026;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signalHandler(int) {interrupted=1;}
double number(const std::string& value) {
    size_t used=0;const double result=std::stod(value,&used);
    if(used!=value.size() || !std::isfinite(result)) throw std::runtime_error("Invalid angle: "+value);
    return result;
}
class ServoTest {
public:
    ServoTest(DeviceIo& io,const HardwareConfig& config,const Params& p)
        : output_(io,config,p) {set(0);}
    void set(double angle) {
        const auto duty=output_.set(angle);const auto& info=output_.info();
        std::cout<<"steer="<<angle<<" deg, duty="<<duty<<'/'<<info.duty_max<<", frequency="<<info.freq<<" Hz\n";
    }
private:ServoOutput output_;
};
}
int main(int argc,char** argv) {
    try {
        std::string configFile="config/competition.ini",hardwareFile="config/hardware.ini";
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") {std::cout<<"hardware_servo_test [--config file] [--hardware-config file] [--offset DEG] [--sign -1|1] [--max DEG]\nOnly the factory servo device is written; motors/encoders are not initialized.\n";return 0;}
            if(i+1>=argc) throw std::runtime_error("Missing value: "+arg);
            const std::string value=argv[++i];
            if(arg=="--config") configFile=value;
            else if(arg=="--hardware-config") hardwareFile=value;
            else if(arg!="--offset" && arg!="--sign" && arg!="--max") throw std::runtime_error("Unknown argument: "+arg);
        }
        auto params=Params::load(configFile);const auto config=HardwareConfig::load(hardwareFile);
        for(int i=1;i+1<argc;i+=2) {
            const std::string arg=argv[i];
            if(arg=="--offset") params.steer_offset_deg=number(argv[i+1]);
            if(arg=="--sign") params.steer_sign=number(argv[i+1]);
            if(arg=="--max") params.max_steer_deg=number(argv[i+1]);
        }
        params.validate();HardwareLock lock;LinuxDeviceIo io(config);
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        ServoTest servo(io,config,params);
        std::cout<<"Enter vehicle angle, c for center, q to quit. Ctrl+C centers the servo.\nsteer> "<<std::flush;
        while(!interrupted) {
            struct pollfd input{STDIN_FILENO,POLLIN,0};
            const int ready=poll(&input,1,100);
            if(ready<0) {if(interrupted) break;throw std::runtime_error("Cannot poll terminal");}
            if(!ready) continue;
            std::string line;if(!std::getline(std::cin,line)) break;
            if(line=="q" || line=="Q") break;
            if(line=="c" || line=="C") servo.set(0);
            else if(!line.empty()) servo.set(number(line));
            std::cout<<"steer> "<<std::flush;
        }
        return 0;
    }catch(const std::exception& e) {std::cerr<<"hardware_servo_test: "<<e.what()<<'\n';return 1;}
}
