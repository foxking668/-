#include "core.hpp"
#include <iomanip>
#include <iostream>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

using namespace car2026;
int main(int argc,char** argv) {
    try {
        if(argc!=2 && argc!=3) {std::cerr<<"Usage: vision_stream config/competition.ini [perception_stage]\n";return 2;}
#ifdef _WIN32
        _setmode(_fileno(stdin),_O_BINARY);
#endif
        Params params=Params::load(argv[1]);Vision vision(params);Mission mission(params);
        Stage perceptionStage=Stage::Depart;
        if(argc==3) {
            bool found=false;
            for(int n=0;n<=int(Stage::Fault);++n) if(std::string(argv[2])==stageName(Stage(n))) {perceptionStage=Stage(n);found=true;}
            if(!found) throw std::runtime_error("Unknown perception stage");
        }
        Telemetry telemetry;double previousTime=-1;
        // Each frame: telemetry line, P6 header, exactly width*height*3 RGB bytes.
        // Telemetry: time absolute_distance speed unwrapped_yaw yaw_valid yaw_measured.
        while(true) {
            std::string line;if(!std::getline(std::cin,line)) break;
            std::istringstream input(line);
            if(!(input>>telemetry.time>>telemetry.distance>>telemetry.speed>>telemetry.yaw>>telemetry.yawValid>>telemetry.yawMeasured)) throw std::runtime_error("Bad telemetry header");
            if(telemetry.time<previousTime) throw std::runtime_error("Replay timestamp moved backwards");
            previousTime=telemetry.time;
            std::string magic;int w,h,maxValue;
            if(!(std::cin>>magic>>w>>h>>maxValue)||magic!="P6"||w<32||h<24||w>1920||h>1080||maxValue!=255) throw std::runtime_error("Bad PPM frame header");
            if(std::cin.get()!='\n') throw std::runtime_error("PPM header must end with newline");
            Image image(w,h);
            static_assert(sizeof(Pixel)==3,"RGB pixels must be packed");
            std::cin.read(reinterpret_cast<char*>(image.pixels.data()),w*h*3);
            if(std::cin.gcount()!=w*h*3) throw std::runtime_error("Truncated frame");
            auto observation=vision.analyze(image,argc==3?perceptionStage:mission.stage());
            auto avoidance=vision.avoidCones(observation,image);
            Command command;
            if(argc==3) {command.stage=perceptionStage;command.reason="PERCEPTION_ONLY";}
            else command=mission.update(observation,telemetry,avoidance);
            std::cout<<std::defaultfloat<<std::setprecision(17)<<"{\"time\":"<<telemetry.time
                     <<std::fixed<<std::setprecision(5)<<",\"stage\":\""<<stageName(command.stage)
                     <<"\",\"laps\":"<<command.completedLaps<<",\"speed\":"<<command.speed<<",\"steer\":"<<command.steer
                     <<",\"line_confidence\":"<<observation.blackPath.confidence<<",\"road_confidence\":"<<observation.roadPath.confidence
                     <<",\"cones\":"<<observation.cones.size()<<",\"ring\":"<<observation.ringCandidate<<",\"stripe\":"<<observation.stripe
                     <<",\"stripe_y\":"<<observation.stripeY<<",\"bar\":"<<observation.bar<<",\"bar_observable\":"<<observation.barObservable
                     <<",\"garage\":"<<observation.garage<<",\"speak\":"<<command.speak<<",\"reason\":\""<<command.reason<<"\",\"path\":[";
            for(size_t i=0;i<observation.blackPath.x.size();++i) {if(i) std::cout<<',';std::cout<<observation.blackPath.x[i];}
            std::cout<<"],\"road_path\":[";
            for(size_t i=0;i<observation.roadPath.x.size();++i) {if(i) std::cout<<',';std::cout<<observation.roadPath.x[i];}
            std::cout<<"]}\n"<<std::flush;
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
