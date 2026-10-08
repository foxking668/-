#include "core.hpp"
#include "visual_observer.hpp"
#include "straight_follow.hpp"
#include <iomanip>
#include <iostream>
#include <optional>
#ifdef _WIN32
#include <fcntl.h>
#include <io.h>
#endif

using namespace car2026;
int main(int argc,char** argv) {
    try {
        if(argc!=2 && argc!=3 && argc!=5 && argc!=6) {
            std::cerr<<"Usage: vision_stream config.ini [perception_stage [--observe-steering forward|reverse | --observe-straight target_x target_heading]]\n";return 2;
        }
#ifdef _WIN32
        _setmode(_fileno(stdin),_O_BINARY);
#endif
        Params params=Params::load(argv[1]);Vision vision(params);Mission mission(params);
        Stage perceptionStage=Stage::Depart;
        if(argc>=3) {
            bool found=false;
            for(int n=0;n<=int(Stage::Fault);++n) if(std::string(argv[2])==stageName(Stage(n))) {perceptionStage=Stage(n);found=true;}
            if(!found) throw std::runtime_error("Unknown perception stage");
        }
        std::optional<VisualSteeringObserver> observer;
        std::optional<StraightLineFollower> straightFollower;
        StraightImageTarget straightTarget;
        if(argc==5) {
            if(std::string(argv[3])!="--observe-steering") throw std::runtime_error("Unknown observer option");
            if(perceptionStage!=Stage::GarageAlign && perceptionStage!=Stage::GarageAdvance && perceptionStage!=Stage::GarageReverse)
                throw std::runtime_error("Steering observation requires a parking perception stage");
            observer.emplace(params,parseManualMotion(argv[4]));
        }
        if(argc==6) {
            if(std::string(argv[3])!="--observe-straight") throw std::runtime_error("Unknown straight observation option");
            if(perceptionStage!=Stage::Depart && perceptionStage!=Stage::ToCones &&
               perceptionStage!=Stage::ToRing && perceptionStage!=Stage::ToCross && perceptionStage!=Stage::ToGarage)
                throw std::runtime_error("Straight observation requires an ordinary forward black-line stage");
            const auto number=[](const char* value) {
                const std::string text=value;size_t consumed=0;
                const double result=std::stod(text,&consumed);
                if(consumed!=text.size() || !std::isfinite(result)) throw std::runtime_error("Invalid straight target number");
                return result;
            };
            straightTarget={number(argv[4]),number(argv[5])};
            straightFollower.emplace(params,straightTarget);
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
            auto observation=vision.analyze(image,argc>=3?perceptionStage:mission.stage());
            auto avoidance=vision.avoidCones(observation,image);
            Command command;
            if(argc>=3) {command.stage=perceptionStage;command.reason="PERCEPTION_ONLY";}
            else command=mission.update(observation,telemetry,avoidance);
            std::cout<<std::defaultfloat<<std::setprecision(17)<<"{\"time\":"<<telemetry.time
                     <<std::fixed<<std::setprecision(5)<<",\"stage\":\""<<stageName(command.stage)
                     <<"\",\"laps\":"<<command.completedLaps<<",\"speed\":"<<command.speed<<",\"steer\":"<<command.steer
                     <<",\"line_confidence\":"<<observation.blackPath.confidence<<",\"road_confidence\":"<<observation.roadPath.confidence
                     <<",\"line_ambiguous\":"<<observation.blackPath.ambiguous
                     <<",\"line_discontinuous\":"<<observation.blackPath.discontinuous
                     <<",\"cones\":"<<observation.cones.size()<<",\"ring\":"<<observation.ringCandidate<<",\"stripe\":"<<observation.stripe
                     <<",\"stripe_y\":"<<observation.stripeY<<",\"bar\":"<<observation.bar<<",\"bar_observable\":"<<observation.barObservable
                     <<",\"garage\":"<<observation.garage<<",\"speak\":"<<command.speak<<",\"reason\":\""<<command.reason<<"\",\"path\":[";
            for(size_t i=0;i<observation.blackPath.x.size();++i) {if(i) std::cout<<',';std::cout<<observation.blackPath.x[i];}
            std::cout<<"],\"road_path\":[";
            for(size_t i=0;i<observation.roadPath.x.size();++i) {if(i) std::cout<<',';std::cout<<observation.roadPath.x[i];}
            std::cout<<']';
            if(observer) {
                const auto suggestion=observer->observe(observation,telemetry.time,telemetry.time);
                std::cout<<",\"observer_state\":\""<<suggestion.state<<"\",\"reference_id\":"<<suggestion.referenceId
                         <<",\"has_reference\":"<<suggestion.hasReference<<",\"suggestion_valid\":"<<suggestion.hasSuggestion
                         <<",\"suggested_command\":";
                if(suggestion.hasSuggestion) std::cout<<suggestion.suggestedCommand;else std::cout<<"null";
                std::cout<<",\"lateral_error_image\":";
                if(suggestion.hasSuggestion) std::cout<<suggestion.lateralError;else std::cout<<"null";
                std::cout<<",\"heading_error_image\":";
                if(suggestion.hasSuggestion) std::cout<<suggestion.headingFeatureError;else std::cout<<"null";
                std::cout<<",\"actuator_writes\":0";
            }
            if(straightFollower) {
                const auto result=straightFollower->observe(observation,telemetry.time,telemetry.time);
                std::cout<<",\"straight_state\":\""<<result.state<<"\",\"is_straight\":"<<result.isStraight
                         <<",\"straight_target_x\":"<<straightTarget.nearX
                         <<",\"straight_target_heading_image\":"<<straightTarget.headingFeature
                         <<",\"straight_aligned\":"<<result.aligned<<",\"straight_confirmed_frames\":"<<result.confirmedFrames
                         <<",\"straight_suggestion_valid\":"<<result.hasSuggestion<<",\"straight_suggested_command\":";
                if(result.hasSuggestion) std::cout<<result.suggestedCommand;else std::cout<<"null";
                std::cout<<",\"straight_lateral_error_image\":";
                if(result.hasErrors) std::cout<<result.lateralError;else std::cout<<"null";
                std::cout<<",\"straight_heading_error_image\":";
                if(result.hasErrors) std::cout<<result.headingFeatureError;else std::cout<<"null";
                std::cout<<",\"straight_fit_residual_image\":";
                if(result.isStraight) std::cout<<result.features.maximumResidual;else std::cout<<"null";
                std::cout<<",\"straight_fit_supported_rows\":"<<result.features.supportedRows
                         <<",\"actuator_writes\":0";
            }
            std::cout<<"}\n"<<std::flush;
        }
        return 0;
    } catch(const std::exception& e) {std::cerr<<e.what()<<'\n';return 1;}
}
