#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include "ZebraTest/ZebraDetector.hpp"
#include <opencv2/videoio.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fstream>
#include <iostream>
namespace Other {
double RuntimeConfig::getCenterPratio(){return .5;}
double RuntimeConfig::getErrorGain(){return 4;}
double RuntimeConfig::getMotorAimSpeed(){return 9;}
}
namespace ImageProcess {
bool FixLine::checkZebraSimple(){throw std::runtime_error("legacy zebra called");}
FixLineState FixLine::handleZebraSimple(){throw std::runtime_error("legacy zebra called");}
}
int main(int argc,char** argv) {
    if(argc!=4){std::cerr<<"zebra_replay VIDEO CONFIG OUTPUT.csv (run beside image_params.txt)\n";return 2;}
    try {
        auto settings=ZebraTest::loadSettings(argv[2]);cv::VideoCapture cap;
        std::ifstream manifest;double fps=0;
        const std::string input=argv[1];
        if(input.size()>=7&&input.substr(input.size()-7)==".frames") {manifest.open(input);manifest>>fps;std::string rest;std::getline(manifest,rest);}
        else {cap.open(input);fps=cap.get(cv::CAP_PROP_FPS);}
        if(fps<=0){std::cerr<<"Cannot decode video/manifest; for codec-free host replay use .frames manifest\n";return 2;}
        std::ofstream out(argv[3]);out<<"frame,time_ms,lane_valid,input_valid,near_valid,near_hit,strong,far_hit,stripes,rows,contrast,nearest_y,reason,simulated_phase,simulated_count\n";
        NewTrack::reset();NewTrack::setZebraValidationMode(true,settings.illumination_band_rows);cv::Mat source,frame;int i=0,hit=0,first=-1,last=-1;ZebraTest::Controller controller(settings);
        for(;;) {
            if(manifest.is_open()){std::string path;if(!std::getline(manifest,path))break;source=cv::imread(path);if(source.empty())throw std::runtime_error("cannot read manifest image");}
            else if(!cap.read(source))break;
            cv::resize(source,frame,{80,60},0,0,cv::INTER_NEAREST);const auto time=static_cast<long long>(i*1000/fps);
            auto nt=NewTrack::processFrameAt(frame,false,time);
            auto e=ZebraTest::detect(nt.validationGray,nt.validationRaw,nt.validationLeft,nt.validationRight,nt.valid,nt.binaryQuality,time,settings);
            controller.observe(e);
            const auto phase=controller.phase();
            const double speed=(phase==ZebraTest::Phase::Settle||phase==ZebraTest::Phase::Hold||phase==ZebraTest::Phase::LanePause)?0:7;
            const auto command=controller.step({time,speed,speed,true,false});
            if(command.override_pwm)controller.acknowledge(time,true);
            out<<i<<','<<time<<','<<nt.valid<<','<<e.input_valid<<','<<e.near.valid<<','<<e.near.hit<<','<<e.near.strong<<','<<e.far.hit<<','<<e.near.max_stripes<<','<<e.near.hit_rows<<','<<e.near.contrast<<','<<e.near.nearest_y<<','<<e.near.reason<<','<<ZebraTest::phaseName(controller.phase())<<','<<controller.count()<<'\n';
            if(e.near.hit){++hit;if(first<0)first=i;last=i;}++i;
        }
        std::cout<<"frames="<<i<<" hits="<<hit<<" first="<<first<<" simulated_events="<<controller.count()<<" last="<<last<<'\n';return out?0:2;
    }catch(const std::exception& e){std::cerr<<e.what()<<'\n';return 2;}
}
