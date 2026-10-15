#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include <cassert>
#include <fstream>
#include <sstream>
#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif
namespace Other {
double RuntimeConfig::getCenterPratio(){return .42;}
double RuntimeConfig::getErrorGain(){return 4;}
double RuntimeConfig::getMotorAimSpeed(){return 9;}
}
namespace ImageProcess {
bool FixLine::checkZebraSimple(){throw std::runtime_error("legacy zebra must not run");}
FixLineState FixLine::handleZebraSimple(){throw std::runtime_error("legacy zebra must not run");}
}
std::vector<std::string> split(const std::string& text) {
    std::stringstream stream(text);std::string item;std::vector<std::string> result;
    while(std::getline(stream,item,',')) result.push_back(item);return result;
}
std::string column(const NewTrack::FrameResult& result,const char* key) {
    const auto names=split(NewTrack::diagnosticCsvHeader()),values=split(result.diagnosticCsv);
    assert(names.size()==values.size());const auto it=std::find(names.begin(),names.end(),key);assert(it!=names.end());return values[it-names.begin()];
}
int main(int argc,char** argv) {
    assert(argc==2);std::ifstream source("cone_avoidance.ini");assert(source.good());std::ostringstream text;text<<source.rdbuf();
#ifdef _WIN32
    assert(_chdir(argv[1])==0);
#else
    assert(chdir(argv[1])==0);
#endif
    std::ofstream("cone_avoidance.ini")<<text.str();
    std::ofstream("image_params.txt")<<"new_track_enable=1\nnew_track_cone_sequence_enable=1\nnew_track_blue_cone_enable=1\n"
        <<"new_track_work_mode=1\nnew_track_debug_print=0\nnew_track_start_right_enable=0\n"
        <<"new_track_top_crop_ratio=0\nnew_track_bottom_crop_ratio=0\nnew_track_ring_require_marker=0\n"
        <<"new_track_ring_startup_block_ms=1000\nnew_track_ring_marker_window_ms=2000\nnew_track_zebra_enable=0\nnew_track_crossing_enable=0\n";
    cv::Mat road(60,80,CV_8UC1,cv::Scalar(15));cv::rectangle(road,{10,0,60,60},{230},-1);
    auto cone=[&](int x,int bottom) {auto image=road.clone();cv::fillConvexPoly(image,
        std::vector<cv::Point>{{x,bottom-22},{x-7,bottom},{x+7,bottom}},cv::Scalar(20));return image;};
    NewTrack::reset();auto r=NewTrack::processFrameAt(road,true,1000);
    auto process=[&](long long t,const cv::Mat& image) {r=NewTrack::processFrameAt(image,true,t);return r;};
    for(int t=1100;t<5000;t+=100) {process(t,cone(57,35));assert(column(r,"cone_phase")=="IDLE" && column(r,"ring_gate_open")=="0");}
    process(5000,cone(57,35));assert(column(r,"cone_phase")=="IDLE");
    process(5033,cone(57,35));assert(column(r,"cone_phase")=="LEFT_1" && column(r,"cone_entry_reason")=="VISUAL_WINDOW");
    assert(r.valid && r.steerError==-30 && column(r,"speed_command")=="9" && column(r,"center_black_used")=="0");
    process(5066,cone(57,48));process(5099,cone(57,54));
    for(int t=5132;t<=5264;t+=33) process(t,road);
    assert(column(r,"cone_phase")=="RIGHT_2" && r.steerError==58);
    auto pass=[&](long long t,int x,const char* expected) {
        process(t,cone(x,33));process(t+33,cone(x,45));process(t+66,cone(x,54));
        for(int dt=99;dt<=231;dt+=33) process(t+dt,road);
        assert(column(r,"cone_phase")==expected && column(r,"speed_command")=="9" && column(r,"ring_gate_open")=="0");
    };
    pass(5297,23,"LEFT_3");pass(5561,57,"RETURN_RIGHT");
    for(int t=5825;t<=6056;t+=33) process(t,road);
    assert(column(r,"cone_phase")=="POST" && column(r,"cone_route_complete")=="1" && column(r,"ring_gate_open")=="1");
    assert(column(r,"speed_command")=="7");
    process(6089,cv::Mat(60,80,CV_8UC1,cv::Scalar(0)));assert(column(r,"stop_command")=="1");
    process(30000,road);assert(column(r,"ring_gate_open")=="1");
    NewTrack::processFrameAt(cv::Mat(),false,30033);NewTrack::processFrameAt(cv::Mat(),false,35000);
    process(36000,road);assert(column(r,"ring_gate_open")=="1");
    // No cone: deadline starts the route, but elapsed minimum cannot fake passage.
    NewTrack::reset();process(1000,road);process(5999,road);assert(column(r,"cone_phase")=="IDLE");
    process(6000,road);assert(column(r,"cone_entry_reason")=="TIME_FALLBACK");
    for(int t=6100;t<=8500;t+=100) process(t,road);
    assert(column(r,"cone_phase")=="FAULT" && column(r,"stop_command")=="1" && column(r,"stop_reason")=="CONE_VISUAL_TIMEOUT");
    assert(column(r,"ring_gate_open")=="0");
    NewTrack::reset();process(1000,road);process(6000,road);const auto remaining=column(r,"cone_remaining_ms");
    NewTrack::processFrameAt(cv::Mat(),false,6033);process(9000,road);assert(column(r,"cone_remaining_ms")==remaining);
    Other::gMotorStop=true;process(9033,road);assert(column(r,"stop_reason")=="OPERATOR_STOP");Other::gMotorStop=false;
    std::remove("cone_avoidance.ini");NewTrack::reset();process(1000,road);
    assert(column(r,"cone_tuning_valid")=="0" && column(r,"stop_command")=="1");
    std::cout<<"PASS cone route pipeline: grayscale entry/window/deadline, visual four stages, zone speed, ring release, pause, timeout, stop, missing INI\n";
}
