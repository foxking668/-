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
    assert(r.valid && r.steerError==-30 && column(r,"speed_command")=="6" && column(r,"center_black_used")=="0");
    process(5066,cone(57,48));process(5099,cone(57,54));process(5132,road);process(5165,road);
    assert(column(r,"cone_phase")=="RIGHT_2" && r.steerError==58);
    auto pass=[&](long long t,int x,const char* expected) {
        process(t,cone(x,33));process(t+33,cone(x,45));process(t+66,cone(x,54));
        process(t+99,road);process(t+132,road);
        assert(column(r,"cone_phase")==expected && column(r,"speed_command")=="6" && column(r,"ring_gate_open")=="0");
    };
    pass(5198,23,"LEFT_3");pass(5363,57,"RETURN_RIGHT");process(5528,road);process(5561,road);
    assert(column(r,"cone_phase")=="POST" && column(r,"cone_route_complete")=="1" && column(r,"ring_gate_open")=="1");
    assert(column(r,"speed_command")=="7");
    process(5594,cv::Mat(60,80,CV_8UC1,cv::Scalar(0)));assert(column(r,"stop_command")=="1");
    process(30000,road);assert(column(r,"ring_gate_open")=="1");
    NewTrack::processFrameAt(cv::Mat(),false,30033);NewTrack::processFrameAt(cv::Mat(),false,35000);
    process(36000,road);assert(column(r,"ring_gate_open")=="1");
    // Regression: entry by deadline WITHOUT a cone must not hold -30 until 2500ms.
    NewTrack::reset();process(1000,road);process(5999,road);assert(column(r,"cone_phase")=="IDLE");
    process(6000,road);assert(column(r,"cone_entry_reason")=="TIME_FALLBACK" && r.steerError==-30);
    process(6100,road);assert(column(r,"cone_control_mode")=="PULSE");
    process(6200,road);assert(column(r,"cone_phase")=="LEFT_1" && column(r,"cone_control_mode")=="GUIDE");
    assert(column(r,"cone_pulse_remaining_ms")=="0" && r.steerError!=-30 && column(r,"speed_command")=="6");
    process(6233,cv::Mat(60,80,CV_8UC1,cv::Scalar(0)));
    assert(column(r,"stop_command")=="1" && column(r,"stop_reason")=="CONE_GUIDANCE_UNAVAILABLE");
    process(6266,road);assert(column(r,"cone_control_mode")=="GUIDE" && column(r,"stop_command")=="0");
    for(int t=6366;t<=9266;t+=100) {process(t,road);if(column(r,"cone_phase")=="FAULT") break;}
    assert(column(r,"cone_phase")=="FAULT" && column(r,"stop_command")=="1" && column(r,"stop_reason")=="CONE_VISUAL_TIMEOUT");
    assert(r.steerError==0 && column(r,"ring_gate_open")=="0");
    NewTrack::reset();process(1000,road);process(6000,road);const auto remaining=column(r,"cone_remaining_ms");
    NewTrack::processFrameAt(cv::Mat(),false,6033);process(9000,road);
    assert(std::stoll(column(r,"cone_remaining_ms"))==std::stoll(remaining)-33);
    Other::gMotorStop=true;process(9033,road);assert(column(r,"stop_reason")=="OPERATOR_STOP");Other::gMotorStop=false;
    // Real motor-thread clock: pulse deadline must stop drive BEFORE the 150ms camera watchdog.
    auto shortPulse=text.str();
    for(const auto& pair:std::vector<std::pair<std::string,std::string>>{{"entry_time_ms=4500","entry_time_ms=0"},
        {"entry_early_ms=500","entry_early_ms=0"},{"entry_late_ms=500","entry_late_ms=0"},
        {"first_left_duration_ms=200","first_left_duration_ms=80"}}) {
        const auto at=shortPulse.find(pair.first);assert(at!=std::string::npos);shortPulse.replace(at,pair.first.size(),pair.second);
    }
    std::ofstream("cone_avoidance.ini")<<shortPulse;NewTrack::reset();
    const auto pulseStarted=std::chrono::steady_clock::now();r=NewTrack::processFrame(road,false);
    assert(r.valid && r.steerError==-30 && !NewTrack::forceMotorStop());
    std::this_thread::sleep_for(std::chrono::milliseconds(90));
    const auto cutoffAge=std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now()-pulseStarted).count();
    assert(cutoffAge<150 && NewTrack::forceMotorStop());
    std::cout<<"PASS motor pulse cutoff observed at "<<cutoffAge<<"ms, before 150ms camera watchdog\n";
    r=NewTrack::processFrame(road,false);assert(r.valid && r.steerError!=-30 && !NewTrack::forceMotorStop());
    std::remove("cone_avoidance.ini");NewTrack::reset();process(1000,road);
    assert(column(r,"cone_tuning_valid")=="0" && column(r,"stop_command")=="1");
    std::cout<<"PASS cone route pipeline: grayscale entry/window/deadline, visual four stages, bounded pulse/visual fallback, zone speed 6, ring release, pause, timeout, stop, missing INI\n";
}
