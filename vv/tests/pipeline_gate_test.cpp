#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include <cassert>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace Other {
double RuntimeConfig::getCenterPratio() {return .5;}
double RuntimeConfig::getErrorGain() {return 4.0;}
double RuntimeConfig::getMotorAimSpeed() {return 9;}
}
namespace ImageProcess {
bool FixLine::checkZebraSimple() {throw std::runtime_error("unexpected legacy zebra detector");}
FixLineState FixLine::handleZebraSimple() {throw std::runtime_error("unexpected legacy zebra handler");}
}
namespace {
std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> fields;std::stringstream input(text);std::string value;
    while(std::getline(input,value,',')) fields.push_back(value);
    if(!text.empty() && text.back()==',') fields.emplace_back();
    return fields;
}
std::map<std::string,std::string> fields(const NewTrack::FrameResult& result) {
    const auto keys=split(NewTrack::diagnosticCsvHeader()),values=split(result.diagnosticCsv);
    assert(keys.size()==values.size());std::map<std::string,std::string> out;
    for(std::size_t i=0;i<keys.size();++i) out[keys[i]]=values[i];
    return out;
}
void setup(const std::filesystem::path& base,const char* name,const char* gateSettings) {
    const auto directory=base/name;
    if(std::filesystem::exists(directory)) throw std::runtime_error("test subdirectory already exists");
    std::filesystem::create_directory(directory);std::filesystem::current_path(directory);
    std::ofstream parameters("image_params.txt");
    parameters<<"new_track_enable=1\nnew_track_work_mode=1\nnew_track_ring_enable=1\n"
        <<"new_track_debug_print=0\nnew_track_start_right_enable=0\nnew_track_startup_crossing_sync_enable=0\n"
        <<"new_track_top_crop_ratio=0\nnew_track_bottom_crop_ratio=0\n"
        <<"new_track_blue_cone_enable=0\nnew_track_zebra_enable=0\nnew_track_crossing_enable=0\n"
        <<"new_track_binary_mode=1\nnew_track_ring_low_camera_enable=1\n"
        <<"new_track_ring_probe_enable=1\nnew_track_ring_probe_wait_ms=0\n"
        <<"new_track_ring_probe_require_opening=1\nnew_track_ring_entry_confirm_frames=3\n"
        <<gateSettings;
    parameters.close();NewTrack::reset();
}
cv::Mat greyRing(int kind) {
    cv::Mat frame(60,80,CV_8UC3,cv::Scalar(80,80,80));
    for(int y=0;y<60;++y) {
        int left=20,right=43+y/2;
        if(kind==1) {
            if(y<10 || (y>=29&&y<=43)) left=0;
            else if(y>=10&&y<=28) left=26-std::abs(y-18);
        } else if(kind==2) left=y<18?30:(y<=32?0:20);
        for(int x=left;x<=right;++x) frame.at<cv::Vec3b>(y,x)={180,180,180};
    }
    return frame;
}
void noSteeringTask(const NewTrack::FrameResult& result) {
    assert(result.valid && result.ringStage==NewTrack::RingStage::NORMAL);
    const auto csv=fields(result);assert(csv.at("probe_phase")!="2" && csv.at("probe_phase")!="3");
    assert(!result.lowCameraConfirmed);
}
}
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    const auto base=std::filesystem::absolute(argv[1]);
    if(!std::filesystem::is_directory(base) || !std::filesystem::is_empty(base)) return 2;
    // Under the old HSV V>=46 path, both 80-grey track paint and 180-grey road
    // become white. This scene proves the new threshold runs in the live pipeline.
    setup(base,"grey_pipeline","new_track_ring_startup_block_ms=60000\nnew_track_ring_require_marker=0\n");
    cv::Mat grey(60,80,CV_8UC3,cv::Scalar(80,80,80));
    cv::rectangle(grey,{10,0,60,60},{180,180,180},-1);cv::line(grey,{40,0},{40,59},{80,80,80},2);
    const auto result=NewTrack::processFrameAt(grey,true,1000000);const auto diagnostic=fields(result);
    assert(result.valid && result.binaryQuality && result.binaryThreshold==80);
    assert(result.binaryWhitePercent>50 && result.binaryWhitePercent<80 && result.centerBlackReliable);
    assert(diagnostic.at("binary_threshold")=="80" && diagnostic.at("binary_quality")=="1");
    assert(std::abs(std::stod(diagnostic.at("steer_error"))-result.steerError)<1e-7);
    // Present real three-corner geometry repeatedly while the time gate is closed.
    setup(base,"time_block","new_track_ring_startup_block_ms=60000\nnew_track_ring_require_marker=0\n");
    bool sawCorners=false;const auto threeCorners=greyRing(1);
    for(int i=0;i<30;++i) {
        const auto blocked=NewTrack::processFrameAt(threeCorners,true,2000000+i*500);
        noSteeringTask(blocked);sawCorners|=blocked.leftDownY>=0 && blocked.leftMidY>=0 && blocked.leftUpY>=0;
        assert(fields(blocked).at("ring_gate_open")=="0");
    }
    assert(sawCorners); // The gate assertion is not vacuous: usable corners were observed.
    // Time is open, but no three-block marker has armed the entrance gate.
    setup(base,"marker_block","new_track_ring_startup_block_ms=0\nnew_track_ring_require_marker=1\n");
    for(int i=0;i<12;++i) {
        const auto blocked=NewTrack::processFrameAt(threeCorners,true,3000000+i*40);
        noSteeringTask(blocked);assert(!blocked.markerArmed && blocked.markerBlocks<3);
        assert(fields(blocked).at("ring_gate_open")=="0");
    }
    // One actual entrance-shaped frame amid straight-road frames must not start
    // a physical left probe; the pre-probe debounce belongs in the live pipeline.
    setup(base,"single_frame_noise","new_track_ring_startup_block_ms=0\nnew_track_ring_require_marker=0\n");
    const auto straight=greyRing(0),entrance=greyRing(2);
    for(int i=0;i<11;++i) noSteeringTask(NewTrack::processFrameAt(i==5?entrance:straight,true,4000000+i*40));
    std::cout<<"PASS actual pipeline: grey threshold reaches controls/CSV, visible corners cannot bypass time/marker gates, one entrance frame cannot start a left probe\n";
}
