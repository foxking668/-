#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <fstream>
#include <sstream>
#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif
// Only hardware-independent target-speed/calibration getters are substituted.
// All image processing, contour repair, cone boundaries, task detectors and diagnostics are production code.
namespace Other {
double RuntimeConfig::getCenterPratio() { return .5; }
double RuntimeConfig::getErrorGain() { return 4.0; }
double RuntimeConfig::getMotorAimSpeed() { return 9; }
}
// The unrelated legacy zebra entry points are linked as fail-if-called guards.
// NewTrack uses its own real zebra detector and must never call these old-map methods.
namespace ImageProcess {
bool FixLine::checkZebraSimple() { throw std::runtime_error("unexpected legacy zebra detector in NewTrack"); }
FixLineState FixLine::handleZebraSimple() { throw std::runtime_error("unexpected legacy zebra handler in NewTrack"); }
}
std::vector<std::string> split(const std::string& s) {
    std::vector<std::string> fields;std::string field;std::stringstream in(s);
    while(std::getline(in,field,',')) fields.push_back(field);
    if(!s.empty() && s.back()==',') fields.emplace_back();return fields;
}
int main(int argc,char** argv) {
    if(argc!=2) return 2;
#ifdef _WIN32
    if(_chdir(argv[1])!=0) return 2;
#else
    if(chdir(argv[1])!=0) return 2;
#endif
    if(std::ifstream("image_params.txt").good()) {
        std::cerr<<"Refusing to overwrite an existing parameter file; use an empty test directory.\n";return 2;
    }
    std::ofstream params("image_params.txt");
    params<<"new_track_enable=1\nnew_track_work_mode=1\nnew_track_debug_print=0\n"
        <<"new_track_start_right_enable=0\nnew_track_top_crop_ratio=0\nnew_track_bottom_crop_ratio=0\n"
        <<"new_track_blue_cone_enable=1\nnew_track_ring_startup_block_ms=60000\n";
    params.close();
    cv::Mat frame(60,80,CV_8UC3,cv::Scalar(0,0,0));
    cv::rectangle(frame,{10,0,60,60},{230,230,230},-1);
    cv::line(frame,{40,0},{40,59},{0,0,0},2);
    std::vector<cv::Point> points{{26,28},{22,43},{30,43}};cv::fillConvexPoly(frame,points,{255,0,0});
    NewTrack::reset();auto first=NewTrack::processFrame(frame,true);
    auto active=NewTrack::processFrame(frame,true);
    assert(first.valid && active.valid && active.debugFrame.size()==cv::Size(960,800));
    const auto header=split(NewTrack::diagnosticCsvHeader()),fields=split(active.diagnosticCsv);
    assert(fields.size()==header.size());
    auto value=[&](const char* key)->std::string {
        const auto i=std::find(header.begin(),header.end(),key);assert(i!=header.end());return fields[i-header.begin()];
    };
    assert(value("source")=="CONE" && value("cone_side")=="LEFT" && value("cone_active")=="1");
    assert(!active.centerBlackReliable && value("center_black_used")=="0");
    assert(value("speed_command")=="7" && std::stod(value("steer_error"))==active.steerError);
    assert(active.steerError>0); // LEFT cone turns away to the right, using the replaced left edge.
    cv::imwrite("synthetic_live_pipeline.png",active.debugFrame);
    const auto withoutView=NewTrack::processFrame(frame,false);assert(withoutView.valid && withoutView.debugFrame.empty());
    assert(withoutView.steerError==active.steerError); // Rendering and recording do not change the control result.
    NewTrack::reset();frame.setTo(cv::Scalar(0,0,0));const auto invalid=NewTrack::processFrame(frame,true);
    assert(!invalid.valid && !invalid.debugFrame.empty());
    assert(split(invalid.diagnosticCsv).size()==header.size());
    std::remove("image_params.txt");
    std::cout<<"PASS actual live pipeline: cone replaces boundary, black priority, exact command/CSV, unchanged steering, invalid current frame\n";
}
