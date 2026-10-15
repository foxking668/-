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
double testMotorSpeed=9;
double RuntimeConfig::getMotorAimSpeed() { return testMotorSpeed; }
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
    NewTrack::reset();auto first=NewTrack::processFrameAt(frame,true,1000);
    auto active=NewTrack::processFrameAt(frame,true,1033);
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
    // The inset must survive drawing all four panels; the colour panel must retain blue.
    assert(active.debugFrame.at<cv::Vec3b>(622,357)==cv::Vec3b(0,0,0));
    const double panelScale=316.0/frame.rows;
    const int colourX=(480-int(frame.cols*panelScale))/2+int(26*panelScale);
    const auto colour=active.debugFrame.at<cv::Vec3b>(48+int(40*panelScale),colourX);
    assert(colour[0]>240 && colour[1]<10 && colour[2]<10);
    cv::imwrite("synthetic_live_pipeline.png",active.debugFrame);
    const auto withoutView=NewTrack::processFrameAt(frame,false,1066);assert(withoutView.valid && withoutView.debugFrame.empty());
    assert(withoutView.steerError==active.steerError); // Rendering and recording do not change the control result.
    NewTrack::reset();frame.setTo(cv::Scalar(0,0,0));const auto invalid=NewTrack::processFrame(frame,true);
    assert(!invalid.valid && !invalid.debugFrame.empty());
    assert(split(invalid.diagnosticCsv).size()==header.size());
    // Full production pipeline with the independent tuning file and real capture timestamps.
    std::ofstream tuning("cone_avoidance.ini");
    tuning<<"left_pratio=0.65\nleft_min_error=30\nleft_duration_ms=500\nleft_speed=1\n"
          <<"wait_duration_ms=200\nwait_speed=7\nright_min_error=58\nright_duration_ms=700\nright_speed=3\n"
          <<"post_duration_ms=5500\npost_speed=7\ntrigger_y_percent=45\n";tuning.close();
    params.open("image_params.txt",std::ios::app);
    params<<"new_track_cone_sequence_enable=1\nnew_track_ring_side_enable=0\nnew_track_ring_entry_cue_enable=0\n"
          <<"new_track_zebra_enable=0\nnew_track_crossing_enable=0\n";params.close();
    frame=cv::Mat(60,80,CV_8UC3,cv::Scalar(0,0,0));
    cv::rectangle(frame,{10,0,60,60},{230,230,230},-1);
    cv::line(frame,{40,0},{40,59},{0,0,0},2);
    points={{54,28},{50,43},{58,43}};cv::fillConvexPoly(frame,points,{255,0,0});
    auto column=[&](const NewTrack::FrameResult& result,const char* key) {
        const auto names=split(NewTrack::diagnosticCsvHeader()),data=split(result.diagnosticCsv);
        assert(names.size()==data.size());const auto it=std::find(names.begin(),names.end(),key);
        assert(it!=names.end());return data[it-names.begin()];
    };
    NewTrack::reset();NewTrack::processFrameAt(frame,true,1000);
    auto timed=NewTrack::processFrameAt(frame,true,1033);
    assert(column(timed,"cone_phase")=="LEFT" && timed.steerError<=-30 && column(timed,"speed_command")=="1");
    assert(column(timed,"center_black_used")=="0" && column(timed,"source")=="CONE_LEFT");
    for(int t=1133;t<=1533;t+=100) timed=NewTrack::processFrameAt(frame,true,t);
    assert(column(timed,"cone_phase")=="WAIT" && column(timed,"speed_command")=="7");
    for(int t=1633;t<=1733;t+=100) timed=NewTrack::processFrameAt(frame,true,t);
    assert(column(timed,"cone_phase")=="RIGHT" && timed.steerError>=58 && column(timed,"speed_command")=="3");
    for(int t=1833;t<=2433;t+=100) timed=NewTrack::processFrameAt(frame,true,t);
    assert(column(timed,"cone_phase")=="POST" && column(timed,"speed_command")=="7");
    cv::Mat clearFrame(60,80,CV_8UC3,cv::Scalar(0,0,0));
    cv::rectangle(clearFrame,{10,0,60,60},{230,230,230},-1);
    cv::line(clearFrame,{40,0},{40,59},{0,0,0},2);
    Other::testMotorSpeed=5;
    NewTrack::processFrameAt(clearFrame,true,2533);timed=NewTrack::processFrameAt(clearFrame,true,2633);
    assert(column(timed,"cone_phase")=="POST" && column(timed,"speed_command")=="5");
    Other::testMotorSpeed=9;
    // Motor stop freezes action time; resuming never charges the paused wall-clock gap.
    NewTrack::reset();NewTrack::processFrameAt(frame,true,3000);NewTrack::processFrameAt(frame,true,3033);
    Other::gMotorStop=true;timed=NewTrack::processFrameAt(frame,true,3133);
    const auto remaining=column(timed,"cone_remaining_ms");assert(column(timed,"stop_command")=="1");
    timed=NewTrack::processFrameAt(frame,true,5000);assert(column(timed,"cone_remaining_ms")==remaining);
    Other::gMotorStop=false;timed=NewTrack::processFrameAt(frame,true,5033);
    assert(column(timed,"cone_remaining_ms")==remaining && column(timed,"stop_command")=="0");
    NewTrack::processFrameAt(cv::Mat(),false,5100);timed=NewTrack::processFrameAt(frame,true,6000);
    assert(column(timed,"cone_remaining_ms")==remaining);
    // An incomplete deployment is explicitly stopped instead of silently using compiled defaults.
    std::remove("cone_avoidance.ini");NewTrack::reset();timed=NewTrack::processFrameAt(frame,true,7000);
    assert(column(timed,"cone_tuning_valid")=="0" && column(timed,"stop_command")=="1");
    std::remove("image_params.txt");
    std::cout<<"PASS actual live pipeline: cone/black priority, locked timed LEFT/WAIT/RIGHT/POST speeds, pause/camera recovery, missing-file stop, exact CSV\n";
}
