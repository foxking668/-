#include "../tools/reference_cc_lane.hpp"
#include <opencv2/imgproc.hpp>
#include <iostream>
using namespace car2026::capture;
int main() {
    try {
        ReferenceCcLane lane;
        auto check=[](bool ok,const char* name) {if(!ok) throw std::runtime_error(name);};
        check(!lane.analyze(cv::Mat()).line.reliable,"empty frame must not authorize motors");
        bool rejected=false;
        try {lane.analyze(cv::Mat(60,80,CV_8UC1,cv::Scalar(255)));}
        catch(const std::runtime_error&) {rejected=true;}
        check(rejected,"gray input must not silently use BGR conversion");
        check(!lane.analyze(cv::Mat(60,80,CV_8UC3,cv::Scalar(0,0,0))).line.reliable,"all black loses lane");
        auto road=[](int center) {
            cv::Mat frame(60,80,CV_8UC3,cv::Scalar(0,0,0));
            cv::rectangle(frame,cv::Rect(8,0,64,60),cv::Scalar(255,255,255),cv::FILLED);
            cv::rectangle(frame,cv::Rect(center-1,0,3,60),cv::Scalar(0,0,0),cv::FILLED);
            return frame;
        };
        for(int center: {28,34,39}) {
            lane.reset();
            const auto result=lane.analyze(road(center));
            check(result.line.reliable && result.centerBlackReliable,"real black center detected");
            check(std::abs(result.rawTarget-center)<0.001,"black center native coordinate");
            const float expected=float(std::atan2(center-80*.42,28.)*180/std::acos(-1.)*4);
            check(std::abs(result.error-expected)<1e-5,"original center ratio and 4x atan2 mapping");
            check(std::abs(ccLaneServoCommand(result,15)-std::clamp(double(expected),-15.,15.))<1e-7,"relative servo sign and limit");
        }
        lane.reset();
        const auto first=lane.analyze(road(34));
        cv::Mat larger;
        cv::resize(road(34),larger,cv::Size(640,480),0,0,cv::INTER_NEAREST);
        check(lane.analyze(larger).error==first.error,"camera resize keeps the original native target");
        for(int i=0;i<1000;++i) {
            const auto next=lane.analyze(road(34));
            check(next.line.reliable && next.error==first.error,"repeat allocation and width memory stability");
        }
        lane.reset();
        check(lane.analyze(road(34)).error==first.error,"new trial resets road memory");
        std::cout<<"CC ordinary lane image checks passed\n";
        return 0;
    } catch(const std::exception& error) {std::cerr<<error.what()<<'\n';return 1;}
}
