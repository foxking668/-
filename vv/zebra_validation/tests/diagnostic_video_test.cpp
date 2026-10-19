#include "NewTrack/DiagnosticView.hpp"
#include "Experiment/VideoRecorder.hpp"
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/videoio.hpp>
#include <cassert>
#include <fstream>
#include <iostream>
#include <thread>
int main(int argc,char** argv) {
    assert(argc==2);const std::string dir=argv[1];
    cv::Mat bgr(56,80,CV_8UC3,cv::Scalar(0,0,0)),raw(56,80,CV_8UC1,cv::Scalar(0));
    cv::rectangle(raw,{10,0,60,56},255,-1);cv::line(raw,{40,0},{40,55},0,2);
    cv::cvtColor(raw,bgr,cv::COLOR_GRAY2BGR);
    cv::Mat closed;cv::morphologyEx(raw,closed,cv::MORPH_CLOSE,cv::getStructuringElement(cv::MORPH_RECT,{11,1}));
    const auto rawCopy=raw.clone();
    NewTrack::DiagnosticView::Snapshot s;s.left.assign(56,10);s.right.assign(56,69);s.center.assign(56,39);s.black.assign(56,40);
    s.valid=true;s.targetX=39;s.blackUsed=true;s.lines={"SYNTHETIC TEST FRAME - NOT A VEHICLE RUN","ring=0 source=BOUNDARY", "left/right/center arrays from this test frame"};
    auto composite=NewTrack::DiagnosticView::render(bgr,raw,closed,s);
    assert(composite.size()==cv::Size(960,800) && cv::countNonZero(raw!=rawCopy)==0);
    // RAW panel interior pixels preserve black and white values after nearest-neighbor scaling.
    const double scale=316.0/56;const int width=static_cast<int>(80*scale),height=316;
    const int px=480+(480-width)/2,py=24+24+(316-height)/2;
    assert(composite.at<cv::Vec3b>(py+height/2,px+static_cast<int>(20*scale))==cv::Vec3b(255,255,255));
    assert(composite.at<cv::Vec3b>(py+height/2,px+static_cast<int>(40*scale))==cv::Vec3b(0,0,0));
    cv::imwrite(dir+"/synthetic_diagnostic_preview.png",composite);
    Experiment::VideoRecorder recorder;Experiment::VideoOptions opts;opts.fps=10;opts.queueCapacity=4;
    opts.fileName="processed.avi";opts.metadataHeader="ring_stage,target_x,left_by_y";
    std::string error;assert(recorder.start(dir,composite.size(),opts,error));
    assert(recorder.shouldSelect(1000000000LL));
    auto result=recorder.submit(composite,17,1000000000LL,"0,39,10;10;10");assert(result==Experiment::RecordSubmit::Queued);
    composite.setTo(cv::Scalar(99,99,99)); // Writer must own a clone of the submitted live snapshot.
    assert(!recorder.shouldSelect(1050000000LL));
    result=recorder.submit(composite,18,1050000000LL,"0,39,10;10;10");assert(result==Experiment::RecordSubmit::RateSkipped);
    for(int i=1;i<=5;++i) {
        cv::Mat image=NewTrack::DiagnosticView::render(bgr,raw,closed,s);
        result=recorder.submit(image,18+i,1000000000LL+i*100000000LL,"1,39,10;10;10");
        if(result==Experiment::RecordSubmit::Overflow) {std::this_thread::sleep_for(std::chrono::milliseconds(30));--i;}
        else assert(result==Experiment::RecordSubmit::Queued || result==Experiment::RecordSubmit::RateSkipped);
        std::this_thread::sleep_for(std::chrono::milliseconds(15));
    }
    recorder.stop();const auto stats=recorder.stats();assert(stats.closed && stats.verified && stats.written==stats.decoded && stats.written>1);
    cv::VideoCapture readback(recorder.videoPath());cv::Mat first;assert(readback.read(first));
    assert(first.at<cv::Vec3b>(py+height/2,px+static_cast<int>(20*scale))[0]>220); // Original white survived clone and codec.
    std::ifstream csv(dir+"/video_timestamps.csv");std::string line;std::getline(csv,line);
    assert(line=="video_frame_index,frame_index,capture_time_ns,ring_stage,target_x,left_by_y");
    std::getline(csv,line);assert(line=="0,17,1000000000,0,39,10;10;10");
    assert(!recorder.start(dir,composite.size(),opts,error)); // Existing run is never overwritten.
    std::cout<<"PASS diagnostic video: unchanged raw, one-frame arrays, async clone, frame timing, CSV, full decode, no overwrite\n";
}
