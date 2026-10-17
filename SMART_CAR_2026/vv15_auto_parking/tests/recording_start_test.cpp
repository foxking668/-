#include "Experiment/ProcessedRecording.hpp"
#include <fstream>
#include <iostream>
#include <algorithm>
#include <cassert>
// Header contents are validated with the real NewTrack pipeline separately.
namespace NewTrack { std::string diagnosticCsvHeader() {return "fixture_state";} }
int main(int argc,char** argv) {
    if(argc!=2) return 2;
    std::string expected=argv[1],actual=Experiment::executableDirectory();
    std::replace(expected.begin(),expected.end(),'\\','/');std::replace(actual.begin(),actual.end(),'\\','/');
    assert(actual==expected);
    const auto params=actual+"/recording_fixture_params.txt";
    if(std::ifstream(params).good()) return 2;
    std::ofstream config(params);config<<"processed_video_fps=10\nprocessed_video_queue_capacity=4\n";config.close();
    std::string error;const bool ready=Experiment::startProcessedRecording(params,error);
    if(!ready) {std::cerr<<error<<'\n';return 1;}
    auto video=Experiment::gProcessedVideoRecorder.videoPath();std::replace(video.begin(),video.end(),'\\','/');
    assert(video.find(actual+"/video/run_")==0 && video.substr(video.size()-13)=="processed.avi");
    cv::Mat image(800,960,CV_8UC3,cv::Scalar(0,0,0));
    const auto queued=Experiment::gProcessedVideoRecorder.submit(image,1,1000000000,"synthetic");
    assert(queued==Experiment::RecordSubmit::Queued);Experiment::gProcessedVideoRecorder.stop();
    assert(Experiment::gProcessedVideoRecorder.stats().verified);
    config.open(params,std::ios::trunc);config<<"processed_video_fps=31\n";config.close();
    const bool bad=Experiment::startProcessedRecording(params,error);assert(!bad && !error.empty());
    std::remove(params.c_str());
    std::cout<<"PASS recording startup: executable-relative video directory, separate run, parameters, decoder, invalid fps rejection\n";
}
