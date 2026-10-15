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
    const auto tuning=actual+"/cone_avoidance.ini";
    if(std::ifstream(tuning).good()) return 2;
    std::ofstream fixture(tuning);fixture<<"# exact snapshot fixture\nleft_duration_ms=500\n";fixture.close();
    std::ofstream config(params);config<<"processed_video_fps=10\nprocessed_video_queue_capacity=4\nnew_track_cone_sequence_enable=1\n";config.close();
    std::string error;const bool ready=Experiment::startProcessedRecording(params,error);
    if(!ready) {std::cerr<<error<<'\n';return 1;}
    auto video=Experiment::gProcessedVideoRecorder.videoPath();std::replace(video.begin(),video.end(),'\\','/');
    assert(video.find(actual+"/video/run_")==0 && video.substr(video.size()-13)=="processed.avi");
    const auto run=video.substr(0,video.find_last_of('/'));
    std::ifstream imageSnapshot(run+"/image_params.snapshot.txt"),coneSnapshot(run+"/cone_avoidance.snapshot.ini");
    const std::string imageText((std::istreambuf_iterator<char>(imageSnapshot)),{});
    const std::string coneText((std::istreambuf_iterator<char>(coneSnapshot)),{});
    assert(imageText.find("new_track_cone_sequence_enable=1")!=std::string::npos);
    assert(coneText=="# exact snapshot fixture\nleft_duration_ms=500\n");
    cv::Mat image(800,960,CV_8UC3,cv::Scalar(0,0,0));
    const auto queued=Experiment::gProcessedVideoRecorder.submit(image,1,1000000000,"synthetic");
    assert(queued==Experiment::RecordSubmit::Queued);Experiment::gProcessedVideoRecorder.stop();
    assert(Experiment::gProcessedVideoRecorder.stats().verified);
    config.open(params,std::ios::trunc);config<<"processed_video_fps=31\n";config.close();
    const bool bad=Experiment::startProcessedRecording(params,error);assert(!bad && !error.empty());
    std::remove(params.c_str());
    std::remove(tuning.c_str());
    std::cout<<"PASS recording startup: executable-relative video directory, separate run, parameters, decoder, invalid fps rejection\n";
}
