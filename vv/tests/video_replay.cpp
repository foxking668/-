#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include <opencv2/videoio.hpp>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>

// Only hardware-independent parameter getters are substituted. All thresholding,
// tracing, task gates, repair and final steering below are the production pipeline.
namespace { double centerRatio=.42, aimSpeed=9; }
namespace Other {
double RuntimeConfig::getCenterPratio() { return centerRatio; }
double RuntimeConfig::getMotorAimSpeed() { return aimSpeed; }
}
namespace ImageProcess {
bool FixLine::checkZebraSimple() { throw std::runtime_error("unexpected legacy zebra detector"); }
FixLineState FixLine::handleZebraSimple() { throw std::runtime_error("unexpected legacy zebra handler"); }
}

namespace {
std::vector<std::string> split(const std::string& text) {
    std::vector<std::string> values; std::stringstream input(text); std::string value;
    while(std::getline(input,value,',')) values.push_back(value);
    if(!text.empty() && text.back()==',') values.emplace_back();
    return values;
}
std::string trim(std::string value) {
    const auto first=value.find_first_not_of(" \t\r\n");
    if(first==std::string::npos) return {};
    return value.substr(first,value.find_last_not_of(" \t\r\n")-first+1);
}
std::string readParams(const std::filesystem::path& source) {
    std::ifstream input(source); if(!input) throw std::runtime_error("missing launch-directory image_params.txt");
    std::ostringstream buffer; buffer<<input.rdbuf();
    std::istringstream lines(buffer.str()); std::string line;
    while(std::getline(lines,line)) {
        const auto equal=line.find('='); if(equal==std::string::npos) continue;
        const auto key=trim(line.substr(0,equal)); const auto value=trim(line.substr(equal+1));
        if(key=="center_pratio") centerRatio=std::stod(value);
        if(key=="motor_aim_speed") aimSpeed=std::stod(value);
    }
    return buffer.str();
}
struct ScalarRangeStats {
    double low=std::numeric_limits<double>::infinity(), high=-std::numeric_limits<double>::infinity();
    void add(double value) {if(std::isfinite(value)) {low=std::min(low,value);high=std::max(high,value);}}
    void print(const char* name) const {
        std::cout<<name<<"=";
        if(std::isfinite(low)) std::cout<<low<<".."<<high; else std::cout<<"unavailable";
        std::cout<<'\n';
    }
};
}

int main(int argc, char** argv) {
    try {
        if(argc<4 || argc>5) {
            std::cerr<<"Usage: video_replay VIDEO EMPTY_WORK_DIR CSV [CLOCK_OFFSET_MS]\n"
                <<"Copies launch-directory image_params.txt to EMPTY_WORK_DIR; never writes image files.\n"
                <<"CLOCK_OFFSET_MS shifts only the replay clock origin, not lap elapsed time or gates.\n";
            return 2;
        }
        namespace fs=std::filesystem;
        const auto launch=fs::current_path();
        const auto video=fs::absolute(argv[1]), work=fs::absolute(argv[2]), csvPath=fs::absolute(argv[3]);
        long long offset=0;
        if(argc==5) {std::size_t parsed=0;offset=std::stoll(argv[4],&parsed);if(parsed!=std::strlen(argv[4]) || offset<0) throw std::runtime_error("offset must be a nonnegative integer");}
        if(offset>86400000) throw std::runtime_error("clock offset exceeds one day");
        if(!fs::is_directory(work) || !fs::is_empty(work)) throw std::runtime_error("work directory must already exist and be empty");
        if(fs::exists(csvPath)) throw std::runtime_error("refusing to overwrite existing CSV");
        const auto parameterText=readParams(launch/"image_params.txt");
        cv::VideoCapture capture(video.string());
        if(!capture.isOpened()) throw std::runtime_error("cannot open video");
        const double fps=capture.get(cv::CAP_PROP_FPS);
        if(!std::isfinite(fps) || fps<=0 || fps>1000) throw std::runtime_error("video has invalid FPS");
        const int advertisedFrames=static_cast<int>(capture.get(cv::CAP_PROP_FRAME_COUNT));
        {std::ofstream parameters(work/"image_params.txt");parameters<<parameterText;if(!parameters) throw std::runtime_error("cannot copy isolated parameters");}
        fs::current_path(work);
        if(fs::exists(csvPath)) throw std::runtime_error("CSV must not overwrite the copied parameter file");
        std::ofstream csv(csvPath);
        if(!csv) throw std::runtime_error("cannot create CSV");
        const auto headerText=NewTrack::diagnosticCsvHeader();
        const auto header=split(headerText);
        std::map<std::string,std::size_t> columns;
        for(std::size_t i=0;i<header.size();++i) columns[header[i]]=i;
        csv<<"frame,time_ms,capture_ms,"<<headerText<<'\n';
        NewTrack::reset();
        int frames=0,valid=0,quality=0,candidates=0,confirmed=0,virtualPoints=0;
        int candidateRun=0,maxCandidateRun=0,closedCandidates=0,closedProbeLeft=0,stageTransitions=0;
        int closedStartTransitions=0,closedProbeStarts=0,previousStage=0,previousProbe=0;
        std::map<int,int> stages,probePhases;
        ScalarRangeStats thresholds,whitePercent,steerErrors;cv::Mat decoded,resized;
        while(capture.read(decoded)) {
            // Exactly the production Camera.cpp downsample; do not apply area interpolation.
            cv::resize(decoded,resized,cv::Size(CAMERA_OPENCV_WIDTH,CAMERA_OPENCV_HEIGHT),0,0,cv::INTER_NEAREST);
            const long long timeMs=std::llround(frames*1000.0/fps),captureMs=1000000+offset+timeMs;
            // Rendering is held in memory only. The true pipeline produces its CSV with this switch.
            const auto result=NewTrack::processFrameAt(resized,true,captureMs);
            const auto values=split(result.diagnosticCsv);
            if(values.size()!=header.size()) throw std::runtime_error("diagnostic CSV field mismatch at frame "+std::to_string(frames));
            auto integer=[&](const char* name,int fallback=-1) {
                const auto column=columns.find(name);
                if(column==columns.end() || values[column->second].empty()) return fallback;
                return std::stoi(values[column->second]);
            };
            csv<<frames<<','<<timeMs<<','<<captureMs<<','<<result.diagnosticCsv<<'\n';
            const int stage=static_cast<int>(result.ringStage),probe=integer("probe_phase");
            ++stages[stage];if(probe>=0) ++probePhases[probe];
            valid+=result.valid;quality+=result.binaryQuality;
            candidates+=result.lowCameraCandidate;confirmed+=result.lowCameraConfirmed;
            virtualPoints+=(result.virtualDownX>=0 && result.virtualDownY>=0);
            if(result.lowCameraCandidate) ++candidateRun;else candidateRun=0;
            maxCandidateRun=std::max(maxCandidateRun,candidateRun);
            if(integer("ring_gate_open")==0) {
                closedCandidates+=result.lowCameraCandidate;
                closedProbeLeft+=(probe==2);
                closedStartTransitions+=(previousStage==0 && stage!=0);
                closedProbeStarts+=(previousProbe!=2 && probe==2);
            }
            if(result.binaryThreshold>=0) thresholds.add(result.binaryThreshold);
            whitePercent.add(result.binaryWhitePercent);if(result.valid) steerErrors.add(result.steerError);
            if(stage!=previousStage) {
                ++stageTransitions;std::cout<<"stage_transition frame="<<frames<<" time_ms="<<timeMs<<" "<<previousStage<<"->"<<stage<<'\n';
                previousStage=stage;
            }
            previousProbe=probe;
            ++frames;
        }
        if(!csv) throw std::runtime_error("CSV write failed");
        if(frames==0 || (advertisedFrames>0 && frames!=advertisedFrames)) throw std::runtime_error("video was not completely decoded");
        std::cout<<std::setprecision(7)<<"replay_frames="<<frames<<" advertised_frames="<<advertisedFrames<<" fps="<<fps
            <<" duration_ms="<<std::llround(frames*1000.0/fps)<<" clock_offset_ms="<<offset<<'\n';
        std::cout<<"valid_frames="<<valid<<" binary_quality_frames="<<quality<<" low_camera_candidates="<<candidates
            <<" low_camera_confirmed="<<confirmed<<" virtual_down_frames="<<virtualPoints
            <<" candidate_max_run="<<maxCandidateRun<<" stage_transitions="<<stageTransitions<<'\n';
        if(columns.count("ring_gate_open")) std::cout<<"gate_closed_visual_candidates="<<closedCandidates
            <<" gate_closed_left_probe_frames="<<closedProbeLeft<<" gate_closed_start_transitions="<<closedStartTransitions
            <<" gate_closed_probe_start_transitions="<<closedProbeStarts<<'\n';
        thresholds.print("threshold_range");whitePercent.print("white_percent_range");steerErrors.print("steer_error_range");
        std::cout<<"ring_stage_frames=";for(const auto& count:stages) std::cout<<count.first<<':'<<count.second<<' ';std::cout<<'\n';
        std::cout<<"probe_phase_frames=";for(const auto& count:probePhases) std::cout<<count.first<<':'<<count.second<<' ';std::cout<<'\n';
        std::cout<<"CSV="<<csvPath.string()<<"\nPARAMETER_COPY="<<(work/"image_params.txt").string()<<'\n';
        return 0;
    } catch(const std::exception& error) {std::cerr<<"video_replay: "<<error.what()<<'\n';return 2;}
}
