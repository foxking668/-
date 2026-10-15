#include "ProcessedRecording.hpp"
#include "NewTrack/DiagnosticView.hpp"
#include "NewTrack/NewTrack.hpp"
#include <fstream>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>
#include <cerrno>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#ifdef _WIN32
#include <windows.h>
#else
#include <unistd.h>
#include <sys/stat.h>
#endif
namespace Experiment {
std::string executableDirectory() {
    char path[4096]{};
#ifdef _WIN32
    const auto n=GetModuleFileNameA(nullptr,path,sizeof(path));
    if(n==0 || n>=sizeof(path)) throw std::runtime_error("cannot resolve executable directory");
#else
    const auto n=readlink("/proc/self/exe",path,sizeof(path)-1);
    if(n<=0 || n>=static_cast<long>(sizeof(path)-1)) throw std::runtime_error("cannot resolve /proc/self/exe");
    path[n]=0;
#endif
    std::string result(path);
    const auto slash=result.find_last_of("/\\");
    if(slash==std::string::npos) throw std::runtime_error("invalid executable path");
    return result.substr(0,slash);
}
static bool makeDirectory(const std::string& path,bool existingAllowed) {
#ifdef _WIN32
    if(CreateDirectoryA(path.c_str(),nullptr)) return true;
    return existingAllowed && GetLastError()==ERROR_ALREADY_EXISTS &&
        (GetFileAttributesA(path.c_str())&FILE_ATTRIBUTE_DIRECTORY);
#else
    if(mkdir(path.c_str(),0755)==0) return true;
    struct stat info{};
    return existingAllowed && errno==EEXIST && stat(path.c_str(),&info)==0 && S_ISDIR(info.st_mode);
#endif
}
bool startProcessedRecording(const std::string& parameterPath,std::string& error) {
    try {
        VideoOptions options;options.fps=10;options.queueCapacity=4;options.fileName="processed.avi";
        options.metadataHeader=NewTrack::diagnosticCsvHeader();
        std::ifstream parameters(parameterPath);
        if(!parameters.good()) throw std::runtime_error("cannot read recording parameters");
        bool coneSequenceEnabled=false;
        std::string line;
        while(std::getline(parameters,line)) {
            line=line.substr(0,line.find('#'));
            const auto eq=line.find('=');if(eq==std::string::npos) continue;
            std::string key=line.substr(0,eq);key.erase(std::remove_if(key.begin(),key.end(),[](unsigned char c){return std::isspace(c);}),key.end());
            if(key=="processed_video_fps") options.fps=std::stoi(line.substr(eq+1));
            if(key=="processed_video_queue_capacity") options.queueCapacity=std::stoi(line.substr(eq+1));
            if(key=="new_track_cone_sequence_enable") coneSequenceEnabled=std::stoi(line.substr(eq+1))!=0;
        }
        const auto root=executableDirectory()+"/video";
        if(!makeDirectory(root,true)) throw std::runtime_error("cannot create video directory beside binary");
        const auto now=std::chrono::system_clock::now();const auto time=std::chrono::system_clock::to_time_t(now);
        std::tm local{};
#ifdef _WIN32
        localtime_s(&local,&time);const auto pid=GetCurrentProcessId();
#else
        localtime_r(&time,&local);const auto pid=getpid();
#endif
        std::ostringstream name;name<<root<<"/run_"<<std::put_time(&local,"%Y%m%d_%H%M%S")<<'_'<<pid<<'_'
            <<std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()).count()%1000;
        if(!makeDirectory(name.str(),false)) throw std::runtime_error("cannot create unique recording run directory");
        auto snapshot=[&](const std::string& source,const std::string& target) {
            std::ifstream input(source,std::ios::binary);
            std::ofstream output(name.str()+"/"+target,std::ios::binary);
            if(!input || !output) throw std::runtime_error("cannot create parameter snapshot: "+source);
            output<<input.rdbuf();output.flush();
            if(!output || input.bad()) throw std::runtime_error("failed parameter snapshot: "+source);
        };
        snapshot(parameterPath,"image_params.snapshot.txt");
        const auto slash=parameterPath.find_last_of("/\\");
        const std::string tuning=(slash==std::string::npos?"":parameterPath.substr(0,slash+1))+"cone_avoidance.ini";
        if(coneSequenceEnabled || std::ifstream(tuning).good()) snapshot(tuning,"cone_avoidance.snapshot.ini");
        return gProcessedVideoRecorder.start(name.str(),NewTrack::DiagnosticView::canvasSize(),options,error);
    } catch(const std::exception& e) {error=e.what();return false;}
}
}
