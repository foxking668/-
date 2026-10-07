#include "parking_rehearsal.hpp"
#include "capture_sensors.hpp"
#include "capture_files.hpp"
#include "hardware_linux.hpp"
#include "rehearsal_servo.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <chrono>
#include <ctime>
#include <filesystem>
#include <iostream>
#include <memory>
#include <poll.h>
#include <termios.h>
namespace fs=std::filesystem;
using namespace car2026::capture;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signalHandler(int) {interrupted=1;}
int64_t monotonicNs() {return rehearsalMonotonicNs();}
struct Options {
    std::string capture="manual_capture.ini",hardware="config/calibration_hardware.ini",vehicle="config/calibration_vehicle.ini",
        tuning="config/parking_tuning.ini",output="captures/parking_tuning";
    bool help=false,check=false,partial=false;
    static Options parse(int argc,char** argv) {
        Options result;std::set<std::string> seen;
        for(int i=1;i<argc;++i) {
            const std::string key=argv[i];
            if(!seen.insert(key).second) throw std::runtime_error("Duplicate option: "+key);
            if(key=="--help") result.help=true;
            else if(key=="--check-config") result.check=true;
            else if(key=="--allow-partial") result.partial=true;
            else {
                std::string* target=nullptr;
                if(key=="--config") target=&result.capture;
                else if(key=="--hardware-config") target=&result.hardware;
                else if(key=="--vehicle-config") target=&result.vehicle;
                else if(key=="--tuning-config") target=&result.tuning;
                else if(key=="--output") target=&result.output;
                if(!target || i+1>=argc) throw std::runtime_error("Unknown option or missing value: "+key);
                *target=argv[++i];
                if(target->empty() || target->rfind("--",0)==0) throw std::runtime_error("Missing option value: "+key);
            }
        }
        return result;
    }
};
RehearsalKey readKey() {
    std::vector<std::string> lines;pollfd input{STDIN_FILENO,POLLIN,0};
    for(unsigned count=0;count<32;++count) {
        int ready;do {ready=poll(&input,1,0);} while(ready<0 && errno==EINTR && !interrupted);
        if(ready<0) throw std::runtime_error("Terminal poll failed");
        if(!ready) break;
        if(input.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("Terminal disconnected");
        char buffer[4096];const auto size=read(STDIN_FILENO,buffer,sizeof(buffer));
        if(size<=0) throw std::runtime_error("Terminal read failed or EOF");
        std::string text(buffer,size_t(size));
        if(text.back()!='\n') {tcflush(STDIN_FILENO,TCIFLUSH);return RehearsalKey::Invalid;}
        size_t start=0;
        for(size_t end=text.find('\n');end!=std::string::npos;end=text.find('\n',start)) {
            lines.push_back(text.substr(start,end-start));start=end+1;
        }
        if(count==31) {tcflush(STDIN_FILENO,TCIFLUSH);return RehearsalKey::Invalid;}
    }
    return parseRehearsalInput(lines);
}
void saveSnapshot(const fs::path& directory,unsigned revision,const ParkingTuning& tuning) {
    auto output=outputFile(directory/("config_"+std::to_string(revision)+".ini"));
    output<<tuning.source;closeText(output,directory/("config_"+std::to_string(revision)+".ini"));
}
constexpr const char* csvHeader="event,elapsed_s,read_start_ns,read_end_ns,stage,state,trial,config_revision,clip,video_frame,configured_command,last_written_command,channel,value,unit,valid,read_return,raw_hex,detail";
class Recording {
public:
    explicit Recording(fs::path directory):directory_(std::move(directory)) {
        for(size_t i=0;i<files_.size();++i) {files_[i]=outputFile(directory_/stageFiles[i]);files_[i]<<csvHeader<<'\n';files_[i].flush();}
    }
    void row(const ParkingRehearsal& trial,const std::string& event,int64_t begin,int64_t end,int64_t origin,
             std::optional<double> written,const std::string& detail="",const SensorSpec* sensor=nullptr,const Sample* sample=nullptr) {
        std::ostringstream file;file<<std::setprecision(17);
        file<<csvString(event)<<','<<double(end-origin)/1e9<<','<<begin<<','<<end<<','<<trial.stage()+1<<','
            <<csvString(trial.stateName())<<','<<trial.trial()<<','<<trial.revision()<<','<<csvString(clip_)<<',';
        if(videoFrames_) file<<videoFrames_-1;
        file<<','<<trial.command()<<',';if(written) file<<*written;
        file<<','<<csvString(sensor ? sensor->name : "")<<',';
        if(sample && sample->valid) file<<sample->value;
        file<<','<<csvString(sensor ? sensor->unit : "")<<',';
        if(sample) file<<sample->valid<<','<<sample->returned<<','<<csvString(sample->raw);
        else file<<",,";
        file<<','<<csvString(sample ? sample->status : detail)<<'\n';
        if(trial.paused()) deferredRows_.push_back({size_t(trial.stage()),file.str()});
        else {flushDeferred();files_[size_t(trial.stage())]<<file.str();}
    }
    void flushDeferred() {
        for(const auto& row:deferredRows_) files_[row.first]<<row.second;
        deferredRows_.clear();
    }
    void holdEvent(const ServoHoldEvent& event,int64_t origin) {
        auto& file=files_[size_t(event.stage)];
        file<<csvString(event.event+(event.error.empty()?"":"_FAILED"))<<','<<double(event.endNs-origin)/1e9
            <<','<<event.beginNs<<','<<event.endNs<<','<<event.stage+1<<",TIMER,"<<event.trial<<','<<event.revision
            <<",,,,"<<(event.error.empty()?"0":"")<<",,,,,,,"<<csvString(event.error)<<'\n';
    }
    void frame(const cv::Mat& image,double fps) {
        // Rotate clips without ending the session or changing its stage.
        if(!writer_.isOpened() || videoFrames_>=1800) {
            closeVideo();videoFrames_=0;
            clip_="camera_"+std::to_string(++clipNumber_)+".avi";
            if(!writer_.open((directory_/clip_).string(),cv::CAP_OPENCV_MJPEG,
                cv::VideoWriter::fourcc('M','J','P','G'),fps,image.size(),image.channels()!=1))
                throw std::runtime_error("Cannot create video output");
        }
        writer_.write(image);++videoFrames_;
    }
    void flush() {for(auto& file:files_) file.flush();}
    void close() {
        flushDeferred();closeVideo();
        for(size_t i=0;i<files_.size();++i) {
            closeText(files_[i],directory_/stageFiles[i]);std::ifstream input(directory_/stageFiles[i]);
            verifyTextOutput(input,csvHeader);
        }
    }
private:
    void closeVideo() {
        if(!writer_.isOpened()) return;
        writer_.release();syncFile(directory_/clip_);
        if(fs::file_size(directory_/clip_)==0) throw std::runtime_error("Video output empty");
        cv::VideoCapture verify((directory_/clip_).string(),cv::CAP_OPENCV_MJPEG);cv::Mat first;
        if(!verify.read(first) || first.empty()) throw std::runtime_error("Video readback failed");
    }
    fs::path directory_;std::array<std::ofstream,6> files_;cv::VideoWriter writer_;
    std::vector<std::pair<size_t,std::string>> deferredRows_;
    std::string clip_;unsigned clipNumber_=0,videoFrames_=0;
};
void showStatus(const ParkingRehearsal& model,std::optional<double> written) {
    std::cout<<"【当前阶段 "<<model.stage()+1<<" "<<stageNames[model.stage()]<<"】 "<<model.stateName()
        <<" | 文件命令="<<model.command()<<" | 最近写入=";
    if(written) std::cout<<*written;else std::cout<<"未写入";
    if(model.paused()) std::cout<<" | 记录已暂停；保存文件仍直接更新舵机，C仅恢复记录";
    else if(model.state()==ParkingRehearsal::State::AwaitApply)
        std::cout<<" | 停稳后Enter授权设置阶段 "<<model.nextStage()+1<<" "<<stageNames[model.nextStage()];
    else if(model.state()==ParkingRehearsal::State::Settling) std::cout<<" | 保持静止，等待舵机";
    else if(model.state()==ParkingRehearsal::State::AwaitStart)
        std::cout<<(model.stage()==5 ? " | Enter开始静态停止确认记录，保持车身静止" : " | Enter授权开始本段手动推/拉");
    else if(model.state()==ParkingRehearsal::State::Running)
        std::cout<<" | 当前段进行中：停稳后Enter结束本段；记录仍继续，P暂停，Q结束";
    std::cout<<std::endl;
}
int run(const Options& options,const Config& config,const car2026::HardwareConfig& hardware,
        const car2026::Params& params,const ParkingTuning& tuning,bool partial) {
    if(!isatty(STDIN_FILENO)) throw std::runtime_error("Interactive terminal required; piped permissions are rejected");
    termios terminal{};
    if(tcgetattr(STDIN_FILENO,&terminal)!=0 || !(terminal.c_lflag&ICANON))
        throw std::runtime_error("Canonical terminal required (press keys then Enter)");
    car2026::HardwareLock lock;
    car2026::LinuxDeviceIo io(hardware); // Construction only stores protocol configuration.
    std::unique_ptr<RehearsalServo> servo;
    cv::VideoCapture camera(hardware.camera_device,cv::CAP_V4L2);
    if(!camera.isOpened()) throw std::runtime_error("Cannot open camera");
    camera.set(cv::CAP_PROP_FRAME_WIDTH,config.width);camera.set(cv::CAP_PROP_FRAME_HEIGHT,config.height);
    camera.set(cv::CAP_PROP_FPS,config.fps);camera.set(cv::CAP_PROP_BUFFERSIZE,1);
    const int64_t origin=monotonicNs();
    const fs::path directory=fs::path(options.output)/("rehearsal_"+std::to_string(std::time(nullptr))+"_"+std::to_string(getpid())+"_"+std::to_string(origin));
    fs::create_directories(directory.parent_path());
    if(!fs::create_directory(directory)) throw std::runtime_error("Session already exists");
    ParkingRehearsal model(tuning);Recording recording(directory);std::optional<double> written;
    saveSnapshot(directory,1,tuning);
    fs::copy_file(options.capture,directory/"capture_config.ini");
    fs::copy_file(options.hardware,directory/"hardware_config.ini");
    fs::copy_file(options.vehicle,directory/"vehicle_config.ini");
    SavedTuningWatcher watcher(tuning,params.max_steer_deg);
    unsigned savedRevision=1;double lastStatus=-1,lastFlush=0,lastFilePoll=-1;std::string lastState,lastFileError;
    std::array<Sample,8> latestSamples;bool hasSamples=false;
    std::string failure;std::deque<ServoHoldEvent> deferredEvents;
    const auto collectHoldEvents=[&](bool final=false) {
        if(!servo) return;
        auto events=servo->takeEvents();
        for(const auto& event:events) {
            deferredEvents.push_back(event);
            std::cout<<(event.error.empty()?"【时间到，舵机已自动回正】":"【定时回正失败，请停止推/拉】")
                <<" 阶段="<<event.stage+1<<"；记录继续，阶段不自动切换。"<<std::endl;
        }
        if(!model.paused() || final) {
            for(const auto& event:deferredEvents) recording.holdEvent(event,origin);
            deferredEvents.clear();
        }
        for(const auto& event:events) if(!event.error.empty()) throw std::runtime_error("AUTO_CENTER_FAILED: "+event.error);
    };
    const auto beginTrialHold=[&](const std::string& source) {
        const auto duration=model.tuning().stages[size_t(model.stage())].holdSeconds;
        const auto start=servo->beginHold(duration,model.stage(),model.revision(),model.trial());
        if(start) recording.row(model,"HOLD_START",*start,monotonicNs(),origin,written,"hold_time_s="+std::to_string(duration)+" source="+source);
        return start.has_value();
    };
    std::cout<<"SESSION "<<directory<<"\nSERVO ONLY; no motor/GPIO-output/IMU initialization.\n"
             <<"保存调参文件即授权执行；新角度写入成功后直接计时，无需R/C/再次回车。记录直到P暂停或Q结束。字母键后按Enter。\n";
    try {
        while(!model.finished()) {
            collectHoldEvents();
            auto key=interrupted ? RehearsalKey::Quit : readKey();
            if(key==RehearsalKey::Reload) std::cout<<"程序自动监测文件；直接保存即可，不需要R。\n";
            if(key==RehearsalKey::Invalid) std::cout<<"输入无效；仅Enter/P/C/R/Q，连续多行不授权。\n";
            if(servo) written=servo->written();
            const auto before=monotonicNs();const bool wasPaused=model.paused();
            const auto oldState=model.state();
            auto request=model.update(double(before-origin)/1e9,key);bool savedExecution=false;
            const double fileNow=double(monotonicNs()-origin)/1e9;
            if(!model.finished() && key!=RehearsalKey::Pause && !request && fileNow-lastFilePoll>=.2) {
                lastFilePoll=fileNow;
                std::optional<ParkingTuning> candidate;
                try {
                    candidate=watcher.observe(ParkingTuning::readSource(options.tuning),fileNow);
                    lastFileError.clear();
                } catch(const std::exception& error) {
                    if(lastFileError!=error.what()) std::cout<<"CONFIG_REJECTED "<<error.what()<<"；原参数及当前计时继续。\n";
                    lastFileError=error.what();
                }
                if(candidate) {
                    request=model.applySaved(*candidate,double(monotonicNs()-origin)/1e9);
                    savedExecution=request.has_value();
                    std::cout<<"CONFIG_AUTO_APPLIED revision="<<model.revision()<<" stage="<<model.stage()+1
                        <<(savedExecution ? "；直接设置舵机并重新计时。" : "；更新其他阶段参数，当前段不重启。")<<std::endl;
                }
            }
            if(!savedExecution && oldState==ParkingRehearsal::State::AwaitStart && model.state()==ParkingRehearsal::State::Running && servo) {
                beginTrialHold("ENTER");
            }
            if(model.revision()!=savedRevision) {
                saveSnapshot(directory,model.revision(),model.tuning());savedRevision=model.revision();
                recording.row(model,"CONFIG_AUTO_APPLIED",before,monotonicNs(),origin,written,savedExecution?"EXECUTE_SAVED_STAGE":"FUTURE_STAGE_ONLY");
            }
            if(key!=RehearsalKey::None && key!=RehearsalKey::Reload && (!wasPaused || !model.paused()))
                recording.row(model,"KEY",before,monotonicNs(),origin,written,rehearsalKeyName(key));
            if(model.finished()) break; // Q never waits for another camera read.
            // Metadata initialization precedes the fresh frame; a slow device open cannot age it.
            if(request && !servo) servo=std::make_unique<RehearsalServo>(io,hardware,params);
            cv::Mat frame;const auto frameStart=monotonicNs();
            if(!camera.read(frame) || frame.empty()) throw std::runtime_error("Camera read failed");
            const auto frameEnd=monotonicNs();
            if(frame.cols!=config.width || frame.rows!=config.height || frame.depth()!=CV_8U ||
               (frame.channels()!=1 && frame.channels()!=3)) throw std::runtime_error("Unexpected camera frame format");
            const auto frameWritten=written;
            if(request) {
                collectHoldEvents();
                if(interrupted) {model.stop("USER_QUIT");break;}
                if(double(monotonicNs()-frameStart)/1e9>params.frame_timeout_s)
                    throw std::runtime_error("Camera frame stale before authorized servo write");
                const auto writeStart=monotonicNs();const auto duty=servo->set(*request);
                const auto writeEnd=monotonicNs();written=*request;
                model.acknowledge(true,double(writeEnd-origin)/1e9);
                const bool timerArmed=savedExecution ? beginTrialHold("FILE_SAVE") : false;
                recording.row(model,"SERVO_WRITE",writeStart,writeEnd,origin,written,"duty="+std::to_string(duty));
                if(savedExecution) {
                    std::cout<<"【保存参数已执行】阶段="<<model.stage()+1<<" command="<<*request
                        <<" hold_time_s="<<model.tuning().stages[size_t(model.stage())].holdSeconds
                        <<(timerArmed ? "；计时已从写入成功开始。" : "；命令已回正或保持时间为0，不启用定时回正。")
                        <<"记录暂停状态不变。"<<std::endl;
                }
                tcflush(STDIN_FILENO,TCIFLUSH); // No typed-ahead permission after hardware latency.
            }
            const auto now=monotonicNs();const double elapsed=double(now-origin)/1e9;
            if(!model.paused()) {
                recording.frame(frame,config.fps);
                recording.row(model,"FRAME",frameStart,frameEnd,origin,frameWritten);
                for(size_t i=0;i<config.sensors.size();++i) {
                    const auto& sensor=config.sensors[i];const auto start=monotonicNs();
                    latestSamples[i]=readSensor(sensor,config.factoryEncoderStatusZero,interrupted);
                    recording.row(model,"SENSOR",start,monotonicNs(),origin,written,"",&sensor,&latestSamples[i]);
                }
                hasSamples=true;
            }
            if(elapsed-lastFlush>=1) {
                recording.flush();lastFlush=elapsed;
                if(fs::space(directory).available<64u*1024u*1024u) throw std::runtime_error("Output storage below 64 MiB");
            }
            if(lastState!=model.stateName() || elapsed-lastStatus>=1) {
                if(servo) written=servo->written();
                showStatus(model,written);
                if(servo) std::cout<<"本段保持时间="<<model.tuning().stages[size_t(model.stage())].holdSeconds
                    <<"s | 定时回正剩余="<<servo->remaining()<<"s (P暂停记录不暂停计时)"<<std::endl;
                if(!model.paused() && hasSamples) {
                    for(size_t index:{size_t(6),size_t(7)}) {
                        std::cout<<config.sensors[index].name<<'=';
                        if(latestSamples[index].valid) std::cout<<latestSamples[index].value<<' '<<config.sensors[index].unit;
                        else std::cout<<"INVALID("<<latestSamples[index].status<<')';
                        std::cout<<' ';
                    }
                    std::cout<<"(原始值，未自动换算距离)"<<std::endl;
                }
                lastState=model.stateName();lastStatus=elapsed;
            }
        }
    } catch(const std::exception& error) {
        failure=error.what();model.stop("ERROR");
        std::cerr<<"【停止推/拉】ERROR "<<failure<<'\n';
    }
    // Manual Q/Ctrl+C or unavoidable error cleanup; never an automatic parking-success event.
    bool exitZeroAttempted=false,exitZeroSucceeded=false;
    if(servo) {
        exitZeroAttempted=servo->attempted();
        ServoHoldEvent exit{model.stage(),model.revision(),model.trial(),monotonicNs(),0,"","EXIT_ZERO"};
        try {
            servo->close();exitZeroSucceeded=exitZeroAttempted;
            std::cout<<(exitZeroAttempted ? "SERVO_ZERO software write completed\n" : "SERVO_ZERO not attempted; no authorized write occurred\n");
        } catch(const std::exception& error) {exit.error=error.what();failure+="; ZERO_FAILED: "+exit.error;}
        exit.endNs=monotonicNs();
        if(exitZeroAttempted) recording.holdEvent(exit,origin);
    }
    try {collectHoldEvents(true);} catch(const std::exception& error) {failure+="; "+std::string(error.what());}
    recording.close();
    auto metadata=outputFile(directory/"session.txt");
    metadata<<"version="<<rehearsalVersion<<"\nresult="<<(failure.empty()?model.outcome():"ERROR")
        <<"\nerror="<<failure<<"\nmotor_writes=0\nimu_initialized=0\npartial="<<partial
        <<"\nservo_zero_on_exit="<<(exitZeroAttempted ? (exitZeroSucceeded ? "software_write_completed":"failed") : "not_attempted")
        <<"\nmanual_parking_success=not_inferred\nvideo_timing=use_csv_monotonic_timestamps_not_nominal_fps\n";
    closeText(metadata,directory/"session.txt");
    std::cout<<"SAVED "<<directory<<" result="<<(failure.empty()?model.outcome():"ERROR")<<std::endl;
    return failure.empty() ? (partial?2:0) : 1;
}
}
int main(int argc,char** argv) {
    try {
        const auto options=Options::parse(argc,argv);
        if(options.help) {
            std::cout<<"parking_rehearsal version="<<rehearsalVersion<<"\n"
                <<"--config file --hardware-config file --vehicle-config file --tuning-config file --output directory\n"
                <<"--allow-partial --check-config (no hardware access)\n"
                <<"Enter: initial/manual-stage permission; P: pause recording; C: resume recording; Q: finish.\n"
                <<"auto_reload=save; saved settings execute immediately and restart hold_time_s after successful write. Timer continues while paused.\n"
                <<"SERVO ONLY. Six stage CSV files.\n";return 0;
        }
        auto config=loadConfig(options.capture);const auto hardware=car2026::HardwareConfig::load(options.hardware);
        const auto params=car2026::Params::load(options.vehicle);const auto tuning=ParkingTuning::load(options.tuning);
        tuning.validate(params.max_steer_deg);fillEncoderPaths(config,hardware);
        const auto missing=validateSensors(config,options.partial);
        for(const auto& name:missing) std::cout<<"UNCONFIGURED "<<name<<'\n';
        if(options.check) {std::cout<<"Config checked without hardware access\n";return missing.empty()?0:2;}
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        return run(options,config,hardware,params,tuning,!missing.empty());
    } catch(const std::exception& error) {std::cerr<<"parking_rehearsal: "<<error.what()<<'\n';return 1;}
}
