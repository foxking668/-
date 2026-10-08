#include "parking_rehearsal.hpp"
#include "capture_sensors.hpp"
#include "capture_files.hpp"
#include "hardware_linux.hpp"
#include "rehearsal_servo.hpp"
#include "rehearsal_session.hpp"
#include "rehearsal_terminal.hpp"
#include "rehearsal_speed.hpp"
#include "rehearsal_motor.hpp"
#include "rehearsal_timing.hpp"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
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
std::vector<std::string> readTerminalLines() {
    std::vector<std::string> lines;pollfd input{STDIN_FILENO,POLLIN,0};
    for(unsigned count=0;count<32;++count) {
        int ready;do {ready=poll(&input,1,0);} while(ready<0 && errno==EINTR);
        if(ready<0) throw std::runtime_error("Terminal poll failed");
        if(!ready) break;
        if(input.revents&(POLLERR|POLLHUP|POLLNVAL)) throw std::runtime_error("Terminal disconnected");
        char buffer[4096];const auto size=read(STDIN_FILENO,buffer,sizeof(buffer));
        if(size<=0) throw std::runtime_error("Terminal read failed or EOF");
        std::string text(buffer,size_t(size));
        if(text.back()!='\n') {tcflush(STDIN_FILENO,TCIFLUSH);return {"INVALID_BATCH"};}
        size_t start=0;
        for(size_t end=text.find('\n');end!=std::string::npos;end=text.find('\n',start)) {
            lines.push_back(text.substr(start,end-start));start=end+1;
        }
        if(count==31) {tcflush(STDIN_FILENO,TCIFLUSH);return {"INVALID_BATCH"};}
    }
    return lines;
}
RehearsalKey readKey() {return parseRehearsalInput(readTerminalLines());}

SaveChoice askSave() {
    SaveConfirmationTerminal terminal(STDIN_FILENO);
    const auto choice=readSaveConfirmation([&] {return terminal.readKey();},std::cout);
    terminal.restore();return choice;
}
LineObservation observeLine(const cv::Mat& frame,LineFollowSession& session,const LineFollowTuning& tuning) {
    const int top=int(frame.rows*tuning.cropTop),bottom=int(frame.rows*tuning.cropBottom);
    if(bottom-top<16) throw std::runtime_error("Camera too small for line crop");
    cv::Mat gray,small;
    const auto crop=frame(cv::Rect(0,top,frame.cols,bottom-top));
    if(frame.channels()==3) cv::cvtColor(crop,gray,cv::COLOR_BGR2GRAY);else gray=crop;
    cv::resize(gray,small,cv::Size(lineWidth,lineHeight),0,0,cv::INTER_AREA);
    LineGray pixels{};
    for(int y=0;y<lineHeight;++y) std::copy_n(small.ptr<unsigned char>(y),lineWidth,pixels.begin()+y*lineWidth);
    return session.tracker.analyze(pixels,tuning);
}
class SessionRetention {
public:
    explicit SessionRetention(const fs::path& directory):files_(directory) {}
    ~SessionRetention() noexcept {
        if(asked_) return;
        try {
            const auto choice=decide();
            if(choice==SaveChoice::Yes) std::cerr<<"KEPT_INCOMPLETE "<<files_.directory()<<"；本次异常退出，文件未完成验证。\n";
        } catch(const std::exception& error) {std::cerr<<"SAVE_UNCONFIRMED "<<files_.directory()<<": "<<error.what()<<"；暂存记录保留，未认定已保存。\n";}
    }
    SaveChoice decide() {
        asked_=true;
        SaveChoice choice;
        try {choice=askSave();}
        catch(const std::exception& error) {
            throw std::runtime_error("SAVE_UNCONFIRMED "+files_.directory().string()+": "+error.what()+"; pending records retained");
        }
        if(choice==SaveChoice::No) {files_.discard();std::cout<<"DISCARDED "<<files_.directory()<<std::endl;}
        return choice;
    }
    void saved() {files_.saved();}
private:
    SessionFiles files_;bool asked_=false;
};
void saveSnapshot(const fs::path& directory,unsigned revision,const ParkingTuning& tuning) {
    auto output=outputFile(directory/("config_"+std::to_string(revision)+".ini"));
    output<<tuning.source;closeText(output,directory/("config_"+std::to_string(revision)+".ini"));
}
constexpr const char* csvHeader="event,elapsed_s,read_start_ns,read_end_ns,stage,state,segment,trial,config_revision,clip,video_frame,configured_command,last_written_command,channel,value,unit,valid,read_return,raw_hex,detail";
class Recording {
public:
    explicit Recording(fs::path directory):directory_(std::move(directory)) {
        for(size_t i=0;i<files_.size();++i) {files_[i]=outputFile(directory_/stageFiles[i]);files_[i]<<csvHeader<<'\n';files_[i].flush();}
    }
    void row(const ParkingRehearsal& trial,const std::string& event,int64_t begin,int64_t end,int64_t origin,
             std::optional<double> written,const std::string& detail="",const SensorSpec* sensor=nullptr,const Sample* sample=nullptr) {
        std::ostringstream file;file<<std::setprecision(17);
        file<<csvString(event)<<','<<double(end-origin)/1e9<<','<<begin<<','<<end<<','<<trial.stage()+1<<','
            <<csvString(trial.stateName())<<','<<csvString(trial.segmentName())<<','<<trial.trial()<<','<<trial.revision()<<','<<csvString(clip_)<<',';
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
            <<','<<event.beginNs<<','<<event.endNs<<','<<event.stage+1<<",TIMER,"<<csvString(rehearsalSegmentName(event.stage,event.segment))<<','<<event.trial<<','<<event.revision
            <<",,,,"<<(event.error.empty()?"0":"")<<",,,,,,,"<<csvString(event.error)<<'\n';
    }
    void motorEvent(const MotorEvent& event,int64_t origin) {
        auto& file=files_[size_t(event.stage)];
        file<<csvString(event.event+(event.error.empty() || event.event=="MOTOR_START_FAILED" ? "" : "_FAILED"))<<','<<double(event.endNs-origin)/1e9
            <<','<<event.beginNs<<','<<event.endNs<<','<<event.stage+1<<",MOTOR,PRIMARY,"<<event.trial<<','<<event.revision
            <<",,,,,,,,,,,"<<csvString(motorEventDetail(event))<<'\n';
    }
    void speed(const ParkingRehearsal& model,const RehearsalSpeedWindow& window,int64_t origin,std::optional<double> written) {
        const auto add=[&](const std::string& name,double value,const std::string& unit) {
            derived(model,"SPEED",name,value,unit,window.endNs,origin,written,"encoder_window_0.5s_or_longer");
        };
        for(size_t i=0;i<2;++i) {
            const std::string side=i==0 ? "left" : "right";
            add("speed_"+side,window.rate(i),"counts/s");
            add("speed_"+side+"_window",window.seconds[i],"s");
            if(window.scale[i]>0) add("speed_"+side+"_cm_s",window.rate(i)*window.scale[i],"cm/s");
        }
        if(window.calibrated()) add("speed_center_cm_s",window.centerCmPerSecond(),"cm/s");
    }
    void line(const ParkingRehearsal& model,const LineObservation& line,double error,int64_t end,int64_t origin,std::optional<double> written) {
        const auto add=[&](const std::string& name,double value,const std::string& unit) {
            derived(model,"LINE_FEEDBACK",name,value,unit,end,origin,written,line.reliable ? "detected" : "unreliable");
        };
        add("line_reliable",line.reliable,"bool");add("line_rows",line.rows,"rows");add("line_near_rows",line.nearRows,"rows");
        if(line.reliable) {add("line_target_x",line.target,"px");add("line_near_x",line.nearX,"px");add("line_far_x",line.farX,"px");add("line_filtered_error",error,"px");}
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
    void derived(const ParkingRehearsal& model,const std::string& event,const std::string& name,double value,
                 const std::string& unit,int64_t end,int64_t origin,std::optional<double> written,const std::string& status) {
        SensorSpec sensor{name,"","derived",unit};Sample sample;
        sample.valid=true;sample.value=value;sample.status=status;
        row(model,event,end,end,origin,written,"",&sensor,&sample);
    }
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
        <<" | 步骤="<<model.segmentTitle()
        <<" | 文件命令="<<model.command()<<" | 最近写入=";
    if(written) std::cout<<*written;else std::cout<<"未写入";
    if(model.paused()) std::cout<<" | 记录已暂停；电机停机，C恢复记录后须Enter重新运动";
    else if(model.state()==ParkingRehearsal::State::AwaitApply)
        std::cout<<" | 停稳后按一次Enter：开始记录并设置阶段 "<<model.nextStage()+1<<" "<<stageNames[model.nextStage()];
    else if(model.state()==ParkingRehearsal::State::Settling) std::cout<<" | 保持静止，等待舵机";
    else if(model.state()==ParkingRehearsal::State::Running) {
        if(model.correctionSequenceActive()) std::cout<<" | 两步转向进行中：主转弯后自动反向修正，最后回正；Q提前结束，P停电机并暂停记录";
        else std::cout<<" | 当前段进行中：定时回正后按模式收尾；停稳后Enter结束本段，P暂停记录，Q提前结束";
    }
    std::cout<<std::endl;
    const auto& settings=model.currentTuning();
    std::cout<<"电机原始PWM 左="<<settings.motorLeft<<" 右="<<settings.motorRight<<" | 运行上限="<<settings.motorSeconds
        <<"s | 方向="<<(model.stage()==0 ? "前进" : model.stage()==5 ? "停止" : "倒退")
        <<(settings.powered() ? " | 已配置电机试验" : " | 电机禁用")<<std::endl;
}
int run(const Options& options,const Config& config,const car2026::HardwareConfig& hardware,
        const car2026::Params& params,const ParkingTuning& tuning,bool partial) {
    if(!isatty(STDIN_FILENO)) throw std::runtime_error("Interactive terminal required; piped permissions are rejected");
    termios terminal{};
    if(tcgetattr(STDIN_FILENO,&terminal)!=0 || !(terminal.c_lflag&ICANON))
        throw std::runtime_error("Canonical terminal required (press keys then Enter)");
    car2026::HardwareLock lock;
    car2026::LinuxDeviceIo io(hardware); // Construction only stores protocol configuration.
    // Independent logging/readback caches: servo and motor workers must not share mutable DeviceIo sets.
    car2026::LinuxDeviceIo motorIo(hardware);
    std::unique_ptr<RehearsalServo> servo;
    std::unique_ptr<RehearsalMotor> motor;
    cv::VideoCapture camera(hardware.camera_device,cv::CAP_V4L2);
    if(!camera.isOpened()) throw std::runtime_error("Cannot open camera");
    camera.set(cv::CAP_PROP_FRAME_WIDTH,config.width);camera.set(cv::CAP_PROP_FRAME_HEIGHT,config.height);
    camera.set(cv::CAP_PROP_FPS,config.fps);camera.set(cv::CAP_PROP_BUFFERSIZE,1);
    const int64_t origin=monotonicNs();
    const fs::path directory=fs::path(options.output)/("rehearsal_"+std::to_string(std::time(nullptr))+"_"+std::to_string(getpid())+"_"+std::to_string(origin));
    fs::create_directories(directory.parent_path());
    if(!fs::create_directory(directory)) throw std::runtime_error("Session already exists");
    SessionRetention retention(directory);
    ParkingRehearsal model(tuning);Recording recording(directory);std::optional<double> written;
    RehearsalSpeedRecorder speed(hardware.encoder_mode,hardware.encoder_left_sign*params.encoder_left_sign,
        hardware.encoder_right_sign*params.encoder_right_sign);
    RehearsalTiming timing(hardware.encoder_mode);
    saveSnapshot(directory,1,tuning);
    fs::copy_file(options.capture,directory/"capture_config.ini");
    fs::copy_file(options.hardware,directory/"hardware_config.ini");
    fs::copy_file(options.vehicle,directory/"vehicle_config.ini");
    SavedTuningWatcher watcher(tuning,params.max_steer_deg);
    unsigned savedRevision=1;double lastStatus=-1,lastFlush=0,lastFilePoll=-1;std::string lastState,lastFileError;
    std::array<Sample,8> latestSamples;bool hasSamples=false;
    std::string failure;std::deque<ServoHoldEvent> deferredEvents;
    std::optional<MotorEvent> stopWait;bool stopConfirmed=false,motorExitZero=false;
    LineFollowSession lineSession;LineObservation lineObservation;
    bool pendingMotion=false;
    const auto collectMotorEvents=[&](bool final=false) {
        if(!motor) return;
        for(const auto& event:motor->takeEvents()) {
            recording.motorEvent(event,origin);
            std::cout<<event.event<<" "<<motorEventDetail(event)<<std::endl;
            if(event.event=="MOTOR_START") timing.motorStart(event.endNs);
            else if(event.motorZeroNs) timing.motorStop(event.motorZeroNs,event.event+(event.error.empty() ? "" : "_FAILED"));
            else timing.end(event.endNs,event.event+"_FAILED");
            if(!event.error.empty()) throw std::runtime_error("MOTOR_ZERO_FAILED: "+event.error);
            if(!final && event.event=="MOTOR_STOP_WATCHDOG") throw std::runtime_error("Motor stopped: camera/encoder heartbeat expired");
            if(!final && (event.event=="MOTOR_STOP_DURATION" || event.event=="MOTOR_STOP_STEER_COMPLETED" || event.event=="MOTOR_STOP_LINE_LOST")) {
                stopWait=event;
                pendingMotion=false;lineSession.reset();
                if(servo) written=servo->written(); // Stop worker already centered under the same motor lock.
                std::cout<<"【电机已写零，舵机回正】继续记录停稳情况；不再启动新动作。"<<std::endl;
            }
        }
    };
    const auto collectHoldEvents=[&](bool final=false) {
        if(!servo) return;
        auto events=servo->takeEvents();
        for(const auto& event:events) {
            deferredEvents.push_back(event);
            const bool matches=event.stage==model.stage() && event.trial==model.trial() && event.segment==model.segment();
            const bool lastAction=event.segment==1 || !isBendStage(event.stage) || model.currentTuning().correction.holdSeconds==0;
            if(!final && event.error.empty() && matches && lastAction && motor && model.currentTuning().powered() &&
               model.state()==ParkingRehearsal::State::Running && !model.motorRestartRequired()) {
                motor->stop("MOTOR_STOP_STEER_COMPLETED");collectMotorEvents();
            } else if(event.error.empty() && !stopWait) model.holdCompleted(event.stage,event.trial,event.segment);
            const char* message=!event.error.empty() ? "【定时回正失败，请停止推/拉】" :
                model.pendingCorrection() ? "【主转弯时间到，正在自动切换反向修正】" : "【时间到，舵机已自动回正】";
            std::cout<<message<<" 阶段="<<event.stage+1<<" 步骤="<<rehearsalSegmentName(event.stage,event.segment)<<"。"<<std::endl;
        }
        if(!model.paused() || final) {
            for(const auto& event:deferredEvents) recording.holdEvent(event,origin);
            deferredEvents.clear();
        }
        for(const auto& event:events) if(!event.error.empty()) throw std::runtime_error("AUTO_CENTER_FAILED: "+event.error);
    };
    const auto beginTrialHold=[&](const std::string& source) {
        const auto duration=model.holdSeconds();
        const auto start=servo->beginHold(duration,model.stage(),model.revision(),model.trial(),model.segment());
        if(start) recording.row(model,"HOLD_START",*start,monotonicNs(),origin,written,"hold_time_s="+std::to_string(duration)+" source="+source);
        return start.has_value();
    };
    std::cout<<"SESSION "<<directory<<"\nSingle-stage motor tuning; no IMU initialization. motor_run_time_s=0 disables motor access.\n"
             <<"按一次Enter开始记录并设置舵机；静止等待结束后自动提示开始推/拉并计时，无须第二次Enter。\n"
             <<"可选两步修正；P暂停记录并停电机，C只恢复记录，Enter重新运动；Q提前结束。Y/N直接按键。\n"
             <<"速度默认counts/s；只有填写实测speed_*_cm_per_count后才输出cm/s，不使用未标定轮周长默认值。\n";
    try {
        while(!model.finished()) {
            collectMotorEvents();
            collectHoldEvents();
            if(model.finished()) break;
            auto key=interrupted ? RehearsalKey::Quit : readKey();
            if(motor && ((key==RehearsalKey::Pause && model.currentTuning().powered()) || key==RehearsalKey::Quit)) {
                motor->stop(key==RehearsalKey::Pause ? "MOTOR_STOP_PAUSE" : "MOTOR_STOP_QUIT");collectMotorEvents();
                if(servo) {servo->set(0);written=0;}
                pendingMotion=false;lineSession.reset();
            }
            if(stopWait && key!=RehearsalKey::Quit && key!=RehearsalKey::Pause && key!=RehearsalKey::Continue) key=RehearsalKey::None;
            if(key==RehearsalKey::Reload) std::cout<<"程序自动监测文件；直接保存即可，不需要R。\n";
            if(key==RehearsalKey::Invalid) std::cout<<"输入无效；仅Enter/P/C/R/Q，连续多行不授权。\n";
            if(servo) written=servo->written();
            const auto before=monotonicNs();const bool wasPaused=model.paused();
            const auto oldState=model.state();
            auto request=stopWait ? std::optional<double>{} : model.update(double(before-origin)/1e9,key);bool savedExecution=false;
            if(key==RehearsalKey::Pause || key==RehearsalKey::Quit) {pendingMotion=false;lineSession.reset();}
            if(stopWait && (key==RehearsalKey::Pause || key==RehearsalKey::Continue)) model.update(double(before-origin)/1e9,key);
            if(stopWait && key==RehearsalKey::Quit) model.stop("USER_QUIT");
            const bool correctionExecution=request.has_value() && model.automaticCorrectionWrite();
            const double fileNow=double(monotonicNs()-origin)/1e9;
            if(!model.finished() && !stopWait && key!=RehearsalKey::Pause && !request && fileNow-lastFilePoll>=.2) {
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
                    // Preview decides whether this save affects the active action. Stop before replacing it.
                    auto preview=model;const auto previewRequest=preview.applySaved(*candidate,double(monotonicNs()-origin)/1e9);
                    if((previewRequest || preview.finished()) && motor) {motor->stop("MOTOR_STOP_RECONFIG");collectMotorEvents();}
                    if(previewRequest || preview.finished()) timing.end(monotonicNs(),"CONFIG_REPLACED");
                    if(!stopWait) request=model.applySaved(*candidate,double(monotonicNs()-origin)/1e9);
                    savedExecution=request.has_value();
                    std::cout<<"CONFIG_AUTO_APPLIED revision="<<model.revision()<<" stage="<<model.stage()+1
                        <<(model.finished() ? "；结束当前修正试验并回正。" : savedExecution ? "；直接设置当前步骤舵机并重新计时。" : "；更新其他阶段参数，当前段不重启。")<<std::endl;
                }
            }
            bool motionReady=!stopWait && !savedExecution && oldState==ParkingRehearsal::State::Settling && model.state()==ParkingRehearsal::State::Running;
            if(oldState==ParkingRehearsal::State::Running && model.state()==ParkingRehearsal::State::AwaitApply) {
                if(motor) {motor->stop("MOTOR_STOP_STAGE");collectMotorEvents();}
                timing.end(before,"STAGE_END");
                pendingMotion=false;lineSession.reset();
            }
            if(model.revision()!=savedRevision) {
                saveSnapshot(directory,model.revision(),model.tuning());savedRevision=model.revision();
                recording.row(model,"CONFIG_AUTO_APPLIED",before,monotonicNs(),origin,written,
                    model.finished()?"END_CURRENT_CORRECTION":savedExecution?"EXECUTE_SAVED_STAGE":"FUTURE_STAGE_ONLY");
            }
            if(key!=RehearsalKey::None && key!=RehearsalKey::Reload && (!wasPaused || !model.paused()))
                recording.row(model,"KEY",before,monotonicNs(),origin,written,rehearsalKeyName(key));
            if(model.finished()) break; // Q never waits for another camera read.
            // Metadata initialization precedes the fresh frame; a slow device open cannot age it.
            if(request && !servo) servo=std::make_unique<RehearsalServo>(io,hardware,params);
            if(request && model.currentTuning().powered() && !motor && !model.motorRestartRequired()) {
                motor=std::make_unique<RehearsalMotor>(motorIo,hardware,params);
                motor->setStopAction([&] {servo->set(0);});
            }
            cv::Mat frame;const auto frameStart=monotonicNs();
            if(!camera.read(frame) || frame.empty()) throw std::runtime_error("Camera read failed");
            const auto frameEnd=monotonicNs();
            if(frame.cols!=config.width || frame.rows!=config.height || frame.depth()!=CV_8U ||
               (frame.channels()!=1 && frame.channels()!=3)) throw std::runtime_error("Unexpected camera frame format");
            const auto frameWritten=written;
            if(request) {collectMotorEvents();if(stopWait) request.reset();}
            if(request) {
                pendingMotion=false;lineSession.reset();
                collectHoldEvents();
                if(model.finished()) break;
                if(interrupted) {model.stop("USER_QUIT");break;}
                if(double(monotonicNs()-frameStart)/1e9>params.frame_timeout_s)
                    throw std::runtime_error("Camera frame stale before authorized servo write");
                const auto writeStart=monotonicNs();const auto duty=servo->set(*request);
                const auto writeEnd=monotonicNs();written=*request;
                model.acknowledge(true,double(writeEnd-origin)/1e9);
                recording.row(model,"SERVO_WRITE",writeStart,writeEnd,origin,written,"duty="+std::to_string(duty));
                if((savedExecution || correctionExecution) && model.state()==ParkingRehearsal::State::Running) motionReady=true;
                if(correctionExecution) std::cout<<"【"<<model.segmentTitle()<<"已执行，继续按原方向后拉】command="<<*request
                    <<" hold_time_s="<<model.holdSeconds()<<"；到时回正，当前弯道动作完成。"<<std::endl;
                if(savedExecution) {
                    std::cout<<"【保存参数已执行】阶段="<<model.stage()+1<<" command="<<*request
                        <<" step="<<model.segmentName()<<" hold_time_s="<<model.holdSeconds()
                        <<"；按状态提示等待/执行；计时将在动作允许后开始。"
                        <<"记录暂停状态不变。"<<std::endl;
                }
                tcflush(STDIN_FILENO,TCIFLUSH); // No typed-ahead permission after hardware latency.
            }
            const bool following=model.stage()==0 && model.currentTuning().line.enabled;
            const auto& lineTuning=model.currentTuning().line;
            if(motionReady) {
                pendingMotion=true;
                if(following && model.currentTuning().powered()) {
                    lineSession.wait(double(monotonicNs()-origin)/1e9);
                    recording.row(model,"WAIT_LINE",frameEnd,monotonicNs(),origin,written,"motor_timer_not_started");
                    std::cout<<"【确认主线 / WAIT_LINE】保持静止，连续确认黑线后自动启动；此时尚未计前移时间。"<<std::endl;
                }
            }
            if(following && model.recordingEnabled() && !stopWait) {
                lineObservation=observeLine(frame,lineSession,lineTuning);
                const double lineNow=double(monotonicNs()-origin)/1e9;
                lineSession.observe(lineObservation,lineNow);
                if(lineSession.expired(lineNow,lineTuning))
                    throw std::runtime_error("LINE_ACQUIRE_TIMEOUT: no continuous reliable black line; motors not started");
            }
            if(motor && motor->active() && double(monotonicNs()-frameStart)/1e9>params.frame_timeout_s)
                throw std::runtime_error("Camera frame stale during motor motion");
            const auto now=monotonicNs();const double elapsed=double(now-origin)/1e9;
            if(model.recordingEnabled()) {
                recording.frame(frame,config.fps);
                recording.row(model,"FRAME",frameStart,frameEnd,origin,frameWritten);
                WheelSpeedSample wheels[2];
                for(size_t i=0;i<config.sensors.size();++i) {
                    const auto& sensor=config.sensors[i];const auto start=monotonicNs();
                    latestSamples[i]=readSensor(sensor,config.factoryEncoderStatusZero,interrupted);
                    const auto end=monotonicNs();
                    recording.row(model,"SENSOR",start,end,origin,written,"",&sensor,&latestSamples[i]);
                    if(i<2) wheels[i]={latestSamples[i].valid,latestSamples[i].value,end};
                }
                const bool validWheels=timing.add(wheels[0],wheels[1]);
                if(motor && motor->active()) {
                    if(!validWheels || !wheels[0].valid || !wheels[1].valid) throw std::runtime_error("Invalid encoder feedback during motor motion");
                    motor->heartbeat();
                }
                collectMotorEvents();
                if(pendingMotion && !stopWait && !model.motorRestartRequired() &&
                   (!following || !model.currentTuning().powered() || lineSession.ready())) {
                    const bool correction=correctionExecution || (model.segment()==1 && motor && motor->active());
                    if(!correction) timing.begin(model.stage(),model.trial(),model.revision(),monotonicNs());
                    if(!correction) speed.reset();
                    if(model.currentTuning().powered() && !correction) {
                        // Fresh camera + valid encoder pair required before any nonzero motor command.
                        if(!wheels[0].valid || !wheels[1].valid || double(monotonicNs()-frameStart)/1e9>params.frame_timeout_s)
                            throw std::runtime_error("Fresh camera and encoder pair required before motor start");
                        const auto& settings=model.currentTuning();const double direction=model.stage()==0 ? 1 : -1;
                        motor->start(direction*settings.motorLeft,direction*settings.motorRight,settings.motorSeconds,model.stage(),model.revision(),model.trial());
                        if(following) {
                            lineSession.started();
                            recording.row(model,"LINE_ACQUIRED",frameEnd,monotonicNs(),origin,written,"consecutive_real_detections_before_motor_start");
                        }
                        collectMotorEvents();
                    }
                    if(!stopWait) beginTrialHold(correction ? "AUTO_CORRECTION" : "MOTION_READY");
                    recording.row(model,"MOTION_READY",before,monotonicNs(),origin,written);
                    std::cout<<(model.currentTuning().powered() ? "【电机开始执行】" : model.stage()==5 ? "【停止确认，请保持静止】" : "【开始推/拉】")<<std::endl;
                    pendingMotion=false;
                }
                if(following && !stopWait && motor && motor->active()) {
                    const double lineNow=double(monotonicNs()-origin)/1e9;
                    if(lineObservation.reliable) {
                        const double command=lineSession.controller.update(lineObservation,lineTuning,lineNow);
                        if(interrupted) {motor->stop("MOTOR_STOP_QUIT");model.stop("USER_QUIT");}
                        else {
                            const auto start=monotonicNs();int64_t end=start;
                            const bool applied=motor->whileActive([&] {servo->adjust(command);end=monotonicNs();});
                            if(applied) {
                                written=command;
                                recording.row(model,"LINE_STEER_WRITE",start,end,origin,written,"feedback_does_not_restart_motor_timer");
                            }
                        }
                    } else if(lineSession.lost(lineNow,lineTuning)) {
                        motor->stop("MOTOR_STOP_LINE_LOST");
                        std::cout<<"【丢线停车】黑线持续不可靠，本次结束，须手动重新启动。"<<std::endl;
                    }
                    collectMotorEvents();
                }
                if(following) recording.line(model,lineObservation,lineSession.controller.error(),monotonicNs(),origin,written);
                if(stopWait) {
                    if(timing.stationaryAfter(stopWait->endNs)) {
                        stopConfirmed=true;model.stop(stopWait->event=="MOTOR_STOP_LINE_LOST" ? "LINE_LOST" : "MOTOR_TRIAL_COMPLETED");
                    }
                }
                if(model.state()==ParkingRehearsal::State::Running) {
                    const auto window=speed.add(model.stage(),model.segment(),model.trial(),model.revision(),
                        model.tuning().speedLeftCmPerCount,model.tuning().speedRightCmPerCount,wheels[0],wheels[1]);
                    if(window) recording.speed(model,*window,origin,written);
                } else speed.reset();
                hasSamples=true;
            } else {speed.reset();timing.gap();hasSamples=false;}
            if(stopWait && !model.finished() && monotonicNs()-stopWait->endNs>=int64_t(2e9)) {
                failure="Encoder zero tail not confirmed within 2s";model.stop("STOP_UNCONFIRMED");
            }
            if(elapsed-lastFlush>=1) {
                recording.flush();lastFlush=elapsed;
                if(fs::space(directory).available<64u*1024u*1024u) throw std::runtime_error("Output storage below 64 MiB");
            }
            if(lastState!=model.stateName() || elapsed-lastStatus>=1) {
                if(servo) written=servo->written();
                showStatus(model,written);
                if(following) std::cout<<"巡线="<<(lineSession.waiting() ? "WAIT_LINE" : motor && motor->active() ? "ACTIVE" : "OBSERVE/STOPPED")
                    <<" | 黑线有效="<<lineObservation.reliable<<" | 真实点行="<<lineObservation.rows<<" | 目标x="<<lineObservation.target
                    <<"/160 | 滤波误差="<<lineSession.controller.error()<<std::endl;
                if(servo) std::cout<<"当前步骤保持时间="<<model.holdSeconds()
                    <<"s | 定时回正剩余="<<servo->remaining()<<"s"<<std::endl;
                if(speed.latest()) {
                    const auto& window=*speed.latest();
                    std::cout<<"编码器窗口速度：左="<<window.rate(0)<<" 右="<<window.rate(1)<<" counts/s";
                    if(window.calibrated()) std::cout<<" | 车速估计="<<window.centerCmPerSecond()<<" cm/s";
                    else std::cout<<" | 厘米速度未标定";
                    std::cout<<std::endl;
                }
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
    if(motor) {
        try {motor->close();motorExitZero=true;}catch(const std::exception& error) {failure+="; MOTOR_ZERO_FAILED: "+std::string(error.what());}
        try {collectMotorEvents(true);}catch(const std::exception& error) {failure+="; "+std::string(error.what());}
    }
    timing.end(monotonicNs(),model.outcome());
    if(servo) {
        exitZeroAttempted=servo->attempted();
        ServoHoldEvent exit{model.stage(),model.revision(),model.trial(),monotonicNs(),0,"","EXIT_ZERO"};
        exit.segment=model.segment();
        try {
            servo->close();exitZeroSucceeded=exitZeroAttempted;
            std::cout<<(exitZeroAttempted ? "SERVO_ZERO software write completed\n" : "SERVO_ZERO not attempted; no authorized write occurred\n");
        } catch(const std::exception& error) {
            exit.error=error.what();failure+="; ZERO_FAILED: "+exit.error;
            std::cerr<<"【退出回正失败，请停止推/拉】ZERO_FAILED "<<exit.error<<std::endl;
        }
        exit.endNs=monotonicNs();
        if(exitZeroAttempted) recording.holdEvent(exit,origin);
    }
    try {collectHoldEvents(true);} catch(const std::exception& error) {failure+="; "+std::string(error.what());}
    recording.close();
    auto speedSummary=outputFile(directory/"stage_speed_summary.csv");
    speedSummary<<std::setprecision(17);speed.writeSummary(speedSummary);
    closeText(speedSummary,directory/"stage_speed_summary.csv");
    auto timingSummary=outputFile(directory/"stage_timing_summary.csv");timing.writeSummary(timingSummary,origin);
    closeText(timingSummary,directory/"stage_timing_summary.csv");
    for(const auto& row:timing.rows()) if(row.stage==0 && row.end)
        std::cout<<"阶段1执行持续时间="<<double(row.end-row.ready)/1e9<<"s；不包含启动等待和Y/N确认。"<<std::endl;
    if(retention.decide()==SaveChoice::No) return failure.empty() ? (partial?2:0) : 1;
    auto metadata=outputFile(directory/"session.txt");
    metadata<<"version="<<rehearsalVersion<<"\nresult="<<(failure.empty()?model.outcome():"ERROR")
        <<"\nerror="<<failure<<"\nmotor_initialized="<<bool(motor)<<"\nmotor_zero_on_exit="<<(motor ? (motorExitZero ? "software_write_completed" : "failed") : "not_attempted")
        <<"\nencoder_stop_confirmed="<<stopConfirmed<<"\nimu_initialized=0\npartial="<<partial
        <<"\nmotor_units=raw_pwm\nmotor_limit=per_device_duty_max\nnormalized_motor_scale_used=0"
        <<"\nservo_zero_on_exit="<<(exitZeroAttempted ? (exitZeroSucceeded ? "software_write_completed":"failed") : "not_attempted")
        <<"\nmanual_parking_success=not_inferred\nvideo_timing=use_csv_monotonic_timestamps_not_nominal_fps\n"
        <<"recording_start=one_enter_or_valid_file_save\nspeed_summary=stage_speed_summary.csv\nspeed_default_unit=counts/s\ntiming_summary=stage_timing_summary.csv\n";
    closeText(metadata,directory/"session.txt");retention.saved();
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
                <<"Enter: record/set servo, settle, then start configured motor/timers. P: pause recording and stop motors; C: resume recording only; Enter required to move again. Q: finish.\n"
                <<"auto_reload=save; powered settings settle before motor start; paused motors require fresh Enter. motor_run_time_s=0 disables motors.\n"
                <<"motor_units=raw_pwm; integer amplitudes, 2000 writes 2000. No tuning cap; per-device duty_max and uint16 wire bound enforced.\n"
                <<"Saved timed trial ends after center; save_confirmation=single-key Y/N (no Enter, one prompt); restart=manual.\n"
                <<"Stage 2: left bend then right correction; stage_2_step=right_correction selects correction-only tuning.\n"
                <<"Stage 4: right bend then left correction; stage_4_step=left_correction selects correction-only tuning.\n"
                <<"Stage 1: optional line_follow_enable=1; acquire reliable black line before powered motion, dt-filtered PD steering, timed stop/center or sustained lost-line stop. No branch selection.\n"
                <<"Single-stage motor tuning. Six stage CSV files plus stage_speed_summary.csv and stage_timing_summary.csv. No IMU. Speed: counts/s; calibrated cm/s only with explicit speed_*_cm_per_count.\n";return 0;
        }
        auto config=loadConfig(options.capture);const auto hardware=car2026::HardwareConfig::load(options.hardware);
        const auto params=car2026::Params::load(options.vehicle);const auto tuning=ParkingTuning::load(options.tuning);
        tuning.validate(params.max_steer_deg);fillEncoderPaths(config,hardware);
        writeTuningSummary(std::cout,tuning,fs::absolute(options.tuning).lexically_normal().string());
        const auto missing=validateSensors(config,options.partial);
        for(const auto& name:missing) std::cout<<"UNCONFIGURED "<<name<<'\n';
        if(options.check) {std::cout<<"Config checked without hardware access\n";return missing.empty()?0:2;}
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        return run(options,config,hardware,params,tuning,!missing.empty());
    } catch(const std::exception& error) {std::cerr<<"parking_rehearsal: "<<error.what()<<'\n';return 1;}
}
