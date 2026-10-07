#include "capture_data.hpp"
#include "hardware_linux.hpp"
#include "device_read_buffer.hpp"
#include "servo_output.hpp"
#include "manual_capture_options.hpp"
#include "manual_steering_probe.hpp"
#include "opencv_image.hpp"
#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>
#include <atomic>
#include <chrono>
#include <cerrno>
#include <csignal>
#include <cstring>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <mutex>
#include <poll.h>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

namespace fs=std::filesystem;
using car2026::capture::Config;
using car2026::capture::SensorSpec;
namespace {
constexpr const char* recorderVersion="2026-10-07.6";
constexpr const char* sensorHeader="cycle,channel,read_start_ns,read_end_ns,elapsed_s,value,unit,valid,status,read_return,raw_hex";
constexpr const char* frameHeader="frame_index,read_start_ns,read_end_ns,elapsed_s";
constexpr const char* markerHeader="monotonic_ns,elapsed_s,marker";
constexpr const char* observerHeader="frame_index,frame_read_start_s,processed_time_s,frame_age_s,motion,reference_id,has_reference,state,line_confidence,line_ambiguous,line_discontinuous,lateral_error_image,heading_error_image,suggestion_valid,suggested_command,motor_writes,servo_writes";
constexpr const char* probeHeader="frame_index,processed_time_s,encoder_fresh,stationary,encoder_age_s,key,probe_state,decision,requested_command,last_successful_command,servo_api_attempts,servo_api_successes,motor_writes,reason,encoder_reason,zero_duration_s,last_encoder_left_delta,last_encoder_right_delta,movement_epoch,auto_phase";
constexpr const char* servoEventHeader="request,monotonic_ns,elapsed_s,command,duty,status,reason,encoder_reason,encoder_age_s,zero_duration_s,frame_age_s";
volatile std::sig_atomic_t interrupted=0;
void signalHandler(int) {interrupted=1;}
int64_t monotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::steady_clock::now().time_since_epoch()).count();
}
std::string utcStamp() {
    const auto current=std::time(nullptr);std::tm calendar{};gmtime_r(&current,&calendar);
    char buffer[32];std::strftime(buffer,sizeof(buffer),"%Y%m%dT%H%M%SZ",&calendar);return buffer;
}
std::ofstream outputFile(const fs::path& path) {
    std::ofstream file(path.string());if(!file) throw std::runtime_error("Cannot create "+path.string());
    file.exceptions(std::ios::failbit|std::ios::badbit);file<<std::setprecision(17);return file;
}
uint64_t nonemptyFileSize(const fs::path& path) {
    struct stat info{};
    if(stat(path.string().c_str(),&info)!=0)
        throw std::runtime_error("Cannot stat "+path.string()+": "+std::strerror(errno));
    if(info.st_size<=0) throw std::runtime_error("Output file is empty: "+path.string());
    return uint64_t(info.st_size);
}
void syncFile(const fs::path& path) {
    const int fd=open(path.string().c_str(),O_RDWR|O_CLOEXEC);
    if(fd<0) throw std::runtime_error("Cannot sync-open "+path.string()+": "+std::strerror(errno));
    int result;do {result=fsync(fd);} while(result<0 && errno==EINTR);
    const int error=errno;const int closed=close(fd);
    if(result<0) throw std::runtime_error("fsync "+path.string()+": "+std::strerror(error));
    if(closed<0) throw std::runtime_error("Close after fsync failed: "+path.string());
}
void closeText(std::ofstream& file,const fs::path& path) {
    file.flush();file.close(); // Exceptions are enabled, including delayed flush/close failures.
    nonemptyFileSize(path);syncFile(path);
}
uint64_t verifyTextFile(const fs::path& path,const std::string& firstLine) {
    nonemptyFileSize(path);
    std::ifstream input(path.string(),std::ios::binary);
    if(!input) throw std::runtime_error("Cannot reopen output: "+path.string());
    try {return car2026::capture::verifyTextOutput(input,firstLine);}
    catch(const std::exception& error) {throw std::runtime_error(path.string()+": "+error.what());}
}
void verifyProbe(const fs::path& path,const std::string& expected) {
    nonemptyFileSize(path);
    const int fd=open(path.string().c_str(),O_RDONLY|O_CLOEXEC);
    if(fd<0) throw std::runtime_error("Probe reopen failed: "+path.string());
    std::string actual;char buffer[256];ssize_t result;int error=0;
    for(;;) {
        do {result=read(fd,buffer,sizeof(buffer));} while(result<0 && errno==EINTR);
        if(result<0) {error=errno;break;}
        if(result==0) break;
        actual.append(buffer,size_t(result));
        if(actual.size()>expected.size()) break;
    }
    close(fd);
    if(error) throw std::runtime_error("Probe readback failed: "+std::string(std::strerror(error)));
    if(actual!=expected) throw std::runtime_error("Probe bytes do not match: "+path.string());
}
int checkOutput(const fs::path& base) {
    fs::create_directories(base);
    const auto directory=base/("output_check_"+utcStamp()+"_"+std::to_string(getpid())+"_"+std::to_string(monotonicNs()));
    if(!fs::create_directory(directory)) throw std::runtime_error("Output-check directory already exists");
    std::cout<<"OUTPUT_CHECK "<<directory.string()<<"\nNo camera or sensor access.\n";
    const std::string payload="manual_capture_output_check\n0123456789\n";
    int failures=0;
    // Compare native Linux writes with C++ streams to isolate shared output failures.
    try {
        const auto path=directory/"posix.txt";
        const int fd=open(path.string().c_str(),O_WRONLY|O_CREAT|O_EXCL|O_CLOEXEC,0600);
        if(fd<0) throw std::runtime_error("POSIX probe open failed: "+std::string(std::strerror(errno)));
        size_t offset=0;int error=0;
        while(offset<payload.size()) {
            ssize_t written;
            do {written=write(fd,payload.data()+offset,payload.size()-offset);} while(written<0 && errno==EINTR);
            if(written<=0) {error=written<0 ? errno : EIO;break;}
            offset+=size_t(written);
        }
        const int closed=close(fd);
        if(error || closed<0) throw std::runtime_error("POSIX probe write/close failed: "+std::string(std::strerror(error ? error : errno)));
        syncFile(path);verifyProbe(path,payload);std::cout<<"PASS POSIX write/fsync/readback bytes="<<payload.size()<<'\n';
    } catch(const std::exception& error) {++failures;std::cout<<"FAIL POSIX: "<<error.what()<<'\n';}
    try {
        const auto path=directory/"cpp_stream.txt";auto file=outputFile(path);
        file<<payload;closeText(file,path);verifyProbe(path,payload);
        std::cout<<"PASS C++ stream/close/fsync/readback bytes="<<payload.size()<<'\n';
    } catch(const std::exception& error) {++failures;std::cout<<"FAIL C++ stream: "<<error.what()<<'\n';}
    return failures ? 1 : 0;
}
car2026::capture::Sample readSensor(const SensorSpec& sensor,bool factoryEncoderStatusZero) {
    using car2026::capture::Sample;
    if(sensor.path.empty()) {Sample result;result.status="unconfigured";return result;}
    // Only sensor inputs are opened, read-only. No GPIO export/direction writes.
    const int fd=open(sensor.path.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if(fd<0) {Sample result;result.status="open_failed:"+std::string(std::strerror(errno));return result;}
    const size_t size=car2026::capture::readBufferSize(sensor);
    std::vector<uint8_t> bytes(size,0xa5);ssize_t count;
    const bool factoryEncoder=factoryEncoderStatusZero &&
        (sensor.name=="encoder_left" || sensor.name=="encoder_right");
    car2026::GuardedDeviceRead<car2026::EncoderCount> guardedEncoder(
        std::numeric_limits<car2026::EncoderCount>::min());
    void* destination=factoryEncoder ? guardedEncoder.data() : bytes.data();
    // Never finish a short transfer with a second read: delta counters may clear.
    do {count=read(fd,destination,bytes.size());} while(count<0 && errno==EINTR && !interrupted);
    const int error=errno;close(fd);
    if(factoryEncoder) {
        std::memcpy(bytes.data(),guardedEncoder.data(),bytes.size());
        if(guardedEncoder.guardDamaged()) throw std::runtime_error("Encoder payload guard damaged: "+sensor.path);
    }
    auto result=car2026::capture::decode(sensor,long(factoryEncoder && count==0 ? size : count),bytes);
    result.returned=long(count); // Always preserve the actual syscall result.
    if(factoryEncoder && (count==0 || count==ssize_t(sizeof(car2026::EncoderCount)))) {
        result.value=car2026::decodeEncoderRead(guardedEncoder.value(sensor.path),count,sensor.path);
        if(count==0) result.status="ok_factory_status_zero";
    }
    if(count<0) result.status="read_failed:"+std::string(std::strerror(error));
    return result;
}
struct Stats {std::atomic<uint64_t> valid{0},invalid{0};};
struct SharedRestGate {
    std::mutex mutex;car2026::capture::EncoderRestGate gate;
    struct Snapshot {int64_t timeNs;double elapsed;car2026::capture::EncoderRestStatus rest;};
    Snapshot snapshot(int64_t origin) {
        // Capture time under the same lock as status: a sampler update cannot
        // appear to come from the future while a caller waits for this lock.
        std::lock_guard<std::mutex> held(mutex);
        const auto time=monotonicNs();const double elapsed=double(time-origin)/1e9;
        return {time,elapsed,gate.status(elapsed)};
    }
};
void fillEncoderPaths(Config& config,const car2026::HardwareConfig& hardware) {
    // Only these verified interfaces have defaults. No guessed GPIO pin numbers.
    if(config.sensors[0].path.empty()) config.sensors[0].path=hardware.encoder_left;
    if(config.sensors[1].path.empty()) config.sensors[1].path=hardware.encoder_right;
    if(config.factoryEncoderStatusZero &&
       (config.sensors[0].path!=hardware.encoder_left || config.sensors[1].path!=hardware.encoder_right))
        throw std::runtime_error("Factory status-zero reads are restricted to configured board encoder paths");
}
void sensorLoop(const Config& config,const fs::path& directory,int64_t origin,
                std::atomic<bool>& stop,std::array<Stats,8>& stats,std::exception_ptr& failure,SharedRestGate& rest) {
    try {
        auto file=outputFile(directory/"sensors.csv");
        file<<sensorHeader<<'\n';file.flush();nonemptyFileSize(directory/"sensors.csv");
        uint64_t cycle=0;auto next=std::chrono::steady_clock::now(),lastFlush=next;
        const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double,std::milli>(config.sampleMs));
        while(!stop.load()) {
            std::array<car2026::capture::Sample,2> encoders;
            int64_t encoderPairStart=0;
            for(size_t index=0;index<config.sensors.size();++index) {
                const auto& sensor=config.sensors[index];const auto before=monotonicNs();
                if(index==0) encoderPairStart=before;
                const auto sample=readSensor(sensor,config.factoryEncoderStatusZero);const auto after=monotonicNs();
                if(index<encoders.size()) encoders[index]=sample;
                if(index==1) {
                    std::lock_guard<std::mutex> held(rest.mutex);
                    rest.gate.sample(encoders[0].valid,encoders[0].value,encoders[1].valid,encoders[1].value,
                                     double(after-origin)/1e9,double(after-encoderPairStart)/1e9);
                }
                file<<cycle<<','<<sensor.name<<','<<before<<','<<after<<','<<double(after-origin)/1e9<<',';
                if(sample.valid) file<<sample.value;
                file<<','<<car2026::capture::csvString(sensor.unit)<<','<<sample.valid<<','
                    <<car2026::capture::csvString(sample.status)<<','<<sample.returned<<','<<sample.raw<<'\n';
                if(sample.valid) ++stats[index].valid;else ++stats[index].invalid;
            }
            ++cycle;const auto now=std::chrono::steady_clock::now();
            if(now-lastFlush>=std::chrono::seconds(1)) {file.flush();lastFlush=now;}
            next+=period;if(next<now) next=now+period;
            std::this_thread::sleep_until(next);
        }
        closeText(file,directory/"sensors.csv");
    } catch(...) {failure=std::current_exception();stop.store(true);}
}
bool recordMarkers(std::ofstream& markers,int64_t origin,std::string& pending,bool& inputEnded,
                   car2026::capture::ProbeInput* probeInput=nullptr,bool allowEof=false) {
    if(inputEnded) return !probeInput || allowEof;
    // Drain already queued canonical terminal lines in probe mode. Q anywhere
    // in this bounded batch cancels all pending commands before any PWM write.
    size_t drained=0;
    do {
        struct pollfd input{STDIN_FILENO,POLLIN,0};
        const int ready=poll(&input,1,0);
        if(ready<0) {if(errno==EINTR) return true;throw std::runtime_error("Terminal poll failed");}
        if(ready<=0) break;
        if(input.revents&(POLLERR|POLLNVAL)) throw std::runtime_error("Terminal input unavailable");
        if(!(input.revents&(POLLIN|POLLHUP))) break;
        // Avoid iostream prefetch: queued lines must not wait for another keypress.
        char buffer[128];const ssize_t count=read(STDIN_FILENO,buffer,sizeof(buffer));
        if(count<0) {if(errno==EINTR) return true;throw std::runtime_error("Terminal read failed");}
        if(count==0) {inputEnded=true;return !probeInput || allowEof;}
        drained+=size_t(count);
        pending.append(buffer,size_t(count));
        size_t end;
        while((end=pending.find('\n'))!=std::string::npos) {
            const auto line=car2026::capture::trimmed(pending.substr(0,end));pending.erase(0,end+1);
            if(probeInput) probeInput->add(line);
            else if(line=="q" || line=="Q") return false;
            if(line.size()==1) {
                char marker=line[0];if(marker>='a' && marker<='e') marker=char(marker-'a'+'A');
                if(marker>='A' && marker<='E') {
                    const auto now=monotonicNs();markers<<now<<','<<double(now-origin)/1e9<<','<<marker<<'\n';
                    markers.flush();std::cout<<"MARK "<<marker<<'\n';
                }
            }
        }
        if(pending.size()>128) {
            if(probeInput) throw std::runtime_error("Probe input line too long");
            pending.clear();
        }
        if(probeInput && probeInput->quit) return false;
        if(probeInput && drained>=4096) throw std::runtime_error("Probe terminal batch too long");
    } while(probeInput);
    return !probeInput || !probeInput->quit;
}
int capture(Config config,const car2026::HardwareConfig& hardware,const std::string& captureConfig,
            const std::string& hardwareConfig,const fs::path& base,double duration,bool allowPartial,
            const car2026::capture::Options& options,const car2026::Params* vehicleParams) {
    const auto missing=car2026::capture::validateSensors(config,allowPartial);
    car2026::HardwareLock lock; // Uses the same exclusion lock as the vehicle.
    cv::VideoCapture camera(hardware.camera_device,cv::CAP_V4L2);
    if(!camera.isOpened()) throw std::runtime_error("Cannot open camera "+hardware.camera_device);
    camera.set(cv::CAP_PROP_FRAME_WIDTH,config.width);camera.set(cv::CAP_PROP_FRAME_HEIGHT,config.height);
    camera.set(cv::CAP_PROP_FPS,config.fps);camera.set(cv::CAP_PROP_BUFFERSIZE,1);
    fs::create_directories(base);
    const auto directory=base/("manual_"+utcStamp()+"_"+std::to_string(getpid())+"_"+std::to_string(monotonicNs()));
    if(!fs::create_directory(directory)) throw std::runtime_error("Session directory already exists");
    fs::copy_file(captureConfig,directory/"capture_config.ini");
    fs::copy_file(hardwareConfig,directory/"hardware_config.ini");
    if(vehicleParams) fs::copy_file(options.vehicleFile,directory/"vehicle_config.ini");
    nonemptyFileSize(directory/"capture_config.ini");nonemptyFileSize(directory/"hardware_config.ini");
    const auto origin=monotonicNs();
    auto metadata=outputFile(directory/"session.txt");
    const auto observerMotion=options.observeSteering.value_or(car2026::ManualMotion::Reverse);
    const bool hasObserver=options.observeSteering.has_value() || options.isProbe();
    const std::string mode=options.autoProbe ? "AUTO_REVERSE_STEERING_PROBE_SERVO_ONLY" :
        (options.isProbe() ? "MANUAL_REVERSE_STEERING_PROBE_SERVO_ONLY" :
        (options.observeSteering ? "VISUAL_STEERING_OBSERVER_NO_ACTUATOR_WRITES" :
        (options.steerCommand ? "MANUAL_PUSH_SERVO_ONLY" : "MANUAL_CAPTURE_NO_ACTUATOR_WRITES")));
    metadata<<"mode="<<mode<<"\nrecorder_version="<<recorderVersion<<"\nimu=not_installed\norigin_ns="<<origin
            <<"\nstart_utc="<<utcStamp()<<"\nencoder_mode="<<hardware.encoder_mode
            <<"\nencoder_values=raw signed counts; first delta read includes the untimed startup interval\n"
              "sensor_values=raw declared units; no guessed sign, scale, distance or steering angle\n"
              "timestamps=host steady clock; video read brackets are NOT hardware exposure timestamps\n"
              "alignment=use frames.csv and sensors.csv; AVI nominal FPS alone is not elapsed time\n"
            <<"nominal_video_fps="<<config.fps<<"\nsample_period_ms="<<config.sampleMs
            <<"\nfactory_encoder_status_zero="<<config.factoryEncoderStatusZero<<'\n';
    if(config.factoryEncoderStatusZero)
        metadata<<"encoder_contract=observed factory status-zero ABI; raw buffer copy cannot be proven for every int32 value\n";
    if(options.steerCommand)
        metadata<<"steer_command_deg="<<*options.steerCommand<<"\nsteering_angle_measured=0\n"
                  "motor_writes=0\nservo_exit=configured_zero_command; NOT guaranteed physically straight\n";
    if(options.observeSteering)
        metadata<<"observer_motion="<<car2026::manualMotionName(*options.observeSteering)
                <<"\nobserver_target=first stable locally straight image reference; NOT a measured bay axis or heading\n"
                  "suggestions_are_applied=0\nmotor_writes=0\nservo_writes=0\n"
                  "observer_age=processing completion minus camera read start; exposure age is unknown\n";
    if(options.isProbe())
        metadata<<"observer_motion=reverse\nobserver_target="<<(options.autoProbe ? "automatic stationary local image reference" : "manual R stationary local image reference")<<'\n'<<
                  "suggestions_are_applied=0\nsteering_angle_measured=0\nmotor_writes=0\n"
                  "probe_rest=both delta counts exactly zero for >=0.8s, valid samples <=0.25s old; not physical rest proof\n"
                  "servo_exit=configured_zero_command; stop pulling before Q/Ctrl+C/duration expiry\n";
    if(options.autoProbe)
        metadata<<"probe_commands=automatic 0 then fixed +"<<car2026::capture::AutomaticSteeringProbe::trialCommand<<" then 0; never uses image suggestions\n"
                  "manual_pull_target_cm=10; human distance marker, NOT encoder-calibrated travel\n"
                  "probe_loss=stop pulling, stationary zero and finish; no rearm or repeated nonzero trial\n"
                  "auto_time_limits_s=initial rest 15, reference 15, start pulling 12, active pull 10\n";
    else if(options.isProbe())
        metadata<<"probe_commands=keyboard 0,+2,-2; only while encoder-rest gate passes\n"
                  "probe_loss=latched hold; stop pulling; zero only after encoder-rest gate; R required to rearm\n";
    for(const auto& sensor:config.sensors)
        metadata<<sensor.name<<"="<<sensor.path<<" format="<<sensor.format<<" unit="<<sensor.unit<<'\n';
    metadata.flush();nonemptyFileSize(directory/"session.txt");
    auto frames=outputFile(directory/"frames.csv"),markers=outputFile(directory/"markers.csv");
    frames<<frameHeader<<'\n';markers<<markerHeader<<'\n';
    frames.flush();markers.flush();
    nonemptyFileSize(directory/"frames.csv");nonemptyFileSize(directory/"markers.csv");
    std::cout<<"SESSION "<<directory.string()<<'\n';
    if(options.autoProbe)
        std::cout<<"SERVO ONLY; no motor/GPIO/IMU initialization or writes. NOT READY: keep vehicle stationary.\n";
    else if(options.steerCommand || options.isProbe())
        std::cout<<"SERVO ONLY; no motor/GPIO/IMU initialization or writes. Wait for STEER_READY before manual pushing.\n"
                   "Stop pushing before Q/Ctrl+C or duration expiry; exit attempts configured zero steering.\n";
    else std::cout<<"No motor/servo/IMU initialization.\n";
    if(options.observeSteering)
        std::cout<<"OBSERVE ONLY motion="<<car2026::manualMotionName(*options.observeSteering)
                 <<"; suggestions are NOT applied. Keep the initial image steady until REFERENCE_READY.\n";
    if(options.autoProbe)
        std::cout<<"【等待准备，请勿推拉】当前尚未允许移动；下面的状态日志不是开始信号。\n"
                   "请预先标出后退10厘米的位置；稍后程序会单独发出允许后拉的提示。\n"
                   "无需输入按键；收到停止或结束提示就保持静止，等保存完成。\n"<<std::flush;
    else if(options.isProbe())
        std::cout<<"PROBE ONLY: no automatic image-error steering. Keep stopped; R+Enter centers and builds a reference.\n"
                   "Wait for PROBE_REFERENCE_READY. While stopped: + or - or 0 +Enter; wait for PROBE_STEER_READY.\n"
                   "Then gently pull backward. S+Enter latches hold; HOLD means STOP PULLING. Do not switch commands while moving.\n";
    if(!options.autoProbe) std::cout<<"Enter A..E to mark, Q or Ctrl+C to finish.\n";
    if(!missing.empty()) {
        std::cout<<"PARTIAL: unconfigured channels:";
        for(const auto& name:missing) std::cout<<' '<<name;
        std::cout<<'\n';
    }
    std::cout<<"factory_encoder_status_zero="<<config.factoryEncoderStatusZero<<'\n';
    std::atomic<bool> stop{false};std::array<Stats,8> stats;SharedRestGate restGate;
    std::exception_ptr sensorFailure,videoFailure;
    cv::VideoWriter video;uint64_t count=0;int width=0,height=0;
    std::unique_ptr<car2026::LinuxDeviceIo> servoIo;
    std::unique_ptr<car2026::ServoOutput> servo;
    std::optional<car2026::Vision> observerVision;
    std::optional<car2026::VisualSteeringObserver> observer;
    std::ofstream observerFile;car2026::SteeringObservation latestSuggestion;
    std::ofstream probeFile,servoEvents;car2026::capture::ManualSteeringProbe probe;
    car2026::capture::AutomaticSteeringProbe autoProbe(options.autoProbe ? duration : 45);
    car2026::capture::ProbeAction latestProbe;
    car2026::capture::EncoderRestStatus latestRest;
    std::string lastProbeReport;
    bool probeReferenceActive=false;
    uint64_t servoRequests=0,servoAttempts=0,servoSuccesses=0;std::optional<double> lastServoCommand;
    bool exitZeroWriteOk=false;
    std::string lastServoCancellation;
    if(hasObserver) {
        observerVision.emplace(*vehicleParams);observer.emplace(*vehicleParams,observerMotion);
        observerFile=outputFile(directory/"visual_observer.csv");observerFile<<observerHeader<<'\n';observerFile.flush();
    }
    if(options.isProbe()) {
        probeFile=outputFile(directory/"steering_probe.csv");probeFile<<probeHeader<<'\n';probeFile.flush();
        servoEvents=outputFile(directory/"servo_events.csv");servoEvents<<servoEventHeader<<'\n';servoEvents.flush();
    }
    auto recordServoEvent=[&](uint64_t request,const SharedRestGate::Snapshot& snapshot,double command,
                             std::optional<uint16_t> duty,const char* status,const std::string& reason,double frameTime) {
        const double elapsed=snapshot.elapsed;const auto& rest=snapshot.rest;
        servoEvents<<request<<','<<snapshot.timeNs<<','<<elapsed<<','<<command<<',';
        if(duty) servoEvents<<*duty;
        servoEvents<<','<<status<<','<<car2026::capture::csvString(reason)<<','<<rest.reason<<','
                   <<rest.age<<','<<rest.zeroDuration<<',';
        if(std::isfinite(frameTime) && frameTime>=0 && elapsed>=frameTime) servoEvents<<elapsed-frameTime;
        servoEvents<<'\n';servoEvents.flush();
    };
    auto probeServoWrite=[&](double command,double frameTime,bool closing=false) {
        lastServoCancellation.clear();
        if(!servo) {
            servoIo=std::make_unique<car2026::LinuxDeviceIo>(hardware);
            servo=std::make_unique<car2026::ServoOutput>(*servoIo,hardware,*vehicleParams);
        }
        const auto before=restGate.snapshot(origin);const auto request=++servoRequests;
        // Normal requests are logged before writing. Shutdown must attempt zero
        // even when the log disk is full; record that attempt afterwards.
        if(!closing) {
            recordServoEvent(request,before,command,{},"PREPARE",{},frameTime);
            // Metadata reads and disk flushes may take time. Recheck immediately
            // before writing, so a delayed request cannot move a pulling car.
            const auto checked=restGate.snapshot(origin);const double now=checked.elapsed;
            const auto& rest=checked.rest;
            car2026::capture::ServoRequestGate gate;
            gate.exitRequested=interrupted;gate.samplerStopped=stop.load();gate.now=now;gate.deadline=duration;
            gate.command=command;gate.frameTime=frameTime;gate.frameTimeout=vehicleParams->frame_timeout_s;gate.rest=rest;
            const auto issue=gate.issue();
            if(!issue.empty()) {
                lastServoCancellation="SERVO_CANCELLED_"+issue;
                recordServoEvent(request,checked,command,{},"CANCELLED_GATE_CHANGED",lastServoCancellation,frameTime);
                std::cout<<"SERVO_REQUEST_CANCELLED reason="<<lastServoCancellation<<" command="<<command
                         <<" encoder="<<rest.reason<<" encoder_age_s="<<rest.age
                         <<" zero_s="<<rest.zeroDuration<<" frame_age_s="<<now-frameTime<<'\n'<<std::flush;
                return false;
            }
        }
        ++servoAttempts;
        uint16_t duty=0;
        try {
            if(closing) {servo->close();exitZeroWriteOk=true;}else duty=servo->set(command);
        } catch(...) {
            const auto after=restGate.snapshot(origin);
            recordServoEvent(request,after,command,{},"FAILED",{},frameTime);throw;
        }
        ++servoSuccesses;lastServoCommand=command;
        const auto after=restGate.snapshot(origin);
        if(closing) recordServoEvent(request,before,command,{},"EXIT_ATTEMPT",{},frameTime);
        recordServoEvent(request,after,command,closing ? std::optional<uint16_t>{} : duty,"SOFTWARE_OK",{},frameTime);
        return true;
    };
    std::thread sampler(sensorLoop,std::cref(config),directory,origin,std::ref(stop),
                        std::ref(stats),std::ref(sensorFailure),std::ref(restGate));
    try {
        auto lastReport=std::chrono::steady_clock::now();
        std::string pendingInput;bool inputEnded=false;
        while(!interrupted && !stop.load() && double(monotonicNs()-origin)/1e9<duration) {
            const auto before=monotonicNs();cv::Mat image;
            const bool ok=camera.read(image);const auto after=monotonicNs();
            if(!ok || image.empty()) throw std::runtime_error("Camera frame read failed");
            if(!video.isOpened()) {
                width=image.cols;height=image.rows;
                video.open((directory/"camera.avi").string(),cv::CAP_OPENCV_MJPEG,cv::VideoWriter::fourcc('M','J','P','G'),
                           config.fps,cv::Size(width,height));
                if(!video.isOpened()) throw std::runtime_error("Cannot initialize MJPG video writer");
                metadata<<"video_writer_backend="<<video.getBackendName()<<'\n';metadata.flush();
                std::cout<<"VIDEO backend="<<video.getBackendName()<<" size="<<width<<'x'<<height<<'\n';
            }
            if(image.cols!=width || image.rows!=height) throw std::runtime_error("Camera dimensions changed");
            video.write(image);frames<<count++<<','<<before<<','<<after<<','<<double(after-origin)/1e9<<'\n';
            car2026::Observation observation;double frameTime=double(before-origin)/1e9,processedTime=0;
            if(observer) {
                observation=observerVision->analyze(car2026::processCameraFrame(image,*vehicleParams),car2026::Stage::GarageReverse);
                processedTime=double(monotonicNs()-origin)/1e9;
                const bool previouslyReferenced=latestSuggestion.hasReference;
                if(!options.isProbe() || probeReferenceActive)
                    latestSuggestion=observer->observe(observation,frameTime,processedTime);
                else {latestSuggestion={};latestSuggestion.state="WAIT_MANUAL_REFERENCE";}
                if(!options.isProbe() && !previouslyReferenced && latestSuggestion.hasReference)
                    std::cout<<"REFERENCE_READY id="<<latestSuggestion.referenceId<<"; image target only, no actuator output\n"<<std::flush;
            }
            car2026::capture::ProbeInput probeInput;
            const bool keepRunning=recordMarkers(markers,origin,pendingInput,inputEnded,
                                                  options.isProbe() ? &probeInput : nullptr,options.autoProbe);
            const bool canWrite=keepRunning && !interrupted && !stop.load() && double(monotonicNs()-origin)/1e9<duration;
            if(options.isProbe()) {
                const auto decisionSnapshot=restGate.snapshot(origin);
                const double decisionTime=decisionSnapshot.elapsed;
                const auto& rest=decisionSnapshot.rest;
                latestRest=rest;
                if(probeReferenceActive && decisionTime-frameTime>vehicleParams->frame_timeout_s) {
                    latestSuggestion.hasSuggestion=false;latestSuggestion.state="STALE_FRAME";
                    latestSuggestion.frameAge=decisionTime-frameTime;
                }
                if(options.autoProbe && probeInput.key==car2026::capture::ProbeKey::Stop) autoProbe.stop("USER_STOP");
                auto action=options.autoProbe ? autoProbe.update(rest,latestSuggestion,decisionTime) :
                    probe.update(canWrite ? probeInput.key : car2026::capture::ProbeKey::Stop,rest,latestSuggestion,decisionTime);
                if(!canWrite) {
                    action.command.reset();action.resetReference=false;action.decision="EXIT_NO_NEW_COMMAND";
                }
                if(action.command && canWrite) {
                    const bool written=probeServoWrite(*action.command,frameTime);
                    const auto writtenSnapshot=restGate.snapshot(origin);const double writtenTime=writtenSnapshot.elapsed;
                    if(options.autoProbe) {
                        if(autoProbe.acknowledge(written,writtenSnapshot.rest,writtenTime,
                                                writtenTime-frameTime<=vehicleParams->frame_timeout_s &&
                                                !interrupted && !stop.load() && writtenTime<duration,lastServoCancellation))
                            std::cout<<"\n【开始后拉】AUTO_PULL_READY command=+"<<int(car2026::capture::AutomaticSteeringProbe::trialCommand)
                                     <<" elapsed_s="<<writtenTime<<" session="<<directory.filename().string()
                                     <<"; 现在才允许缓慢后拉10厘米，然后立即停稳；若先提示停止就提前停。\n"<<std::flush;
                        if(!written) {
                            action.command.reset();action.decision="SERVO_REQUEST_CANCELLED";action.reason=lastServoCancellation;
                            if(std::string(autoProbe.phaseName())=="WAIT_STILL") {
                                action.decision="AUTO_WAIT_STILL";action.state="UNARMED";
                            }
                        }
                        if(std::string(autoProbe.phaseName())=="WAIT_STOP") {
                            action.decision="AUTO_STOP_PENDING";action.reason=autoProbe.outcome();
                        }
                        if(autoProbe.finished()) {action.decision="AUTO_ZERO_AND_FINISH";action.reason=autoProbe.outcome();}
                    } else if(!written) {
                        action=probe.update(car2026::capture::ProbeKey::Stop,{},latestSuggestion,writtenTime);
                        action.reason=lastServoCancellation;
                    }
                }
                if(action.resetReference) {
                    observer->reset();observerVision.emplace(*vehicleParams);
                    probeReferenceActive=action.decision=="CENTER_THEN_BUILD_REFERENCE";
                }
                probeFile<<count-1<<','<<decisionTime<<','<<rest.fresh<<','<<rest.stationary<<','<<rest.age<<','
                         <<car2026::capture::probeKeyName(probeInput.key)<<','<<action.state<<','<<action.decision<<',';
                if(action.command && canWrite) probeFile<<*action.command;
                probeFile<<',';if(lastServoCommand) probeFile<<*lastServoCommand;
                probeFile<<','<<servoAttempts<<','<<servoSuccesses<<",0,"<<car2026::capture::csvString(action.reason)<<','
                         <<rest.reason<<','<<rest.zeroDuration<<',';
                if(std::isfinite(rest.leftDelta)) probeFile<<rest.leftDelta;
                probeFile<<',';if(std::isfinite(rest.rightDelta)) probeFile<<rest.rightDelta;
                probeFile<<','<<rest.movementEpoch<<','<<(options.autoProbe ? autoProbe.phaseName() : "MANUAL")<<'\n';
                latestProbe=action;
                const std::string report=action.state+":"+action.decision+":"+action.reason;
                if(report!=lastProbeReport || probeInput.key!=car2026::capture::ProbeKey::None) {
                    lastProbeReport=report;
                    std::cout<<action.decision<<" state="<<action.state;
                    if(!action.reason.empty()) std::cout<<" reason="<<action.reason;
                    if(action.command && canWrite) std::cout<<" command="<<*action.command<<" (software write only)";
                    std::cout<<std::endl;
                    if(options.autoProbe && std::string(autoProbe.phaseName())=="WAIT_STOP")
                        std::cout<<"【停止后拉】AUTO_STOP_PULLING; 停稳后程序自动回零并保存。\n"<<std::flush;
                }
                if(options.autoProbe && probeInput.key!=car2026::capture::ProbeKey::None)
                    std::cout<<"AUTO模式无需输入按键；当前阶段="<<autoProbe.phaseName()<<"，未收到实际允许移动提示时请保持静止。\n";
                else if(probeInput.key==car2026::capture::ProbeKey::Invalid)
                    std::cout<<"INVALID_PROBE_INPUT: use one R,+,-,0,S,Q followed by Enter; repeated +++ is not a command.\n";
            }
            if(observer) {
                observerFile<<count-1<<','<<frameTime<<','<<processedTime<<','<<latestSuggestion.frameAge<<','
                            <<car2026::manualMotionName(observerMotion)<<','<<latestSuggestion.referenceId<<','
                            <<latestSuggestion.hasReference<<','<<latestSuggestion.state<<','<<observation.blackPath.confidence<<','
                            <<observation.blackPath.ambiguous<<','<<observation.blackPath.discontinuous<<',';
                if(latestSuggestion.hasSuggestion) observerFile<<latestSuggestion.lateralError;
                observerFile<<',';
                if(latestSuggestion.hasSuggestion) observerFile<<latestSuggestion.headingFeatureError;
                observerFile<<','<<latestSuggestion.hasSuggestion<<',';
                if(latestSuggestion.hasSuggestion) observerFile<<latestSuggestion.suggestedCommand;
                observerFile<<",0,"<<servoSuccesses<<'\n';
            }
            if(!canWrite) break;
            if(options.autoProbe && autoProbe.finished()) {
                std::cout<<"【本次已结束，请勿继续推拉】AUTO_TRIAL_FINISHED result="<<autoProbe.outcome()<<"; 保持静止，正在保存。\n"<<std::flush;
                break;
            }
            if(options.steerCommand && !servo) {
                if(interrupted || stop.load()) break;
                servoIo=std::make_unique<car2026::LinuxDeviceIo>(hardware);
                servo=std::make_unique<car2026::ServoOutput>(*servoIo,hardware,*vehicleParams);
                if(interrupted || stop.load()) break;
                const auto commandStart=monotonicNs();
                const auto duty=servo->set(*options.steerCommand);const auto commandEnd=monotonicNs();
                metadata<<"servo_command_start_ns="<<commandStart<<"\nservo_command_end_ns="<<commandEnd
                        <<"\nservo_duty="<<duty<<"\nservo_duty_max="<<servo->info().duty_max
                        <<"\nservo_frequency_hz="<<servo->info().freq<<'\n';metadata.flush();
                std::cout<<"STEER_READY command="<<*options.steerCommand<<" duty="<<duty
                         <<" frequency="<<servo->info().freq<<" Hz; command is NOT measured wheel angle\n"<<std::flush;
            }
            const auto now=std::chrono::steady_clock::now();
            if(now-lastReport>=std::chrono::seconds(1)) {
                frames.flush();lastReport=now;
                if(observer) observerFile.flush();
                if(options.isProbe()) probeFile.flush();
                if(fs::space(directory).available<64u*1024u*1024u) throw std::runtime_error("Recording disk space below 64 MiB");
                std::cout<<"frames="<<count<<" elapsed="<<std::fixed<<std::setprecision(1)<<double(after-origin)/1e9;
                for(size_t i=0;i<stats.size();++i)
                    std::cout<<' '<<config.sensors[i].name<<"(ok/bad)="<<stats[i].valid.load()<<'/'<<stats[i].invalid.load();
                if(observer) {
                    std::cout<<" observe="<<latestSuggestion.state<<" suggested=";
                    if(latestSuggestion.hasSuggestion) std::cout<<latestSuggestion.suggestedCommand;else std::cout<<"unavailable";
                    std::cout<<" (NOT applied)";
                }
                if(options.isProbe()) {
                    std::cout<<" probe="<<latestProbe.state<<" reason="<<latestProbe.reason<<" encoder="<<latestRest.reason
                             <<" zero_s="<<latestRest.zeroDuration;
                    if(options.autoProbe) std::cout<<" auto="<<autoProbe.phaseName();
                }
                std::cout<<std::endl;
            }
        }
        frames.flush();markers.flush();
    } catch(...) {videoFailure=std::current_exception();}
    stop.store(true);
    // Attempt zero steering before potentially slow joins, AVI close and output checks.
    std::exception_ptr servoFailure;
    if(servo && servo->commandAttempted()) {
        try {
            if(options.isProbe()) probeServoWrite(0,0,true);
            else {servo->close();exitZeroWriteOk=true;}
            std::cout<<"SERVO_ZERO write completed (software only)\n";
        } catch(...) {servoFailure=std::current_exception();}
    }
    sampler.join();video.release();camera.release();
    if(options.steerCommand || options.isProbe()) metadata<<"servo_command_attempted="<<bool(servo && servo->commandAttempted())<<'\n';
    if(options.isProbe())
        metadata<<"servo_api_attempts="<<servoAttempts<<"\nservo_api_successes="<<servoSuccesses<<'\n';
    if(options.autoProbe) metadata<<"auto_phase="<<autoProbe.phaseName()<<"\nauto_result="<<autoProbe.outcome()<<'\n';
    if(servo && servo->commandAttempted()) metadata<<"servo_return_zero_write_ok="<<exitZeroWriteOk<<'\n';
    closeText(frames,directory/"frames.csv");closeText(markers,directory/"markers.csv");
    if(observer) closeText(observerFile,directory/"visual_observer.csv");
    if(options.isProbe()) {
        closeText(probeFile,directory/"steering_probe.csv");closeText(servoEvents,directory/"servo_events.csv");
    }
    bool sensorsGood=missing.empty();
    metadata<<"end_utc="<<utcStamp()<<"\nframe_rows="<<count<<"\nactual_width="<<width<<"\nactual_height="<<height<<'\n';
    for(size_t i=0;i<stats.size();++i) {
        const auto valid=stats[i].valid.load(),invalid=stats[i].invalid.load();
        metadata<<config.sensors[i].name<<"_valid="<<valid<<" invalid="<<invalid<<'\n';
        if(!valid || invalid) sensorsGood=false;
    }
    metadata<<"all_channels_configured="<<missing.empty()<<"\nall_sensor_samples_valid="<<sensorsGood
            <<"\ncapture_loop_failed="<<bool(videoFailure || sensorFailure || servoFailure)<<'\n';
    for(const auto& failure:{videoFailure,sensorFailure,servoFailure}) if(failure) {
        try {std::rethrow_exception(failure);}
        catch(const std::exception& error) {metadata<<"capture_error="<<error.what()<<'\n';}
    }
    metadata.flush();
    if(videoFailure) std::rethrow_exception(videoFailure);
    if(sensorFailure) std::rethrow_exception(sensorFailure);
    if(servoFailure) std::rethrow_exception(servoFailure);
    if(!count) throw std::runtime_error("No video frames recorded");
    const auto frameRows=verifyTextFile(directory/"frames.csv",frameHeader);
    if(frameRows!=count) throw std::runtime_error("Frame CSV row count does not match captured frames");
    if(observer && verifyTextFile(directory/"visual_observer.csv",observerHeader)!=count)
        throw std::runtime_error("Observer CSV row count does not match captured frames");
    if(options.isProbe() && verifyTextFile(directory/"steering_probe.csv",probeHeader)!=count)
        throw std::runtime_error("Steering probe CSV row count does not match captured frames");
    if(options.isProbe()) verifyTextFile(directory/"servo_events.csv",servoEventHeader);
    uint64_t expectedSensorRows=0;
    for(const auto& stat:stats) expectedSensorRows+=stat.valid.load()+stat.invalid.load();
    const auto sensorRows=verifyTextFile(directory/"sensors.csv",sensorHeader);
    if(!sensorRows || sensorRows!=expectedSensorRows) throw std::runtime_error("Sensor CSV row count does not match sampling statistics");
    verifyTextFile(directory/"markers.csv",markerHeader);
    const auto videoBytes=nonemptyFileSize(directory/"camera.avi");syncFile(directory/"camera.avi");
    cv::VideoCapture recorded((directory/"camera.avi").string(),cv::CAP_OPENCV_MJPEG);cv::Mat firstFrame;
    if(!recorded.isOpened() || !recorded.read(firstFrame) || firstFrame.empty() ||
       firstFrame.cols!=width || firstFrame.rows!=height)
        throw std::runtime_error("Recorded AVI failed first-frame readback");
    recorded.release();
    metadata<<"output_verified=nonempty files, CSV row counts, AVI first frame; not full-video validation\n"
            <<"video_bytes="<<videoBytes<<'\n';closeText(metadata,directory/"session.txt");
    verifyTextFile(directory/"session.txt","mode="+mode);
    std::cout<<"VERIFIED video_bytes="<<videoBytes<<" frame_rows="<<frameRows<<" sensor_rows="<<sensorRows<<'\n';
    std::cout<<"SAVED "<<directory.string()<<"\n";
    return sensorsGood ? 0 : 2;
}
}
int main(int argc,char** argv) {
    try {
        std::cout<<"manual_capture version="<<recorderVersion<<'\n';
        const auto options=car2026::capture::Options::parse(std::vector<std::string>(argv+1,argv+argc));
        if(options.help) {
                std::cout<<"manual_capture [--config file] [--hardware-config file] [--output directory]"
                             " [--duration seconds] [--allow-partial] [--check-config] [--check-output]\n"
                             " [--steer-command -15..15 --vehicle-config file --duration <=60]\n"
                             " [--observe-steering forward|reverse --vehicle-config file --duration <=60]\n"
                             " [--steering-probe reverse --vehicle-config file --duration <=60]\n"
                             " [--auto-probe reverse --vehicle-config file --duration <=60]\n"
                             "Auto probe: fixed +5 command after rest/reference gates; cue then manually pull 10cm and stop; automatic zero and save.\n"
                             "Probe: keyboard R reference, +/0/- commands +2/0/-2 while stopped; S hold; SERVO ONLY.\n"
                             "Probe NEVER applies image suggestions; conflicts with fixed steering and observation.\n"
                             "Observation: no actuator writes; image-reference suggestions only; conflicts with --steer-command.\n"
                             "Default: no actuator writes. Explicit steering: SERVO ONLY, return to configured zero on exit.\n"
                             "No motor/GPIO-output/IMU initialization. Check modes never access hardware.\n";return 0;
        }
        if(options.checkOutput) return checkOutput(options.output);
        auto config=car2026::capture::loadConfig(options.configFile);
        const auto hardware=car2026::HardwareConfig::load(options.hardwareFile);
        if(options.isProbe() && hardware.encoder_mode!="delta")
            throw std::runtime_error("Steering probe requires the configured delta encoder contract");
        std::optional<car2026::Params> vehicleParams;
        if(options.steerCommand || options.observeSteering || options.isProbe()) {
            vehicleParams=car2026::Params::load(options.vehicleFile);
            if(options.steerCommand && std::abs(*options.steerCommand)>vehicleParams->max_steer_deg)
                throw std::runtime_error("Steering command exceeds vehicle-config command limit");
            if(options.isProbe() && vehicleParams->max_steer_deg<2)
                throw std::runtime_error("Steering probe needs a configured command limit of at least 2");
            if(options.autoProbe && vehicleParams->max_steer_deg<car2026::capture::AutomaticSteeringProbe::trialCommand)
                throw std::runtime_error("Automatic probe needs a configured command limit of at least 5");
        }
        fillEncoderPaths(config,hardware);
        if(options.check) {
            const auto missing=car2026::capture::validateSensors(config,options.allowPartial);
            for(const auto& name:missing) std::cout<<"UNCONFIGURED "<<name<<'\n';
            std::cout<<"Config checked without hardware access\n";return missing.empty() ? 0 : 2;
        }
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        return capture(config,hardware,options.configFile,options.hardwareFile,options.output,
                       options.duration,options.allowPartial,options,vehicleParams ? &*vehicleParams : nullptr);
    } catch(const std::exception& error) {
        std::cerr<<"manual_capture: "<<error.what()<<'\n';return 1;
    }
}
