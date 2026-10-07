#include "capture_data.hpp"
#include "hardware_linux.hpp"
#include "device_read_buffer.hpp"
#include "servo_output.hpp"
#include "manual_capture_options.hpp"
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
#include <poll.h>
#include <thread>
#include <sys/stat.h>
#include <unistd.h>

namespace fs=std::filesystem;
using car2026::capture::Config;
using car2026::capture::SensorSpec;
namespace {
constexpr const char* recorderVersion="2026-10-07.1";
constexpr const char* sensorHeader="cycle,channel,read_start_ns,read_end_ns,elapsed_s,value,unit,valid,status,read_return,raw_hex";
constexpr const char* frameHeader="frame_index,read_start_ns,read_end_ns,elapsed_s";
constexpr const char* markerHeader="monotonic_ns,elapsed_s,marker";
constexpr const char* observerHeader="frame_index,frame_read_start_s,processed_time_s,frame_age_s,motion,reference_id,has_reference,state,line_confidence,line_ambiguous,line_discontinuous,lateral_error_image,heading_error_image,suggestion_valid,suggested_command,motor_writes,servo_writes";
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
void fillEncoderPaths(Config& config,const car2026::HardwareConfig& hardware) {
    // Only these verified interfaces have defaults. No guessed GPIO pin numbers.
    if(config.sensors[0].path.empty()) config.sensors[0].path=hardware.encoder_left;
    if(config.sensors[1].path.empty()) config.sensors[1].path=hardware.encoder_right;
    if(config.factoryEncoderStatusZero &&
       (config.sensors[0].path!=hardware.encoder_left || config.sensors[1].path!=hardware.encoder_right))
        throw std::runtime_error("Factory status-zero reads are restricted to configured board encoder paths");
}
void sensorLoop(const Config& config,const fs::path& directory,int64_t origin,
                std::atomic<bool>& stop,std::array<Stats,8>& stats,std::exception_ptr& failure) {
    try {
        auto file=outputFile(directory/"sensors.csv");
        file<<sensorHeader<<'\n';file.flush();nonemptyFileSize(directory/"sensors.csv");
        uint64_t cycle=0;auto next=std::chrono::steady_clock::now(),lastFlush=next;
        const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double,std::milli>(config.sampleMs));
        while(!stop.load()) {
            for(size_t index=0;index<config.sensors.size();++index) {
                const auto& sensor=config.sensors[index];const auto before=monotonicNs();
                const auto sample=readSensor(sensor,config.factoryEncoderStatusZero);const auto after=monotonicNs();
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
bool recordMarkers(std::ofstream& markers,int64_t origin,std::string& pending,bool& inputEnded) {
    if(inputEnded) return true;
    struct pollfd input{STDIN_FILENO,POLLIN,0};
    const int ready=poll(&input,1,0);
    if(ready<0) {if(errno==EINTR) return true;throw std::runtime_error("Terminal poll failed");}
    if(ready<=0 || !(input.revents&POLLIN)) return true;
    // Avoid iostream prefetch: queued lines must not wait for another keypress.
    char buffer[128];const ssize_t count=read(STDIN_FILENO,buffer,sizeof(buffer));
    if(count<0) {if(errno==EINTR) return true;throw std::runtime_error("Terminal read failed");}
    if(count==0) {inputEnded=true;return true;}
    pending.append(buffer,size_t(count));
    size_t end;
    while((end=pending.find('\n'))!=std::string::npos) {
        const auto line=car2026::capture::trimmed(pending.substr(0,end));pending.erase(0,end+1);
        if(line=="q" || line=="Q") return false;
        if(line.size()==1) {
            char marker=line[0];if(marker>='a' && marker<='e') marker=char(marker-'a'+'A');
            if(marker>='A' && marker<='E') {
                const auto now=monotonicNs();markers<<now<<','<<double(now-origin)/1e9<<','<<marker<<'\n';
                markers.flush();std::cout<<"MARK "<<marker<<'\n';
            }
        }
    }
    if(pending.size()>128) pending.clear();
    return true;
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
    const std::string mode=options.observeSteering ? "VISUAL_STEERING_OBSERVER_NO_ACTUATOR_WRITES" :
        (options.steerCommand ? "MANUAL_PUSH_SERVO_ONLY" : "MANUAL_CAPTURE_NO_ACTUATOR_WRITES");
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
                <<"\nobserver_target=first stable image reference; NOT a measured bay axis or heading\n"
                  "suggestions_are_applied=0\nmotor_writes=0\nservo_writes=0\n"
                  "observer_age=processing completion minus camera read start; exposure age is unknown\n";
    for(const auto& sensor:config.sensors)
        metadata<<sensor.name<<"="<<sensor.path<<" format="<<sensor.format<<" unit="<<sensor.unit<<'\n';
    metadata.flush();nonemptyFileSize(directory/"session.txt");
    auto frames=outputFile(directory/"frames.csv"),markers=outputFile(directory/"markers.csv");
    frames<<frameHeader<<'\n';markers<<markerHeader<<'\n';
    frames.flush();markers.flush();
    nonemptyFileSize(directory/"frames.csv");nonemptyFileSize(directory/"markers.csv");
    std::cout<<"SESSION "<<directory.string()<<'\n';
    if(options.steerCommand)
        std::cout<<"SERVO ONLY; no motor/GPIO/IMU initialization or writes. Wait for STEER_READY before manual pushing.\n"
                   "Stop pushing before Q/Ctrl+C or duration expiry; exit attempts configured zero steering.\n";
    else std::cout<<"No motor/servo/IMU initialization.\n";
    if(options.observeSteering)
        std::cout<<"OBSERVE ONLY motion="<<car2026::manualMotionName(*options.observeSteering)
                 <<"; suggestions are NOT applied. Keep the initial image steady until REFERENCE_READY.\n";
    std::cout<<"Enter A..E to mark, Q or Ctrl+C to finish.\n";
    if(!missing.empty()) {
        std::cout<<"PARTIAL: unconfigured channels:";
        for(const auto& name:missing) std::cout<<' '<<name;
        std::cout<<'\n';
    }
    std::cout<<"factory_encoder_status_zero="<<config.factoryEncoderStatusZero<<'\n';
    std::atomic<bool> stop{false};std::array<Stats,8> stats;
    std::exception_ptr sensorFailure,videoFailure;
    cv::VideoWriter video;uint64_t count=0;int width=0,height=0;
    std::unique_ptr<car2026::LinuxDeviceIo> servoIo;
    std::unique_ptr<car2026::ServoOutput> servo;
    std::optional<car2026::Vision> observerVision;
    std::optional<car2026::VisualSteeringObserver> observer;
    std::ofstream observerFile;car2026::SteeringObservation latestSuggestion;
    if(options.observeSteering) {
        observerVision.emplace(*vehicleParams);observer.emplace(*vehicleParams,*options.observeSteering);
        observerFile=outputFile(directory/"visual_observer.csv");observerFile<<observerHeader<<'\n';observerFile.flush();
    }
    std::thread sampler(sensorLoop,std::cref(config),directory,origin,std::ref(stop),
                        std::ref(stats),std::ref(sensorFailure));
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
            if(observer) {
                const auto observation=observerVision->analyze(car2026::processCameraFrame(image,*vehicleParams),car2026::Stage::GarageReverse);
                const double frameTime=double(before-origin)/1e9,processedTime=double(monotonicNs()-origin)/1e9;
                const bool previouslyReferenced=latestSuggestion.hasReference;
                latestSuggestion=observer->observe(observation,frameTime,processedTime);
                observerFile<<count-1<<','<<frameTime<<','<<processedTime<<','<<latestSuggestion.frameAge<<','
                            <<car2026::manualMotionName(*options.observeSteering)<<','<<latestSuggestion.referenceId<<','
                            <<latestSuggestion.hasReference<<','<<latestSuggestion.state<<','<<observation.blackPath.confidence<<','
                            <<observation.blackPath.ambiguous<<','<<observation.blackPath.discontinuous<<',';
                if(latestSuggestion.hasSuggestion) observerFile<<latestSuggestion.lateralError;
                observerFile<<',';
                if(latestSuggestion.hasSuggestion) observerFile<<latestSuggestion.headingFeatureError;
                observerFile<<','<<latestSuggestion.hasSuggestion<<',';
                if(latestSuggestion.hasSuggestion) observerFile<<latestSuggestion.suggestedCommand;
                observerFile<<",0,0\n";
                if(!previouslyReferenced && latestSuggestion.hasReference)
                    std::cout<<"REFERENCE_READY id="<<latestSuggestion.referenceId<<"; image target only, no actuator output\n"<<std::flush;
            }
            if(!recordMarkers(markers,origin,pendingInput,inputEnded)) break;
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
                if(fs::space(directory).available<64u*1024u*1024u) throw std::runtime_error("Recording disk space below 64 MiB");
                std::cout<<"frames="<<count<<" elapsed="<<std::fixed<<std::setprecision(1)<<double(after-origin)/1e9;
                for(size_t i=0;i<stats.size();++i)
                    std::cout<<' '<<config.sensors[i].name<<"(ok/bad)="<<stats[i].valid.load()<<'/'<<stats[i].invalid.load();
                if(observer) {
                    std::cout<<" observe="<<latestSuggestion.state<<" suggested=";
                    if(latestSuggestion.hasSuggestion) std::cout<<latestSuggestion.suggestedCommand;else std::cout<<"unavailable";
                    std::cout<<" (NOT applied)";
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
            servo->close();
            std::cout<<"SERVO_ZERO write completed (software only)\n";
        } catch(...) {servoFailure=std::current_exception();}
    }
    sampler.join();video.release();camera.release();
    if(options.steerCommand) metadata<<"servo_command_attempted="<<bool(servo && servo->commandAttempted())<<'\n';
    if(servo && servo->commandAttempted()) metadata<<"servo_return_zero_write_ok="<<!servoFailure<<'\n';
    closeText(frames,directory/"frames.csv");closeText(markers,directory/"markers.csv");
    if(observer) closeText(observerFile,directory/"visual_observer.csv");
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
                             "Observation: no actuator writes; image-reference suggestions only; conflicts with --steer-command.\n"
                             "Default: no actuator writes. Explicit steering: SERVO ONLY, return to configured zero on exit.\n"
                             "No motor/GPIO-output/IMU initialization. Check modes never access hardware.\n";return 0;
        }
        if(options.checkOutput) return checkOutput(options.output);
        auto config=car2026::capture::loadConfig(options.configFile);
        const auto hardware=car2026::HardwareConfig::load(options.hardwareFile);
        std::optional<car2026::Params> vehicleParams;
        if(options.steerCommand || options.observeSteering) {
            vehicleParams=car2026::Params::load(options.vehicleFile);
            if(options.steerCommand && std::abs(*options.steerCommand)>vehicleParams->max_steer_deg)
                throw std::runtime_error("Steering command exceeds vehicle-config command limit");
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
