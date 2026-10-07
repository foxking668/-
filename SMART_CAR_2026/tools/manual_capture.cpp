#include "capture_data.hpp"
#include "hardware_linux.hpp"
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
#include <poll.h>
#include <thread>
#include <unistd.h>

namespace fs=std::filesystem;
using car2026::capture::Config;
using car2026::capture::SensorSpec;
namespace {
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
    std::ofstream file(path);if(!file) throw std::runtime_error("Cannot create "+path.string());
    file.exceptions(std::ios::failbit|std::ios::badbit);file<<std::setprecision(17);return file;
}
car2026::capture::Sample readSensor(const SensorSpec& sensor) {
    using car2026::capture::Sample;
    if(sensor.path.empty()) {Sample result;result.status="unconfigured";return result;}
    // Only sensor inputs are opened, read-only. No GPIO export/direction writes.
    const int fd=open(sensor.path.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if(fd<0) {Sample result;result.status="open_failed:"+std::string(std::strerror(errno));return result;}
    const size_t size=sensor.format=="i16le" ? 2 : sensor.format=="u8" ? 1 : 256;
    std::vector<uint8_t> bytes(size,0xa5);ssize_t count;
    // Never finish a short transfer with a second read: delta counters may clear.
    do {count=read(fd,bytes.data(),bytes.size());} while(count<0 && errno==EINTR && !interrupted);
    const int error=errno;close(fd);
    auto result=car2026::capture::decode(sensor,long(count),bytes);
    if(count<0) result.status="read_failed:"+std::string(std::strerror(error));
    return result;
}
struct Stats {std::atomic<uint64_t> valid{0},invalid{0};};
void fillEncoderPaths(Config& config,const car2026::HardwareConfig& hardware) {
    // Only these verified interfaces have defaults. No guessed GPIO pin numbers.
    if(config.sensors[0].path.empty()) config.sensors[0].path=hardware.encoder_left;
    if(config.sensors[1].path.empty()) config.sensors[1].path=hardware.encoder_right;
}
void sensorLoop(const Config& config,const fs::path& directory,int64_t origin,
                std::atomic<bool>& stop,std::array<Stats,8>& stats,std::exception_ptr& failure) {
    try {
        auto file=outputFile(directory/"sensors.csv");
        file<<"cycle,channel,read_start_ns,read_end_ns,elapsed_s,value,unit,valid,status,read_return,raw_hex\n";
        uint64_t cycle=0;auto next=std::chrono::steady_clock::now(),lastFlush=next;
        const auto period=std::chrono::duration_cast<std::chrono::steady_clock::duration>(
            std::chrono::duration<double,std::milli>(config.sampleMs));
        while(!stop.load()) {
            for(size_t index=0;index<config.sensors.size();++index) {
                const auto& sensor=config.sensors[index];const auto before=monotonicNs();
                const auto sample=readSensor(sensor);const auto after=monotonicNs();
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
        file.flush();
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
            const std::string& hardwareConfig,const fs::path& base,double duration,bool allowPartial) {
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
    const auto origin=monotonicNs();
    auto metadata=outputFile(directory/"session.txt");
    metadata<<"mode=MANUAL_CAPTURE_NO_ACTUATOR_WRITES\nimu=not_installed\norigin_ns="<<origin
            <<"\nstart_utc="<<utcStamp()<<"\nencoder_mode="<<hardware.encoder_mode
            <<"\nencoder_values=raw signed counts; first delta read includes the untimed startup interval\n"
              "sensor_values=raw declared units; no guessed sign, scale, distance or steering angle\n"
              "timestamps=host steady clock; video read brackets are NOT hardware exposure timestamps\n"
              "alignment=use frames.csv and sensors.csv; AVI nominal FPS alone is not elapsed time\n"
            <<"nominal_video_fps="<<config.fps<<"\nsample_period_ms="<<config.sampleMs<<'\n';
    for(const auto& sensor:config.sensors)
        metadata<<sensor.name<<"="<<sensor.path<<" format="<<sensor.format<<" unit="<<sensor.unit<<'\n';
    metadata.flush();
    auto frames=outputFile(directory/"frames.csv"),markers=outputFile(directory/"markers.csv");
    frames<<"frame_index,read_start_ns,read_end_ns,elapsed_s\n";
    markers<<"monotonic_ns,elapsed_s,marker\n";
    std::cout<<"SESSION "<<directory.string()<<"\nNo motor/servo/IMU initialization."
               " Enter A..E to mark, Q or Ctrl+C to finish.\n";
    if(!missing.empty()) std::cout<<"PARTIAL: grayscale/ultrasonic interfaces remain unconfigured.\n";
    std::atomic<bool> stop{false};std::array<Stats,8> stats;
    std::exception_ptr sensorFailure,videoFailure;
    cv::VideoWriter video;uint64_t count=0;int width=0,height=0;
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
                video.open((directory/"camera.avi").string(),cv::VideoWriter::fourcc('M','J','P','G'),
                           config.fps,cv::Size(width,height));
                if(!video.isOpened()) throw std::runtime_error("Cannot initialize MJPG video writer");
            }
            if(image.cols!=width || image.rows!=height) throw std::runtime_error("Camera dimensions changed");
            video.write(image);frames<<count++<<','<<before<<','<<after<<','<<double(after-origin)/1e9<<'\n';
            if(!recordMarkers(markers,origin,pendingInput,inputEnded)) break;
            const auto now=std::chrono::steady_clock::now();
            if(now-lastReport>=std::chrono::seconds(1)) {
                frames.flush();lastReport=now;
                if(fs::space(directory).available<64u*1024u*1024u) throw std::runtime_error("Recording disk space below 64 MiB");
                std::cout<<"frames="<<count<<" elapsed="<<std::fixed<<std::setprecision(1)<<double(after-origin)/1e9;
                for(size_t i=0;i<stats.size();++i)
                    std::cout<<' '<<config.sensors[i].name<<"(ok/bad)="<<stats[i].valid.load()<<'/'<<stats[i].invalid.load();
                std::cout<<std::endl;
            }
        }
        frames.flush();markers.flush();
    } catch(...) {videoFailure=std::current_exception();}
    stop.store(true);sampler.join();video.release();camera.release();
    bool sensorsGood=missing.empty();
    metadata<<"end_utc="<<utcStamp()<<"\nframe_rows="<<count<<"\nactual_width="<<width<<"\nactual_height="<<height<<'\n';
    for(size_t i=0;i<stats.size();++i) {
        const auto valid=stats[i].valid.load(),invalid=stats[i].invalid.load();
        metadata<<config.sensors[i].name<<"_valid="<<valid<<" invalid="<<invalid<<'\n';
        if(!valid || invalid) sensorsGood=false;
    }
    metadata<<"all_channels_configured="<<missing.empty()<<"\nall_sensor_samples_valid="<<sensorsGood
            <<"\ncapture_loop_failed="<<bool(videoFailure || sensorFailure)<<'\n';
    for(const auto& failure:{videoFailure,sensorFailure}) if(failure) {
        try {std::rethrow_exception(failure);}
        catch(const std::exception& error) {metadata<<"capture_error="<<error.what()<<'\n';}
    }
    metadata.flush();
    if(videoFailure) std::rethrow_exception(videoFailure);
    if(sensorFailure) std::rethrow_exception(sensorFailure);
    if(!count) throw std::runtime_error("No video frames recorded");
    std::cout<<"SAVED "<<directory.string()<<"\n";
    return sensorsGood ? 0 : 2;
}
}
int main(int argc,char** argv) {
    try {
        std::string configFile="manual_capture.ini",hardwareFile="config/hardware.ini";
        fs::path output="captures";double duration=180;bool allowPartial=false,check=false;
        for(int i=1;i<argc;++i) {
            const std::string arg=argv[i];
            if(arg=="--help") {
                std::cout<<"manual_capture [--config file] [--hardware-config file] [--output directory]"
                             " [--duration seconds] [--allow-partial] [--check-config]\n"
                             "No motor, servo, GPIO-output or IMU initialization.\n";return 0;
            }
            if(arg=="--allow-partial") {allowPartial=true;continue;}
            if(arg=="--check-config") {check=true;continue;}
            if(i+1>=argc) throw std::runtime_error("Missing option value: "+arg);
            const std::string value=argv[++i];
            if(arg=="--config") configFile=value;
            else if(arg=="--hardware-config") hardwareFile=value;
            else if(arg=="--output") output=value;
            else if(arg=="--duration") duration=car2026::capture::finiteNumber(value);
            else throw std::runtime_error("Unknown option: "+arg);
        }
        if(duration<=0 || duration>3600) throw std::runtime_error("duration must be between 0 and 3600 seconds");
        auto config=car2026::capture::loadConfig(configFile);
        const auto hardware=car2026::HardwareConfig::load(hardwareFile);
        fillEncoderPaths(config,hardware);
        if(check) {
            const auto missing=car2026::capture::validateSensors(config,allowPartial);
            for(const auto& name:missing) std::cout<<"UNCONFIGURED "<<name<<'\n';
            std::cout<<"Config checked without hardware access\n";return missing.empty() ? 0 : 2;
        }
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        return capture(config,hardware,configFile,hardwareFile,output,duration,allowPartial);
    } catch(const std::exception& error) {
        std::cerr<<"manual_capture: "<<error.what()<<'\n';return 1;
    }
}
