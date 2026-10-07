#include "core.hpp"
#include "imu.hpp"
#include "hardware_linux.hpp"
#include "Contral/PID/PID.h"
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>
#include <opencv2/highgui.hpp>
#include <atomic>
#include <chrono>
#include <csignal>
#include <iomanip>
#include <memory>
#include <iostream>
#include <mutex>
#include <thread>

using namespace car2026;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signalHandler(int) { interrupted=1; }
double seconds() {
    return std::chrono::duration<double>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct Bus {
    std::mutex mutex;
    Telemetry telemetry;
    Command command;
    double commandTime=0;
    std::string fault;
};
class Drive {
public:
    explicit Drive(const Params& p,const HardwareConfig& config)
        : p_(p),board_(io_,config,p),imu_(io_,config,p),voice_(config) {
        for(auto* pid : {&leftPid_,&rightPid_}) {
            if(PID_Initialize(pid)!=PID_RESULT_SUCCESS ||
               PID_Incremental_Set_Kpid(pid,p.pid_kp,p.pid_ki,p.pid_kd)!=PID_RESULT_SUCCESS ||
               PID_Incremental_Open_OutLimit(pid)!=PID_RESULT_SUCCESS ||
               PID_Incremental_Set_OutLimitValue(pid,p.pwm_limit)!=PID_RESULT_SUCCESS)
                throw std::runtime_error("Cannot initialize motor PID");
        }
    }
    ~Drive() {stop();}
    bool speakZebra() {return voice_.speakZebra();}
    void stop() noexcept {
        if(!board_.stop()) std::cerr<<"[2026 ERROR] At least one shutdown device write failed\n";
    }
    void run(Bus& bus,std::atomic<bool>& running) {
        double last=seconds(),distance=0,estimatedYaw=0,targetSpeed=0,stallSince=0;
        int wrongDirectionCount=0;
        try {
            while(running && !interrupted) {
                double now=seconds(),dt=now-last;last=now;
                if(dt<=0 || dt>0.15) throw std::runtime_error("Hardware control sampling gap");
                const auto wheels=board_.sampleWheels(now);
                double l=wheels.leftRps,r=wheels.rightRps;
                const bool encoderValid=true; // Failed device reads/range checks throw, never fabricate zero.
                Command command;double age;
                {std::lock_guard<std::mutex> lock(bus.mutex);command=bus.command;age=now-bus.commandTime;}
                const double measuredSpeed=(l+r)*.5*p_.wheel_circumference_m;
                distance+=wheels.distanceM;
                // Ackermann estimate is available for bench use; it is not claimed as measured IMU yaw.
                estimatedYaw-=measuredSpeed*std::tan(command.steer*p_.steer_to_wheel_ratio*pi/180)/p_.wheelbase_m*180/pi*dt;
                imu_.poll(now,l==0 && r==0);
                Telemetry telemetry{now,distance,measuredSpeed,imu_.valid(now)?imu_.yaw():estimatedYaw,
                                    encoderValid,imu_.valid(now),encoderValid};
                if(p_.require_imu) telemetry.yawValid=telemetry.yawMeasured;
                std::string fault;
                if(board_.stopInputActive()) fault="Configured board stop input is active";
                if(age>p_.frame_timeout_s) fault="Camera/control command watchdog expired";
                if(p_.require_imu && std::abs(command.speed)>0 && !telemetry.yawMeasured) fault="IMU freshness/continuity lost";
                if(std::abs(command.speed)>.025 && (std::abs(l)*p_.wheel_circumference_m<.01 || std::abs(r)*p_.wheel_circumference_m<.01)) {
                    if(stallSince==0) stallSince=now;
                    if(now-stallSince>p_.stall_timeout_s) fault="Motor stalled or encoder stopped";
                } else stallSince=0;
                if(std::abs(command.speed)>.025 && (l*p_.wheel_circumference_m*command.speed<-.001 || r*p_.wheel_circumference_m*command.speed<-.001)) ++wrongDirectionCount;else wrongDirectionCount=0;
                if(wrongDirectionCount>=5) fault="Encoder sign or motor direction is reversed";
                {
                    std::lock_guard<std::mutex> lock(bus.mutex);bus.telemetry=telemetry;
                    if(!fault.empty() && bus.fault.empty()) bus.fault=fault;
                    if(!bus.fault.empty()) {command.speed=0;command.steer=0;fault=bus.fault;}
                }
                if(command.speed==0 || age>p_.frame_timeout_s) {
                    board_.setMotorCommands(0,0);targetSpeed=0;
                    PID_Incremental_Clear_TempData(&leftPid_);PID_Incremental_Clear_TempData(&rightPid_);
                } else {
                    // Cross zero before reversing. This also prevents residual forward PID output.
                    if(targetSpeed*command.speed<0) {
                        targetSpeed=0;board_.setMotorCommands(0,0);
                        PID_Incremental_Clear_TempData(&leftPid_);PID_Incremental_Clear_TempData(&rightPid_);
                    } else {
                        targetSpeed+=clamp(command.speed-targetSpeed,-p_.accel_m_s2*dt,p_.accel_m_s2*dt);
                        const double aimRps=targetSpeed/p_.wheel_circumference_m;
                        board_.setMotorCommands(PID_Incremental_CalcResult_ByNowTureValue(&leftPid_,l,aimRps),
                                                PID_Incremental_CalcResult_ByNowTureValue(&rightPid_,r,aimRps));
                    }
                }
                board_.setSteering(command.steer);
                board_.setBeep(!fault.empty());
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
        } catch(const std::exception& e) {
            std::lock_guard<std::mutex> lock(bus.mutex);bus.fault=e.what();
        }
        stop();
    }
private:
    HardwareLock hardwareLock_;
    Params p_;LinuxDeviceIo io_;FactoryBoard board_;HardwareImu imu_;VoiceOutput voice_;
    st_PID_Attr leftPid_{},rightPid_{};
};

Image convert(const cv::Mat& bgr,const Params& p) {
    cv::Mat small;cv::resize(bgr,small,cv::Size(int(p.process_width),int(p.process_height)),0,0,cv::INTER_AREA);
    Image image(small.cols,small.rows);
    for(int y=0;y<small.rows;++y) {
        auto row=small.ptr<cv::Vec3b>(y);
        for(int x=0;x<small.cols;++x) image.at(x,y)={row[x][2],row[x][1],row[x][0]};
    }
    return image;
}
void drawPath(cv::Mat& frame,const Path& path,cv::Scalar color) {
    for(size_t y=0;y<path.x.size();++y) if(path.x[y]>=0)
        cv::circle(frame,cv::Point(int(path.x[y]*frame.cols),int(double(y)/path.x.size()*frame.rows)),1,color,-1);
}
}

int main(int argc,char** argv) {
    bool drive=false,preview=false,hardwareCheck=false;
    std::string config="config/competition.ini",hardwareConfig="config/hardware.ini",video,record,telemetryFile;
    std::unique_ptr<Drive> hardware;std::atomic<bool> running{true};std::thread worker;
    Bus bus;
    try {
        for(int i=1;i<argc;++i) {
            std::string arg=argv[i];
            if(arg=="--drive") drive=true;
            else if(arg=="--preview") preview=true;
            else if(arg=="--config" && i+1<argc) config=argv[++i];
            else if(arg=="--hardware-config" && i+1<argc) hardwareConfig=argv[++i];
            else if(arg=="--hardware-check") hardwareCheck=true;
            else if(arg=="--video" && i+1<argc) video=argv[++i];
            else if(arg=="--record" && i+1<argc) record=argv[++i];
            else if(arg=="--telemetry" && i+1<argc) telemetryFile=argv[++i];
            else if(arg=="--help") {std::cout<<"SMART_CAR_2026 [--drive] [--config file] [--hardware-config file] [--hardware-check] [--preview] [--video file] [--record file.avi] [--telemetry file.csv]\nDefault: camera perception only, no motion hardware initialized.\nHardware check reads devices without writing outputs; delta encoder reads consume counts.\n";return 0;}
            else throw std::runtime_error("Unknown/incomplete argument: "+arg);
        }
        Params params=Params::load(config);
        const HardwareConfig boardConfig=HardwareConfig::load(hardwareConfig);
        if(hardwareCheck) {
            if(drive || !video.empty()) throw std::runtime_error("--hardware-check cannot combine with drive/video");
            HardwareLock lock;LinuxDeviceIo io;inspectHardware(io,boardConfig,std::cout);return 0;
        }
        if(drive && !video.empty()) throw std::runtime_error("Recorded video cannot drive hardware");
        if(drive && !params.motion_calibrated) throw std::runtime_error("Set motion_calibrated=1 only after wheel, steering, encoder and parking calibration");
        if(drive && params.require_imu && boardConfig.imu_backend=="none") throw std::runtime_error("require_imu=1 requires IIO or serial IMU");
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        cv::VideoCapture capture;
        if(video.empty()) {
            capture.open(boardConfig.camera_device,cv::CAP_V4L2);
            capture.set(cv::CAP_PROP_FRAME_WIDTH,params.camera_width);capture.set(cv::CAP_PROP_FRAME_HEIGHT,params.camera_height);
            capture.set(cv::CAP_PROP_BUFFERSIZE,1);
        } else capture.open(video);
        if(!capture.isOpened()) throw std::runtime_error("Camera/video could not be opened");
        std::cout<<"Camera actual resolution "<<capture.get(cv::CAP_PROP_FRAME_WIDTH)<<"x"<<capture.get(cv::CAP_PROP_FRAME_HEIGHT)<<'\n';
        cv::Mat frame;
        if(!capture.read(frame)||frame.empty()) throw std::runtime_error("Camera startup frame missing");
        cv::VideoWriter recorder;
        if(!record.empty()) {
            recorder.open(record,cv::VideoWriter::fourcc('M','J','P','G'),20,frame.size());
            if(!recorder.isOpened()) throw std::runtime_error("Recording path/codec unavailable");
        }
        Vision vision(params);Mission mission(params);
        std::ofstream telemetryLog;
        if(!telemetryFile.empty()) {
            if(!drive) throw std::runtime_error("Telemetry recording requires real drive sensors");
            telemetryLog.open(telemetryFile);
            if(!telemetryLog) throw std::runtime_error("Telemetry output unavailable");
            telemetryLog<<"time,distance,speed,yaw,yaw_valid,yaw_measured\n"<<std::setprecision(17);
        }
        if(drive) {
            hardware=std::make_unique<Drive>(params,boardConfig);
            {std::lock_guard<std::mutex> lock(bus.mutex);bus.commandTime=seconds();}
            worker=std::thread([&]{hardware->run(bus,running);});
            // Motors remain zero until a fresh measured IMU frame is available.
            const double startupTimeout=boardConfig.imu_backend=="iio" ? boardConfig.imu_calibration_samples*0.02+3 : 2;
            const double deadline=seconds()+startupTimeout;
            while(params.require_imu && seconds()<deadline && !interrupted) {
                bool ready;std::string fault;
                {std::lock_guard<std::mutex> lock(bus.mutex);bus.commandTime=seconds();ready=bus.telemetry.yawMeasured;fault=bus.fault;}
                if(!fault.empty()) throw std::runtime_error(fault);
                if(ready) break;
                std::this_thread::sleep_for(std::chrono::milliseconds(20));
            }
            std::lock_guard<std::mutex> lock(bus.mutex);
            if(params.require_imu && !bus.telemetry.yawMeasured) throw std::runtime_error("IMU startup/calibration timed out; keep chassis stationary and verify gyro scale");
        }
        // Hardware setup and IMU startup may take seconds. The frame read before
        // initialization is only for camera/recorder setup, not a motion command.
        if(drive && (!capture.read(frame) || frame.empty()))
            throw std::runtime_error("Camera frame missing after hardware initialization");
        int exitCode=0;double lastPrint=0;Stage previous=Stage::Depart;
        do {
            const double now=seconds();
            auto image=convert(frame,params);
            auto observation=vision.analyze(image,mission.stage());
            auto avoidance=vision.avoidCones(observation,image);
            Command command;
            if(drive) {
                Telemetry telemetry;
                {std::lock_guard<std::mutex> lock(bus.mutex);telemetry=bus.telemetry;if(!bus.fault.empty()) mission.fail(bus.fault);}
                command=mission.update(observation,telemetry,avoidance);
                if(telemetryLog.is_open()) {
                    telemetryLog<<telemetry.time<<','<<telemetry.distance<<','<<telemetry.speed<<','<<telemetry.yaw<<','<<telemetry.yawValid<<','<<telemetry.yawMeasured<<'\n';
                    if(!telemetryLog) throw std::runtime_error("Telemetry write failed");
                }
                {std::lock_guard<std::mutex> lock(bus.mutex);bus.command=command;bus.commandTime=now;}
            } else {
                command.stage=mission.stage();command.reason="PERCEPTION_ONLY";
                // No invented distance or yaw from an ordinary video. Use replay telemetry CSV for full task simulation.
            }
            if(command.speak) {
                std::cout<<"人行道前停车礼让行人\n"<<std::flush;
                if(hardware && !hardware->speakZebra()) std::cerr<<"[2026 WARNING] Voice playback unavailable/failed\n";
            }
            if(command.stage!=previous || now-lastPrint>.5) {
                std::cout<<"[2026] "<<stageName(command.stage)<<" laps="<<command.completedLaps<<" speed="<<command.speed<<" steer="<<command.steer
                         <<" black="<<observation.blackPath.confidence<<" road="<<observation.roadPath.confidence<<" cones="<<observation.cones.size()
                         <<" stripe="<<observation.stripe<<" bar="<<observation.bar<<" garage="<<observation.garage<<" "<<command.reason<<'\n';
                lastPrint=now;previous=command.stage;
            }
            if(recorder.isOpened()) recorder.write(frame);
            if(preview) {
                drawPath(frame,observation.blackPath,{0,0,255});drawPath(frame,observation.roadPath,{0,255,0});
                cv::putText(frame,stageName(command.stage),{10,30},cv::FONT_HERSHEY_SIMPLEX,.7,{0,0,255},2);
                cv::imshow("2026 black/red road/green",frame);if(cv::waitKey(1)==27) break;
            }
            if(command.stage==Stage::Fault) {exitCode=1;break;}
            if(command.stage==Stage::Complete) break;
            if(interrupted) break;
        } while(capture.read(frame) && !frame.empty());
        if(drive && !interrupted && mission.stage()!=Stage::Complete && mission.stage()!=Stage::Fault) exitCode=1;
        running=false;if(worker.joinable()) worker.join();hardware.reset();
        return exitCode;
    } catch(const std::exception& e) {
        running=false;if(worker.joinable()) worker.join();hardware.reset();
        std::cerr<<"[2026 ERROR] "<<e.what()<<'\n';return 1;
    }
}
