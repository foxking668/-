#include "Parking/AutoParking.hpp"
#include "Parking/AutoParkingRuntime.hpp"
#include "Parking/SteeringOutput.hpp"
#include "Parking/SteeringValidation.hpp"
#include "headfile.hpp"
#include <atomic>
#include <filesystem>
#include <memory>

namespace AutoParking {
namespace {
using Clock=std::chrono::steady_clock;
std::mutex mutex;
std::unique_ptr<Controller> controller;
std::atomic<bool> enabledFlag{false},manualFlag{false},takeoverFlag{false},cameraFlag{true};
Lane latestLane;
std::array<std::ofstream,6> logs;
std::ofstream events;
std::string directory;
Phase reportedPhase=Phase::Cruise;
int reportedStage=0;
double lastStatus=-1;
double nowSeconds() {return std::chrono::duration<double>(Clock::now().time_since_epoch()).count();}
void row(int stage,const char* event,double now,const Output& out,double elapsed,double left=0,double right=0) {
    if(stage<1 || stage>6 || !logs[stage-1].is_open()) return;
    auto& file=logs[stage-1];
    file<<std::fixed<<std::setprecision(6)<<now<<','<<event<<','<<stage<<','<<elapsed<<','
        <<out.steer<<','<<out.left<<','<<out.right<<','<<left<<','<<right<<','<<latestLane.at<<'\n';
    if(!file.good()) throw std::runtime_error("Parking CSV write failed");
}
void event(const char* name,double now,const std::string& detail="") {
    if(!events.is_open()) return;
    events<<std::fixed<<std::setprecision(6)<<now<<','<<name<<','<<detail<<'\n';events.flush();
    if(!events.good()) throw std::runtime_error("Parking event log write failed");
}
void openLogs() {
    const auto wall=std::chrono::system_clock::now().time_since_epoch();
    const auto tag=std::chrono::duration_cast<std::chrono::milliseconds>(wall).count();
    directory="captures/parking_auto/run_"+std::to_string(tag)+"_"+std::to_string(getpid());
    // create_directory deliberately rejects an existing session instead of truncating its logs.
    std::filesystem::create_directories("captures/parking_auto");
    if(!std::filesystem::create_directory(directory)) throw std::runtime_error("Parking log directory already exists");
    for(int i=0;i<6;++i) {
        logs[i].open(directory+"/"+files[i]);
        if(!logs[i]) throw std::runtime_error("Cannot open parking stage CSV");
        logs[i]<<"steady_s,event,stage,motor_elapsed_s,steer_command,left_pwm_ns,right_pwm_ns,left_rps,right_rps,last_valid_lane_s\n";
        row(i+1,i>0 && i<5 && controller->config().stages[i].seconds==0?"SKIPPED":"CONFIGURED",
            nowSeconds(),{},0);
    }
    events.open(directory+"/events.csv");
    if(!events) throw std::runtime_error("Cannot open parking event CSV");
    events<<"steady_s,event,detail\n";
    std::ofstream snapshot(directory+"/parking_tuning.snapshot.ini");
    controller->config().write(snapshot);snapshot.flush();
    if(!snapshot.good()) throw std::runtime_error("Cannot save actual parking configuration");
    std::filesystem::copy_file("image_params.txt",directory+"/image_params.snapshot.txt");
}
bool steerWrite(Contral::Steer* steer,double command) {
    return writeSteeringCommand(steer,command,Other::RuntimeConfig::getSteerOffsetDeg());
}
void zero(Contral::Motor* motor) {
    if(!motor) return;
    stopMotorOutputs(motor);
    Other::gMotorAimSpeed=0;
}
void flush() {
    for(auto& f:logs) if(f.is_open()) {f.flush();if(!f.good()) throw std::runtime_error("Parking CSV flush failed");}
}
}
void initialize(const std::string& path) {
    std::lock_guard<std::mutex> lock(mutex);
    const Config config=Config::read(path);
    // The check-only entry must not generate a default image file and guess its offset.
    if(!std::ifstream("image_params.txt").good())
        throw std::runtime_error("Cannot read image_params.txt for actual steering offset preflight");
    const double offset=Other::RuntimeConfig::getSteerOffsetDeg();
    validateSteeringCommands(config,offset);
    controller=std::make_unique<Controller>(config);
    latestLane={};manualFlag.store(controller->config().manual);
    takeoverFlag.store(controller->active());cameraFlag.store(true);
    reportedPhase=controller->phase();reportedStage=0;lastStatus=-1;
    enabledFlag.store(controller->config().enabled);
    std::cout<<"[AutoParking] "<<version<<" start_mode="<<(manualMode()?"manual":"race")<<" enabled="<<enabled()<<" trigger_zebra_count="
             <<controller->config().zebraCount<<" reverse_right_only="<<controller->config().rightOnly<<std::endl;
    std::cout<<"[AutoParking] steering_fixed_limit=none steer_offset_deg="<<offset
             <<" servo_period_ns="<<Contral::SteeringPulse::periodNs
             <<" pulse_preflight=passed; not a mechanical range guarantee"<<std::endl;
    for(int i=0;i<6;++i) {
        const auto& s=controller->config().stages[i];
        std::cout<<"[AutoParking] stage="<<i+1<<" "<<names[i]<<" pwm="<<s.pwm<<" time="<<s.seconds
                 <<" steer="<<s.steer<<" hold="<<s.hold<<" correction="<<s.correction<<'/'<<s.correctionSeconds<<std::endl;
    }
}
bool enabled() {return enabledFlag.load();}
bool manualMode() {return manualFlag.load();}
bool awaitingManualStart() {
    std::lock_guard<std::mutex> lock(mutex);
    return controller && controller->phase()==Phase::AwaitManualStart;
}
bool authorizeManualStart() {
    std::lock_guard<std::mutex> lock(mutex);
    if(!controller || controller->phase()!=Phase::AwaitManualStart) return false;
    try {
        openLogs();
        const double now=nowSeconds();
        latestLane={}; // Only a fresh post-authorization image can start the motors.
        if(!controller->authorizeManualStart(now)) return false;
        event("MANUAL_START_AUTHORIZED",now,"skip_zebra_stop_and_reverse");
        std::cout<<"[AutoParking] 已授权；等待有效巡线图像，随后自动前进计时。日志="<<directory<<std::endl;
        return true;
    } catch(const std::exception& e) {
        controller->fault(e.what());cameraFlag.store(false);
        std::cerr<<"[AutoParking] 手动入口未启动: "<<e.what()<<std::endl;return false;
    }
}
bool takeoverActive() {return takeoverFlag.load();}
bool cameraNeeded() {return cameraFlag.load();}
void zebraDetected(int count) {
    std::lock_guard<std::mutex> lock(mutex);
    if(!controller) return;
    const double now=nowSeconds();
    if(!controller->onZebra(count,now)) return;
    takeoverFlag.store(true);
    try {
        openLogs();event("SECOND_ZEBRA_CONFIRMED",now,"waiting_for_stop_and_reverse_to_finish");
        std::cout<<"[AutoParking] 第二圈斑马线确认；完成现有停车动作后自动前移。日志="<<directory<<std::endl;
    } catch(const std::exception& e) {controller->fault(e.what());cameraFlag.store(false);}
}
void publishLane(bool valid,double error) {
    std::lock_guard<std::mutex> lock(mutex);
    // Retain only a recent valid sample. Invalid frames cannot renew its watchdog timestamp.
    if(valid && std::isfinite(error)) latestLane={true,nowSeconds(),error};
}
bool writeCruiseSteering(Contral::Steer* steer,double error) {
    std::lock_guard<std::mutex> lock(mutex);
    if(takeoverFlag.load()) return true;
    return steerWrite(steer,error);
}
bool motorStep(Contral::Motor* motor,Contral::Steer* steer,double leftRps,double rightRps,bool held) {
    std::lock_guard<std::mutex> lock(mutex);
    if(!controller || !controller->active()) return false;
    const bool awaitingPermission=controller->phase()==Phase::AwaitManualStart;
    try {
        const double before=nowSeconds();const int previousStage=controller->stage();
        const double previousElapsed=controller->elapsed(before);
        if(Other::gMotorStop) controller->fault("EXTERNAL_STOP");
        Output out=controller->tick(before,held,latestLane);
        cameraFlag.store(controller->cameraNeeded());
        if(controller->finished()) {
            zero(motor);
            if(!awaitingPermission && controller->phase()!=reportedPhase) steerWrite(steer,0);
        } else {
            // Stop both wheels before any between-stage steering change.
            // During motion, steer first and issue the pair only if steering succeeded.
            bool ok=true;
            if(out.left==0 && out.right==0) {
                ok=stopMotorOutputs(motor);
                // The manual standby gate owns zero PWM but writes no servo until Enter.
                if(controller->phase()!=Phase::AwaitManualStart) {const bool s=steerWrite(steer,out.steer);ok=ok&&s;}
            } else {
                ok=steerWrite(steer,out.steer) && motor!=nullptr;
                if(ok) {const bool l=motor->setLeftPwm(out.left);const bool r=motor->setRightPwm(out.right);ok=l&&r;}
            }
            const double ackAt=nowSeconds();
            const bool firstOutput=!controller->motorStarted() && (out.left!=0 || out.right!=0);
            controller->acknowledge(out,ok,ackAt);
            if(!ok) {zero(motor);if(!awaitingPermission) steerWrite(steer,0);out={};}
            if(firstOutput && ok) event("MOTOR_OUTPUT_STARTED",ackAt,"stage="+std::to_string(controller->stage()));
            Other::gMotorAimSpeed=0; // Open-loop PWM is not a target rps.
        }
        const double now=nowSeconds();
        if(previousStage>0 && previousStage!=controller->stage())
            row(previousStage,"STAGE_END",now,{},previousElapsed,leftRps,rightRps);
        if(reportedPhase!=controller->phase() || reportedStage!=controller->stage()) {
            reportedPhase=controller->phase();reportedStage=controller->stage();
            event("STATE",now,std::string(phaseName(reportedPhase))+" stage="+std::to_string(reportedStage));
            std::cout<<"[AutoParking] "<<phaseName(reportedPhase)<<" stage="<<reportedStage
                     <<" reason="<<controller->reason()<<std::endl;
            flush();
        }
        row(controller->stage(),"SAMPLE",now,out,controller->elapsed(now),leftRps,rightRps);
        if(!controller->finished() && controller->phase()!=Phase::AwaitManualStart && now-lastStatus>=1) {
            lastStatus=now;
            std::cout<<"[AutoParking] 当前阶段="<<controller->stage()<<" "<<phaseName(controller->phase())
                     <<" 电机计时="<<controller->elapsed(now)<<"s 左/右PWM="<<out.left<<'/'<<out.right
                     <<" 编码器="<<leftRps<<'/'<<rightRps<<"rps"<<std::endl;
        }
        if(controller->finished()) {
            flush();
            if(events.is_open()) events.flush();
        }
    } catch(const std::exception& e) {
        controller->fault(e.what());cameraFlag.store(false);zero(motor);
        if(reportedPhase!=Phase::Fault) std::cerr<<"[AutoParking] 故障停车: "<<e.what()<<std::endl;
        reportedPhase=Phase::Fault;
    }
    return true;
}
void cameraReleased() {
    std::lock_guard<std::mutex> lock(mutex);
    try {event("CAMERA_RELEASED",nowSeconds(),"reverse_stages_have_no_video");}
    catch(const std::exception& e) {if(controller) controller->fault(e.what());}
    std::cout<<"[AutoParking] 阶段1结束，录像已停止、摄像头已释放。"<<std::endl;
}
void shutdown() {
    std::lock_guard<std::mutex> lock(mutex);
    if(!controller || !controller->active() || directory.empty()) return;
    try {event("PROGRAM_EXIT",nowSeconds(),controller->reason());flush();}
    catch(const std::exception& e) {std::cerr<<"[AutoParking] 保存错误: "<<e.what()<<std::endl;}
    for(auto& f:logs) f.close();
    events.close();
    std::cout<<"[AutoParking] 六个阶段CSV已保存: "<<directory<<std::endl;
}
}
