#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include "NewTrack/DiagnosticView.hpp"
#include "Parking/StandaloneTestRuntime.hpp"
#include "Parking/SteeringDiagnostics.hpp"
#include "Experiment/ProcessedRecording.hpp"
#include "Experiment/VideoRecorder.hpp"
#include <filesystem>
#include <poll.h>

using namespace Other;
using namespace Contral;
namespace fs = std::filesystem;
namespace {
constexpr const char *version = "VV_STANDALONE_20261022.1";
void stopSignal(int) { gIsClose.store(true, std::memory_order_relaxed); }
struct Options { ParkingTest::Mode mode=ParkingTest::Mode::Full; bool check=false, video=true, help=false; };
Options parse(int argc, char **argv) {
    Options o;
    for (int i=1;i<argc;++i) {
        const std::string arg=argv[i];
        if (arg=="--mode" && i+1<argc) o.mode=ParkingTest::parseMode(argv[++i]);
        else if (arg=="--check-config") o.check=true;
        else if (arg=="--no-video") o.video=false;
        else if (arg=="--help" || arg=="-h") o.help=true;
        else throw std::runtime_error("unknown or incomplete option: "+arg);
    }
    return o;
}
// Canonical terminal input: Enter starts; Q+Enter or Ctrl+C stops. No background reader.
bool command(std::string &text) {
    pollfd input{STDIN_FILENO,POLLIN,0};
    const int ready=poll(&input,1,50);
    if (ready<0) { if(errno==EINTR) return false; throw std::runtime_error("stdin poll failed"); }
    if (ready==0) return false;
    if (input.revents&POLLIN) {
        if (!std::getline(std::cin,text)) throw std::runtime_error("stdin closed");
        return true;
    }
    if (input.revents&(POLLHUP|POLLERR|POLLNVAL)) throw std::runtime_error("stdin disconnected");
    return false;
}
fs::path createRun() {
    const fs::path parent="runs";
    fs::create_directories(parent);
    const auto stamp=std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
    const fs::path run=parent/(std::to_string(stamp)+"_"+std::to_string(getpid()));
    if (!fs::create_directory(run)) throw std::runtime_error("session directory already exists");
    fs::copy_file("image_params.txt",run/"image_params.start.txt");
    return run;
}
void requireParameterFile() {
    std::ifstream input("image_params.txt");
    if (!input) throw std::runtime_error("image_params.txt missing beside executable; refusing to generate replacement defaults");
    std::string line;
    int versions=0;
    while (std::getline(input,line)) {
        line=line.substr(0,line.find('#'));
        const auto equals=line.find('=');
        if (equals==std::string::npos) continue;
        auto key=line.substr(0,equals);
        key.erase(std::remove_if(key.begin(),key.end(),[](unsigned char c){return std::isspace(c);}),key.end());
        if (key=="config_version") {
            auto value=line.substr(equals+1);
            value.erase(std::remove_if(value.begin(),value.end(),[](unsigned char c){return std::isspace(c);}),value.end());
            if (value!="2026083101") throw std::runtime_error("wrong config_version; refusing automatic template replacement");
            ++versions;
        }
    }
    if (versions!=1) throw std::runtime_error("requires exactly one config_version=2026083101; refusing automatic template replacement");
}
void requireHardware() {
    for (const char *path : {"/dev/mem",CAMERA_PATH_OR_INDEX,"/sys/class/pwm/pwmchip2","/sys/class/pwm/pwmchip8"})
        if (!fs::exists(path)) throw std::runtime_error(std::string("hardware path missing: ")+path);
}
void requireMotorReady() {
    for (int channel : {MOTOR_LEFT_PWM_INDEX,MOTOR_RIGHT_PWM_INDEX}) {
        const auto base="/sys/class/pwm/pwmchip"+std::to_string(MOTOR_LEFT_PWM_CHIP)+"/pwm"+std::to_string(channel);
        if (Parking::readPwmSysfsValue(base+"/period")!=MOTOR_PWM_PERIOD ||
            Parking::readPwmSysfsValue(base+"/enable")!=1 || Parking::readPwmSysfsValue(base+"/duty_cycle")!=0)
            throw std::runtime_error("motor PWM is not enabled with the original zero-duty setup: "+base);
    }
}
void printConfig(const Parking::Config &c, const Options &o) {
    std::cout << version << " mode=" << ParkingTest::modeName(o.mode) << '\n'
        << "parameters=" << fs::absolute("image_params.txt").string() << '\n'
        << "speed_rps=" << RuntimeConfig::getMotorAimSpeed() << " center_pratio=" << RuntimeConfig::getCenterPratio()
        << " error_gain=" << RuntimeConfig::getErrorGain() << '\n'
        << "parking_center=" << c.center << " servo_offset=" << c.servoOffset
        << " approach_ms=" << c.approachMs << " settle_ms=" << c.settleMs
        << " left_enabled=" << c.leftEnabled << " right_enabled=" << c.rightEnabled << '\n';
    for (std::size_t i=0;i<c.stages.size();++i) {
        const auto &s=c.stages[i];
        std::cout << "stage=" << (i==0?"S2":i==1?"S4":"S5") << " enabled=" << s.enabled
            << " offset=" << s.steerOffset << " reverse_pwm=" << s.pwm
            << " total_ms=" << s.totalMs << " hold_ms=" << s.holdMs << '\n';
    }
}
}

int main(int argc,char **argv) {
    std::thread imageThread,motorThread;
    bool initialized=false, started=false;
    fs::path run;
    std::string outcome="ERROR";
    int result=1;
    std::atomic<bool> workerFailed{false};
    try {
        const auto options=parse(argc,argv);
        if (options.help) {
            std::cout << version << "\nUsage: ./vv_parking_test [--mode full|approach-only|reverse-only|s2|s4|s5] [--check-config] [--no-video]\n"
                << "full: Enter -> follow to zebra -> original reverse/hold -> follow 4s -> S2/S4/S5 -> stop\n"
                << "approach-only: same zebra sequence and forward approach, then stop before S2\n"
                << "reverse-only/s2/s4/s5: start from a manually positioned reverse origin\n";
            return 0;
        }
        fs::current_path(Experiment::executableDirectory());
        requireParameterFile();
        RuntimeConfig::reloadIfNeeded();
        if (!NewTrack::enabled() || RuntimeConfig::getExperimentEnable()!=0)
            throw std::runtime_error("requires new_track_enable=1 and experiment_enable=0");
        auto request=NewTrack::standaloneParkingRequest();
        request.config=ParkingTest::selectStages(request.config,options.mode);
        const auto error=Parking::validate(request.config);
        if (!error.empty()) throw std::runtime_error(error);
        printConfig(request.config,options);
        if (options.check) {
            std::cout << "CONFIG_OK; hardware was not opened; no motion\n";
            return 0;
        }
        signal(SIGINT,stopSignal); signal(SIGTERM,stopSignal);
        requireHardware();
        run=createRun();
        initialized=true;
        globalInit(); // Same PWM/GPIO/encoder/PID/camera initialization as the supplied VV.
        requireMotorReady();
        std::cout << "初始化完成。摆好车辆后按 Enter 开始；Q+Enter 或 Ctrl+C 停止。\n";
        while (!gIsClose.load()) {
            std::string input;
            if (!command(input)) continue;
            if (input=="q" || input=="Q") { gIsClose.store(true); break; }
            if (input.empty()) { started=true; break; }
        }
        if (started) {
            ParkingTest::runtime().configure(options.mode,run.string());
            NewTrack::beginStandaloneParkingTest();
            if (options.video) {
                Experiment::VideoOptions recording;
                recording.fps=10; recording.queueCapacity=4; recording.fileName="processed.avi";
                recording.metadataHeader=NewTrack::diagnosticCsvHeader();
                std::string failure;
                if (!Experiment::gProcessedVideoRecorder.start(run.string(),NewTrack::DiagnosticView::canvasSize(),recording,failure))
                    throw std::runtime_error("recording startup failed: "+failure);
                Experiment::gProcessedRecordingEnabled.store(true);
            }
            if (!ParkingTest::seeksZebra(options.mode)) {
                request=NewTrack::standaloneParkingRequest();
                if (!Parking::mailbox().submit(1,request)) throw std::runtime_error("cannot arm selected reverse test");
            }
            auto worker=[&](void (*entry)()) {
                try { entry(); }
                catch (const std::exception &e) {
                    ParkingTest::runtime().event(std::string("WORKER_ERROR=")+e.what());
                    workerFailed.store(true); gIsClose.store(true);
                }
                catch (...) { workerFailed.store(true); gIsClose.store(true); }
            };
            imageThread=std::thread(worker,threadRunImageHandle);
            motorThread=std::thread(worker,threadMotorHandle);
            std::cout << "TEST_STARTED mode=" << ParkingTest::modeName(options.mode) << " logs=" << fs::absolute(run).string() << '\n';
            while (!gIsClose.load() && !ParkingTest::runtime().finished()) {
                std::string input;
                if (command(input) && (input=="q" || input=="Q")) gIsClose.store(true);
            }
            const auto phase=ParkingTest::runtime().phase();
            if (workerFailed.load()) { outcome="WORKER_ERROR"; result=1; }
            else if (phase==Parking::Phase::Fault) { outcome="CONTROLLER_FAULT"; result=1; }
            else if (phase==Parking::Phase::Done) { outcome="SEQUENCE_DONE_NOT_PHYSICAL_SUCCESS"; result=0; }
            else { outcome="OPERATOR_STOP"; result=130; }
        } else { outcome="CANCELLED_BEFORE_START"; result=130; }
    } catch (const std::exception &e) {
        std::cerr << "[StandaloneTest] " << e.what() << '\n';
        ParkingTest::runtime().event(std::string("ERROR=")+e.what());
    }
    gIsClose.store(true);
    // Stop control before draining/decoding video. Threads are joined even if creation failed.
    if (motorThread.joinable()) motorThread.join();
    if (imageThread.joinable()) imageThread.join();
    if (workerFailed.load()) { outcome="WORKER_ERROR"; result=1; }
    if (gMotor) { auto *motor=static_cast<Motor *>(gMotor); motor->setLeftPwm(0); motor->setRightPwm(0); }
    Experiment::gProcessedRecordingEnabled.store(false);
    Experiment::gProcessedVideoRecorder.stop();
    const auto video=Experiment::gProcessedVideoRecorder.stats();
    if (!video.error.empty() && result==0) { outcome="SEQUENCE_DONE_RECORDING_ERROR"; result=1; }
    if (initialized) globalDestroy();
    if (!run.empty()) {
        std::ofstream status(run/"result.txt");
        status << "version=" << version << "\noutcome=" << outcome << "\nexit_code=" << result
            << "\nvideo_written=" << video.written << "\nvideo_verified=" << video.verified
            << "\nvideo_error=" << video.error << '\n';
        std::cout << "TEST_END " << outcome << " logs=" << fs::absolute(run).string() << '\n';
    }
    return result;
}
