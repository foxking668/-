#include "hardware_linux.hpp"
#include "encoder_bench_stats.hpp"
#include "bench_options.hpp"
#include <chrono>
#include <csignal>
#include <iomanip>
#include <iostream>
#include <thread>

using namespace car2026;
namespace {
volatile std::sig_atomic_t interrupted=0;
void signalHandler(int) {interrupted=1;}
using Clock=std::chrono::steady_clock;
double secondsSince(Clock::time_point start) {return std::chrono::duration<double>(Clock::now()-start).count();}
void help() {
    std::cout<<"hardware_bench_test v2 (verified factory output writes) [--hardware-config file] MODE\n"
             <<"  --interfaces  read-only interface inventory (default; encoder reads consume counts)\n"
             <<"  --imu         read-only IIO check, no encoder reads or output writes\n"
             <<"  --motor-zero  write ONLY zero to both motor PWMs; no direction/servo/encoder operations\n"
             <<"  --encoders [--seconds 15] [--interval-ms 100] [--wheel left|right --turns N]\n"
             <<"    raw int32 CSV and totals under CONFIGURED encoder_mode; manual forward turns only\n"
             <<"  --motor-pulse --wheel left|right|both --raw-duty SIGNED --duration-ms 300 --wheels-raised\n"
             <<"    POWERED TEST: initializes FactoryBoard (also centers servo, clears buzzer);\n"
             <<"    raw magnitude <=20% of each selected duty_max AND configured pwm_limit; duration 50..500ms\n"
             <<"    secure chassis, both drive wheels off ground, linkage unobstructed, emergency power-off ready\n"
             <<"  [--config config/competition.ini] is used only for motor-pulse safety/calibration settings\n"
             <<"No config is rewritten. Keep motion_calibrated=0 until commissioning is complete.\n";
}
void zeroMotors(DeviceIo& io,const HardwareConfig& config) {
    std::cout<<"ZERO-ONLY motor output check; no nonzero PWM, direction, servo or encoder operations.\n"
             <<"factory_write_readback="<<config.factory_write_readback<<'\n'<<std::flush;
    bool success=true;
    for(const auto& path : {config.motor_left_pwm,config.motor_right_pwm}) {
        try {
            io.writePwmDuty(path,0);
            // Strict-mode typed write already required exactly two bytes; the
            // zero-only preflight still verifies readback in that mode as well.
            if(!config.factory_write_readback) verifyFactoryPwmWrite(io,path,0,sizeof(uint16_t));
            std::cout<<path<<" zero write/check passed (software only; not physical stop proof).\n";
        }catch(const std::exception& e) {
            success=false;std::cerr<<"Zero check '"<<path<<"': "<<e.what()<<'\n';
        }
    }
    if(!success) throw std::runtime_error("Zero-only check failed; do not proceed to powered tests");
}
void collectEncoders(DeviceIo& io,const HardwareConfig& config,double duration,int interval,const std::string& wheel,double turns) {
    std::cout<<"# Read-only manual wheel test; NO motor/servo/GPIO writes. Stop other controllers.\n"
             <<"# encoder_mode="<<config.encoder_mode<<"; totals/PPR candidates are valid ONLY if this mode is verified.\n"
             <<"# Rotate only the selected wheel physically FORWARD through exactly the declared full turns, then leave stationary.\n"
             <<"# Initial untimed counts discarded. measurement_seconds="<<duration<<" wheel="<<wheel<<" declared_turns="<<turns<<'\n';
    const auto leftBaseline=io.readEncoderCount(config.encoder_left),rightBaseline=io.readEncoderCount(config.encoder_right);
    EncoderBenchStats left(config.encoder_mode,leftBaseline),right(config.encoder_mode,rightBaseline);
    std::cout<<"elapsed_s,left_raw_i32,right_raw_i32\n"<<std::setprecision(17)<<std::flush;
    const auto start=Clock::now();auto next=start;
    while(!interrupted && secondsSince(start)<duration) {
        const auto l=io.readEncoderCount(config.encoder_left),r=io.readEncoderCount(config.encoder_right);
        const double elapsed=secondsSince(start);
        std::cout<<elapsed<<','<<l<<','<<r<<'\n'<<std::flush;
        left.add(l);right.add(r);
        next+=std::chrono::milliseconds(interval);
        if(next<Clock::now()) next=Clock::now(); // No catch-up burst of destructive delta reads.
        std::this_thread::sleep_until(next);
    }
    std::cout<<"# CONFIGURED_MODE_TOTAL left_net="<<left.net()<<" right_net="<<right.net()
             <<" left_abs_steps="<<left.absolute()<<" right_abs_steps="<<right.absolute()<<" samples="<<left.samples()<<'\n';
    if(interrupted) {std::cout<<"# INTERRUPTED: do not use this run for PPR/sign calibration.\n";return;}
    if(turns>0) {
        const auto net=wheel=="left"?left.net():right.net();
        const auto other=wheel=="left"?right.net():left.net();
        if(net==0) throw std::runtime_error("No net selected-wheel counts: no PPR/sign candidate");
        std::cout<<"# CANDIDATE_ONLY wheel="<<wheel<<" counts_per_declared_wheel_rev="<<std::abs(double(net))/turns
                 <<" combined_encoder_sign_for_declared_forward="<<(net>0?1:-1)<<" other_wheel_net="<<other<<'\n'
                 <<"# Verify exact turns, mode, wheel identity and repeatability; sign is the PRODUCT of hardware.ini and competition.ini signs.\n";
    }
}
void pulseMotor(DeviceIo& io,const HardwareConfig& config,const Params& params,const std::string& wheel,int rawDuty,int duration) {
    std::cout<<"POWERED BENCH TEST ONLY: both drive wheels raised, linkage clear; FactoryBoard also centers servo.\n"
             <<"factory_write_readback="<<config.factory_write_readback
             <<"; status-zero writes require matching typed readback when enabled; no partial-write retries.\n"<<std::flush;
    FactoryBoard board(io,config,params); // Zero both outputs before any metadata/sensor setup.
    const bool useLeft=wheel=="left" || wheel=="both",useRight=wheel=="right" || wheel=="both";
    auto command=[&](const PwmInfo& info,bool selected) {
        if(!selected) return 0.0;
        if(std::abs(rawDuty)>double(info.duty_max)*.2) throw std::runtime_error("Requested raw PWM exceeds 20% bench cap");
        const double value=rawDuty*config.motor_command_range/info.duty_max;
        if(std::abs(value)>params.pwm_limit) throw std::runtime_error("Requested raw PWM exceeds configured controller pwm_limit");
        return value;
    };
    const double leftCommand=command(board.leftPwmInfo(),useLeft),rightCommand=command(board.rightPwmInfo(),useRight);
    const auto leftBefore=io.readEncoderCount(config.encoder_left),rightBefore=io.readEncoderCount(config.encoder_right);
    std::cout<<"wheel="<<wheel<<" raw_duty="<<rawDuty<<" duration_ms="<<duration
             <<" left_command="<<leftCommand<<" right_command="<<rightCommand<<'\n'<<std::flush;
    if(interrupted) throw std::runtime_error("Interrupted before pulse; no nonzero motor command sent");
    const auto start=Clock::now();
    board.setMotorCommands(leftCommand,rightCommand);
    // No encoder reads or console output while powered. Optional output readback
    // happens inside setMotorCommands/stop and, like writes, may block. The pulse
    // includes initial write/readback time; hardware cutoff remains mandatory.
    while(!interrupted && secondsSince(start)*1000<duration) std::this_thread::sleep_for(std::chrono::milliseconds(5));
    if(!board.stop()) throw std::runtime_error("Bench stop write failed: use emergency POWER OFF; physical stop NOT confirmed");
    const double stopElapsed=secondsSince(start);
    const auto leftAfter=io.readEncoderCount(config.encoder_left),rightAfter=io.readEncoderCount(config.encoder_right);
    std::cout<<"Zero-output writes/checks succeeded after "<<stopElapsed<<" s; this is NOT proof of mechanical standstill or pin waveform.\n"
             <<"encoder_mode="<<config.encoder_mode<<" left_raw="<<leftAfter<<" right_raw="<<rightAfter
             <<" left_count_step="<<encoderCountDelta(leftAfter,leftBefore,config.encoder_mode)
             <<" right_count_step="<<encoderCountDelta(rightAfter,rightBefore,config.encoder_mode)<<'\n'
             <<"Record physical direction/start/no-start for each wheel. No PID or calibration changes were made.\n";
}
}
int main(int argc,char** argv) {
    try {
        const auto options=BenchOptions::parse(std::vector<std::string>(argv+1,argv+argc));
        if(options.help) {help();return 0;}
        const auto config=HardwareConfig::load(options.hardwareFile);
        HardwareLock lock;LinuxDeviceIo io(config);
        std::signal(SIGINT,signalHandler);std::signal(SIGTERM,signalHandler);
        if(options.mode=="interfaces") inspectHardware(io,config,std::cout);
        else if(options.mode=="imu") inspectIioImu(io,config,std::cout);
        else if(options.mode=="encoders") collectEncoders(io,config,options.duration,int(options.interval),options.wheel,options.turns);
        else if(options.mode=="motor-zero") zeroMotors(io,config);
        else pulseMotor(io,config,Params::load(options.paramsFile),options.wheel,int(options.rawDuty),int(options.pulseDuration));
        return interrupted?2:0;
    }catch(const std::exception& e) {std::cerr<<"hardware_bench_test: "<<e.what()<<'\n';return 1;}
}
