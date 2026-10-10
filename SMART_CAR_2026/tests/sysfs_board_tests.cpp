#include "sysfs_board.hpp"
#include "servo_output.hpp"
#include "../tools/rehearsal_motor.hpp"
#include "../tools/rehearsal_rps.hpp"
#include <iostream>
#include <mutex>
using namespace car2026;
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* message) {++checks;if(!ok) throw std::runtime_error(message);}
template<class F> void rejects(F f,const char* message) {bool rejected=false;try {f();}catch(...) {rejected=true;}check(rejected,message);}
class MemoryNodes final:public SysfsNodes {
    std::mutex mutex_;
    std::map<std::string,std::string> files_;
    std::vector<std::pair<std::string,std::string>> writes_;
public:
    std::string failedPath,failedValue;
    bool corruptPeriod=false,rejectUnsetDisable=true;
    bool exists(const std::string& p) override {std::lock_guard<std::mutex> held(mutex_);return files_.count(p)!=0;}
    std::string read(const std::string& p) override {
        std::lock_guard<std::mutex> held(mutex_);
        if(corruptPeriod && p.find("pwmchip8/pwm2/period")!=std::string::npos) return "49999";
        return files_.at(p);
    }
    void write(const std::string& p,const std::string& v) override {
        std::lock_guard<std::mutex> held(mutex_);writes_.emplace_back(p,v);
        if(rejectUnsetDisable && v=="0" && p.find("/enable")!=std::string::npos && files_.at(p.substr(0,p.size()-7)+"/period")=="0")
            throw std::runtime_error("Invalid argument: unset PWM period");
        if(p==failedPath && v==failedValue) throw std::runtime_error("Injected output failure");
        if(p=="/sys/class/gpio/export") {
            const auto dir="/sys/class/gpio/gpio"+v;
            files_[dir+"/value"]="0";files_[dir+"/direction"]="in";
        } else if(p.find("/export")!=std::string::npos) {
            const auto dir=p.substr(0,p.size()-7)+"/pwm"+v;
            for(const auto& name:{"period","duty_cycle","enable"}) files_[dir+"/"+name]="0";
            files_[dir+"/polarity"]="normal";
        } else if(p.find("/direction")!=std::string::npos) {
            files_.at(p)=v=="low" ? "out" : v;
            if(v=="low") files_[p.substr(0,p.size()-10)+"/value"]="0";
        } else files_.at(p)=v;
    }
    std::vector<std::pair<std::string,std::string>> writes() {std::lock_guard<std::mutex> held(mutex_);return writes_;}
};
size_t findWrite(const std::vector<std::pair<std::string,std::string>>& writes,const std::string& p,const std::string& v) {
    for(size_t i=0;i<writes.size();++i) if(writes[i]==std::make_pair(p,v)) return i;
    return writes.size();
}
}
int main() {
 try {
    const auto hardware=HardwareConfig::load("deploy/config/new_car_hardware.ini");
    const auto params=Params::load("deploy/config/new_car_vehicle.ini");
    const auto tuning=ParkingTuning::load("deploy/config/parking_tuning.new_car.ini",true);tuning.validate(params.max_steer_deg);
    check(tuning.stages[0].motorLeft==1 && tuning.stages[0].speed.leftRps==9 && tuning.stages[0].motorSeconds==1,"initial stage enables cc reference speed for one second");
    check(hardware.motor_backend=="sysfs" && !hardware.factory_write_readback,"new-car decimal backend");
    check(hardware.encoder_left_sign==1 && hardware.encoder_right_sign==1,"native reference encoder signs");
    check(params.steer_offset_deg==-15 && !params.require_imu,"new-car steering offset without IMU");
    check(params.max_steer_deg==30,"delivered new-car profile opens command range to 30");
    check(sysfsNumber("50000\n")==50000,"sysfs whitespace readback accepted");
    for(const auto& invalid:{"-1","+1","50000ns","1.5","4294967296",""}) rejects([&]{sysfsNumber(invalid);},"malformed integer rejected");
    rejects([]{sysfsPwmDirectory("/sys/class/pwm/pwmchip8/pwm2junk/duty_cycle");},"PWM channel traversal rejected");
    check(newCarServoNs(-15,3040000)==1494667,"new-car mechanical neutral matches reference");
    check(newCarServoNs(0,3040000)==1520000,"unoffset pulse matches reference");
    check(newCarServoNs(15,3040000)>newCarServoNs(-15,3040000),"positive steering increases native pulse");
    rejects([]{newCarServoNs(10000,3040000);},"invalid servo pulse blocked");
    check(std::abs(newCarEncoderRps(10000,1)-9.765625)<1e-9,"100MHz period converts to native fractional RPS");
    check(std::abs(newCarEncoderRps(10000,0)+9.765625)<1e-9,"encoder direction controls speed sign");
    check(newCarEncoderRps(0,1)==0,"no measured period gives zero native reading");
    rejects([]{newCarEncoderRps(1,2);},"invalid encoder direction rejected");
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);
        check(nodes.writes().empty(),"sysfs construction does not authorize outputs");
        ServoOutput servo(io,hardware,params);
        check(nodes.writes().empty(),"servo metadata requires no GPIO/PWM outputs");
        check(servo.set(10)==1511556,"servo pulse remains uint32 and includes mechanical offset");
        const auto initialization=nodes.writes();const std::string servoDir="/sys/class/pwm/pwmchip2/pwm0";
        check(findWrite(initialization,servoDir+"/period","3040000")<findWrite(initialization,hardware.servo_pwm,"1520000"),"reference servo writes period before initial midpoint");
        check(findWrite(initialization,hardware.servo_pwm,"1520000")<findWrite(initialization,servoDir+"/enable","1"),"reference servo writes initial midpoint before enabling output");
        check(findWrite(initialization,servoDir+"/enable","0")==initialization.size(),"reference servo has no initial disable write");
        check(!nodes.exists(hardware.motor_enable_gpio),"servo-only writes do not initialize motors");
        check(servo.set(-30)==1444000,"negative 30 retains exact cc pulse formula and offset");
        check(servo.set(30)==1545333,"positive 30 retains exact cc pulse formula and offset");
        const auto beforeRejected=nodes.writes().size();
        for(double command:{-30.01,30.01}) rejects([&]{servo.set(command);},"expanded native servo still rejects commands beyond configured limit");
        check(nodes.writes().size()==beforeRejected,"out-of-range steering cannot write a PWM value");
        servo.close();check(sysfsNumber(nodes.read(hardware.servo_pwm))==1494667,"shutdown uses new-car calibrated zero");
        rejects([&]{io.writePwmDuty(hardware.servo_pwm,100);},"servo cannot use uint16 motor output API");
    }
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);RehearsalMotor motor(io,hardware,params);
        check(nodes.read(hardware.motor_enable_gpio)=="0","motor initialization leaves nSLEEP off");
        check(nodes.read(hardware.motor_left_pwm)=="0" && nodes.read(hardware.motor_right_pwm)=="0","motor initialization zeros both channels");
        check(nodes.read("/sys/class/pwm/pwmchip8/pwm2/period")=="50000","period is reference 20kHz");
        check(nodes.read("/sys/class/pwm/pwmchip8/pwm2/polarity")=="normal","reference PWM polarity left unchanged");
        const auto initialization=nodes.writes();
        const auto leftDir="/sys/class/pwm/pwmchip8/pwm2";
        check(findWrite(initialization,std::string(leftDir)+"/enable","0")==initialization.size(),"reference initialization has no pre-period PWM disable");
        check(findWrite(initialization,std::string(leftDir)+"/enable","1")<findWrite(initialization,std::string(leftDir)+"/period","50000"),"reference motor enables before period");
        check(findWrite(initialization,std::string(leftDir)+"/period","50000")<findWrite(initialization,hardware.motor_left_pwm,"0"),"reference motor sets period before zero duty");
        check(findWrite(initialization,hardware.motor_right_pwm,"0")<findWrite(initialization,"/sys/class/gpio/gpio12/direction","out"),"reference initializes both PWM channels before output GPIO directions");
        const auto priorWrites=nodes.writes().size();motor.start(3000,4000,.04,0,1,1);
        const auto writes=nodes.writes();
        check(nodes.read(hardware.motor_left_pwm)=="3000" && nodes.read(hardware.motor_right_pwm)=="4000","tuning values written exactly, no old scaling");
        check(nodes.read(hardware.motor_left_dir)=="0" && nodes.read(hardware.motor_right_dir)=="0","forward GPIO is zero on both wheels");
        const std::vector<std::pair<std::string,std::string>> startWrites(writes.begin()+priorWrites,writes.end());
        check(findWrite(startWrites,hardware.motor_enable_gpio,"1")>findWrite(startWrites,hardware.motor_right_pwm,"4000"),"motion release follows both configured PWM writes");
        std::this_thread::sleep_for(std::chrono::milliseconds(90));
        check(!motor.active(),"independent timer stops new-car motors");
        check(nodes.read(hardware.motor_left_pwm)=="0" && nodes.read(hardware.motor_right_pwm)=="0" && nodes.read(hardware.motor_enable_gpio)=="0","timer clears both duties and nSLEEP");
        bool detailed=false;for(const auto& event:motor.takeEvents()) if(event.event=="MOTOR_START") detailed=motorEventDetail(event).find("units=duty_ns")!=std::string::npos;
        check(detailed,"native duty units in motor event logs");
        motor.start(-20000,-15000,.2,1,1,2);
        check(nodes.read(hardware.motor_left_dir)=="1" && nodes.read(hardware.motor_right_dir)=="1","reverse GPIO is one on both wheels");
        check(nodes.read(hardware.motor_left_pwm)=="20000","no artificial 2000 or 12000 cap");
        motor.stop("MOTOR_STOP_PAUSE");check(nodes.read(hardware.motor_enable_gpio)=="0","manual pause deasserts enable");
        rejects([&]{motor.start(50001,3000,.2,0,1,3);},"out-of-period duty rejected before starting either wheel");
        check(nodes.read(hardware.motor_left_pwm)=="0" && nodes.read(hardware.motor_right_pwm)=="0","rejected tuning leaves both motors at zero");
        motor.start(50000,0,.2,0,1,4);check(nodes.read(hardware.motor_left_pwm)=="50000" && nodes.read(hardware.motor_right_pwm)=="0","full physical range and stationary right wheel supported");
        motor.close();
    }
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);RehearsalMotor motor(io,hardware,params);
        nodes.write("/sys/class/pwm/pwmchip8/pwm2/enable","0");
        nodes.write("/sys/class/pwm/pwmchip8/pwm1/enable","0");
        motor.start(3000,3000,.2,0,1,1);
        check(nodes.read("/sys/class/pwm/pwmchip8/pwm2/enable")=="1" && nodes.read("/sys/class/pwm/pwmchip8/pwm1/enable")=="1","authorized start repairs disabled PWM channels after configured period");
        const auto events=motor.takeEvents();
        check(events.front().pwmEnabled[0]==1 && events.front().pwmEnabled[1]==1 && events.front().driverEnabled==1,"start diagnostics include both enables and driver nSLEEP");
        motor.stop("MOTOR_STOP_QUIT");
    }
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);RehearsalMotor motor(io,hardware,params);
        nodes.write("/sys/class/pwm/pwmchip8/pwm1/enable","0");
        nodes.failedPath="/sys/class/pwm/pwmchip8/pwm1/enable";nodes.failedValue="1";
        rejects([&]{motor.start(3000,3000,.2,0,1,1);},"PWM enable failure rejects motor start");
        check(nodes.read(hardware.motor_left_pwm)=="0" && nodes.read(hardware.motor_right_pwm)=="0" && nodes.read(hardware.motor_enable_gpio)=="0","enable failure zeros both PWM channels before returning error");
    }
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);RehearsalMotor motor(io,hardware,params);
        nodes.failedPath=hardware.motor_right_pwm;nodes.failedValue="3000";
        rejects([&]{motor.start(3000,3000,.2,0,1,1);},"right wheel failure aborts motor start");
        check(nodes.read(hardware.motor_left_pwm)=="0" && nodes.read(hardware.motor_right_pwm)=="0" && nodes.read(hardware.motor_enable_gpio)=="0","partial start failure clears both wheels and total enable");
    }
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);RehearsalMotor motor(io,hardware,params);
        motor.start(3000,3000,.2,0,1,1);
        nodes.failedPath=hardware.motor_left_pwm;nodes.failedValue="0";
        rejects([&]{motor.stop("MOTOR_STOP_QUIT");},"failed left zero is reported");
        check(nodes.read(hardware.motor_right_pwm)=="0" && nodes.read(hardware.motor_enable_gpio)=="0","right zero and total sleep still attempted after left stop failure");
    }
    {
        MemoryNodes nodes;nodes.corruptPeriod=true;SysfsBoardIo io(nodes,hardware);
        rejects([&]{RehearsalMotor motor(io,hardware,params);},"period readback mismatch aborts initialization");
        check(nodes.read(hardware.motor_enable_gpio)=="0","readback initialization failure leaves enable disabled");
    }
    {
        MemoryNodes nodes;SysfsBoardIo io(nodes,hardware);
        rejects([&]{FactoryBoard board(io,hardware,params);},"generic competition controller cannot use native-RPS sysfs profile");
        check(nodes.writes().empty(),"wrong controller rejected before hardware outputs");
    }
    {
        NativeRpsRecorder recorder;
        recorder.add(0,0,1,1,{true,1.25,1000000000},{true,-2.5,1000000000});
        recorder.add(0,0,1,1,{true,1.75,2000000000},{true,-3.5,2000000000});
        std::ostringstream output;recorder.writeSummary(output);
        check(output.str().find(",rps,2,1,1.5,1.75,2,1,-3,3.5")!=std::string::npos,"speed summary preserves signed fractional RPS and weighted means");
        recorder.gap();recorder.add(0,0,1,1,{true,20,3000000000},{true,30,3000000000});
        output.str("");recorder.writeSummary(output);
        check(output.str().find(",rps,3,1,1.5,20,3,1,-3,30")!=std::string::npos,"paused recording gap not integrated into speed average");
    }
    std::cout<<checks<<" new-car sysfs checks passed (mock hardware only)\n";
    return 0;
 }catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
