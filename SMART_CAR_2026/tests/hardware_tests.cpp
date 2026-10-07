#include "hardware.hpp"
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <limits>
using namespace car2026;
namespace {
int checks=0;
void check(bool v,const char* message) {++checks;if(!v) throw std::runtime_error(message);}
void near(double a,double b,const char* message) {check(std::abs(a-b)<1e-6,message);}
void rejects(const std::function<void()>& fn,const char* message) {
    bool threw=false;try {fn();}catch(const std::exception&) {threw=true;}check(threw,message);
}
struct Write {std::string path;std::vector<uint8_t> bytes;};
class FakeIo : public DeviceIo {
public:
    std::map<std::string,std::vector<uint8_t>> binary;
    std::map<std::string,std::string> text;
    std::vector<Write> writes;
    std::string failWrite;
    bool failOnce=false,clearCounts=true;
    template<class T> void put(const std::string& path,T value) {
        const auto* p=reinterpret_cast<const uint8_t*>(&value);binary[path]={p,p+sizeof(value)};
    }
    void readBinary(const std::string& path,void* target,size_t bytes) override {
        const auto& source=binary.at(path);
        if(source.size()!=bytes) throw std::runtime_error("Fake short read");
        std::memcpy(target,source.data(),bytes);
        if(clearCounts && bytes==sizeof(int16_t)) put(path,int16_t(0));
    }
    void writeBinary(const std::string& path,const void* source,size_t bytes) override {
        if(path==failWrite) {if(failOnce) failWrite.clear();throw std::runtime_error("Injected write failure");}
        const auto* p=static_cast<const uint8_t*>(source);writes.push_back({path,{p,p+bytes}});
    }
    std::string readText(const std::string& path) override {return text.at(path);}
    uint16_t lastDuty(const std::string& path) const {
        for(auto i=writes.rbegin();i!=writes.rend();++i) if(i->path==path) {
            check(i->bytes.size()==2,"PWM write is exactly uint16");uint16_t v=0;std::memcpy(&v,i->bytes.data(),2);return v;
        }
        throw std::runtime_error("No PWM output");
    }
    void populate(const HardwareConfig& c) {
        const PwmInfo motor{20000,0,10000,0,50000,100000000},servo{300,0,10000,0,3333333,100000000};
        put(c.motor_left_pwm,motor);put(c.motor_right_pwm,motor);put(c.servo_pwm,servo);
        put(c.encoder_left,int16_t(0));put(c.encoder_right,int16_t(0));
        for(int i=0;i<4;++i) put(c.key_prefix+std::to_string(i),uint8_t(1));
        for(int i=0;i<2;++i) put(c.switch_prefix+std::to_string(i),uint8_t('1'));
        text[c.imu_iio_device+"/name"]="IMU660RA\n";
        text[c.imu_iio_device+"/in_anglvel_scale"]=std::to_string(pi/180);
        text[c.imu_iio_device+"/in_anglvel_z_raw"]="0\n";
    }
};
}
int main(int argc,char** argv) {
    try {
        HardwareConfig c=HardwareConfig::load(argc>1?argv[1]:"config/hardware.ini");Params p;c.voice_enabled=0;
        const PwmInfo info{300,0,10000,0,3333333,100000000};
        check(motorDuty(12000,50000,info)==2400,"PID command retains physical 24 percent limit");
        check(motorDuty(-12000,50000,info)==2400,"Reverse uses positive magnitude");
        check(motorDuty(1e6,50000,info)==10000,"PWM saturation respects driver's range");
        check(servoDuty(0,c,info)==4470,"Factory center pulse");
        check(servoDuty(-22,c,info)==5170,"Factory left pulse");
        check(servoDuty(22,c,info)==3370,"Factory right pulse");
        PwmInfo slow=info;slow.freq=50;check(servoDuty(0,c,slow)==745,"Same pulse duration at 50 Hz");
        rejects([&]{motorDuty(std::numeric_limits<double>::quiet_NaN(),50000,info);},"NaN command rejected");
        HardwareConfig bad=c;bad.servo_right_us=bad.servo_center_us;rejects([&]{bad.validate();},"Misordered servo calibration rejected");
        bad=c;bad.motor_right_pwm=bad.motor_left_pwm;rejects([&]{bad.validate();},"Aliased motor outputs rejected");
        bad=c;bad.encoder_mode="guess";rejects([&]{bad.validate();},"Unknown encoder semantics rejected");
        bad=c;bad.stop_key_index=4;rejects([&]{bad.validate();},"Invalid stop key rejected");
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Startup zeros both motors");
            check(io.lastDuty(c.servo_pwm)==4470,"Startup straight pulse");
            io.writes.clear();board.setMotorCommands(-12000,12000);
            check(io.writes.size()==4 && io.writes[0].path==c.motor_left_pwm && io.writes[1].path==c.motor_left_dir && io.writes[1].bytes==std::vector<uint8_t>{'0'},"Direction changes after zero PWM with ASCII GPIO");
            check(io.lastDuty(c.motor_left_pwm)==2400,"Output scales to driver");
            board.sampleWheels(1);io.put(c.encoder_left,int16_t(-20));io.put(c.encoder_right,int16_t(20));
            auto s=board.sampleWheels(1.02);near(s.leftRps,20/(1024*.02),"Left sign corrected");
            near(s.rightRps,s.leftRps,"Wheel units match");near(s.distanceM,20*.2/1024,"Distance comes from counts");
            s=board.sampleWheels(1.04);near(s.leftRps,0,"Cleared counts produce stopped speed");near(s.distanceM,0,"No phantom stopped odometry");
            io.put(c.encoder_left,int16_t(10));io.put(c.encoder_right,int16_t(-10));s=board.sampleWheels(1.09);
            near(s.leftRps,-10/(1024*.05),"Reverse uses actual interval");near(s.distanceM,10*.2/1024,"Reverse travel is positive");
            rejects([&]{board.sampleWheels(1.3);},"Long sampling gap rejected");
            board.setBeep(true);check(io.writes.back().bytes==std::vector<uint8_t>{'1'},"Buzzer uses factory GPIO");
            check(board.stop(),"Shutdown succeeds");check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Shutdown zeros both sides");
        }
        {
            FakeIo io;io.populate(c);io.binary.erase(c.motor_right_pwm);
            rejects([&]{FactoryBoard board(io,c,p);},"Partial initialization failure propagates");
            check(io.lastDuty(c.motor_left_pwm)==0,"Partially initialized motor stopped");
            check(io.lastDuty(c.motor_right_pwm)==0,"Unreadable metadata still receives zero command");
        }
        {
            FakeIo io;io.populate(c);io.binary.erase(c.motor_left_pwm);
            rejects([&]{FactoryBoard board(io,c,p);},"First metadata failure propagates");
            check(io.lastDuty(c.motor_right_pwm)==0,"First metadata failure cannot skip other motor stop");
        }
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);board.setMotorCommands(1000,1000);
            io.failWrite=c.motor_left_pwm;io.failOnce=true;
            rejects([&]{board.setMotorCommands(2000,2000);},"Write failure propagates");
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Fault rollback stops both motors");
            io.failWrite=c.motor_left_pwm;check(!board.stop(),"Failed shutdown reported");
            check(io.lastDuty(c.motor_right_pwm)==0,"One failure cannot block other shutdown");
        }
        {
            HardwareConfig stopConfig=c;stopConfig.stop_key_index=0;
            FakeIo io;io.populate(c);FactoryBoard board(io,stopConfig,p);
            check(!board.stopInputActive(),"Idle key not stopping");io.put(c.key_prefix+"0",uint8_t(0));
            check(board.stopInputActive(),"Assigned low-active key stops");
            io.put(c.key_prefix+"0",uint8_t(7));rejects([&]{board.stopInputActive();},"Invalid GPIO is not silently accepted");
        }
        {
            HardwareConfig cumulative=c;cumulative.encoder_mode="cumulative16";
            FakeIo io;io.populate(c);io.clearCounts=false;FactoryBoard board(io,cumulative,p);
            io.put(c.encoder_left,int16_t(-32760));io.put(c.encoder_right,int16_t(32760));board.sampleWheels(2);
            io.put(c.encoder_left,int16_t(32756));io.put(c.encoder_right,int16_t(-32756));auto s=board.sampleWheels(2.02);
            near(s.leftRps,20/(1024*.02),"Cumulative counter wraps backward");near(s.rightRps,s.leftRps,"Cumulative wraps forward");
            s=board.sampleWheels(2.04);near(s.leftRps,0,"Unchanged cumulative counter stopped");
        }
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);board.sampleWheels(3);
            io.put(c.encoder_left,int16_t(32767));rejects([&]{board.sampleWheels(3.02);},"Saturated delta count rejected");
            io.binary[c.encoder_left].resize(1);rejects([&]{board.sampleWheels(3.04);},"Short encoder count is not fabricated as zero");
        }
        {
            FakeIo io;io.populate(c);io.binary[c.servo_pwm].resize(2);
            rejects([&]{FactoryBoard board(io,c,p);},"Short metadata read rejected");
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Metadata failure leaves motors zero");
        }
        {
            HardwareConfig gyro=c;gyro.imu_calibration_samples=20;gyro.imu_gyro_scale_deg_s=.1;
            FakeIo io;io.populate(c);const auto raw=c.imu_iio_device+"/in_anglvel_z_raw";
            io.text[raw]="10";IioYaw yaw(io,gyro,1);
            for(int i=0;i<20;++i) yaw.poll(i*.02,true);
            check(yaw.ready() && yaw.fresh(.38),"Stationary calibration completes");near(yaw.value(),0,"Calibration adds no yaw");
            io.text[raw]="110";yaw.poll(.40,false);yaw.poll(.42,false);near(yaw.value(),.3,"Bias-corrected integration uses actual time");
            check(!yaw.fresh(.6),"Stale yaw invalid");rejects([&]{yaw.poll(.7,false);},"Gyro gap stops continuity");
            rejects([&]{yaw.poll(.72,false);},"Gyro cannot silently restart");
            IioYaw moving(io,gyro,1);rejects([&]{moving.poll(1,false);},"Calibration requires stationary wheels");
        }
        {
            FakeIo io;io.populate(c);io.text[c.imu_iio_device+"/in_anglvel_scale"]="invalid";
            rejects([&]{IioYaw yaw(io,c,1);},"Scale is not guessed");
            io.text[c.imu_iio_device+"/name"]="wrong_sensor";rejects([&]{IioYaw yaw(io,c,1);},"Unknown model rejected");
        }
        {
            HardwareConfig gyro=c;gyro.imu_calibration_samples=20;
            FakeIo io;io.populate(c);io.text[c.imu_iio_device+"/in_anglvel_scale"]="0.001";
            IioYaw yaw(io,gyro,-1);for(int i=0;i<20;++i) yaw.poll(i*.02,true);
            io.text[c.imu_iio_device+"/in_anglvel_z_raw"]="100";yaw.poll(.4,false);
            near(yaw.value(),-0.1*180/pi*.01,"Auto IIO scale converts radians to degrees with yaw sign");
            io.text[c.imu_iio_device+"/in_anglvel_z_raw"]="bad";
            rejects([&]{yaw.poll(.42,false);},"Malformed gyro sample fails");
            check(!yaw.fresh(.42),"Malformed gyro sample latches invalid yaw");
        }
        {
            HardwareConfig gyro=c;gyro.imu_calibration_samples=20;gyro.imu_gyro_scale_deg_s=1;
            FakeIo io;io.populate(c);const auto raw=c.imu_iio_device+"/in_anglvel_z_raw";
            IioYaw yaw(io,gyro,1);for(int i=0;i<19;++i) {io.text[raw]=(i%2?"10":"-10");yaw.poll(i*.02,true);}
            rejects([&]{yaw.poll(.38,true);},"Noisy/moving calibration rejected");
        }
        std::cout<<"PASS "<<checks<<" hardware checks using simulated devices (no board attached)\n";return 0;
    }catch(const std::exception& e) {std::cerr<<"FAIL after "<<checks<<" hardware checks: "<<e.what()<<'\n';return 1;}
}
