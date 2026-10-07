#include "../tools/rehearsal_servo.hpp"
#include <iostream>
#include <atomic>
#include <mutex>
using namespace car2026;
using namespace car2026::capture;
namespace {
int checks=0;
void check(bool ok,const char* name) {++checks;if(!ok) throw std::runtime_error(name);}
class ServoOnlyIo final:public DeviceIo {
public:
    std::mutex mutex;std::vector<uint16_t> duties;std::atomic<bool> failNext{false};
    void readBinary(const std::string&,void*,size_t) override {throw std::runtime_error("Generic input forbidden");}
    void writeBinary(const std::string&,const void*,size_t) override {throw std::runtime_error("Generic output forbidden");}
    std::string readText(const std::string&) override {throw std::runtime_error("Text input forbidden");}
    PwmInfo readPwmMetadata(const std::string& path) override {
        if(path!=HardwareConfig{}.servo_pwm) throw std::runtime_error("Only servo metadata permitted");
        return {300,4470,10000,1490000,3333333,100000000};
    }
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        if(path!=HardwareConfig{}.servo_pwm) throw std::runtime_error("Motor/GPIO output forbidden");
        std::lock_guard<std::mutex> held(mutex);duties.push_back(duty);
        if(failNext.exchange(false)) throw std::runtime_error("Injected write failure");
    }
    size_t size() {std::lock_guard<std::mutex> held(mutex);return duties.size();}
};
std::deque<ServoHoldEvent> waitEvent(RehearsalServo& servo) {
    const auto deadline=std::chrono::steady_clock::now()+std::chrono::seconds(2);
    while(std::chrono::steady_clock::now()<deadline) {
        auto events=servo.takeEvents();if(!events.empty()) return events;
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
    }
    throw std::runtime_error("Timer did not deliver event");
}
}
int main() {
 try {
    HardwareConfig hardware;Params params;ServoOnlyIo io;
    {
        RehearsalServo servo(io,hardware,params);
        check(io.size()==0,"construct does not write");
        bool rejected=false;try {servo.beginHold(.01,1,1,1);}catch(const std::exception&) {rejected=true;}
        check(rejected,"timer cannot arm before authorized command");
        servo.set(10);std::this_thread::sleep_for(std::chrono::milliseconds(50));
        check(io.size()==1 && servo.takeEvents().empty(),"waiting for push permission does not start timer");
        servo.beginHold(.03,1,2,3);
        auto events=waitEvent(servo);
        check(events.size()==1 && events.front().error.empty(),"timer writes center");
        check(events.front().stage==1 && events.front().revision==2 && events.front().trial==3,"timer preserves stage/config/trial context");
        check(events.front().beginNs<=events.front().endNs,"actual write timestamps");
        check(servo.written()==0 && io.size()==2,"zero command tracked");
        std::this_thread::sleep_for(std::chrono::milliseconds(50));check(io.size()==2,"expired timer does not repeat zero");
        servo.set(-5);servo.beginHold(0,1,1,1);
        std::this_thread::sleep_for(std::chrono::milliseconds(50));check(io.size()==3,"hold zero disables timer");
        servo.beginHold(.05,1,1,1);servo.set(5);
        std::this_thread::sleep_for(std::chrono::milliseconds(100));check(io.size()==4 && servo.takeEvents().empty(),"new authorized command cancels old timer");
        servo.beginHold(.03,3,1,1);io.failNext=true;events=waitEvent(servo);
        check(events.size()==1 && !events.front().error.empty(),"automatic zero failure surfaced");
        check(servo.written()==5,"failed zero not reported as successful");
        servo.close();const auto size=io.size();servo.close();check(io.size()==size,"close idempotent");
        check(io.duties.back()==4470,"exit centers configured PWM");
    }
    {
        ServoOnlyIo idle;RehearsalServo servo(idle,hardware,params);servo.close();
        check(idle.size()==0,"exit before authorization never writes");
    }
    {
        ServoOnlyIo cancelled;RehearsalServo servo(cancelled,hardware,params);servo.set(10);servo.beginHold(.08,1,1,1);servo.close();
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        check(cancelled.size()==2,"exit cancels pending timer before cleanup zero");
    }
    std::cout<<checks<<" timed servo checks passed\n";return 0;
 } catch(const std::exception& error) {std::cerr<<"FAIL "<<error.what()<<'\n';return 1;}
}
