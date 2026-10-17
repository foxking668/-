#include "Contral/PWM/PwmController.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <thread>
int checks=0;
void check(bool yes,const char* why){++checks;if(!yes)throw std::runtime_error(why);}
std::string read(const std::filesystem::path& path){std::ifstream in(path);std::string s;in>>s;return s;}
void channel(const std::filesystem::path& path){std::filesystem::create_directories(path);std::ofstream(path/"period")<<50000;std::ofstream(path/"duty_cycle")<<0;std::ofstream(path/"enable")<<0;}
int main(int argc,char** argv){try{
    check(argc==2,"temp directory required");const std::filesystem::path root=argv[1];std::filesystem::create_directories(root/"pwmchip8");
    const auto chip=root/"pwmchip8";std::ofstream(chip/"export")<<"unchanged";channel(chip/"pwm2");
    Contral::PwmController left(8,2,root.string());
    check(left.initialize(),"existing complete channel reusable");check(left.initialize(),"second initialization idempotent");
    check(read(chip/"export")=="unchanged","existing channel never exported again");
    check(left.readPeriod()==50000 && left.readDutyCycle()==0,"existing metadata read");
    check(left.setDutyCycle(9000) && read(chip/"pwm2/duty_cycle")=="9000","duty write acknowledged");
    check(left.setDutyCycle(0) && read(chip/"pwm2/duty_cycle")=="0","zero truncates previous regular fixture value");
    check(left.enable() && read(chip/"pwm2/enable")=="1","enable write");
    check(left.disable() && read(chip/"pwm2/enable")=="0","disable write");
    std::ofstream(chip/"pwm2/enable")<<2;
    check(!left.initialize(),"invalid enable metadata rejected");std::ofstream(chip/"pwm2/enable")<<0;
    // Simulate delayed sysfs node creation after exactly one export request.
    std::thread creator([&]{std::this_thread::sleep_for(std::chrono::milliseconds(12));channel(chip/"pwm1");});
    Contral::PwmController right(8,1,root.string());const bool initialized=right.initialize();creator.join();
    check(initialized && read(chip/"export")=="1","delayed channel appears after export");
    check(right.initialize() && read(chip/"export")=="1","repeated delayed channel reuse");
    std::filesystem::create_directories(chip/"pwm3");
    Contral::PwmController missing(8,3,root.string());
    check(!missing.initialize(),"incomplete channel not mistaken for ready");
    check(read(chip/"export")=="1","incomplete existing channel is not re-exported");
    const auto failedRoot=root/"missing-root";Contral::PwmController absent(99,0,failedRoot.string());
    check(!absent.initialize(),"missing chip reports failure");
    const auto impossible=root/"removed";channel(impossible/"pwmchip9/pwm0");Contral::PwmController failure(9,0,impossible.string());
    check(failure.initialize() && failure.setDutyCycle(400),"failure fixture initialized");
    std::filesystem::remove(impossible/"pwmchip9/pwm0/duty_cycle");std::filesystem::create_directory(impossible/"pwmchip9/pwm0/duty_cycle");
    check(!failure.setDutyCycle(900) && failure.readDutyCycle()==400,"failed write leaves last acknowledged cache unchanged");
    std::cout<<"PASS "<<checks<<" PWM reuse/delay/zero/failure checks\n";return 0;
}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
