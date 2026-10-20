// Real motor/steer initialization against fake nodes; kernel electrical behavior is not simulated.
#include "headfile.hpp"
#include "ZebraTest/IoStatus.hpp"
#include <cassert>
#include <filesystem>
#include <limits>

namespace fs=std::filesystem;
static int checks=0;
void check(bool ok){++checks;assert(ok);}
void put(const fs::path& path,const std::string& text){fs::create_directories(path.parent_path());std::ofstream(path)<<text;}
int value(const fs::path& path){int v=-1;std::ifstream(path)>>v;return v;}
void channel(const fs::path& dir,const std::string& period){put(dir/"period",period);put(dir/"duty_cycle","0");put(dir/"enable","0");}
int main(){
    const auto root=fs::temp_directory_path()/("zebra-hardware-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto pwm=root/"pwm",gpio=root/"gpio",left=pwm/"pwmchip8/pwm2",right=pwm/"pwmchip8/pwm1",steer=pwm/"pwmchip2/pwm0";
    try{
        channel(left,"50000");channel(right,"50000");channel(steer,"3040000");
        for(int n:{12,13,73}){put(gpio/("gpio"+std::to_string(n))/"direction","out");put(gpio/("gpio"+std::to_string(n))/"value","0");}
        for(int run=0;run<2;++run){
            {
                Contral::Motor motor(8,2,12,8,1,13,73,pwm.string(),gpio.string());
                check(value(left/"enable")==1 && value(right/"enable")==1);
                check(value(left/"duty_cycle")==0 && value(right/"duty_cycle")==0);
                check(value(gpio/"gpio73/value")==1);
                check(motor.setLeftPwm(0));check(motor.setRightPwm(-12000));
                check(value(left/"duty_cycle")==0 && value(right/"duty_cycle")==12000);
                check(value(gpio/"gpio13/value")==1); // Right reverse mapping remains VV17.
            }
            check(value(left/"enable")==0 && value(right/"enable")==0);
            check(value(gpio/"gpio73/value")==0);
        }
        // Failed right-channel setup must roll back the left PWM and driver enable.
        fs::remove(right/"period");bool rejected=false;
        try{Contral::Motor motor(8,2,12,8,1,13,73,pwm.string(),gpio.string());}
        catch(const std::exception& e){const std::string cause=e.what();rejected=cause.find("Right motor PWM prepare")!=std::string::npos && cause.find("pwm1/period")!=std::string::npos && cause.find("errno=")!=std::string::npos;}
        check(rejected);check(value(left/"enable")==0);check(value(gpio/"gpio73/value")==0);
        channel(right,"50000");
        // A GPIO setup failure must also unwind prepared PWMs, not continue to success.
        fs::remove(gpio/"gpio13/direction");rejected=false;
        try{Contral::Motor motor(8,2,12,8,1,13,73,pwm.string(),gpio.string());}
        catch(const std::exception& e){rejected=std::string(e.what()).find("gpio13/direction")!=std::string::npos;}
        check(rejected);check(value(left/"enable")==0 && value(right/"enable")==0);check(value(gpio/"gpio73/value")==0);
        {
            Contral::Steer steering(pwm.string());steering.setSteerOffset(20);steering.init();
            check(value(steer/"duty_cycle")==1553777); // Calibrated +20 applies to FIRST pulse.
            check(value(steer/"enable")==1);steering.init();check(value(steer/"enable")==1);
            check(steering.setAngle(90));check(value(steer/"duty_cycle")==1553777);
            check(!steering.setAngle(std::numeric_limits<float>::quiet_NaN()));
        }
        check(value(steer/"enable")==0);
        fs::remove(steer/"duty_cycle");rejected=false;
        try{Contral::Steer steering(pwm.string());steering.init();}
        catch(const std::exception& e){rejected=std::string(e.what()).find("pwm0/duty_cycle")!=std::string::npos;}
        check(rejected);check(value(steer/"enable")==0);
        fs::remove_all(root);
        std::cout<<"PASS "<<checks<<" hardware startup/rollback checks (fake nodes, no car hardware)\n";
    }catch(...){fs::remove_all(root);throw;}
}
