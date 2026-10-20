// Real PWM/GPIO code, fake filesystem nodes. No physical outputs are accessed.
#include "Contral/PWM/PwmController.hpp"
#include "Contral/GPIO/GPIO.hpp"
#include "ZebraTest/IoStatus.hpp"
#include "ZebraTest/SysfsIo.hpp"
#include <cassert>
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace fs=std::filesystem;
static int checks=0;
void check(bool ok) { ++checks; assert(ok); }
void put(const fs::path& path,const std::string& value) { fs::create_directories(path.parent_path());std::ofstream(path)<<value; }
std::string get(const fs::path& path) {std::ifstream file(path);return {std::istreambuf_iterator<char>(file),{}};}
void channel(const fs::path& dir,const std::string& period="50000",const std::string& duty="0",const std::string& enabled="0") {
    put(dir/"period",period);put(dir/"duty_cycle",duty);put(dir/"enable",enabled);
}
int main() {
    const auto root=fs::temp_directory_path()/("zebra-sysfs-"+std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()));
    const auto pwm=root/"pwm",gpio=root/"gpio",left=pwm/"pwmchip8/pwm2";
    fs::create_directories(root);
    try {
        // Already-exported channels MUST work with no export node at all.
        channel(left,"50000","12000","1");
        {
            Contral::PwmController p(8,2,pwm.string());
            check(p.prepare(50000));check(!fs::exists(pwm/"pwmchip8/export"));
            check(get(left/"duty_cycle")=="0");check(get(left/"enable")=="0");
            check(p.enable());check(get(left/"enable")=="1");
            check(p.setDutyCycle(12000));check(p.readDutyCycle()==12000);
            check(!p.setDutyCycle(50001));check(get(left/"duty_cycle")=="12000");
        }
        check(get(left/"enable")=="0");
        // Shrinking a period below the OLD duty must succeed by clearing duty first.
        channel(left,"3040000","1640042","0");
        {Contral::PwmController p(8,2,pwm.string());check(p.prepare(50000));check(p.readDutyCycle()==0);check(p.readPeriod()==50000);}
        // Linux requires a nonzero period even when writing zero duty on a fresh export.
        channel(left,"0","0","0");
        {Contral::PwmController p(8,2,pwm.string());check(p.initialize());check(!p.setDutyCycle(0));
            check(p.prepare(50000));check(p.readPeriod()==50000);check(p.enable());}
        check(get(left/"enable")=="0");
        channel(left,"0","0","1");
        {Contral::PwmController p(8,2,pwm.string());check(!p.initialize());}
        channel(left);
        // Repeated startup after clean shutdown reuses the same nodes.
        {Contral::PwmController p(8,2,pwm.string());check(p.prepare(50000));check(p.enable());}
        check(get(left/"enable")=="0");
        // Missing nodes are not manufactured by a write; report exact path and errno.
        const auto missing=root/"missing";
        check(!ZebraTest::SysfsIo::writeValue(missing.string(),"1"));check(!fs::exists(missing));
        check(ZebraTest::IoStatus::lastError.find(missing.string())!=std::string::npos);
        check(ZebraTest::IoStatus::lastError.find("errno=")!=std::string::npos);
        // Existing directory with incomplete nodes is a real failure, never 'already ready'.
        fs::remove(left/"duty_cycle");
        {Contral::PwmController p(8,2,pwm.string());check(!p.initialize());}
        check(!fs::exists(left/"duty_cycle"));
        channel(left,"50000","60000");
        {Contral::PwmController p(8,2,pwm.string());check(!p.initialize());}
        channel(left,"not-a-number");
        {Contral::PwmController p(8,2,pwm.string());check(!p.initialize());}
        channel(left,"50000 trailing-junk");
        {Contral::PwmController p(8,2,pwm.string());check(!p.initialize());}
        channel(left,"50000","0","2");
        {Contral::PwmController p(8,2,pwm.string());check(!p.initialize());}
        channel(left);
        // External changes after initialization are detected by the enable readback.
        {Contral::PwmController p(8,2,pwm.string());check(p.prepare(50000));put(left/"period","40000");check(!p.enable());
            check(ZebraTest::IoStatus::lastError.find("PWM_READBACK")!=std::string::npos);}
        channel(left);
        // A write target becoming unusable must fail with diagnostics.
        {Contral::PwmController p(8,2,pwm.string());check(p.initialize());fs::remove(left/"enable");fs::create_directory(left/"enable");
            check(!p.enable());check(ZebraTest::IoStatus::lastError.find("enable")!=std::string::npos);}
        fs::remove(left/"enable");channel(left);
        {Contral::PwmController p(-1,0,pwm.string());check(!p.initialize());}
        {Contral::PwmController p(8,2,pwm.string());check(!p.prepare(0));check(!p.prepare(100,101));}
        // Delayed export materialization, and an export-error race, both require valid nodes.
        for(int race=0;race<2;++race) {
            const auto chip=pwm/("pwmchip"+std::to_string(20+race));
            if(race)fs::create_directories(chip/"export");else put(chip/"export","");
            std::thread exporter([chip] {std::this_thread::sleep_for(std::chrono::milliseconds(12));channel(chip/"pwm0");});
            bool ready=false;
            {Contral::PwmController p(20+race,0,pwm.string());ready=p.initialize();}
            exporter.join();check(ready);
        }
        put(gpio/"gpio73/value","0");put(gpio/"gpio73/direction","out");
        {
            Contral::GPIO pin(73,gpio.string());
            check(!fs::exists(gpio/"export"));check(pin.setDirection("out"));
            check(pin.setValue(true));check(pin.readValue()&&pin.lastReadOk());
            check(pin.setValue(false));check(!pin.readValue()&&pin.lastReadOk());
            check(get(gpio/"gpio73/value")=="0");
            fs::remove(gpio/"gpio73/direction");check(!pin.setDirection("out"));
            check(!fs::exists(gpio/"gpio73/direction"));
        }
        bool threw=false;
        try{Contral::GPIO pin(99,gpio.string());}catch(const std::exception& e){threw=std::string(e.what()).find("gpio99/value")!=std::string::npos;}
        check(threw);
        fs::remove_all(root);
        std::cout<<"PASS "<<checks<<" sysfs regression checks (simulated nodes, no car hardware)\n";
    } catch(...) {fs::remove_all(root);throw;}
}
