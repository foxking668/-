#include "Steer.hpp"
#include "ZebraTest/IoStatus.hpp"
#include <memory>

using namespace Other;
namespace Contral {
Steer::Steer(const std::string& root) : pwm_root_(root) {}

bool Steer::dutyForAngle(float angle,unsigned int& duty) const {
    angle+=steer_offset;
    // Keep the original VV17 pulse conversion and mechanical offset; no new angle limits.
    const float value=(((angle-90.0f)/90.0f)*5.0f)*(float(period)/100.0f)+1520000.0f;
    if(!std::isfinite(value) || value<0 || value>period) {
        ZebraTest::IoStatus::set("STEER_DUTY_RANGE","angle_to_pwm",ERANGE);
        return false;
    }
    duty=static_cast<unsigned int>(value);return true;
}

void Steer::init() {
    if(pwmCtrl)return; // Repeated initialization keeps the existing live owner.
    using ZebraTest::IoStatus::require;
    auto pwm=std::make_unique<PwmController>(STEER_PWM_CHIP,STEER_PWM_INDEX,pwm_root_);
    unsigned int initialDuty=0;
    require(dutyForAngle(90,initialDuty),"Steer initial angle");
    require(pwm->prepare(period,initialDuty),"Steer PWM prepare");
    require(pwm->enable(),"Steer PWM enable");
    // Commit ownership only when every initialization step has succeeded.
    delete static_cast<PwmController*>(pwmCtrl);
    pwmCtrl=pwm.release();
    Log(LOG_LEVEL_GLOBAL).logOutputConsole("Steer Initialize Success",LOG_INFO);
}

bool Steer::setAngle(float angle) {
    unsigned int duty=0;
    if(!dutyForAngle(angle,duty))return false;
    if(!pwmCtrl) {
        ZebraTest::IoStatus::set("STEER_NOT_INITIALIZED","pwmCtrl",ENODEV);return false;
    }
    return static_cast<PwmController*>(pwmCtrl)->setDutyCycle(duty);
}
void Steer::setSteerOffset(float offsetAngle) { steer_offset=offsetAngle; }
Steer::~Steer() { delete static_cast<PwmController*>(pwmCtrl); }
}
