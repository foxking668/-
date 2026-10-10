#include "reference_motor_speed.hpp"
#include <algorithm>
#include "../legacy/Contral/PID/PID.h"
// Existing PID.c is identical to the supplied cc(1) implementation. Keep its
// internal pool lifecycle in this translation unit, away from OpenCV headers.
extern "C" bool PID_Memory_Free_Heap_self(void*);
namespace car2026 { namespace capture {
namespace {
void checked(eu_PID_Result result) {
    if(result!=PID_RESULT_SUCCESS) throw std::runtime_error("Reference motor PID setup failed");
}
void release(st_PID_Attr& attr) noexcept {
    auto* node=attr.pid_temp.head;
    while(node) {
        auto* next=node->next;
        PID_Memory_Free_Heap_self(node->data);
        PID_Memory_Free_Heap_self(node->config);
        PID_Memory_Free_Heap_self(node);
        node=next;
    }
    attr.pid_temp={nullptr,nullptr};
}
}
struct ReferenceMotorSpeed::Impl {
    st_PID_Attr wheels[2]{};
    ReferenceMotorTuning tuning{};
    std::array<double,2> target{{0,0}};
    bool ready=false;
    Impl() {for(auto& wheel:wheels) checked(PID_Initialize(&wheel));}
    ~Impl() {for(auto& wheel:wheels) release(wheel);}
};
ReferenceMotorSpeed::ReferenceMotorSpeed():impl_(std::make_unique<Impl>()) {}
ReferenceMotorSpeed::~ReferenceMotorSpeed()=default;
void ReferenceMotorSpeed::reset(const ReferenceMotorTuning& tuning,double direction,bool leftEnabled,bool rightEnabled) {
    tuning.validate();
    if(direction!=1 && direction!=-1) throw std::runtime_error("Reference motor direction must be +1 or -1");
    impl_->ready=false;impl_->tuning=tuning;
    impl_->target={leftEnabled ? direction*tuning.leftRps : 0,rightEnabled ? direction*tuning.rightRps : 0};
    for(auto& wheel:impl_->wheels) {
        checked(PID_Incremental_Set_Kpid(&wheel,float(tuning.kp),float(tuning.ki),float(tuning.kd)));
        checked(PID_Incremental_Open_OutLimit(&wheel));
        checked(PID_Incremental_Set_OutLimitValue(&wheel,float(tuning.pwmLimit)));
        checked(PID_Incremental_Clear_TempData(&wheel));
    }
    impl_->ready=true;
}
std::array<double,2> ReferenceMotorSpeed::targets() const {return impl_->target;}
std::array<int,2> ReferenceMotorSpeed::update(double left,double right) {
    if(!impl_->ready || !std::isfinite(left) || !std::isfinite(right)) throw std::runtime_error("Invalid reference speed feedback");
    std::array<int,2> result{{0,0}};
    const double measured[]={left,right};
    for(size_t index=0;index<2;++index) {
        if(impl_->target[index]==0) continue; // Disabled wheel stays zero, including overspeed feedback.
        // cc Motor.cpp stores the PID result in float before converting to int.
        const float pwm=float(PID_Incremental_CalcResult_ByNowTureValue(&impl_->wheels[index],float(measured[index]),float(impl_->target[index])));
        if(!std::isfinite(pwm)) throw std::runtime_error("Non-finite reference motor PID output");
        int value=int(std::clamp(double(pwm),-double(impl_->tuning.pwmLimit),double(impl_->tuning.pwmLimit)));
        if(value!=0 && std::abs(value)<100) value=value>0 ? 100 : -100; // Reference Motor::constrainPwm.
        result[index]=value;
    }
    return result;
}
}}
