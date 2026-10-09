#pragma once
#include "rehearsal_speed.hpp"
#include "parking_rehearsal.hpp"
namespace car2026 { namespace capture {
// Retain native speed units. Never manufacture encoder count deltas from RPS.
class NativeRpsRecorder {
    using Key=std::tuple<int,unsigned,unsigned,unsigned>;
    struct Wheel {size_t samples=0;int64_t lastNs=0;double last=0,seconds=0,integral=0,peak=0;};
    std::map<Key,std::array<Wheel,2>> rows_;
public:
    void add(int stage,unsigned segment,unsigned trial,unsigned revision,const WheelSpeedSample& left,const WheelSpeedSample& right) {
        auto& wheels=rows_[Key{stage,segment,trial,revision}];const WheelSpeedSample values[]{left,right};
        for(size_t i=0;i<2;++i) {
            auto& wheel=wheels[i];const auto& value=values[i];
            if(!value.valid || !std::isfinite(value.count)) {wheel.lastNs=0;continue;}
            if(wheel.lastNs && value.endNs>wheel.lastNs && value.endNs-wheel.lastNs<=int64_t(2e9)) {
                const double dt=double(value.endNs-wheel.lastNs)/1e9;
                wheel.integral+=(wheel.last+value.count)*.5*dt;wheel.seconds+=dt;
            }
            ++wheel.samples;wheel.peak=std::max(wheel.peak,std::abs(value.count));wheel.last=value.count;wheel.lastNs=value.endNs;
        }
    }
    void gap() {for(auto& row:rows_) for(auto& wheel:row.second) wheel.lastNs=0;}
    void writeSummary(std::ostream& out) const {
        out<<"stage,step,trial,config_revision,unit,left_samples,left_observed_s,left_mean_rps,left_peak_rps,right_samples,right_observed_s,right_mean_rps,right_peak_rps\n";
        for(const auto& row:rows_) {
            const auto& key=row.first;out<<std::get<0>(key)+1<<','<<rehearsalSegmentName(std::get<0>(key),std::get<1>(key))<<','<<std::get<2>(key)<<','<<std::get<3>(key)<<",rps";
            for(const auto& wheel:row.second) {
                out<<','<<wheel.samples<<','<<wheel.seconds<<',';
                if(wheel.seconds>0) out<<wheel.integral/wheel.seconds;
                out<<','<<wheel.peak;
            }
            out<<'\n';
        }
    }
};
}}
