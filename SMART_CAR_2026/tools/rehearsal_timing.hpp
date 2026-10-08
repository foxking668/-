#pragma once
#include "rehearsal_speed.hpp"
#include <iomanip>
namespace car2026 { namespace capture {
struct StageTiming {
    int stage=0;unsigned trial=0,revision=0;
    int64_t ready=0,end=0,motorStart=0,motorStop=0,firstMotion=0,lastMotion=0,zeroSince=0,lastSample=0;
    size_t validPairs=0;std::string reason;
};
// Consume capture's existing encoder samples; never read/reset an encoder here.
class RehearsalTiming {
public:
    explicit RehearsalTiming(std::string mode):mode_(std::move(mode)) {encoderCountDelta(0,0,mode_);}
    void begin(int stage,unsigned trial,unsigned revision,int64_t now) {
        end(now,"REPLACED");StageTiming row;row.stage=stage;row.trial=trial;row.revision=revision;row.ready=now;
        rows_.push_back(row);active_=rows_.size()-1;baseline_=false;
    }
    void end(int64_t now,const std::string& reason) {
        if(active_ && rows_[*active_].end==0) {rows_[*active_].end=now;rows_[*active_].reason=reason;}
    }
    void motorStart(int64_t now) {if(active_) {rows_[*active_].motorStart=now;rows_[*active_].ready=now;}}
    void motorStop(int64_t now,const std::string& reason) {if(active_) {rows_[*active_].motorStop=now;end(now,reason);baseline_=false;rows_[*active_].zeroSince=0;}}
    bool add(const WheelSpeedSample& left,const WheelSpeedSample& right) {
        const WheelSpeedSample samples[]{left,right};
        if(!active_) return false;
        auto& row=rows_[*active_];
        for(const auto& sample:samples)
            if(!sample.valid || !std::isfinite(sample.count) || std::floor(sample.count)!=sample.count ||
               sample.count<=std::numeric_limits<EncoderCount>::min() || sample.count>=std::numeric_limits<EncoderCount>::max() ||
               sample.endNs<row.ready) {baseline_=false;row.zeroSince=0;return false;}
        const auto now=std::max(left.endNs,right.endNs);
        ++row.validPairs;
        if(!baseline_) {setBaseline(samples);row.lastSample=now;return true;}
        bool moving=false;
        for(size_t i=0;i<2;++i) {
            if(samples[i].endNs<=previous_[i].endNs || samples[i].endNs-previous_[i].endNs>int64_t(2e9)) {
                setBaseline(samples);row.zeroSince=0;return false;
            }
            try {const auto delta=encoderCountDelta(EncoderCount(samples[i].count),EncoderCount(previous_[i].count),mode_);moving=moving || delta!=0;}
            catch(...) {baseline_=false;row.zeroSince=0;return false;}
        }
        if(moving) {if(!row.firstMotion) row.firstMotion=now;row.lastMotion=now;row.zeroSince=0;}
        else if(!row.zeroSince) row.zeroSince=row.lastSample;
        row.lastSample=now;setBaseline(samples);return true;
    }
    bool stationaryAfter(int64_t stopNs) const {
        if(!active_) return false;
        const auto& row=rows_[*active_];
        return row.zeroSince>=stopNs && row.lastSample-row.zeroSince>=int64_t(.5e9);
    }
    void gap() {baseline_=false;if(active_) rows_[*active_].zeroSince=0;}
    void writeSummary(std::ostream& out,int64_t origin) const {
        out<<"stage,trial,config_revision,ready_s,motor_start_s,output_stop_s,stage_duration_s,first_encoder_motion_s,last_encoder_motion_s,encoder_motion_span_s,observed_zero_tail_s,valid_encoder_pairs,reason\n"<<std::setprecision(17);
        for(const auto& row:rows_) {
            out<<row.stage+1<<','<<row.trial<<','<<row.revision<<','<<double(row.ready-origin)/1e9;
            for(auto ns:{row.motorStart,row.motorStop}) {out<<',';if(ns) out<<double(ns-origin)/1e9;}
            out<<',';if(row.end) out<<double(row.end-row.ready)/1e9;
            for(auto ns:{row.firstMotion,row.lastMotion}) {out<<',';if(ns) out<<double(ns-origin)/1e9;}
            out<<',';if(row.firstMotion) out<<double(row.lastMotion-row.firstMotion)/1e9;
            out<<',';if(row.zeroSince) out<<double(row.lastSample-row.zeroSince)/1e9;
            out<<','<<row.validPairs<<','<<row.reason<<'\n';
        }
    }
    const std::vector<StageTiming>& rows() const {return rows_;}
private:
    void setBaseline(const WheelSpeedSample* samples) {previous_[0]=samples[0];previous_[1]=samples[1];baseline_=true;}
    std::string mode_;bool baseline_=false;WheelSpeedSample previous_[2];std::vector<StageTiming> rows_;std::optional<size_t> active_;
};
}}
