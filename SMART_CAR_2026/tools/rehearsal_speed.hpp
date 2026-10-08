#pragma once
#include "hardware.hpp"
#include <algorithm>
#include <cmath>
#include <limits>
#include <map>
#include <optional>
#include <ostream>
#include <tuple>

namespace car2026 { namespace capture {
struct WheelSpeedSample {
    bool valid=false;double count=0;int64_t endNs=0;
};
struct RehearsalSpeedWindow {
    int stage=0;unsigned segment=0,trial=0,revision=0;
    int64_t endNs=0;double seconds[2]{},counts[2]{},absoluteCounts[2]{},scale[2]{};
    double rate(size_t wheel) const {return counts[wheel]/seconds[wheel];}
    double magnitudeRate(size_t wheel) const {return absoluteCounts[wheel]/seconds[wheel];}
    bool calibrated() const {return scale[0]>0 && scale[1]>0;}
    double centerCmPerSecond() const {return (rate(0)*scale[0]+rate(1)*scale[1])/2;}
};
struct RehearsalSpeedTotals {
    size_t windows=0;double seconds[2]{},counts[2]{},absoluteCounts[2]{},movingSeconds[2]{},peak[2]{},scale[2]{};
    void add(const RehearsalSpeedWindow& window) {
        ++windows;
        for(size_t i=0;i<2;++i) {
            seconds[i]+=window.seconds[i];counts[i]+=window.counts[i];absoluteCounts[i]+=window.absoluteCounts[i];
            if(window.absoluteCounts[i]>0) movingSeconds[i]+=window.seconds[i];
            peak[i]=std::max(peak[i],window.magnitudeRate(i));scale[i]=window.scale[i];
        }
    }
};
// No extra device reads: consume the existing encoder samples exactly once.
// Keep each wheel's read interval; camera/frame periods are not encoder periods.
class RehearsalSpeedRecorder {
public:
    using Key=std::tuple<int,unsigned,unsigned,unsigned>;
    RehearsalSpeedRecorder(std::string mode,double leftSign,double rightSign):mode_(std::move(mode)),sign_{leftSign,rightSign} {
        encoderCountDelta(0,0,mode_);
        for(double sign:sign_) if(sign!=1 && sign!=-1) throw std::runtime_error("Invalid speed encoder sign");
    }
    void reset() {baseline_=false;latest_.reset();clearWindow();}
    std::optional<RehearsalSpeedWindow> add(int stage,unsigned segment,unsigned trial,unsigned revision,
            double leftScale,double rightScale,const WheelSpeedSample& left,const WheelSpeedSample& right) {
        const Key key{stage,segment,trial,revision};
        if(!context_ || *context_!=key) {reset();context_=key;}
        const WheelSpeedSample samples[]{left,right};
        for(const auto& sample:samples) {
            if(!sample.valid || sample.endNs<0 || !std::isfinite(sample.count) || std::floor(sample.count)!=sample.count ||
               sample.count<std::numeric_limits<EncoderCount>::min() || sample.count>std::numeric_limits<EncoderCount>::max() ||
               (mode_=="delta" && (sample.count==std::numeric_limits<EncoderCount>::min() || sample.count==std::numeric_limits<EncoderCount>::max()))) {
                reset();return {};
            }
            try {encoderCountDelta(EncoderCount(sample.count),EncoderCount(sample.count),mode_);}
            catch(const std::exception&) {reset();return {};}
        }
        if(!std::isfinite(leftScale) || !std::isfinite(rightScale) || leftScale<0 || rightScale<0 || leftScale>10 || rightScale>10)
            throw std::runtime_error("Invalid speed scale");
        if(!baseline_) {setBaseline(samples);return {};}
        double dt[2];int64_t delta[2];
        for(size_t i=0;i<2;++i) {
            dt[i]=double(samples[i].endNs-previousTime_[i])/1e9;
            // A pause/error/slow read leaves an unknown interval. Do not turn it into a speed spike.
            if(dt[i]<=0 || dt[i]>2) {reset();setBaseline(samples);return {};}
            try {delta[i]=encoderCountDelta(EncoderCount(samples[i].count),previousCount_[i],mode_);}
            catch(const std::exception&) {reset();return {};}
        }
        for(size_t i=0;i<2;++i) {
            seconds_[i]+=dt[i];counts_[i]+=double(delta[i])*sign_[i];absoluteCounts_[i]+=std::abs(double(delta[i]));
        }
        setBaseline(samples);
        if(std::min(seconds_[0],seconds_[1])<.5) return {};
        RehearsalSpeedWindow window;
        window.stage=stage;window.segment=segment;window.trial=trial;window.revision=revision;
        window.endNs=std::max(left.endNs,right.endNs);
        for(size_t i=0;i<2;++i) {
            window.seconds[i]=seconds_[i];window.counts[i]=counts_[i];window.absoluteCounts[i]=absoluteCounts_[i];
        }
        window.scale[0]=leftScale;window.scale[1]=rightScale;
        totals_[key].add(window);latest_=window;clearWindow();return window;
    }
    const std::optional<RehearsalSpeedWindow>& latest() const {return latest_;}
    const std::map<Key,RehearsalSpeedTotals>& totals() const {return totals_;}
    void writeSummary(std::ostream& out) const {
        out<<"stage,segment,trial,config_revision,window_count,left_observed_s,right_observed_s,left_net_counts,right_net_counts,left_abs_counts,right_abs_counts,left_mean_abs_counts_s,right_mean_abs_counts_s,left_moving_mean_abs_counts_s,right_moving_mean_abs_counts_s,left_peak_window_counts_s,right_peak_window_counts_s,left_cm_per_count,right_cm_per_count,center_mean_abs_cm_s,center_moving_mean_abs_cm_s\n";
        for(const auto& entry:totals_) {
            const auto& key=entry.first;const auto& t=entry.second;
            out<<std::get<0>(key)+1<<','<<std::get<1>(key)<<','<<std::get<2>(key)<<','<<std::get<3>(key)<<','<<t.windows;
            for(const auto* pair:{t.seconds,t.counts,t.absoluteCounts}) for(size_t i=0;i<2;++i) out<<','<<pair[i];
            for(size_t i=0;i<2;++i) out<<','<<t.absoluteCounts[i]/t.seconds[i];
            for(size_t i=0;i<2;++i) {out<<',';if(t.movingSeconds[i]>0) out<<t.absoluteCounts[i]/t.movingSeconds[i];}
            for(double peak:t.peak) out<<','<<peak;
            for(double scale:t.scale) out<<','<<scale;
            out<<',';if(t.scale[0]>0 && t.scale[1]>0) out<<(t.absoluteCounts[0]*t.scale[0]/t.seconds[0]+t.absoluteCounts[1]*t.scale[1]/t.seconds[1])/2;
            out<<',';if(t.scale[0]>0 && t.scale[1]>0 && t.movingSeconds[0]>0 && t.movingSeconds[1]>0)
                out<<(t.absoluteCounts[0]*t.scale[0]/t.movingSeconds[0]+t.absoluteCounts[1]*t.scale[1]/t.movingSeconds[1])/2;
            out<<'\n';
        }
    }
private:
    void clearWindow() {for(size_t i=0;i<2;++i) seconds_[i]=counts_[i]=absoluteCounts_[i]=0;}
    void setBaseline(const WheelSpeedSample* samples) {
        for(size_t i=0;i<2;++i) {previousCount_[i]=EncoderCount(samples[i].count);previousTime_[i]=samples[i].endNs;}baseline_=true;
    }
    std::string mode_;double sign_[2];bool baseline_=false;EncoderCount previousCount_[2]{};int64_t previousTime_[2]{};
    double seconds_[2]{},counts_[2]{},absoluteCounts_[2]{};
    std::optional<Key> context_;std::optional<RehearsalSpeedWindow> latest_;std::map<Key,RehearsalSpeedTotals> totals_;
};
}}
