#pragma once
#include "hardware.hpp"
#include <limits>

namespace car2026 {
// Commissioning statistics, not a vehicle speed estimate. Counter mode must
// first be verified on the board; no automatic delta/cumulative guessing.
class EncoderBenchStats {
public:
    EncoderBenchStats(std::string mode,EncoderCount baseline) : mode_(std::move(mode)),previous_(baseline) {
        encoderCountDelta(baseline,baseline,mode_); // Validate cumulative domain/mode.
    }
    void add(EncoderCount count) {
        if(mode_=="delta" && (count==std::numeric_limits<EncoderCount>::min() || count==std::numeric_limits<EncoderCount>::max()))
            throw std::runtime_error("Saturated delta encoder count during bench sampling");
        const int64_t step=encoderCountDelta(count,previous_,mode_);
        const int64_t magnitude=step<0?-step:step;
        const auto high=std::numeric_limits<int64_t>::max(),low=std::numeric_limits<int64_t>::min();
        if((step>0 && net_>high-step) || (step<0 && net_<low-step) || absolute_>high-magnitude)
            throw std::runtime_error("Encoder bench totals overflow");
        net_+=step;absolute_+=magnitude;previous_=count;++samples_;
    }
    int64_t net() const {return net_;}
    int64_t absolute() const {return absolute_;}
    size_t samples() const {return samples_;}
private:
    std::string mode_;EncoderCount previous_;
    int64_t net_=0,absolute_=0;size_t samples_=0;
};
} // namespace car2026
