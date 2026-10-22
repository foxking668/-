#pragma once

namespace Parking {
enum class SteeringUpdateResult { Unchanged, Applied, Failed };

// Motor-thread local. One restart for each distinct target, not every 20ms and
// not when only a stage's motor PWM changes. Failures latch until a new run.
class SteeringUpdate {
public:
    bool needsUpdate(float angle, float offset) const {
        return !failed_ && (!applied_ || angle != angle_ || offset != offset_);
    }
    template<class Write>
    SteeringUpdateResult apply(float angle, float offset, Write write) {
        if (failed_) return SteeringUpdateResult::Failed;
        if (!needsUpdate(angle, offset)) return SteeringUpdateResult::Unchanged;
        if (!write()) {
            failed_ = true;
            return SteeringUpdateResult::Failed;
        }
        angle_ = angle; offset_ = offset; applied_ = true;
        return SteeringUpdateResult::Applied;
    }
private:
    bool applied_ = false, failed_ = false;
    float angle_ = 0, offset_ = 0;
};
}
