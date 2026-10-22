#pragma once
#include "ParkingController.hpp"
#include <atomic>
#include <chrono>
#include <fstream>
#include <iomanip>
#include <mutex>
#include <stdexcept>
#include <string>

// Test selection and passive telemetry. Disabled in the original competition entry.
// This class never writes motors/steering or advances the production controller.
namespace ParkingTest {
enum class Mode { Full, ApproachOnly, ReverseOnly, S2, S4, S5 };
inline const char *modeName(Mode m) {
    switch (m) {
    case Mode::Full: return "full";
    case Mode::ApproachOnly: return "approach-only";
    case Mode::ReverseOnly: return "reverse-only";
    case Mode::S2: return "s2";
    case Mode::S4: return "s4";
    case Mode::S5: return "s5";
    }
    return "unknown";
}
inline Mode parseMode(const std::string &text) {
    for (auto m : {Mode::Full, Mode::ApproachOnly, Mode::ReverseOnly, Mode::S2, Mode::S4, Mode::S5})
        if (text == modeName(m)) return m;
    throw std::runtime_error("unknown mode: " + text);
}
inline bool seeksZebra(Mode m) { return m == Mode::Full || m == Mode::ApproachOnly; }
inline Parking::Config selectStages(Parking::Config c, Mode m) {
    if (m == Mode::ApproachOnly) c.enabled = false;
    if (!seeksZebra(m)) c.approachMs = 0;
    if (m == Mode::S2 || m == Mode::S4 || m == Mode::S5) {
        const int selected = m == Mode::S2 ? 0 : (m == Mode::S4 ? 1 : 2);
        for (int i = 0; i < 3; ++i) c.stages[i].enabled = c.stages[i].enabled && i == selected;
    }
    return c;
}

class Runtime {
public:
    bool enabled() const { return enabled_.load(); }
    Mode mode() const { return mode_; } // immutable from before worker creation until join
    bool finished() const { return finished_.load(); }
    Parking::Phase phase() const { return phase_.load(); }
    void configure(Mode mode, const std::string &directory) {
        mode_ = mode;
        if (!directory.empty()) {
            motorLog_.open(directory + "/motor.csv");
            frameLog_.open(directory + "/frames.csv");
            events_.open(directory + "/events.txt");
            if (!motorLog_ || !frameLog_ || !events_) throw std::runtime_error("cannot open session logs");
            motorLog_ << "elapsed_ms,phase,reason,left_pwm_command,right_pwm_command,target_rps,left_rps,right_rps,left_turns_est,right_turns_est,steer_error_command\n";
            frameLog_ << "elapsed_ms,valid,steer_error_command\n";
        }
        start_ = Clock::now();
        enabled_.store(true);
        event(std::string("MODE=") + modeName(mode));
    }
    void frame(bool valid, float error) {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        error_ = error;
        if (frameLog_.is_open()) frameLog_ << elapsed() << ',' << valid << ',' << error << '\n';
    }
    void armed(const Parking::Config &c) {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (!events_.is_open()) return;
        events_ << elapsed() << " FROZEN_CONFIG center=" << c.center << " servo_offset=" << c.servoOffset
                << " approach_ms=" << c.approachMs << " settle_ms=" << c.settleMs
                << " enabled=" << c.enabled << " left_enabled=" << c.leftEnabled
                << " right_enabled=" << c.rightEnabled << '\n';
        for (std::size_t i = 0; i < c.stages.size(); ++i) {
            const auto &s = c.stages[i];
            events_ << "stage=" << i << " enabled=" << s.enabled << " offset=" << s.steerOffset
                    << " pwm=" << s.pwm << " total_ms=" << s.totalMs << " hold_ms=" << s.holdMs << '\n';
        }
        events_.flush();
    }
    void motor(Parking::Phase phase, const char *reason, int leftPwm, int rightPwm,
               double target, double leftRps, double rightRps) {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        const auto now = Clock::now();
        if (lastMotor_ != Clock::time_point{}) {
            const double seconds = std::chrono::duration<double>(now-lastMotor_).count();
            leftTurns_ += previousLeftRps_*seconds;
            rightTurns_ += previousRightRps_*seconds;
        }
        lastMotor_ = now; previousLeftRps_ = leftRps; previousRightRps_ = rightRps;
        if (motorLog_.is_open()) motorLog_ << elapsed() << ',' << Parking::phaseName(phase) << ',' << reason
            << ',' << leftPwm << ',' << rightPwm << ',' << target << ',' << leftRps << ',' << rightRps
            << ',' << leftTurns_ << ',' << rightTurns_ << ',' << error_ << '\n';
        if (phase_.exchange(phase) != phase && events_.is_open())
            events_ << elapsed() << " PHASE=" << Parking::phaseName(phase) << '\n';
        if (phase == Parking::Phase::Done || phase == Parking::Phase::Fault) finished_.store(true);
    }
    void event(const std::string &text) {
        if (!enabled()) return;
        std::lock_guard<std::mutex> lock(mutex_);
        if (events_.is_open()) { events_ << elapsed() << ' ' << text << '\n'; events_.flush(); }
    }
private:
    using Clock = std::chrono::steady_clock;
    long long elapsed() const { return std::chrono::duration_cast<std::chrono::milliseconds>(Clock::now()-start_).count(); }
    std::atomic<bool> enabled_{false}, finished_{false};
    std::atomic<Parking::Phase> phase_{Parking::Phase::Idle};
    Mode mode_ = Mode::Full;
    std::mutex mutex_;
    Clock::time_point start_{}, lastMotor_{};
    double leftTurns_=0, rightTurns_=0, previousLeftRps_=0, previousRightRps_=0;
    float error_=0;
    std::ofstream motorLog_, frameLog_, events_;
};
inline Runtime &runtime() { static Runtime value; return value; }
}
