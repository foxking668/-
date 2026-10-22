#pragma once
#include "ParkingController.hpp"
#include <mutex>

namespace Parking {
// The lock covers both ownership and the actual write, closing the handover race.
class SteeringAuthority {
public:
    template<class Write> bool normal(Write write) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(owned_) return false;
        write(); return true;
    }
    template<class Write> void parking(Write write) {
        std::lock_guard<std::mutex> lock(mutex_);
        owned_ = true; write();
    }
    void reset() { std::lock_guard<std::mutex> lock(mutex_); owned_ = false; }
private:
    std::mutex mutex_;
    bool owned_ = false;
};
inline SteeringAuthority &steering() { static SteeringAuthority value; return value; }

struct Request { Config config; std::int64_t holdUntilMs = 0; };
class Mailbox {
public:
    bool submit(int completedLaps, const Request &request) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!isSecondLap(completedLaps) || submitted_) return false;
        request_ = request; submitted_ = pending_ = true; return true;
    }
    bool take(Request &out) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(!pending_) return false;
        out = request_; pending_ = false; return true;
    }
    bool submitted() { std::lock_guard<std::mutex> lock(mutex_); return submitted_; }
    void reset() { std::lock_guard<std::mutex> lock(mutex_); submitted_ = pending_ = false; }
private:
    std::mutex mutex_;
    Request request_;
    bool submitted_ = false, pending_ = false;
};
inline Mailbox &mailbox() { static Mailbox value; return value; }

struct ZebraEvent {
    bool triggered = false;
    bool voiceAllowed = false;
};
// Called by the image thread, which also submits the parking request after
// detection. Capture voice permission now: the legitimate triggering event may
// submit parking before its caller sends speech, but later frames stay blocked.
template<class Detector>
ZebraEvent detectZebraEvent(Mailbox &requests, bool sceneAllowed, Detector detect) {
    if (requests.submitted() || !sceneAllowed) return {};
    const bool triggered = detect();
    return {triggered, triggered};
}
}
