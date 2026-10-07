#pragma once
#include "servo_output.hpp"
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <optional>
#include <thread>
namespace car2026 { namespace capture {
inline int64_t rehearsalMonotonicNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
struct ServoHoldEvent {
    int stage;unsigned revision,trial;int64_t beginNs,endNs;
    std::string error;std::string event="AUTO_CENTER";
    unsigned segment=0;
};
// The worker can only write zero. Commands require main-thread Enter or validated file-save authorization.
// Serializes set/zero/close; camera or terminal delays cannot postpone the zero deadline indefinitely.
class RehearsalServo {
public:
    RehearsalServo(DeviceIo& io,const HardwareConfig& config,const Params& params)
        :output_(io,config,params),worker_([this] {watch();}) {}
    ~RehearsalServo() noexcept {try {close();}catch(const std::exception&) {}}
    RehearsalServo(const RehearsalServo&)=delete;
    RehearsalServo& operator=(const RehearsalServo&)=delete;
    uint16_t set(double command) {
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_) throw std::runtime_error("Timed servo closed");
        deadline_.reset();++generation_;wake_.notify_all();
        const auto duty=output_.set(command);written_=command;return duty;
    }
    std::optional<int64_t> beginHold(double seconds,int stage,unsigned revision,unsigned trial,unsigned segment=0) {
        if(!std::isfinite(seconds) || seconds<0 || seconds>120 || stage<0 || stage>5 || segment>1)
            throw std::runtime_error("Invalid servo hold request");
        std::lock_guard<std::mutex> held(mutex_);
        if(closed_) throw std::runtime_error("Timed servo closed");
        if(!written_) throw std::runtime_error("Cannot arm hold before authorized write");
        ++generation_;deadline_.reset();std::optional<int64_t> start;
        if(seconds>0) {
            context_={stage,revision,trial,0,0,""};
            context_.segment=segment;
            const auto now=std::chrono::steady_clock::now();
            start=std::chrono::duration_cast<std::chrono::nanoseconds>(now.time_since_epoch()).count();
            deadline_=now+std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(seconds));
        }
        wake_.notify_all();return start;
    }
    bool attempted() const {std::lock_guard<std::mutex> held(mutex_);return output_.commandAttempted();}
    std::optional<double> written() const {std::lock_guard<std::mutex> held(mutex_);return written_;}
    double remaining() const {
        std::lock_guard<std::mutex> held(mutex_);
        return deadline_ ? std::max(0.,std::chrono::duration<double>(*deadline_-std::chrono::steady_clock::now()).count()) : 0;
    }
    std::deque<ServoHoldEvent> takeEvents() {
        std::lock_guard<std::mutex> held(mutex_);std::deque<ServoHoldEvent> result;result.swap(events_);return result;
    }
    void close() {
        {
            std::lock_guard<std::mutex> held(mutex_);
            if(closed_) return;
            closed_=true;deadline_.reset();++generation_;wake_.notify_all();
        }
        if(worker_.joinable()) worker_.join();
        // No worker remains when the normal exit zero is written.
        output_.close();
    }
private:
    void watch() noexcept {
        std::unique_lock<std::mutex> held(mutex_);
        while(!closed_) {
            if(!deadline_) {wake_.wait(held,[this] {return closed_ || deadline_.has_value();});continue;}
            const auto generation=generation_;const auto deadline=*deadline_;
            if(wake_.wait_until(held,deadline,[&] {return closed_ || generation_!=generation;})) continue;
            deadline_.reset();auto event=context_;event.beginNs=rehearsalMonotonicNs();
            try {output_.set(0);written_=0;}
            catch(const std::exception& error) {event.error=error.what();}
            event.endNs=rehearsalMonotonicNs();events_.push_back(event);
        }
    }
    ServoOutput output_;mutable std::mutex mutex_;std::condition_variable wake_;
    bool closed_=false;unsigned generation_=0;std::optional<std::chrono::steady_clock::time_point> deadline_;
    std::optional<double> written_;ServoHoldEvent context_{};std::deque<ServoHoldEvent> events_;
    std::thread worker_; // Last: all state is initialized before the worker starts.
};
}}
