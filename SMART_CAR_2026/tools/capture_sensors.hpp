#pragma once
#include "capture_data.hpp"
#include "hardware.hpp"
#include "device_read_buffer.hpp"
#include <cerrno>
#include <csignal>
#include <cstring>
#include <fcntl.h>
#include <limits>
#include <unistd.h>
namespace car2026 { namespace capture {
inline void fillEncoderPaths(Config& config,const car2026::HardwareConfig& hardware) {
    // Only these verified interfaces have defaults. No guessed GPIO pin numbers.
    if(config.sensors[0].path.empty()) config.sensors[0].path=hardware.encoder_left;
    if(config.sensors[1].path.empty()) config.sensors[1].path=hardware.encoder_right;
    if(config.factoryEncoderStatusZero &&
       (config.sensors[0].path!=hardware.encoder_left || config.sensors[1].path!=hardware.encoder_right))
        throw std::runtime_error("Factory status-zero reads are restricted to configured board encoder paths");
}

inline Sample readSensor(const SensorSpec& sensor,bool factoryEncoderStatusZero,const volatile std::sig_atomic_t& interrupted) {
    using car2026::capture::Sample;
    if(sensor.path.empty()) {Sample result;result.status="unconfigured";return result;}
    // Only sensor inputs are opened, read-only. No GPIO export/direction writes.
    const int fd=open(sensor.path.c_str(),O_RDONLY|O_NONBLOCK|O_CLOEXEC);
    if(fd<0) {Sample result;result.status="open_failed:"+std::string(std::strerror(errno));return result;}
    struct ReadFd {int value;~ReadFd() {if(value>=0) close(value);}} held{fd};
    const size_t size=car2026::capture::readBufferSize(sensor);
    std::vector<uint8_t> bytes(size,0xa5);ssize_t count;
    const bool factoryEncoder=factoryEncoderStatusZero &&
        (sensor.name=="encoder_left" || sensor.name=="encoder_right");
    car2026::GuardedDeviceRead<car2026::EncoderCount> guardedEncoder(
        std::numeric_limits<car2026::EncoderCount>::min());
    void* destination=factoryEncoder ? guardedEncoder.data() : bytes.data();
    // Never finish a short transfer with a second read: delta counters may clear.
    do {count=read(fd,destination,bytes.size());} while(count<0 && errno==EINTR && !interrupted);
    const int error=errno;close(fd);held.value=-1;
    if(factoryEncoder) {
        std::memcpy(bytes.data(),guardedEncoder.data(),bytes.size());
        if(guardedEncoder.guardDamaged()) throw std::runtime_error("Encoder payload guard damaged: "+sensor.path);
    }
    auto result=car2026::capture::decode(sensor,long(factoryEncoder && count==0 ? size : count),bytes);
    result.returned=long(count); // Always preserve the actual syscall result.
    if(factoryEncoder && (count==0 || count==ssize_t(sizeof(car2026::EncoderCount)))) {
        result.value=car2026::decodeEncoderRead(guardedEncoder.value(sensor.path),count,sensor.path);
        if(count==0) result.status="ok_factory_status_zero";
    }
    if(count<0) result.status="read_failed:"+std::string(std::strerror(error));
    return result;
}
}}
