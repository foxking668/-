#pragma once
#include "hardware.hpp"
#include "imu.hpp"
#include <memory>

namespace car2026 {
class LinuxDeviceIo final : public DeviceIo {
public:
    void readBinary(const std::string&,void*,size_t) override;
    void writeBinary(const std::string&,const void*,size_t) override;
    std::string readText(const std::string&) override;
};
class HardwareLock {
public:
    HardwareLock();
    ~HardwareLock();
    HardwareLock(const HardwareLock&)=delete;
    HardwareLock& operator=(const HardwareLock&)=delete;
private: int fd_=-1;
};
class HardwareImu {
public:
    HardwareImu(DeviceIo&,const HardwareConfig&,const Params&);
    ~HardwareImu();
    HardwareImu(const HardwareImu&)=delete;
    HardwareImu& operator=(const HardwareImu&)=delete;
    void poll(double now,bool stationary);
    bool valid(double now) const;
    double yaw() const;
private:
    int fd_=-1;ImuParser parser_;YawTracker tracker_;double sign_=1;
    std::unique_ptr<IioYaw> iio_;
};
class VoiceOutput {
public:
    explicit VoiceOutput(const HardwareConfig&);
    ~VoiceOutput();
    VoiceOutput(const VoiceOutput&)=delete;
    VoiceOutput& operator=(const VoiceOutput&)=delete;
    bool speakZebra();
private: int fd_=-1;
};
void inspectHardware(DeviceIo&,const HardwareConfig&,std::ostream&);
} // namespace car2026
