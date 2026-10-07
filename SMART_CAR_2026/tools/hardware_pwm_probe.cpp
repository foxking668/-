#include "hardware_linux.hpp"
#include "device_read_buffer.hpp"
#include <array>
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iomanip>
#include <iostream>
#include <unistd.h>

namespace {
// Deliberately inspect raw syscall results instead of bypassing LinuxDeviceIo's
// strict transfer checks. This diagnostic never writes to hardware devices.
class ReadOnlyDevice {
public:
    explicit ReadOnlyDevice(const std::string& path)
        : fd_(open(path.c_str(), O_RDONLY | O_CLOEXEC)) {
        if (fd_ < 0) throw std::runtime_error("Open " + path + ": " + std::strerror(errno));
    }
    ~ReadOnlyDevice() { close(fd_); }
    ReadOnlyDevice(const ReadOnlyDevice&) = delete;
    ReadOnlyDevice& operator=(const ReadOnlyDevice&) = delete;
    ssize_t readOnce(void* data, size_t size) {
        ssize_t result;
        do { result = read(fd_, data, size); } while (result < 0 && errno == EINTR);
        if (result < 0) throw std::runtime_error(std::string("Read: ") + std::strerror(errno));
        return result;
    }
private:
    int fd_;
};

template<size_t Bytes> struct DiagnosticSample {
    std::array<unsigned char, Bytes> buffer{};
    ssize_t returnedBytes = -1;
    bool guardDamaged = false;
};

template<class Bytes> void printHex(const char* label,const Bytes& bytes) {
    std::cout << "  " << label << '=';
    for(auto byte : bytes)
        std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte) << ' ';
    std::cout << std::dec << std::setfill(' ') << '\n';
}

template<size_t Bytes> DiagnosticSample<Bytes> readSample(const std::string& path, int sample) {
    DiagnosticSample<Bytes> result;
    std::array<unsigned char, Bytes> initial{};
    for (size_t i = 0; i < Bytes; ++i) {
        const auto pattern = static_cast<unsigned char>(0xa5u ^ (i * 37u));
        initial[i] = sample == 0 ? pattern : static_cast<unsigned char>(~pattern);
    }
    // A v2 exact-size buffer hid overwrites. Preserve the original requested
    // width, but reserve padding and report changes instead of trusting it.
    using Storage=car2026::GuardedDeviceRead<decltype(initial)>;
    Storage storage(initial,sample==0?0xa5:0x5a);
    const auto beforeInitial=storage.paddingBefore(),afterInitial=storage.paddingAfter();
    // Reopen per sample to match the controller. Never retry a positive short
    // transfer: reads of delta encoders may consume/reset the count.
    ReadOnlyDevice device(path);
    result.returnedBytes = device.readOnce(storage.data(), Bytes);
    std::memcpy(result.buffer.data(),storage.data(),Bytes);
    const auto before=storage.paddingBefore(),after=storage.paddingAfter();
    result.guardDamaged=storage.guardDamaged();
    size_t changedBefore=0,changedAfter=0,observedEnd=0;
    for(size_t i=0;i<Storage::guardBytes;++i) {
        changedBefore+=before[i]!=beforeInitial[i];
        if(after[i]!=afterInitial[i]) {++changedAfter;observedEnd=Bytes+i+1;}
    }
    size_t changed = 0;
    for (size_t i = 0; i < Bytes; ++i) changed += result.buffer[i] != initial[i];
    std::cout << "  sample=" << sample + 1 << " requested=" << Bytes
              << " returned=" << result.returnedBytes << " changed_bytes=" << changed << '\n';
    printHex("raw",result.buffer);
    std::cout << "  changed_before=" << changedBefore << " changed_after=" << changedAfter
              << " observed_end_offset=" << observedEnd << " (exclusive; lower bound, not proven ABI width)\n";
    printHex("raw_after",after);
    if(changedBefore) printHex("raw_before",before);
    if(result.guardDamaged) std::cout << "  [OVERWRITE] bytes outside requested payload changed; do not run vehicle controller.\n";
    return result;
}

bool probe(const std::string& path) {
    bool conventional = true;
    std::cout << "DEVICE " << path << '\n';
    for (int sample = 0; sample < 2; ++sample) {
        const auto result = readSample<sizeof(car2026::PwmInfo)>(path, sample);
        car2026::PwmInfo info{};
        std::memcpy(&info, result.buffer.data(), sizeof(info));
        const bool plausible = info.freq > 0 && info.duty_max > 0 &&
                               info.duty_max <= 65535 && info.duty <= info.duty_max;
        std::cout << "  candidate freq=" << info.freq << " duty=" << info.duty
                  << " duty_max=" << info.duty_max << " duty_ns=" << info.duty_ns
                  << " period_ns=" << info.period_ns << " clk_freq=" << info.clk_freq
                  << " basic_ranges=" << (plausible ? "plausible" : "invalid") << '\n';
        if (result.returnedBytes != static_cast<ssize_t>(sizeof(info)) || !plausible || result.guardDamaged) conventional = false;
    }
    return conventional;
}

bool probeEncoder(const std::string& path) {
    bool conventional = true;
    std::cout << "ENCODER " << path << '\n';
    for (int sample = 0; sample < 2; ++sample) {
        const auto result = readSample<sizeof(car2026::EncoderCount)>(path, sample);
        car2026::EncoderCount count = 0;
        std::memcpy(&count, result.buffer.data(), sizeof(count));
        std::cout << "  candidate count=" << count << " (raw signed int32; not wheel speed)\n";
        if (result.returnedBytes != static_cast<ssize_t>(sizeof(count)) || result.guardDamaged) conventional = false;
    }
    return conventional;
}

bool probeGpio(const std::string& path) {
    bool conventional = true;
    std::cout << "GPIO " << path << '\n';
    for (int sample = 0; sample < 2; ++sample) {
        const auto result = readSample<1>(path, sample);
        const auto byte = result.buffer[0];
        const bool valid = byte == 0 || byte == 1 || byte == '0' || byte == '1';
        std::cout << "  candidate byte=" << unsigned(byte);
        if (valid) std::cout << " level=" << unsigned(byte == '0' || byte == '1' ? byte - '0' : byte);
        std::cout << " value_range=" << (valid ? "plausible" : "invalid") << '\n';
        if (result.returnedBytes != 1 || !valid || result.guardDamaged) conventional = false;
    }
    return conventional;
}
}

int main(int argc, char** argv) {
    try {
        std::string configPath = "config/hardware.ini";
        bool inputs = false, configSeen = false;
        for (int i = 1; i < argc; ++i) {
            const std::string arg = argv[i];
            if (arg == "--inputs" && !inputs) inputs = true;
            else if (arg == "--hardware-config" && !configSeen && i + 1 < argc && argv[i + 1][0] != '-') {
                configPath = argv[++i];configSeen = true;
            } else {
                std::cerr << "Usage: hardware_pwm_probe [--hardware-config config/hardware.ini] [--inputs]\n";
                return 1;
            }
        }
        const auto config = car2026::HardwareConfig::load(configPath);
        car2026::HardwareLock lock;
        std::cout << "Read-only PWM/input probe v4: no motor/servo/GPIO writes.\n"
                     "Encoder requests are now 4 bytes; verify raw data and unchanged padding before driving.\n"
                     "32-byte padding on each side detects limited overwrites, not arbitrary kernel corruption.\n"
                     "Candidate fields are diagnostic evidence, not validated device data.\n";
        if (inputs) std::cout << "Encoder reads may consume/reset counts. Keep wheels stationary; do not run a vehicle controller.\n";
        bool conventional = true;
        auto inspect = [&](const std::string& path, auto operation) {
            try { if (!operation(path)) conventional = false; }
            catch (const std::exception& error) {
                conventional = false;
                std::cerr << "  [PROBE ERROR] " << path << ": " << error.what() << '\n';
            }
        };
        inspect(config.motor_left_pwm,probe);inspect(config.motor_right_pwm,probe);inspect(config.servo_pwm,probe);
        if (inputs) {
            inspect(config.encoder_left,probeEncoder);inspect(config.encoder_right,probeEncoder);
            inspect(config.motor_left_dir,probeGpio);inspect(config.motor_right_dir,probeGpio);
            if (config.board_inputs_enabled) {
                for (int i = 0; i < 4; ++i) inspect(config.key_prefix + std::to_string(i), probeGpio);
                for (int i = 0; i < 2; ++i) inspect(config.switch_prefix + std::to_string(i), probeGpio);
            }
            if (config.beep_enabled) inspect(config.beep_device, probeGpio);
        }
        std::cout << "Exit 2 indicates nonstandard/failed reads; no compatibility mode is enabled.\n";
        return conventional ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "[PROBE ERROR] " << error.what() << '\n';
        return 1;
    }
}
