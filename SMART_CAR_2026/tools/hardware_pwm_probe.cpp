#include "hardware_linux.hpp"
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

bool probe(const std::string& path) {
    bool conventional = true;
    std::cout << "DEVICE " << path << '\n';
    // Reopen for each sample, just as the vehicle's I/O layer does. Complementary
    // byte patterns expose unchanged data even when a driver returns zero.
    for (int sample = 0; sample < 2; ++sample) {
        std::array<unsigned char, sizeof(car2026::PwmInfo)> buffer{}, initial{};
        for (size_t i = 0; i < buffer.size(); ++i) {
            const auto pattern = static_cast<unsigned char>(0xa5u ^ (i * 37u));
            initial[i] = sample == 0 ? pattern : static_cast<unsigned char>(~pattern);
        }
        buffer = initial;
        ReadOnlyDevice device(path);
        const auto result = device.readOnce(buffer.data(), buffer.size());
        size_t changed = 0;
        for (size_t i = 0; i < buffer.size(); ++i) changed += buffer[i] != initial[i];
        std::cout << "  sample=" << sample + 1 << " requested=" << buffer.size()
                  << " returned=" << result << " changed_bytes=" << changed << '\n';
        std::cout << "  raw=";
        for (auto byte : buffer)
            std::cout << std::hex << std::setfill('0') << std::setw(2) << unsigned(byte) << ' ';
        std::cout << std::dec << std::setfill(' ') << '\n';
        car2026::PwmInfo info{};
        std::memcpy(&info, buffer.data(), sizeof(info));
        const bool plausible = info.freq > 0 && info.duty_max > 0 &&
                               info.duty_max <= 65535 && info.duty <= info.duty_max;
        std::cout << "  candidate freq=" << info.freq << " duty=" << info.duty
                  << " duty_max=" << info.duty_max << " duty_ns=" << info.duty_ns
                  << " period_ns=" << info.period_ns << " clk_freq=" << info.clk_freq
                  << " basic_ranges=" << (plausible ? "plausible" : "invalid") << '\n';
        if (result != static_cast<ssize_t>(buffer.size()) || !plausible) conventional = false;
    }
    return conventional;
}
}

int main(int argc, char** argv) {
    try {
        std::string configPath = "config/hardware.ini";
        if (argc == 3 && std::string(argv[1]) == "--hardware-config") configPath = argv[2];
        else if (argc != 1) {
            std::cerr << "Usage: hardware_pwm_probe [--hardware-config config/hardware.ini]\n";
            return 1;
        }
        const auto config = car2026::HardwareConfig::load(configPath);
        car2026::HardwareLock lock;
        std::cout << "Read-only PWM probe: no motor/servo/GPIO writes.\n"
                     "Candidate fields are diagnostic evidence, not validated device data.\n";
        bool conventional = true;
        for (const auto& path : {config.motor_left_pwm, config.motor_right_pwm, config.servo_pwm}) {
            try { if (!probe(path)) conventional = false; }
            catch (const std::exception& error) {
                conventional = false;
                std::cerr << "  [PROBE ERROR] " << path << ": " << error.what() << '\n';
            }
        }
        std::cout << "Exit 2 indicates nonstandard/failed reads; no compatibility mode is enabled.\n";
        return conventional ? 0 : 2;
    } catch (const std::exception& error) {
        std::cerr << "[PROBE ERROR] " << error.what() << '\n';
        return 1;
    }
}
