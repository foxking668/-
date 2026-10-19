#include "JY62.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <sys/select.h>
#include <termios.h>
#include <unistd.h>

namespace Contral
{
    namespace
    {
        speed_t baudToTermios(int baudRate)
        {
            switch (baudRate)
            {
            case 115200:
                return B115200;
            case 57600:
                return B57600;
            case 38400:
                return B38400;
            case 19200:
                return B19200;
            case 9600:
            default:
                return B9600;
            }
        }

        short decodeSigned16(unsigned char low, unsigned char high)
        {
            const unsigned short value = static_cast<unsigned short>(low) |
                                        (static_cast<unsigned short>(high) << 8U);
            return static_cast<short>(value);
        }

        double decodeAngle(unsigned char low, unsigned char high)
        {
            return static_cast<double>(decodeSigned16(low, high)) / 32768.0 * 180.0;
        }
    }

    JY62AngleSensor::JY62AngleSensor()
        : fd_(-1), receivedBytes_(0), validAngleFrames_(0), checksumErrors_(0)
    {
    }

    JY62AngleSensor::~JY62AngleSensor()
    {
        closeDevice();
    }

    bool JY62AngleSensor::configurePort(int baudRate)
    {
        if (fd_ < 0)
        {
            return false;
        }

        termios options{};
        if (tcgetattr(fd_, &options) != 0)
        {
            return false;
        }

        cfmakeraw(&options);
        options.c_cflag |= CLOCAL | CREAD;
        options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);
        options.c_cflag &= static_cast<tcflag_t>(~PARENB);
        options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
        options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
        options.c_cflag |= CS8;
        options.c_cc[VMIN] = 0;
        options.c_cc[VTIME] = 0;

        const speed_t speed = baudToTermios(baudRate);
        if (cfsetispeed(&options, speed) != 0 || cfsetospeed(&options, speed) != 0)
        {
            return false;
        }
        if (tcsetattr(fd_, TCSANOW, &options) != 0)
        {
            return false;
        }

        tcflush(fd_, TCIFLUSH);
        return true;
    }

    bool JY62AngleSensor::openDevice(const std::string &device, int baudRate)
    {
        closeDevice();

        fd_ = open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
        if (fd_ < 0)
        {
            return false;
        }

        device_ = device;
        rxBuffer_.clear();
        receivedBytes_ = 0;
        validAngleFrames_ = 0;
        checksumErrors_ = 0;

        if (!configurePort(baudRate))
        {
            closeDevice();
            return false;
        }
        return true;
    }

    void JY62AngleSensor::closeDevice()
    {
        if (fd_ >= 0)
        {
            close(fd_);
        }
        fd_ = -1;
        device_.clear();
        rxBuffer_.clear();
    }

    bool JY62AngleSensor::isOpen() const
    {
        return fd_ >= 0;
    }

    bool JY62AngleSensor::parseBufferedAngle(JY62AngleData &data)
    {
        constexpr unsigned long frameSize = 11;

        while (rxBuffer_.size() >= frameSize)
        {
            auto header = std::find(rxBuffer_.begin(), rxBuffer_.end(), static_cast<unsigned char>(0x55));
            if (header != rxBuffer_.begin())
            {
                rxBuffer_.erase(rxBuffer_.begin(), header);
                if (rxBuffer_.size() < frameSize)
                {
                    return false;
                }
            }

            unsigned char checksum = 0;
            for (unsigned long i = 0; i < 10; ++i)
            {
                checksum = static_cast<unsigned char>(checksum + rxBuffer_[i]);
            }

            if (checksum != rxBuffer_[10])
            {
                ++checksumErrors_;
                rxBuffer_.erase(rxBuffer_.begin());
                continue;
            }

            const unsigned char frameType = rxBuffer_[1];
            if (frameType == 0x53)
            {
                data.roll = decodeAngle(rxBuffer_[2], rxBuffer_[3]);
                data.pitch = decodeAngle(rxBuffer_[4], rxBuffer_[5]);
                data.yaw = decodeAngle(rxBuffer_[6], rxBuffer_[7]);
                ++validAngleFrames_;
                rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.begin() + frameSize);
                return true;
            }

            // 0x51/0x52 等其他合法帧不是本测试需要的角度帧，直接跳过。
            rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.begin() + frameSize);
        }

        return false;
    }

    bool JY62AngleSensor::readAngle(JY62AngleData &data, int timeoutMs)
    {
        if (fd_ < 0)
        {
            return false;
        }

        if (parseBufferedAngle(data))
        {
            return true;
        }

        if (timeoutMs < 0)
        {
            timeoutMs = 0;
        }

        fd_set readSet;
        FD_ZERO(&readSet);
        FD_SET(fd_, &readSet);

        timeval timeout{};
        timeout.tv_sec = timeoutMs / 1000;
        timeout.tv_usec = (timeoutMs % 1000) * 1000;

        const int selectResult = select(fd_ + 1, &readSet, nullptr, nullptr, &timeout);
        if (selectResult <= 0)
        {
            return false;
        }

        unsigned char temp[256];
        while (true)
        {
            const ssize_t count = read(fd_, temp, sizeof(temp));
            if (count > 0)
            {
                receivedBytes_ += static_cast<std::uint64_t>(count);
                rxBuffer_.insert(rxBuffer_.end(), temp, temp + count);
                if (rxBuffer_.size() > 4096)
                {
                    rxBuffer_.erase(rxBuffer_.begin(), rxBuffer_.end() - 1024);
                }
            }
            else
            {
                if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK && errno != EINTR)
                {
                    return false;
                }
                break;
            }
        }

        return parseBufferedAngle(data);
    }

    const std::string &JY62AngleSensor::device() const
    {
        return device_;
    }

    std::uint64_t JY62AngleSensor::receivedBytes() const
    {
        return receivedBytes_;
    }

    std::uint64_t JY62AngleSensor::validAngleFrames() const
    {
        return validAngleFrames_;
    }

    std::uint64_t JY62AngleSensor::checksumErrors() const
    {
        return checksumErrors_;
    }
}
