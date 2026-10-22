#pragma once

#include <cstdint>
#include <string>
#include <vector>
#include <sys/types.h>

namespace Contral
{
    struct JY62AngleData
    {
        double roll = 0.0;
        double pitch = 0.0;
        double yaw = 0.0;
    };

    /*
     * JY62 串口角度读取器。
     *
     * 本车JY62参考程序使用 115200bps、8N1，并以 0x55 0x53 开头的 11 字节数据帧
     * 输出 Roll / Pitch / Yaw。该类只负责读取、校验和解析角度帧，
     * 不会控制电机、舵机或修改正常比赛状态机。
     */
    class JY62AngleSensor
    {
    private:
        int fd_;
        std::string device_;
        std::vector<unsigned char> rxBuffer_;
        std::uint64_t receivedBytes_;
        std::uint64_t validAngleFrames_;
        std::uint64_t checksumErrors_;

        bool configurePort(int baudRate);
        bool parseBufferedAngle(JY62AngleData &data);

    public:
        JY62AngleSensor();
        ~JY62AngleSensor();

        JY62AngleSensor(const JY62AngleSensor &) = delete;
        JY62AngleSensor &operator=(const JY62AngleSensor &) = delete;

        bool openDevice(const std::string &device, int baudRate = 115200);
        void closeDevice();
        bool isOpen() const;

        /*
         * 等待并读取一帧通过校验的 0x55 0x53 角度帧。
         * timeoutMs=0 时只检查已有缓存和当前可读数据。
         */
        bool readAngle(JY62AngleData &data, int timeoutMs);

        const std::string &device() const;
        std::uint64_t receivedBytes() const;
        std::uint64_t validAngleFrames() const;
        std::uint64_t checksumErrors() const;
    };
}
