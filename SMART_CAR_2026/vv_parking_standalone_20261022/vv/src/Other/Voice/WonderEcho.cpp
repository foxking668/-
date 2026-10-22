#include "WonderEcho.hpp"

#include <fcntl.h>
#include <iostream>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <mutex>
#include <sys/ioctl.h>
#include <unistd.h>

namespace Other
{
    namespace
    {
        std::mutex gWonderEchoMutex;
        int gWonderEchoFd = -1;
        bool gWonderEchoInitTried = false;

        constexpr const char *WONDER_ECHO_I2C_DEVICE = "/dev/i2c-2";
        constexpr int WONDER_ECHO_I2C_ADDR = 0x34;
        constexpr uint8_t WONDER_ECHO_WRITE_REG = 0x6e;

        int i2cWriteBlockData(int file, uint8_t reg, uint8_t length, const uint8_t *values)
        {
            if (file < 0)
            {
                return -1;
            }

            if (length > I2C_SMBUS_BLOCK_MAX)
            {
                return -1;
            }

            union i2c_smbus_data data;
            struct i2c_smbus_ioctl_data args;

            for (int i = 0; i < length; i++)
            {
                data.block[i + 1] = values[i];
            }
            data.block[0] = length;

            args.read_write = I2C_SMBUS_WRITE;
            args.command = reg;
            args.size = I2C_SMBUS_I2C_BLOCK_DATA;
            args.data = &data;

            if (ioctl(file, I2C_SMBUS, &args) == -1)
            {
                return -1;
            }

            return 0;
        }
    }

    int WonderEchoInit()
    {
        std::lock_guard<std::mutex> lock(gWonderEchoMutex);

        if (gWonderEchoFd >= 0)
        {
            return gWonderEchoFd;
        }

        gWonderEchoInitTried = true;

        int file = open(WONDER_ECHO_I2C_DEVICE, O_RDWR);
        if (file < 0)
        {
    /* quiet: WonderEcho open failed */
            return -1;
        }

        if (ioctl(file, I2C_SLAVE, WONDER_ECHO_I2C_ADDR) < 0)
        {
            /* quiet: WonderEcho slave acquire failed */
            close(file);
            return -1;
        }

        gWonderEchoFd = file;
/* quiet: WonderEcho init success */
        return gWonderEchoFd;
    }

    void WonderEchoDestroy()
    {
        std::lock_guard<std::mutex> lock(gWonderEchoMutex);

        if (gWonderEchoFd >= 0)
        {
            close(gWonderEchoFd);
            gWonderEchoFd = -1;
/* quiet: WonderEcho closed */
        }
    }

    bool WonderEchoIsReady()
    {
        std::lock_guard<std::mutex> lock(gWonderEchoMutex);
        return gWonderEchoFd >= 0;
    }

    int WonderEchoSend(uint8_t d1, uint8_t d2)
    {
        /*
         * 兼容你提供的示例工程：
         * 第一个字节必须是 0x00 或 0xFF。
         */
        if (d1 != 0x00 && d1 != 0xFF)
        {
    /* quiet: WonderEcho first byte invalid */
            return -1;
        }

        /*
         * 如果初始化失败过，后续仍允许懒加载重试；
         * 这样即使设备节点稍晚出现，也有机会恢复。
         */
        if (!WonderEchoIsReady())
        {
            if (WonderEchoInit() < 0)
            {
                return -1;
            }
        }

        std::lock_guard<std::mutex> lock(gWonderEchoMutex);

        uint8_t data[2] = {d1, d2};
        int ret = i2cWriteBlockData(gWonderEchoFd, WONDER_ECHO_WRITE_REG, sizeof(data), data);
        if (ret < 0)
        {
    /* quiet: WonderEcho write failed */
            return -1;
        }

        return 0;
    }

    int WonderEchoSpeakZebra()
    {
        /*
         * 你提供的旧工程里，检测到斑马线时调用：
         *     wonderEchoSend(0xFF, 0x11);
         */
        int ret = WonderEchoSend(0xFF, 0x11);
        if (ret == 0)
        {
    /* quiet: WonderEcho speak zebra */
        }
        return ret;
    }
}
