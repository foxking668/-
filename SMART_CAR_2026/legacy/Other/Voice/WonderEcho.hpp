#pragma once

#include <cstdint>

namespace Other
{
    /*
     * 语音播报模块。
     *
     * 使用方式：
     * 1. 程序初始化时调用 WonderEchoInit()；
     * 2. 检测到事件时调用对应播报函数；
     * 3. 程序退出时调用 WonderEchoDestroy()。
     *
     * 当前已接入：
     * - 人行横道 / 斑马线播报：WonderEchoSpeakZebra()
     *
     * 底层协议来自你提供的 wonderEcho 示例工程：
     * - I2C 总线：/dev/i2c-2
     * - 设备地址：0x34
     * - 写入寄存器：0x6e
     * - 斑马线播报命令：0xFF 0x11
     */
    int WonderEchoInit();
    void WonderEchoDestroy();
    bool WonderEchoIsReady();

    int WonderEchoSend(uint8_t d1, uint8_t d2);

    /* 人行横道 / 斑马线播报。 */
    int WonderEchoSpeakZebra();
}
