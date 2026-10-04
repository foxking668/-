#pragma once

#include "headfile.hpp"

namespace Contral
{
    /*
     * 物理寄存器映射工具函数。
     *
     * 用于将 SoC 外设寄存器地址映射到用户态虚拟地址空间，
     * 以便直接访问底层定时器、PWM 等硬件寄存器。
     */
    /**
     * @brief 映射物理寄存器地址
     *
     * @details
     * 通过 /dev/mem 将指定物理地址映射到进程虚拟地址空间，供后续直接读写硬件寄存器使用。
     *
     * @param physical_address 需要映射的物理寄存器地址。
     * @param size 映射区域大小或缓存大小。
     */
    void *mapRegister(uint32_t physical_address, size_t size);

}