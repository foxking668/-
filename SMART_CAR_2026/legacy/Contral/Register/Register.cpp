#include "Register.hpp"

namespace Contral
{
    /*
     * 将物理地址映射为可读写虚拟地址。
     *
     * 该函数通过 `/dev/mem` 直接访问硬件寄存器，因此通常需要 root 权限。
     * 返回值会自动偏移到目标物理地址在页内的实际位置。
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
    void *mapRegister(uint32_t physical_address, size_t size)
    {
        int mem_fd = open("/dev/mem", O_RDWR | O_SYNC);
        if (mem_fd == -1)
        {
            perror("Failed to open /dev/mem");
            throw std::runtime_error("Cannot open /dev/mem");
        }

        void *mapped_addr = mmap(NULL, size, PROT_READ | PROT_WRITE, MAP_SHARED, mem_fd, physical_address & ~(ENCODER_MAP_BYTES - 1));
        if (mapped_addr == MAP_FAILED)
        {
            perror("Failed to map memory");
            close(mem_fd);
            throw std::runtime_error("Cannot map encoder registers");
        }

        close(mem_fd);

        return (void *)((uintptr_t)mapped_addr + (physical_address & (ENCODER_MAP_BYTES - 1)));
    }
}
