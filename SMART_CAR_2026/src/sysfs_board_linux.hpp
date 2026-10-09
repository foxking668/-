#pragma once
#include "sysfs_board.hpp"
#include "capture_data.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>
namespace car2026 {
class LinuxSysfsNodes final:public SysfsNodes {
public:
    bool exists(const std::string& path) override {struct stat info{};return stat(path.c_str(),&info)==0;}
    std::string read(const std::string& path) override {
        const int fd=open(path.c_str(),O_RDONLY|O_CLOEXEC);if(fd<0) fail(path);
        char buffer[128];ssize_t size;do {size=::read(fd,buffer,sizeof(buffer));}while(size<0 && errno==EINTR);
        const int error=errno;close(fd);
        if(size<=0 || size==ssize_t(sizeof(buffer))) {errno=error;fail(path);}
        return std::string(buffer,size_t(size));
    }
    void write(const std::string& path,const std::string& text) override {
        const int fd=open(path.c_str(),O_WRONLY|O_CLOEXEC);if(fd<0) fail(path);
        ssize_t size;do {size=::write(fd,text.data(),text.size());}while(size<0 && errno==EINTR);
        const int error=errno;close(fd);
        if(size!=ssize_t(text.size())) {errno=error;fail(path);}
    }
private:
    [[noreturn]] static void fail(const std::string& path) {throw std::runtime_error("Sysfs I/O "+path+": "+std::strerror(errno));}
};
// Reference measures pulse PERIOD, not a read-and-clear count. Nonzero register
// values may persist after motion; no physical standstill/freshness is inferred.
class NewCarEncoders {
public:
    NewCarEncoders(SysfsNodes& nodes,const HardwareConfig& config):nodes_(nodes),config_(config) {
        sysfsGpio(nodes_,config_.encoder_left,false);sysfsGpio(nodes_,config_.encoder_right,false);
        const long page=sysconf(_SC_PAGESIZE);if(page<=0) throw std::runtime_error("Cannot determine MMIO page size");
        length_=size_t(page);const uint64_t address=0x1611b000;
        const uint64_t aligned=address & ~(uint64_t(page)-1);offset_=size_t(address-aligned);
        if(offset_+0x40>length_) throw std::runtime_error("Encoder registers outside mapped page");
        const int fd=open("/dev/mem",O_RDWR|O_SYNC|O_CLOEXEC);
        if(fd<0) throw std::runtime_error("Open encoder /dev/mem: "+std::string(std::strerror(errno)));
        mapping_=mmap(nullptr,length_,PROT_READ|PROT_WRITE,MAP_SHARED,fd,off_t(aligned));close(fd);
        if(mapping_==MAP_FAILED) throw std::runtime_error("Map encoder registers: "+std::string(std::strerror(errno)));
        reg(0x0c)=0x101;reg(0x3c)=0x101; // counter 0 and 3: count enable + pulse measurement
    }
    ~NewCarEncoders() {if(mapping_!=MAP_FAILED) munmap(mapping_,length_);}
    NewCarEncoders(const NewCarEncoders&)=delete;
    NewCarEncoders& operator=(const NewCarEncoders&)=delete;
    capture::Sample sample(size_t wheel) {
        capture::Sample result;
        try {
            if(wheel>1) throw std::runtime_error("Invalid encoder wheel");
            const uint32_t ticks=reg(wheel==0 ? 0x08 : 0x38);
            const auto direction=sysfsNumber(nodes_.read(wheel==0 ? config_.encoder_left : config_.encoder_right));
            result.value=newCarEncoderRps(ticks,direction)*(wheel==0 ? config_.encoder_left_sign : config_.encoder_right_sign);result.valid=true;
            result.status="ok_mmio_period_freshness_unverified";
            result.raw="period_ticks="+std::to_string(ticks)+" direction="+std::to_string(direction);
        }catch(const std::exception& error) {result.status=error.what();}
        return result;
    }
private:
    volatile uint32_t& reg(size_t offset) {return *reinterpret_cast<volatile uint32_t*>(static_cast<char*>(mapping_)+offset_+offset);}
    SysfsNodes& nodes_;HardwareConfig config_;void* mapping_=MAP_FAILED;size_t length_=0,offset_=0;
};
}
