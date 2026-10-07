#include "hardware_linux.hpp"
#include "device_read_buffer.hpp"
#include <cerrno>
#include <algorithm>
#include <cstring>
#include <dirent.h>
#include <fcntl.h>
#include <iostream>
#include <limits>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/file.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>

namespace car2026 {
namespace {
class FileDescriptor {
public:
    FileDescriptor(const std::string& path,int flags) : fd_(open(path.c_str(),flags|O_CLOEXEC)) {
        if(fd_<0) throw std::runtime_error("Open '"+path+"': "+std::strerror(errno));
    }
    ~FileDescriptor() {close(fd_);}
    int get() const {return fd_;}
    FileDescriptor(const FileDescriptor&)=delete;
    FileDescriptor& operator=(const FileDescriptor&)=delete;
private:int fd_;
};
ssize_t transferOnce(const std::string& path,void* data,size_t bytes,bool writing) {
    if(path.empty()) throw std::runtime_error("Empty device path before "+std::string(writing?"write":"read"));
    FileDescriptor fd(path,writing?O_WRONLY:O_RDONLY);ssize_t count;
    do {count=writing ? write(fd.get(),data,bytes) : read(fd.get(),data,bytes);} while(count<0 && errno==EINTR);
    if(count<0) throw std::runtime_error("I/O "+path+": "+std::strerror(errno));
    return count;
}
void exactTransfer(const std::string& path,void* data,size_t bytes,bool writing) {
    const auto count=transferOnce(path,data,bytes,writing);
    // Do not complete a short transfer with another read: encoder reads may clear counters.
    if(size_t(count)!=bytes) throw std::runtime_error("Short device transfer "+path+": expected "+std::to_string(bytes)+", got "+std::to_string(count));
}
}
LinuxDeviceIo::LinuxDeviceIo(const HardwareConfig& config) : config_(config) {config_.validate();}
void LinuxDeviceIo::readBinary(const std::string& path,void* data,size_t bytes) {exactTransfer(path,data,bytes,false);}
void LinuxDeviceIo::writeBinary(const std::string& path,const void* data,size_t bytes) {exactTransfer(path,const_cast<void*>(data),bytes,true);}
void LinuxDeviceIo::writePwmDuty(const std::string& path,uint16_t duty) {
    if(!config_.factory_write_readback) {DeviceIo::writePwmDuty(path,duty);return;}
    if(path!=config_.motor_left_pwm && path!=config_.motor_right_pwm && path!=config_.servo_pwm)
        throw std::runtime_error("PWM readback compatibility refused unconfigured output: "+path);
    const auto returned=transferOnce(path,&duty,sizeof(duty),true);
    verifyFactoryPwmWrite(*this,path,duty,returned); // One write, no retries on short/status-zero results.
    if(returned==0 && reportedZeroOutputWrites_.insert(path).second)
        std::cerr<<"[2026 INFO] Verified status-zero PWM write: "<<path<<" duty="<<duty<<" readback matched (software only)\n";
}
void LinuxDeviceIo::writeGpioLevel(const std::string& path,uint8_t level) {
    if(level>1) throw std::runtime_error("Invalid GPIO output level: "+path);
    if(!config_.factory_write_readback) {DeviceIo::writeGpioLevel(path,level);return;}
    if(path!=config_.motor_left_dir && path!=config_.motor_right_dir &&
       !(config_.beep_enabled && path==config_.beep_device))
        throw std::runtime_error("GPIO readback compatibility refused unconfigured output: "+path);
    uint8_t ascii=uint8_t('0'+level);
    const auto returned=transferOnce(path,&ascii,sizeof(ascii),true);
    verifyFactoryGpioWrite(*this,path,level,returned);
    if(returned==0 && reportedZeroOutputWrites_.insert(path).second)
        std::cerr<<"[2026 INFO] Verified status-zero GPIO write: "<<path<<" level="<<unsigned(level)<<" readback matched (software only)\n";
}
PwmInfo LinuxDeviceIo::readPwmMetadata(const std::string& path) {
    GuardedDeviceRead<PwmInfo> buffer(pwmMetadataReadBuffer());
    const auto count=transferOnce(path,buffer.data(),sizeof(PwmInfo),false);
    const auto info=buffer.value(path);
    validatePwmMetadataRead(info,count,path);
    if(count==0 && reportedZeroPwmReads_.insert(path).second)
        std::cerr<<"[2026 INFO] Validated zero-return PWM metadata: "<<path<<'\n';
    return info;
}
EncoderCount LinuxDeviceIo::readEncoderCount(const std::string& path) {
    // Not initialized to zero: an untouched buffer must not be fabricated as
    // standstill. This seed is NOT reserved: INT32_MIN is valid in cumulative32.
    // v3 board evidence identified four changed bytes on BOTH encoders.
    GuardedDeviceRead<EncoderCount> buffer(std::numeric_limits<EncoderCount>::min());
    const auto returned=transferOnce(path,buffer.data(),sizeof(EncoderCount),false);
    const auto count=buffer.value(path);
    // Exactly one read. Do not add a second sentinel check: delta reads may clear
    // the counter and would lose motion data. The factory zero-status contract
    // has been observed on both devices; transfer failures must be reported by
    // the driver. User space cannot prove copying for all possible int32 values.
    return decodeEncoderRead(count,returned,path);
}
uint8_t LinuxDeviceIo::readGpioLevel(const std::string& path) {
    GuardedDeviceRead<uint8_t> buffer(uint8_t(0xff)); // Outside GPIO domains.
    const auto returned=transferOnce(path,buffer.data(),sizeof(uint8_t),false);
    const auto level=buffer.value(path);
    return decodeGpioRead(level,returned,path);
}
std::string LinuxDeviceIo::readText(const std::string& path) {
    FileDescriptor fd(path,O_RDONLY);char text[256];ssize_t count;
    do {count=read(fd.get(),text,sizeof(text));} while(count<0 && errno==EINTR);
    if(count<=0 || count==ssize_t(sizeof(text))) throw std::runtime_error("Empty/oversized/unreadable device text: "+path);
    return std::string(text,size_t(count));
}
std::vector<std::string> LinuxDeviceIo::listIioDevicePaths() {
    const std::string root="/sys/bus/iio/devices";
    DIR* opened=opendir(root.c_str());
    if(!opened) {
        if(errno==ENOENT) return {};
        throw std::runtime_error("Cannot enumerate '"+root+"': "+std::strerror(errno));
    }
    std::unique_ptr<DIR,decltype(&closedir)> directory(opened,&closedir);
    std::vector<std::string> paths;
    const std::string prefix="iio:device";
    for(;;) {
        errno=0;const auto* entry=readdir(directory.get());
        if(!entry) {
            if(errno) throw std::runtime_error("Cannot finish IIO inventory: "+std::string(std::strerror(errno)));
            break;
        }
        const std::string name=entry->d_name;
        if(name.compare(0,prefix.size(),prefix)==0 && name.size()>prefix.size() &&
           name.find_first_not_of("0123456789",prefix.size())==std::string::npos)
            paths.push_back(root+"/"+name);
    }
    std::sort(paths.begin(),paths.end());return paths;
}
HardwareLock::HardwareLock() {
    fd_=open("/tmp/smart_car_2026.lock",O_CREAT|O_RDWR|O_CLOEXEC|O_NOFOLLOW,0600);
    if(fd_<0) throw std::runtime_error("Cannot open vehicle lock");
    if(flock(fd_,LOCK_EX|LOCK_NB)<0) {close(fd_);fd_=-1;throw std::runtime_error("Another vehicle process is running");}
}
HardwareLock::~HardwareLock() {if(fd_>=0) close(fd_);}
HardwareImu::HardwareImu(DeviceIo& io,const HardwareConfig& config,const Params& p)
    : parser_(p.imu_format),sign_(p.imu_yaw_sign) {
    if(config.imu_backend=="none") return;
    if(config.imu_backend=="iio") {iio_=std::make_unique<IioYaw>(io,config,sign_);return;}
    const auto& path=p.imu_device.empty()?config.imu_serial_device:p.imu_device;
    fd_=open(path.c_str(),O_RDONLY|O_NOCTTY|O_NONBLOCK|O_CLOEXEC);
    if(fd_<0) throw std::runtime_error("Cannot open IMU serial device: "+path);
    try {
        termios options{};if(tcgetattr(fd_,&options)<0) throw std::runtime_error("IMU is not a serial port");
        cfmakeraw(&options);const speed_t baud=p.imu_baud==9600?B9600:p.imu_baud==57600?B57600:B115200;
        cfsetispeed(&options,baud);cfsetospeed(&options,baud);options.c_cflag|=CLOCAL|CREAD;
        if(tcsetattr(fd_,TCSANOW,&options)<0) throw std::runtime_error("Cannot configure IMU serial port");
        if(tcflush(fd_,TCIFLUSH)<0) throw std::runtime_error("Cannot flush stale IMU frames");
    } catch(...) {close(fd_);fd_=-1;throw;}
}
HardwareImu::~HardwareImu() {if(fd_>=0) close(fd_);}
void HardwareImu::poll(double now,bool stationary) {
    if(iio_) {iio_->poll(now,stationary);return;}
    if(fd_<0) return;
    uint8_t data[512];double last=0;bool received=false;
    for(int batch=0;batch<4;++batch) {
        const ssize_t count=read(fd_,data,sizeof(data));
        if(count<0) {if(errno==EINTR) continue;if(errno==EAGAIN || errno==EWOULDBLOCK) break;throw std::runtime_error("IMU serial read failed");}
        if(count==0) break;
        const auto angles=parser_.feed(data,size_t(count));if(!angles.empty()) {last=angles.back();received=true;}
    }
    if(received) tracker_.ingest(last*sign_,now);
}
bool HardwareImu::valid(double now) const {return iio_?iio_->fresh(now):tracker_.fresh(now);}
double HardwareImu::yaw() const {return iio_?iio_->value():tracker_.value();}
VoiceOutput::VoiceOutput(const HardwareConfig& config) {
    if(!config.voice_enabled) return;
    fd_=open(config.voice_device.c_str(),O_RDWR|O_CLOEXEC);
    if(fd_>=0 && ioctl(fd_,I2C_SLAVE,int(config.voice_address))<0) {close(fd_);fd_=-1;}
    if(fd_<0) std::cerr<<"[2026 WARNING] Voice device unavailable: "<<config.voice_device<<'\n';
}
VoiceOutput::~VoiceOutput() {if(fd_>=0) close(fd_);}
bool VoiceOutput::speakZebra() {
    if(fd_<0) return false;
    // Preserve the original WonderEcho protocol: register 0x6e, command FF 11.
    union i2c_smbus_data data{};data.block[0]=2;data.block[1]=0xff;data.block[2]=0x11;
    struct i2c_smbus_ioctl_data args{};args.read_write=I2C_SMBUS_WRITE;args.command=0x6e;
    args.size=I2C_SMBUS_I2C_BLOCK_DATA;args.data=&data;return ioctl(fd_,I2C_SMBUS,&args)==0;
}
} // namespace car2026
