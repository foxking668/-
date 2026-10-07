#include "hardware_linux.hpp"
#include <cerrno>
#include <cstring>
#include <fcntl.h>
#include <iostream>
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
        if(fd_<0) throw std::runtime_error("Open "+path+": "+std::strerror(errno));
    }
    ~FileDescriptor() {close(fd_);}
    int get() const {return fd_;}
    FileDescriptor(const FileDescriptor&)=delete;
    FileDescriptor& operator=(const FileDescriptor&)=delete;
private:int fd_;
};
void exactTransfer(const std::string& path,void* data,size_t bytes,bool writing) {
    FileDescriptor fd(path,writing?O_WRONLY:O_RDONLY);ssize_t count;
    do {count=writing ? write(fd.get(),data,bytes) : read(fd.get(),data,bytes);} while(count<0 && errno==EINTR);
    if(count<0) throw std::runtime_error("I/O "+path+": "+std::strerror(errno));
    // Do not complete a short transfer with another read: encoder reads may clear counters.
    if(size_t(count)!=bytes) throw std::runtime_error("Short device transfer "+path+": expected "+std::to_string(bytes)+", got "+std::to_string(count));
}
}
void LinuxDeviceIo::readBinary(const std::string& path,void* data,size_t bytes) {exactTransfer(path,data,bytes,false);}
void LinuxDeviceIo::writeBinary(const std::string& path,const void* data,size_t bytes) {exactTransfer(path,const_cast<void*>(data),bytes,true);}
std::string LinuxDeviceIo::readText(const std::string& path) {
    FileDescriptor fd(path,O_RDONLY);char text[256];ssize_t count;
    do {count=read(fd.get(),text,sizeof(text));} while(count<0 && errno==EINTR);
    if(count<=0 || count==ssize_t(sizeof(text))) throw std::runtime_error("Empty/oversized/unreadable device text: "+path);
    return std::string(text,size_t(count));
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
void inspectHardware(DeviceIo& io,const HardwareConfig& config,std::ostream& out) {
    // No motor/servo/GPIO writes. Encoder reads consume counts on delta-mode drivers.
    for(const auto& path : {config.motor_left_pwm,config.motor_right_pwm,config.servo_pwm}) {
        const auto info=readPwmInfo(io,path);out<<path<<" freq="<<info.freq<<" duty="<<info.duty<<" max="<<info.duty_max<<'\n';
    }
    for(const auto& path : {config.encoder_left,config.encoder_right}) {int16_t count=0;io.readBinary(path,&count,sizeof(count));out<<path<<" count="<<count<<'\n';}
    for(const auto& path : {config.motor_left_dir,config.motor_right_dir}) out<<path<<" level="<<int(readGpio(io,path))<<'\n';
    if(config.board_inputs_enabled) {
        for(int i=0;i<4;++i) {const auto path=config.key_prefix+std::to_string(i);out<<path<<" level="<<int(readGpio(io,path))<<'\n';}
        for(int i=0;i<2;++i) {const auto path=config.switch_prefix+std::to_string(i);out<<path<<" level="<<int(readGpio(io,path))<<'\n';}
    }
    if(config.beep_enabled) out<<config.beep_device<<" level="<<int(readGpio(io,config.beep_device))<<'\n';
    if(config.imu_backend=="iio") {IioYaw imu(io,config,1);out<<"IIO model="<<imu.model()<<" axis="<<config.imu_gyro_axis<<" raw="<<readDeviceNumber(io,config.imu_iio_device+"/in_anglvel_"+config.imu_gyro_axis+"_raw")<<'\n';}
    out<<"Camera: "<<config.camera_device<<" (use --preview to acquire frames)\n";
    out<<"Voice: "<<config.voice_device<<" address="<<config.voice_address<<" enabled="<<config.voice_enabled<<" (not written)\n";
}
} // namespace car2026
