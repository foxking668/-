#pragma once
#include "core.hpp"
#include <array>
#include <cstddef>

namespace car2026 {
// Device paths and physical output calibration belong here, not in vision/mission.
#define BOARD_NUMERIC_SETTINGS(X) \
 X(motor_command_range,50000) \
 X(motor_left_forward_level,1) \
 X(motor_right_forward_level,1) \
 X(encoder_left_sign,-1) \
 X(encoder_right_sign,1) \
 X(servo_center_us,1490) \
 X(servo_left_us,1723.333333) \
 X(servo_right_us,1123.333333) \
 X(servo_reference_deg,22) \
 X(board_inputs_enabled,1) \
 X(stop_key_index,-1) \
 X(stop_switch_index,-1) \
 X(input_active_level,0) \
 X(beep_enabled,1) \
 X(voice_enabled,1) \
 X(voice_address,52) \
 X(imu_gyro_scale_deg_s,0) \
 X(imu_calibration_samples,100) \
 X(imu_max_bias_deg_s,20) \
 X(imu_max_stddev_deg_s,0.8)
#define BOARD_STRING_SETTINGS(X) \
 X(motor_left_pwm,"/dev/zf_device_pwm_motor_1") \
 X(motor_right_pwm,"/dev/zf_device_pwm_motor_2") \
 X(motor_left_dir,"/dev/zf_driver_gpio_motor_1") \
 X(motor_right_dir,"/dev/zf_driver_gpio_motor_2") \
 X(servo_pwm,"/dev/zf_device_pwm_servo") \
 X(encoder_left,"/dev/zf_encoder_1") \
 X(encoder_right,"/dev/zf_encoder_2") \
 X(encoder_mode,"delta") \
 X(camera_device,"/dev/video0") \
 X(key_prefix,"/dev/zf_driver_gpio_key_") \
 X(switch_prefix,"/dev/zf_driver_gpio_switch_") \
 X(beep_device,"/dev/zf_driver_gpio_beep") \
 X(voice_device,"/dev/i2c-2") \
 X(imu_backend,"iio") \
 X(imu_iio_device,"/sys/bus/iio/devices/iio:device1") \
 X(imu_gyro_axis,"z") \
 X(imu_serial_device,"/dev/ttyS2")

struct HardwareConfig {
#define FIELD(n,d) double n=d;
    BOARD_NUMERIC_SETTINGS(FIELD)
#undef FIELD
#define FIELD(n,d) std::string n=d;
    BOARD_STRING_SETTINGS(FIELD)
#undef FIELD
    void validate() const;
    static HardwareConfig load(const std::string& path);
};

// Exactly the six uint32 fields read by the supplied zf_driver_pwm library.
struct PwmInfo { uint32_t freq=0,duty=0,duty_max=0,duty_ns=0,period_ns=0,clk_freq=0; };
static_assert(sizeof(PwmInfo)==24,"Factory PWM ABI must be 24 bytes");
class DeviceIo {
public:
    virtual ~DeviceIo()=default;
    virtual void readBinary(const std::string& path,void* data,size_t bytes)=0;
    virtual void writeBinary(const std::string& path,const void* data,size_t bytes)=0;
    virtual std::string readText(const std::string& path)=0;
};
PwmInfo readPwmInfo(DeviceIo& io,const std::string& path);
double readDeviceNumber(DeviceIo& io,const std::string& path);
uint8_t readGpio(DeviceIo& io,const std::string& path);
uint16_t motorDuty(double command,double commandRange,const PwmInfo& info);
uint16_t servoDuty(double angleDeg,const HardwareConfig& config,const PwmInfo& info);

struct WheelSample { double leftRps=0,rightRps=0,distanceM=0; };
class FactoryBoard {
public:
    FactoryBoard(DeviceIo& io,const HardwareConfig& config,const Params& params);
    ~FactoryBoard();
    FactoryBoard(const FactoryBoard&)=delete;
    FactoryBoard& operator=(const FactoryBoard&)=delete;
    WheelSample sampleWheels(double now);
    void setMotorCommands(double left,double right);
    void setSteering(double vehicleAngleDeg);
    bool stop() noexcept;
    void setBeep(bool enabled);
    bool stopInputActive();
    const PwmInfo& leftPwmInfo() const {return leftInfo_;}
    const PwmInfo& rightPwmInfo() const {return rightInfo_;}
private:
    DeviceIo& io_;HardwareConfig config_;Params params_;
    PwmInfo leftInfo_,rightInfo_,servoInfo_;
    bool leftReady_=false,rightReady_=false,servoReady_=false;
    bool initialized_=false,beepReady_=false,beepState_=false;
    int leftDirection_=-1,rightDirection_=-1;
    int16_t previousLeft_=0,previousRight_=0;
    double previousTime_=0;
    bool timed_=false;
    void setMotor(const std::string& pwm,const std::string& dir,const PwmInfo& info,
                  double command,int forwardLevel,int& previousDirection);
};

// Relative yaw from a calibrated physical gyro; not absolute compass heading.
class IioYaw {
public:
    IioYaw(DeviceIo& io,const HardwareConfig& config,double yawSign);
    void poll(double now,bool stationary);
    bool ready() const {return calibrated_;}
    bool fresh(double now) const {return calibrated_ && !lost_ && now-lastTime_>=0 && now-lastTime_<0.15;}
    double value() const {return yaw_;}
    const std::string& model() const {return model_;}
private:
    DeviceIo& io_;HardwareConfig config_;std::string model_,rawPath_;
    double scale_=0,sign_=1,bias_=0,yaw_=0,lastTime_=0,previousRate_=0;
    std::vector<double> calibration_;
    bool calibrated_=false,timed_=false,lost_=false;
};
} // namespace car2026
