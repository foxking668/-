#include "hardware.hpp"
#include <limits>
#include <set>

namespace car2026 {
namespace {
std::string trimmed(const std::string& text) {
    const auto a=text.find_first_not_of(" \t\r\n");
    return a==std::string::npos ? std::string{} : text.substr(a,text.find_last_not_of(" \t\r\n")-a+1);
}
void require(bool condition,const std::string& message) {if(!condition) throw std::runtime_error(message);}
void writeDuty(DeviceIo& io,const std::string& path,uint16_t duty) {io.writeBinary(path,&duty,sizeof(duty));}
void writeGpio(DeviceIo& io,const std::string& path,int level) {
    const uint8_t value=uint8_t('0'+level);io.writeBinary(path,&value,sizeof(value));
}
int16_t readCount(DeviceIo& io,const std::string& path) {
    int16_t count=0;io.readBinary(path,&count,sizeof(count));return count;
}
int countDelta(int16_t now,int16_t before,const std::string& mode) {
    if(mode=="delta") return now;
    const unsigned difference=(unsigned(uint16_t(now))-unsigned(uint16_t(before)))&0xffffu;
    return difference<32768u ? int(difference) : int(difference)-65536;
}
}
void HardwareConfig::validate() const {
#define FINITE(n,d) require(std::isfinite(n),"Nonfinite hardware setting: " #n);
    BOARD_NUMERIC_SETTINGS(FINITE)
#undef FINITE
    require(motor_command_range>0 && motor_command_range<=65535,"Invalid motor_command_range");
    for(double b : {motor_left_forward_level,motor_right_forward_level,board_inputs_enabled,input_active_level,beep_enabled,voice_enabled})
        require(b==0 || b==1,"Hardware level/enable must be 0 or 1");
    for(double s : {encoder_left_sign,encoder_right_sign}) require(s==1 || s==-1,"Hardware encoder sign must be -1 or +1");
    require(servo_right_us>=500 && servo_right_us<servo_center_us && servo_center_us<servo_left_us && servo_left_us<=2500,
            "Expected 500 <= servo_right_us < center < left <= 2500");
    require(servo_reference_deg>=5 && servo_reference_deg<=90,"Invalid servo_reference_deg");
    require(stop_key_index>=-1 && stop_key_index<=3 && std::floor(stop_key_index)==stop_key_index,"Invalid stop_key_index");
    require(stop_switch_index>=-1 && stop_switch_index<=1 && std::floor(stop_switch_index)==stop_switch_index,"Invalid stop_switch_index");
    require(board_inputs_enabled || (stop_key_index==-1 && stop_switch_index==-1),"Stop input requires board_inputs_enabled=1");
    require(voice_address>=3 && voice_address<=119 && std::floor(voice_address)==voice_address,"Invalid 7-bit voice_address");
    require(encoder_mode=="delta" || encoder_mode=="cumulative16","encoder_mode must be delta or cumulative16");
    require(imu_backend=="iio" || imu_backend=="serial" || imu_backend=="none","imu_backend must be iio, serial or none");
    require(imu_gyro_axis=="x" || imu_gyro_axis=="y" || imu_gyro_axis=="z","Invalid imu_gyro_axis");
    require(imu_gyro_scale_deg_s>=0 && imu_gyro_scale_deg_s<=100,"Invalid imu_gyro_scale_deg_s");
    require(imu_calibration_samples>=20 && imu_calibration_samples<=1000 && std::floor(imu_calibration_samples)==imu_calibration_samples,"Invalid imu_calibration_samples");
    require(imu_max_bias_deg_s>0 && imu_max_bias_deg_s<=100 && imu_max_stddev_deg_s>0 && imu_max_stddev_deg_s<=10,"Invalid gyro calibration limits");
    std::set<std::string> devicePaths;
    for(const auto& path : {motor_left_pwm,motor_right_pwm,motor_left_dir,motor_right_dir,servo_pwm,encoder_left,encoder_right,beep_device})
        require(!path.empty() && devicePaths.insert(path).second,"Empty or aliased motor/encoder/servo/beep path: "+path);
    for(const auto& path : {camera_device,key_prefix,switch_prefix,voice_device,imu_iio_device,imu_serial_device})
        require(!path.empty(),"Empty hardware device path");
}
HardwareConfig HardwareConfig::load(const std::string& path) {
    std::ifstream file(path);require(bool(file),"Cannot open hardware config: "+path);
    HardwareConfig config;std::set<std::string> seen;std::string line;int lineNumber=0;
    while(std::getline(file,line)) {
        ++lineNumber;
        if(lineNumber==1 && line.size()>=3 && uint8_t(line[0])==0xef && uint8_t(line[1])==0xbb && uint8_t(line[2])==0xbf) line.erase(0,3);
        line=trimmed(line.substr(0,line.find('#')));if(line.empty()) continue;
        const auto equal=line.find('=');require(equal!=std::string::npos,"Missing '=' in hardware config line "+std::to_string(lineNumber));
        const auto key=trimmed(line.substr(0,equal)),value=trimmed(line.substr(equal+1));
        require(seen.insert(key).second,"Duplicate hardware setting: "+key);
        bool known=false;
#define STRING(n,d) if(key==#n) {config.n=value;known=true;}
        BOARD_STRING_SETTINGS(STRING)
#undef STRING
        if(known) continue;
        size_t consumed=0;const double number=std::stod(value,&consumed);
        require(consumed==value.size() && std::isfinite(number),"Invalid hardware value: "+key);
#define NUMBER(n,d) if(key==#n) {config.n=number;known=true;}
        BOARD_NUMERIC_SETTINGS(NUMBER)
#undef NUMBER
        require(known,"Unknown hardware setting: "+key);
    }
    config.validate();return config;
}
PwmInfo readPwmInfo(DeviceIo& io,const std::string& path) {
    PwmInfo info;io.readBinary(path,&info,sizeof(info));
    require(info.freq>0 && info.duty_max>0 && info.duty_max<=65535 && info.duty<=info.duty_max,"Invalid PWM metadata: "+path);
    return info;
}
double readDeviceNumber(DeviceIo& io,const std::string& path) {
    const auto text=trimmed(io.readText(path));size_t consumed=0;
    const double number=std::stod(text,&consumed);
    require(consumed==text.size() && std::isfinite(number),"Invalid numeric device data: "+path);return number;
}
uint8_t readGpio(DeviceIo& io,const std::string& path) {
    uint8_t level=0;io.readBinary(path,&level,sizeof(level));
    if(level=='0' || level=='1') level=uint8_t(level-'0');
    require(level<=1,"Invalid GPIO level: "+path);return level;
}
uint16_t motorDuty(double command,double range,const PwmInfo& info) {
    require(std::isfinite(command) && std::isfinite(range) && range>0 && info.duty_max>0 && info.duty_max<=65535,"Invalid motor conversion");
    return uint16_t(std::llround(clamp(std::abs(command)/range,0,1)*info.duty_max));
}
uint16_t servoDuty(double angle,const HardwareConfig& config,const PwmInfo& info) {
    require(std::isfinite(angle) && info.freq>=50 && info.freq<=300 && info.duty_max>0 && info.duty_max<=65535,"Invalid servo conversion");
    const double fraction=clamp(angle/config.servo_reference_deg,-1,1);
    const double pulse=config.servo_center_us+(fraction>=0 ? fraction*(config.servo_right_us-config.servo_center_us) : -fraction*(config.servo_left_us-config.servo_center_us));
    require(pulse*info.freq<1e6,"Servo pulse exceeds PWM period");
    return uint16_t(std::llround(pulse*info.freq*info.duty_max/1e6));
}
FactoryBoard::FactoryBoard(DeviceIo& io,const HardwareConfig& config,const Params& params)
    : io_(io),config_(config),params_(params) {
    config_.validate();params_.validate();
    try {
        // Zero BOTH paths independently before reading metadata or initializing sensors.
        leftReady_=true;rightReady_=true;
        require(stop(),"Cannot clear both motor outputs at startup");
        leftInfo_=readPwmInfo(io_,config_.motor_left_pwm);
        rightInfo_=readPwmInfo(io_,config_.motor_right_pwm);
        servoInfo_=readPwmInfo(io_,config_.servo_pwm);
        require(servoInfo_.freq>=50 && servoInfo_.freq<=300,"Servo frequency must be 50..300 Hz");
        servoReady_=true;setSteering(0);
        require(params_.pwm_limit<=config_.motor_command_range,"pwm_limit exceeds motor_command_range");
        setMotorCommands(0,0);
        // Discard counts acquired before this controller started.
        previousLeft_=readCount(io_,config_.encoder_left);previousRight_=readCount(io_,config_.encoder_right);
        if(config_.board_inputs_enabled) stopInputActive();
        if(config_.beep_enabled) {beepReady_=true;writeGpio(io_,config_.beep_device,0);}
        initialized_=true;
    } catch(...) {stop();throw;}
}
FactoryBoard::~FactoryBoard() {stop();}
void FactoryBoard::setMotor(const std::string& pwm,const std::string& dir,const PwmInfo& info,
                            double command,int forwardLevel,int& previousDirection) {
    require(std::isfinite(command),"Nonfinite motor command");
    const uint16_t duty=motorDuty(clamp(command,-params_.pwm_limit,params_.pwm_limit),config_.motor_command_range,info);
    const int direction=command>=0 ? forwardLevel : 1-forwardLevel;
    // Zero first before direction changes; previousDirection changes only after successful GPIO write.
    if(previousDirection!=direction) {writeDuty(io_,pwm,0);writeGpio(io_,dir,direction);previousDirection=direction;}
    writeDuty(io_,pwm,duty);
}
void FactoryBoard::setMotorCommands(double left,double right) {
    try {
        setMotor(config_.motor_left_pwm,config_.motor_left_dir,leftInfo_,left,int(config_.motor_left_forward_level),leftDirection_);
        setMotor(config_.motor_right_pwm,config_.motor_right_dir,rightInfo_,right,int(config_.motor_right_forward_level),rightDirection_);
    } catch(...) {stop();throw;}
}
void FactoryBoard::setSteering(double angle) {
    require(std::isfinite(angle),"Nonfinite steering command");
    const double servoAngle=params_.steer_sign*clamp(angle,-params_.max_steer_deg,params_.max_steer_deg)+params_.steer_offset_deg;
    writeDuty(io_,config_.servo_pwm,servoDuty(servoAngle,config_,servoInfo_));
}
bool FactoryBoard::stop() noexcept {
    bool success=true;
    auto attempt=[&](auto operation) {try {operation();} catch(...) {success=false;}};
    if(leftReady_) attempt([&]{writeDuty(io_,config_.motor_left_pwm,0);});
    if(rightReady_) attempt([&]{writeDuty(io_,config_.motor_right_pwm,0);});
    if(initialized_ && servoReady_) attempt([&]{setSteering(0);});
    if(beepReady_) attempt([&]{writeGpio(io_,config_.beep_device,0);beepState_=false;});
    return success;
}
WheelSample FactoryBoard::sampleWheels(double now) {
    require(std::isfinite(now),"Invalid encoder timestamp");
    // The first call discards the untimed startup interval and establishes the baseline.
    const int16_t left=readCount(io_,config_.encoder_left),right=readCount(io_,config_.encoder_right);
    if(!timed_) {previousLeft_=left;previousRight_=right;previousTime_=now;timed_=true;return {};}
    const double dt=now-previousTime_;
    require(dt>0 && dt<=0.15,"Encoder sampling interval out of range");
    require(params_.encoder_ppr*params_.max_encoder_rps*dt<32768,"Encoder resolution/sampling interval cannot represent configured maximum speed");
    require(config_.encoder_mode!="delta" || (left!=std::numeric_limits<int16_t>::min() && left!=std::numeric_limits<int16_t>::max() && right!=std::numeric_limits<int16_t>::min() && right!=std::numeric_limits<int16_t>::max()),"Encoder count saturated");
    const double l=countDelta(left,previousLeft_,config_.encoder_mode)*config_.encoder_left_sign*params_.encoder_left_sign;
    const double r=countDelta(right,previousRight_,config_.encoder_mode)*config_.encoder_right_sign*params_.encoder_right_sign;
    previousLeft_=left;previousRight_=right;previousTime_=now;
    const double leftRps=l/(params_.encoder_ppr*dt),rightRps=r/(params_.encoder_ppr*dt);
    require(std::abs(leftRps)<=params_.max_encoder_rps && std::abs(rightRps)<=params_.max_encoder_rps,"Encoder speed out of range");
    // Integrate signed wheel counts before taking magnitude; no clamped dt or stale period.
    return {leftRps,rightRps,std::abs((l+r)*0.5)*params_.wheel_circumference_m/params_.encoder_ppr};
}
void FactoryBoard::setBeep(bool enabled) {
    if(!config_.beep_enabled || enabled==beepState_) return;
    writeGpio(io_,config_.beep_device,enabled?1:0);beepState_=enabled;
}
bool FactoryBoard::stopInputActive() {
    if(!config_.board_inputs_enabled) return false;
    bool stop=false;
    for(int i=0;i<4;++i) {const auto level=readGpio(io_,config_.key_prefix+std::to_string(i));if(i==int(config_.stop_key_index) && level==config_.input_active_level) stop=true;}
    for(int i=0;i<2;++i) {const auto level=readGpio(io_,config_.switch_prefix+std::to_string(i));if(i==int(config_.stop_switch_index) && level==config_.input_active_level) stop=true;}
    return stop;
}
IioYaw::IioYaw(DeviceIo& io,const HardwareConfig& config,double sign)
    : io_(io),config_(config),sign_(sign) {
    config_.validate();require(sign==1 || sign==-1,"Invalid IMU yaw sign");
    model_=trimmed(io_.readText(config_.imu_iio_device+"/name"));
    require(model_=="IMU660RA" || model_=="IMU660RB" || model_=="IMU963RA","Unsupported factory IIO IMU: "+model_);
    rawPath_=config_.imu_iio_device+"/in_anglvel_"+config_.imu_gyro_axis+"_raw";
    scale_=config_.imu_gyro_scale_deg_s;
    if(scale_==0) {
        try {scale_=readDeviceNumber(io_,config_.imu_iio_device+"/in_anglvel_"+config_.imu_gyro_axis+"_scale");}
        catch(const std::exception&) {scale_=readDeviceNumber(io_,config_.imu_iio_device+"/in_anglvel_scale");}
        // Linux IIO angular velocity scales are SI radians/second per raw unit.
        scale_*=180/pi;
    }
    require(scale_>0 && scale_<=100,"Missing/invalid gyro scale; set measured imu_gyro_scale_deg_s");
}
void IioYaw::poll(double now,bool stationary) {
    require(!lost_ && std::isfinite(now),"IIO yaw continuity lost");
    try {
    const double raw=readDeviceNumber(io_,rawPath_);
    require(std::floor(raw)==raw && raw>-32768 && raw<32767,"IIO gyro raw data invalid/saturated");
    const double rate=raw*scale_;
    const double dt=timed_ ? now-lastTime_ : 0;
    if(timed_ && (dt<=0 || dt>0.15)) {lost_=true;throw std::runtime_error("IIO gyro sampling gap");}
    lastTime_=now;timed_=true;
    if(!calibrated_) {
        require(stationary,"Wheels moved during IMU calibration");
        calibration_.push_back(rate);
        if(calibration_.size()<size_t(config_.imu_calibration_samples)) return;
        for(double v : calibration_) bias_+=v;
        bias_/=calibration_.size();double variance=0;
        for(double v : calibration_) variance+=(v-bias_)*(v-bias_);
        require(std::abs(bias_)<=config_.imu_max_bias_deg_s && std::sqrt(variance/calibration_.size())<=config_.imu_max_stddev_deg_s,"IMU is moving or gyro bias is excessive; keep chassis stationary");
        previousRate_=0;calibrated_=true;return;
    }
    const double corrected=(rate-bias_)*sign_;
    require(std::abs(corrected)<=300,"IIO gyro rate out of range");
    yaw_+=(previousRate_+corrected)*0.5*dt;previousRate_=corrected;
    } catch(...) {lost_=true;throw;}
}
} // namespace car2026
