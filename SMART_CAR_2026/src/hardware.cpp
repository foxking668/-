#include "hardware.hpp"
#include <limits>
#include <set>
#include <iostream>

namespace car2026 {
namespace {
std::string trimmed(const std::string& text) {
    const auto a=text.find_first_not_of(" \t\r\n");
    return a==std::string::npos ? std::string{} : text.substr(a,text.find_last_not_of(" \t\r\n")-a+1);
}
void require(bool condition,const std::string& message) {if(!condition) throw std::runtime_error(message);}
double parseDeviceNumber(const std::string& contents,const std::string& path) {
    const auto text=trimmed(contents);size_t consumed=0;
    const double number=std::stod(text,&consumed);
    require(consumed==text.size() && std::isfinite(number),"Invalid numeric device data: "+path);
    return number;
}
bool supportedIioModel(const std::string& model) {
    return model=="IMU660RA" || model=="IMU660RB" || model=="IMU963RA";
}
struct IioDevice {std::string path,model;};
IioDevice resolveIioImu(DeviceIo& io,const std::string& configuredPath) {
    if(configuredPath!="auto") {
        const auto model=trimmed(io.readText(configuredPath+"/name"));
        require(supportedIioModel(model),"Configured IIO device '"+configuredPath+"' is '"+model+
                "', not a supported IMU. Use imu_iio_device=auto or a verified IMU path; run collect_hardware_diagnostics.sh.");
        return {configuredPath,model};
    }
    std::vector<IioDevice> matches;std::set<std::string> seen;std::string inventory;
    for(const auto& path : io.listIioDevicePaths()) {
        require(!path.empty() && seen.insert(path).second,"Invalid/duplicate IIO inventory path: "+path);
        std::string model;
        try {model=trimmed(io.readText(path+"/name"));}
        catch(const std::exception& e) {
            throw std::runtime_error("IIO discovery incomplete at '"+path+"': "+e.what()+
                                     ". Refusing to select a potentially ambiguous IMU.");
        }
        require(!model.empty(),"Empty IIO model name at '"+path+"'");
        inventory+=(inventory.empty()?"":"; ")+path+"="+model;
        if(supportedIioModel(model)) matches.push_back({path,model});
    }
    const auto observed=inventory.empty()?std::string("no IIO devices"):inventory;
    require(!matches.empty(),"No supported IIO IMU found. Observed: "+observed+
            ". ADC/ultrasonic devices are not gyroscopes. Check IMU probe/power/wiring/device tree with collect_hardware_diagnostics.sh; do not disable require_imu to bypass this fault.");
    require(matches.size()==1,"Multiple supported IIO IMUs found. Observed: "+observed+
            ". Set imu_iio_device to the verified sensor path; refusing to select the first device.");
    return matches.front();
}
void writeDuty(DeviceIo& io,const std::string& path,uint16_t duty) {io.writePwmDuty(path,duty);}
void writeGpio(DeviceIo& io,const std::string& path,int level) {
    require(level==0 || level==1,"Invalid GPIO output level: "+path);
    io.writeGpioLevel(path,uint8_t(level));
}
void requireFactoryInputRead(std::ptrdiff_t returnedBytes,size_t expectedBytes,const std::string& path) {
    require(returnedBytes==0 || returnedBytes==std::ptrdiff_t(expectedBytes),
            "Short factory input transfer "+path+": expected "+std::to_string(expectedBytes)+
            " or zero success status, got "+std::to_string(returnedBytes));
}
void validateEncoderDomain(EncoderCount count,const std::string& mode) {
    if(mode=="cumulative16")
        require(count>=std::numeric_limits<int16_t>::min() && count<=std::numeric_limits<int16_t>::max(),
                "cumulative16 requires a sign-extended 16-bit counter; full count="+std::to_string(count));
}
}
int64_t encoderCountDelta(EncoderCount now,EncoderCount before,const std::string& mode) {
    if(mode=="delta") return now;
    require(mode=="cumulative16" || mode=="cumulative32","Unknown encoder count mode: "+mode);
    validateEncoderDomain(now,mode);validateEncoderDomain(before,mode);
    // Unsigned modular subtraction avoids signed overflow at wraparound. Convert
    // in int64, not by an implementation-defined narrowing cast to int32.
    uint64_t difference=uint32_t(now)-uint32_t(before);
    const uint64_t modulus=mode=="cumulative16" ? (uint64_t(1)<<16) : (uint64_t(1)<<32);
    difference&=modulus-1;
    require(difference!=modulus/2,"Ambiguous half-range cumulative encoder jump");
    return difference<modulus/2 ? int64_t(difference) : int64_t(difference)-int64_t(modulus);
}
void HardwareConfig::validate() const {
#define FINITE(n,d) require(std::isfinite(n),"Nonfinite hardware setting: " #n);
    BOARD_NUMERIC_SETTINGS(FINITE)
#undef FINITE
    require(motor_command_range>0 && motor_command_range<=65535,"Invalid motor_command_range");
    for(double b : {factory_write_readback,motor_left_forward_level,motor_right_forward_level,board_inputs_enabled,input_active_level,beep_enabled,voice_enabled})
        require(b==0 || b==1,"Hardware level/enable must be 0 or 1");
    for(double s : {encoder_left_sign,encoder_right_sign}) require(s==1 || s==-1,"Hardware encoder sign must be -1 or +1");
    require(servo_right_us>=500 && servo_right_us<servo_center_us && servo_center_us<servo_left_us && servo_left_us<=2500,
            "Expected 500 <= servo_right_us < center < left <= 2500");
    require(servo_reference_deg>=5 && servo_reference_deg<=90,"Invalid servo_reference_deg");
    require(stop_key_index>=-1 && stop_key_index<=3 && std::floor(stop_key_index)==stop_key_index,"Invalid stop_key_index");
    require(stop_switch_index>=-1 && stop_switch_index<=1 && std::floor(stop_switch_index)==stop_switch_index,"Invalid stop_switch_index");
    require(board_inputs_enabled || (stop_key_index==-1 && stop_switch_index==-1),"Stop input requires board_inputs_enabled=1");
    require(voice_address>=3 && voice_address<=119 && std::floor(voice_address)==voice_address,"Invalid 7-bit voice_address");
    require(encoder_mode=="delta" || encoder_mode=="cumulative16" || encoder_mode=="cumulative32",
            "encoder_mode must be delta, cumulative16 or cumulative32");
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
PwmInfo pwmMetadataReadBuffer() {
    // Each untouched field is invalid. In particular, never seed duty with zero:
    // a driver that supplies no data must not look like a stopped, valid device.
    const auto invalid=std::numeric_limits<uint32_t>::max();
    return {invalid,invalid,invalid,invalid,invalid,invalid};
}
void validatePwmMetadataRead(const PwmInfo& info,std::ptrdiff_t returnedBytes,const std::string& path) {
    require(returnedBytes==0 || returnedBytes==std::ptrdiff_t(sizeof(info)),
            "Short PWM metadata transfer "+path+": expected 24 or validated zero return, got "+std::to_string(returnedBytes));
    require(info.freq>0 && info.duty_max>0 && info.duty_max<=65535 && info.duty<=info.duty_max,"Invalid PWM metadata: "+path);
    if(returnedBytes==0) {
        // Board evidence: all six fields were copied although read() returned 0.
        // Require coherent nanosecond timing as well as basic duty ranges before
        // accepting this narrowly scoped factory-driver convention.
        require(info.freq<=1000000000u && info.clk_freq>=info.freq && info.clk_freq<=1000000000u &&
                info.period_ns>0 && info.duty_ns<=info.period_ns,
                "Invalid/incomplete zero-return PWM metadata: "+path);
        const double expectedPeriod=1e9/double(info.freq);
        const double expectedDuty=double(info.period_ns)*info.duty/info.duty_max;
        require(std::abs(double(info.period_ns)-expectedPeriod)<=2 &&
                std::abs(double(info.duty_ns)-expectedDuty)<=2,
                "Inconsistent zero-return PWM timing: "+path);
    }
}
PwmInfo readPwmInfo(DeviceIo& io,const std::string& path) {
    return io.readPwmMetadata(path);
}
void verifyFactoryPwmWrite(DeviceIo& io,const std::string& path,uint16_t duty,std::ptrdiff_t returned) {
    require(returned==0 || returned==std::ptrdiff_t(sizeof(duty)),
            "PWM write '"+path+"': requested 2 bytes, returned "+std::to_string(returned));
    const auto info=io.readPwmMetadata(path);
    // Apply the stronger timing/coherence checks even to a conventional read.
    validatePwmMetadataRead(info,0,path);
    require(info.duty==duty,"PWM write readback mismatch '"+path+"': requested duty="+
            std::to_string(duty)+", observed="+std::to_string(info.duty)+
            ", write returned="+std::to_string(returned));
}
void verifyFactoryGpioWrite(DeviceIo& io,const std::string& path,uint8_t level,std::ptrdiff_t returned) {
    require(level<=1,"Invalid GPIO output level: "+path);
    require(returned==0 || returned==1,"GPIO write '"+path+"': requested 1 byte, returned "+std::to_string(returned));
    const auto observed=io.readGpioLevel(path);
    require(observed<=1 && observed==level,"GPIO write readback mismatch '"+path+"': requested level="+
            std::to_string(level)+", observed="+std::to_string(observed)+
            ", write returned="+std::to_string(returned));
}
double readDeviceNumber(DeviceIo& io,const std::string& path) {
    return parseDeviceNumber(io.readText(path),path);
}
uint8_t readGpio(DeviceIo& io,const std::string& path) {
    return io.readGpioLevel(path);
}
EncoderCount decodeEncoderRead(EncoderCount rawCount,std::ptrdiff_t returnedBytes,const std::string& path) {
    requireFactoryInputRead(returnedBytes,sizeof(rawCount),path);
    // Every int32 bit pattern is a possible cumulative counter. No value can be
    // reserved as an "unwritten" sentinel without breaking wraparound. Zero is
    // a success STATUS under the observed factory ABI, not a byte count or a
    // manufactured zero sample. Saturation/rate checks remain at FactoryBoard.
    return rawCount;
}
uint8_t decodeGpioRead(uint8_t rawLevel,std::ptrdiff_t returnedBytes,const std::string& path) {
    requireFactoryInputRead(returnedBytes,sizeof(rawLevel),path);
    if(rawLevel=='0' || rawLevel=='1') rawLevel=uint8_t(rawLevel-'0');
    require(rawLevel<=1,"Invalid GPIO level: "+path);return rawLevel;
}
void inspectHardware(DeviceIo& io,const HardwareConfig& config,std::ostream& out) {
    config.validate();
    // The inspection only uses portable DeviceIo operations, so exercise this
    // exact sequence in mock-device tests. Avoid initializer_list<string>
    // temporaries in the deployed inspector and finish I/O before printing a row.
    auto pwm=[&](const std::string& path) {
        const auto info=readPwmInfo(io,path);
        out<<path<<" freq="<<info.freq<<" duty="<<info.duty<<" max="<<info.duty_max<<'\n';
    };
    auto encoder=[&](const std::string& path) {
        require(!path.empty(),"Empty encoder path in hardware inspection");
        const auto count=io.readEncoderCount(path);
        out<<path<<" count="<<count<<'\n';
    };
    auto gpio=[&](const std::string& path) {
        require(!path.empty(),"Empty GPIO path in hardware inspection");
        const auto level=readGpio(io,path);
        out<<path<<" level="<<int(level)<<'\n';
    };
    pwm(config.motor_left_pwm);pwm(config.motor_right_pwm);pwm(config.servo_pwm);
    out<<"Motor units: command +/-"<<config.motor_command_range<<" maps to each duty_max; positive = forward.\n"
       <<"Forward GPIO: left="<<int(config.motor_left_forward_level)<<" right="<<int(config.motor_right_forward_level)<<" (ASCII output; inspection does not change levels).\n";
    encoder(config.encoder_left);encoder(config.encoder_right);
    gpio(config.motor_left_dir);gpio(config.motor_right_dir);
    if(config.board_inputs_enabled) {
        for(int i=0;i<4;++i) gpio(config.key_prefix+std::to_string(i));
        for(int i=0;i<2;++i) gpio(config.switch_prefix+std::to_string(i));
    }
    if(config.beep_enabled) gpio(config.beep_device);
    out<<"Camera: "<<config.camera_device<<" (use --preview to acquire frames)\n";
    out<<"Voice: "<<config.voice_device<<" address="<<config.voice_address<<" enabled="<<config.voice_enabled<<" (not written)\n";
    if(config.imu_backend=="iio") inspectIioImu(io,config,out);
}
void inspectIioImu(DeviceIo& io,const HardwareConfig& config,std::ostream& out) {
    config.validate();require(config.imu_backend=="iio","--imu-check requires imu_backend=iio; serial IMUs need their verified serial protocol");
    IioYaw imu(io,config,1);
    const auto raw=readDeviceNumber(io,imu.devicePath()+"/in_anglvel_"+config.imu_gyro_axis+"_raw");
    require(std::floor(raw)==raw && raw>-32768 && raw<32767,"IIO gyro raw data invalid/saturated");
    out<<"IIO model="<<imu.model()<<" device="<<imu.devicePath()<<" axis="<<config.imu_gyro_axis
       <<" raw="<<raw<<" scale_deg_s_per_raw="<<imu.scaleDegPerRaw()<<'\n'
       <<"Read-only IMU interface check: not calibrated; no yaw/motion validation.\n";
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
        previousLeft_=io_.readEncoderCount(config_.encoder_left);previousRight_=io_.readEncoderCount(config_.encoder_right);
        validateEncoderDomain(previousLeft_,config_.encoder_mode);validateEncoderDomain(previousRight_,config_.encoder_mode);
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
    auto attempt=[&](auto operation) {
        try {operation();}
        catch(const std::exception& e) {
            success=false;
            try {std::cerr<<"[2026 ERROR] Stop output verification failed: "<<e.what()<<'\n';} catch(...) {}
        }
        catch(...) {success=false;}
    };
    if(leftReady_) attempt([&]{writeDuty(io_,config_.motor_left_pwm,0);});
    if(rightReady_) attempt([&]{writeDuty(io_,config_.motor_right_pwm,0);});
    if(initialized_ && servoReady_) attempt([&]{setSteering(0);});
    if(beepReady_) attempt([&]{writeGpio(io_,config_.beep_device,0);beepState_=false;});
    return success;
}
WheelSample FactoryBoard::sampleWheels(double now) {
    require(std::isfinite(now),"Invalid encoder timestamp");
    // The first call discards the untimed startup interval and establishes the baseline.
    const EncoderCount left=io_.readEncoderCount(config_.encoder_left),right=io_.readEncoderCount(config_.encoder_right);
    validateEncoderDomain(left,config_.encoder_mode);validateEncoderDomain(right,config_.encoder_mode);
    if(!timed_) {previousLeft_=left;previousRight_=right;previousTime_=now;timed_=true;return {};}
    const double dt=now-previousTime_;
    require(dt>0 && dt<=0.15,"Encoder sampling interval out of range");
    const double halfRange=config_.encoder_mode=="cumulative16" ? 32768.0 : 2147483648.0;
    require(params_.encoder_ppr*params_.max_encoder_rps*dt<halfRange,"Encoder resolution/sampling interval cannot represent configured maximum speed");
    require(config_.encoder_mode!="delta" || (left!=std::numeric_limits<EncoderCount>::min() && left!=std::numeric_limits<EncoderCount>::max() && right!=std::numeric_limits<EncoderCount>::min() && right!=std::numeric_limits<EncoderCount>::max()),"Encoder count saturated");
    const double l=encoderCountDelta(left,previousLeft_,config_.encoder_mode)*config_.encoder_left_sign*params_.encoder_left_sign;
    const double r=encoderCountDelta(right,previousRight_,config_.encoder_mode)*config_.encoder_right_sign*params_.encoder_right_sign;
    const double leftRps=l/(params_.encoder_ppr*dt),rightRps=r/(params_.encoder_ppr*dt);
    require(std::abs(leftRps)<=params_.max_encoder_rps && std::abs(rightRps)<=params_.max_encoder_rps,"Encoder speed out of range");
    previousLeft_=left;previousRight_=right;previousTime_=now;
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
    const auto device=resolveIioImu(io_,config_.imu_iio_device);
    config_.imu_iio_device=device.path;model_=device.model;
    rawPath_=config_.imu_iio_device+"/in_anglvel_"+config_.imu_gyro_axis+"_raw";
    scale_=config_.imu_gyro_scale_deg_s;
    if(scale_==0) {
        std::string scalePath=config_.imu_iio_device+"/in_anglvel_"+config_.imu_gyro_axis+"_scale",scaleText;
        try {scaleText=io_.readText(scalePath);}
        catch(const std::exception&) {scalePath=config_.imu_iio_device+"/in_anglvel_scale";scaleText=io_.readText(scalePath);}
        // Fall back only when the axis attribute cannot be read, not when a
        // present value is malformed. Never mask corrupt per-axis scale data.
        scale_=parseDeviceNumber(scaleText,scalePath);
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
