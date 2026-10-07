#include "hardware.hpp"
#include "device_read_buffer.hpp"
#include "../tools/encoder_bench_stats.hpp"
#include "../tools/bench_options.hpp"
#include <cstring>
#include <functional>
#include <iostream>
#include <map>
#include <limits>
using namespace car2026;
namespace {
int checks=0;
void check(bool v,const char* message) {++checks;if(!v) throw std::runtime_error(message);}
void near(double a,double b,const char* message) {check(std::abs(a-b)<1e-6,message);}
void rejects(const std::function<void()>& fn,const char* message) {
    bool threw=false;try {fn();}catch(const std::exception&) {threw=true;}check(threw,message);
}
void rejectsContaining(const std::function<void()>& fn,const std::string& text,const char* message) {
    try {fn();}catch(const std::exception& e) {check(std::string(e.what()).find(text)!=std::string::npos,message);return;}
    check(false,message);
}
struct Write {std::string path;std::vector<uint8_t> bytes;};
class FakeIo : public DeviceIo {
public:
    std::map<std::string,std::vector<uint8_t>> binary;
    std::map<std::string,std::string> text;
    std::vector<Write> writes;
    std::map<std::string,int> reads;
    std::vector<std::string> iioPaths;
    std::string failWrite;
    bool failOnce=false,clearCounts=true;
    template<class T> void put(const std::string& path,T value) {
        const auto* p=reinterpret_cast<const uint8_t*>(&value);binary[path]={p,p+sizeof(value)};
    }
    void readBinary(const std::string& path,void* target,size_t bytes) override {
        ++reads[path];
        const auto& source=binary.at(path);
        if(source.size()!=bytes) throw std::runtime_error("Fake short read");
        std::memcpy(target,source.data(),bytes);
        if(clearCounts && bytes==sizeof(EncoderCount)) put(path,EncoderCount(0));
    }
    void writeBinary(const std::string& path,const void* source,size_t bytes) override {
        if(path==failWrite) {if(failOnce) failWrite.clear();throw std::runtime_error("Injected write failure");}
        const auto* p=static_cast<const uint8_t*>(source);writes.push_back({path,{p,p+bytes}});
    }
    std::string readText(const std::string& path) override {return text.at(path);}
    std::vector<std::string> listIioDevicePaths() override {return iioPaths;}
    uint16_t lastDuty(const std::string& path) const {
        for(auto i=writes.rbegin();i!=writes.rend();++i) if(i->path==path) {
            check(i->bytes.size()==2,"PWM write is exactly uint16");uint16_t v=0;std::memcpy(&v,i->bytes.data(),2);return v;
        }
        throw std::runtime_error("No PWM output");
    }
    void populate(const HardwareConfig& c) {
        const PwmInfo motor{20000,0,10000,0,50000,100000000},servo{300,0,10000,0,3333333,100000000};
        put(c.motor_left_pwm,motor);put(c.motor_right_pwm,motor);put(c.servo_pwm,servo);
        put(c.encoder_left,EncoderCount(0));put(c.encoder_right,EncoderCount(0));
        put(c.motor_left_dir,uint8_t(0));put(c.motor_right_dir,uint8_t(0));put(c.beep_device,uint8_t(0));
        for(int i=0;i<4;++i) put(c.key_prefix+std::to_string(i),uint8_t(1));
        for(int i=0;i<2;++i) put(c.switch_prefix+std::to_string(i),uint8_t('1'));
        text[c.imu_iio_device+"/name"]="IMU660RA\n";
        text[c.imu_iio_device+"/in_anglvel_scale"]=std::to_string(pi/180);
        text[c.imu_iio_device+"/in_anglvel_z_raw"]="0\n";
        iioPaths={c.imu_iio_device};
    }
};
class ZeroStatusInputIo : public FakeIo {
public:
    std::ptrdiff_t encoderStatus=0,gpioStatus=0;
    EncoderCount readEncoderCount(const std::string& path) override {
        EncoderCount count=0;FakeIo::readBinary(path,&count,sizeof(count));
        return decodeEncoderRead(count,encoderStatus,path);
    }
    uint8_t readGpioLevel(const std::string& path) override {
        uint8_t level=0xff;FakeIo::readBinary(path,&level,sizeof(level));
        return decodeGpioRead(level,gpioStatus,path);
    }
};
class ReadbackOutputIo : public FakeIo {
public:
    std::ptrdiff_t pwmStatus=0,gpioStatus=0;
    bool applyWrites=true;
    void writePwmDuty(const std::string& path,uint16_t duty) override {
        FakeIo::writeBinary(path,&duty,sizeof(duty));
        if(applyWrites) {
            PwmInfo info{};std::memcpy(&info,binary.at(path).data(),sizeof(info));
            info.duty=duty;
            info.duty_ns=uint32_t(uint64_t(info.period_ns)*duty/info.duty_max);
            put(path,info);
        }
        verifyFactoryPwmWrite(*this,path,duty,pwmStatus);
    }
    void writeGpioLevel(const std::string& path,uint8_t level) override {
        if(level>1) throw std::runtime_error("Invalid GPIO output level");
        const uint8_t ascii=uint8_t('0'+level);FakeIo::writeBinary(path,&ascii,sizeof(ascii));
        if(applyWrites) put(path,level);
        verifyFactoryGpioWrite(*this,path,level,gpioStatus);
    }
};
}
int main(int argc,char** argv) {
    try {
        HardwareConfig c=HardwareConfig::load(argc>1?argv[1]:"config/hardware.ini");Params p;c.voice_enabled=0;
        check(c.motor_left_forward_level==0 && c.motor_right_forward_level==0,"New-car config follows user-reported low-level forward mapping");
        check(HardwareConfig{}.motor_left_forward_level==0 && HardwareConfig{}.motor_right_forward_level==0,"C++ direction defaults match shipped config");
        check(c.imu_iio_device=="auto" && HardwareConfig{}.imu_iio_device=="auto","Shipped IMU selection is independent of IIO numbering");
        check(HardwareConfig{}.factory_write_readback==0,"Default output transport remains strict");
        check(c.factory_write_readback==1,"This board config explicitly enables verified output readback");
        {
            HardwareConfig invalid=c;invalid.factory_write_readback=2;
            rejects([&]{invalid.validate();},"Output compatibility flag is binary");
            ReadbackOutputIo io;io.populate(c);
            io.writePwmDuty(c.motor_left_pwm,2000);
            check(io.lastDuty(c.motor_left_pwm)==2000,"Status-zero PWM writes preserve uint16 payload");
            io.writePwmDuty(c.motor_left_pwm,0);
            check(io.readPwmMetadata(c.motor_left_pwm).duty==0,"Status-zero PWM stop requires zero readback");
            io.writeGpioLevel(c.motor_left_dir,1);
            check(io.writes.back().bytes==std::vector<uint8_t>{uint8_t('1')},"GPIO output remains ASCII despite binary readback");
            io.writeGpioLevel(c.motor_left_dir,0);
            check(io.readGpioLevel(c.motor_left_dir)==0,"Status-zero GPIO write verified");
            io.pwmStatus=2;io.gpioStatus=1;
            io.writePwmDuty(c.motor_right_pwm,1000);io.writeGpioLevel(c.motor_right_dir,1);
            check(io.readPwmMetadata(c.motor_right_pwm).duty==1000,"Conventional writes also require readback in compatibility mode");
            io.applyWrites=false;
            rejectsContaining([&]{io.writePwmDuty(c.motor_left_pwm,500);},"requested duty=500, observed=0","Stale PWM metadata rejects false success");
            rejectsContaining([&]{io.writeGpioLevel(c.motor_left_dir,1);},"requested level=1, observed=0","Unapplied GPIO command rejected");
            rejects([&]{io.writeGpioLevel(c.motor_left_dir,2);},"Invalid GPIO rejected before write");
            for(std::ptrdiff_t returned : {-1,1,3}) {
                const int readsBefore=io.reads[c.motor_left_pwm];
                rejects([&]{verifyFactoryPwmWrite(io,c.motor_left_pwm,0,returned);},"PWM short/error writes rejected even with matching zero metadata");
                check(io.reads[c.motor_left_pwm]==readsBefore,"Bad PWM status rejected before readback");
            }
            for(std::ptrdiff_t returned : {-1,2})
                rejects([&]{verifyFactoryGpioWrite(io,c.motor_left_dir,0,returned);},"GPIO error/oversized writes rejected");
            io.put(c.motor_left_pwm,pwmMetadataReadBuffer());
            rejects([&]{verifyFactoryPwmWrite(io,c.motor_left_pwm,0,0);},"Untouched PWM readback rejected");
            io.put(c.motor_left_pwm,PwmInfo{17000,0,10000,1000,58823,1000000000});
            rejects([&]{verifyFactoryPwmWrite(io,c.motor_left_pwm,0,0);},"Contradictory PWM timing rejected on write verification");
            io.put(c.motor_left_dir,uint8_t(0xff));
            rejects([&]{verifyFactoryGpioWrite(io,c.motor_left_dir,0,0);},"Invalid GPIO readback rejected");
        }
        {
            ReadbackOutputIo io;io.populate(c);FactoryBoard board(io,c,p);
            board.setMotorCommands(10000,10000);
            check(io.readPwmMetadata(c.motor_left_pwm).duty==2000 && io.readPwmMetadata(c.motor_right_pwm).duty==2000,
                  "Board outputs both raw2000 with verified status-zero transport");
            check(board.stop(),"Board stop succeeds with verified status-zero writes");
            check(io.readPwmMetadata(c.motor_left_pwm).duty==0 && io.readPwmMetadata(c.motor_right_pwm).duty==0,
                  "Both readbacks zero after verified stop");
            io.failWrite=c.motor_left_pwm;
            rejects([&]{board.setMotorCommands(5000,5000);},"Output failure still aborts actuation");
            check(io.lastDuty(c.motor_right_pwm)==0,"Left write failure still attempts right stop");
            io.failWrite.clear();
        }
        // Existing calibration fixtures use an explicit sensor path. Discovery is
        // exercised separately below, not faked by populating an 'auto/name' file.
        c.imu_iio_device="/sys/bus/iio/devices/iio:device7";
        {
            check(BenchOptions::parse({}).mode=="interfaces","Bench default is read-only inventory");
            check(BenchOptions::parse({"--help"}).help,"Bench help requires no device/config access");
            check(BenchOptions::parse({"--motor-zero"}).mode=="motor-zero","Zero-only output preflight has its own mode");
            for(const std::vector<std::string>& args : {
                    std::vector<std::string>{"--motor-zero","--raw-duty","2000"},
                    {"--motor-zero","--wheel","left"}, {"--motor-zero","--motor-pulse"},
                    {"--motor-zero","--seconds","5"}, {"--motor-zero","--wheels-raised"}})
                rejects([&]{BenchOptions::parse(args);},"Zero-only preflight rejects powered/input/mixed options");
            const std::vector<std::string> pulse={"--motor-pulse","--wheel","left","--raw-duty","1000","--duration-ms","300","--wheels-raised"};
            const auto accepted=BenchOptions::parse(pulse);
            check(accepted.mode=="motor-pulse" && accepted.wheel=="left" && accepted.rawDuty==1000,"Explicit capped bench pulse accepted");
            for(size_t index : {size_t(1),size_t(3),size_t(5),size_t(7)}) {
                auto incomplete=pulse;incomplete.erase(incomplete.begin()+index,incomplete.begin()+index+(index==7?1:2));
                rejects([&]{BenchOptions::parse(incomplete);},"Every powered-test safety/selection option is mandatory");
            }
            for(const std::string raw : {"2001","-2001","0.5","nan","inf","1000x"}) {
                auto invalid=pulse;invalid[4]=raw;
                rejects([&]{BenchOptions::parse(invalid);},"Invalid/overlarge raw duty rejected before hardware initialization");
            }
            for(const std::string ms : {"0","49","501","300.5"}) {
                auto invalid=pulse;invalid[6]=ms;
                rejects([&]{BenchOptions::parse(invalid);},"Invalid pulse duration rejected before writes");
            }
            auto mixed=pulse;mixed.push_back("--encoders");
            rejects([&]{BenchOptions::parse(mixed);},"Mixed bench modes cannot accidentally actuate");
            auto repeated=pulse;repeated.push_back("--raw-duty");repeated.push_back("-1000");
            rejects([&]{BenchOptions::parse(repeated);},"Duplicate duty options cannot silently reverse a pulse");
            rejects([&]{BenchOptions::parse({"--imu","--raw-duty","1000"});},"Read-only mode rejects motor options");
            rejects([&]{BenchOptions::parse({"--encoders","--turns","1"});},"PPR candidate requires selected wheel");
            rejects([&]{BenchOptions::parse({"--encoders","--seconds","121"});},"Sampling window bounded");
            rejects([&]{BenchOptions::parse({"--encoders","--interval-ms","0"});},"Invalid sampling interval rejected");
            rejects([&]{BenchOptions::parse({"--encoders","--wheel","both","--turns","1"});},"Two wheels cannot share a manual revolution declaration");
            const auto manual=BenchOptions::parse({"--encoders","--wheel","right","--turns","2","--seconds","15"});
            check(manual.wheel=="right" && manual.turns==2,"Manual PPR test records exact declared turns separately");
        }
        {
            EncoderBenchStats delta("delta",633186);delta.add(-20);delta.add(0);delta.add(30);
            check(delta.net()==10 && delta.absolute()==50 && delta.samples()==3,"Bench delta totals ignore untimed initial counts and preserve signs");
            EncoderBenchStats cumulative("cumulative32",std::numeric_limits<EncoderCount>::max()-9);
            cumulative.add(std::numeric_limits<EncoderCount>::min()+10);
            check(cumulative.net()==20 && cumulative.absolute()==20,"Bench cumulative totals retain 32-bit forward wrap");
            cumulative.add(std::numeric_limits<EncoderCount>::max()-9);
            check(cumulative.net()==0 && cumulative.absolute()==40,"Bench net vs absolute totals reveal back-and-forth motion");
            EncoderBenchStats narrow("cumulative16",32760);narrow.add(-32756);
            check(narrow.net()==20,"Bench legacy cumulative mode preserves signed-extended wrap");
            rejects([&]{narrow.add(633186);},"Bench legacy mode cannot truncate wide counts");
            check(narrow.samples()==1 && narrow.net()==20,"Failed bench sample does not commit totals");
            rejects([&]{EncoderBenchStats invalid("guess",0);},"Bench does not guess encoder mode");
            rejects([&]{delta.add(std::numeric_limits<EncoderCount>::min());},"Bench rejects saturated delta before suggesting PPR");
            EncoderBenchStats stationary("delta",0);stationary.add(0);stationary.add(0);
            check(stationary.net()==0 && stationary.absolute()==0,"Stationary bench totals remain zero");
        }
        {
            FakeIo io;io.populate(c);HardwareConfig automatic=c;automatic.imu_iio_device="auto";
            const std::string adc="/sys/bus/iio/devices/iio:device0",ultra="/sys/bus/iio/devices/iio:device1";
            io.iioPaths={ultra,c.imu_iio_device,adc};
            io.text[adc+"/name"]="1611c000.adc\n";io.text[ultra+"/name"]="ultrasonic_1\n";
            IioYaw yaw(io,automatic,1);
            check(yaw.devicePath()==c.imu_iio_device && yaw.model()=="IMU660RA","Auto discovery skips ADC/ultrasonic and resolves renumbered IMU");
            check(automatic.imu_iio_device=="auto","Discovery does not mutate caller config");
            check(!yaw.ready(),"Interface discovery does not falsely claim yaw calibration");
            std::ostringstream output;inspectIioImu(io,automatic,output);
            check(output.str().find("device="+c.imu_iio_device)!=std::string::npos,"Independent IMU check reports resolved path");
            check(output.str().find("not calibrated")!=std::string::npos,"IMU check labels validation boundary");
            check(io.writes.empty() && io.reads.empty(),"Independent IMU check performs no output/encoder/PWM transfers");
            for(const auto* model : {"IMU660RB","IMU963RA"}) {
                io.text[c.imu_iio_device+"/name"]=model;IioYaw supported(io,automatic,1);
                check(supported.model()==model,"All existing supported factory models discoverable");
            }
            const std::string second="/sys/bus/iio/devices/iio:device8";
            io.iioPaths.push_back(second);io.text[second+"/name"]="IMU660RA";
            rejectsContaining([&]{IioYaw rejected(io,automatic,1);},"Multiple supported IIO IMUs","Ambiguous discovery refuses first-device selection");
            IioYaw explicitImu(io,c,1);check(explicitImu.devicePath()==c.imu_iio_device,"Explicit verified path allowed when multiple IMUs exist");
            io.iioPaths={adc,ultra};
            rejectsContaining([&]{IioYaw rejected(io,automatic,1);},"ultrasonic_1","Missing IMU reports actual IIO inventory");
            io.iioPaths.clear();
            rejectsContaining([&]{IioYaw rejected(io,automatic,1);},"no IIO devices","Absent IIO subsystem produces actionable fault");
            HardwareConfig wrong=c;wrong.imu_iio_device=ultra;
            rejectsContaining([&]{IioYaw rejected(io,wrong,1);},"not a supported IMU","Explicit ultrasonic selection never silently falls back");
            io.iioPaths={c.imu_iio_device,second};io.text.erase(second+"/name");
            rejectsContaining([&]{IioYaw rejected(io,automatic,1);},"discovery incomplete","Unreadable candidate cannot hide ambiguous IMU");
            io.iioPaths={c.imu_iio_device,c.imu_iio_device};
            rejectsContaining([&]{IioYaw rejected(io,automatic,1);},"duplicate IIO","Duplicate inventory rejected");
            io.iioPaths={c.imu_iio_device};io.text[c.imu_iio_device+"/name"]=" \n";
            rejectsContaining([&]{IioYaw rejected(io,automatic,1);},"Empty IIO model","Empty model rejected");
            io.text[c.imu_iio_device+"/name"]="IMU660RA";io.text[c.imu_iio_device+"/in_anglvel_z_raw"]="32767";
            rejects([&]{std::ostringstream invalid;inspectIioImu(io,automatic,invalid);},"Independent check rejects saturated gyro sample");
            io.text[c.imu_iio_device+"/in_anglvel_z_raw"]="0.5";
            rejects([&]{std::ostringstream invalid;inspectIioImu(io,automatic,invalid);},"Independent check rejects fractional raw gyro sample");
            HardwareConfig serial=c;serial.imu_backend="serial";
            rejects([&]{std::ostringstream invalid;inspectIioImu(io,serial,invalid);},"IIO check never claims serial protocol validation");
            io.iioPaths={adc,ultra};std::ostringstream partial;
            rejects([&]{inspectHardware(io,automatic,partial);},"Whole-board check still fails on missing IMU");
            check(partial.str().find("Camera: "+c.camera_device)!=std::string::npos,"IMU fault no longer hides camera configuration");
            check(partial.str().find("Forward GPIO: left=0 right=0")!=std::string::npos,"Read-only check reports direction configuration distinctly from levels");
            check(io.writes.empty(),"Failed IMU discovery/check never writes outputs");
        }
        {
            GuardedDeviceRead<int16_t> buffer(int16_t(-32768));
            const int16_t count=-1;std::memcpy(buffer.data(),&count,sizeof(count));
            check(buffer.value("encoder")==-1,"Guarded read preserves legal two-byte data");
            auto* bytes=static_cast<uint8_t*>(buffer.data());bytes[sizeof(int16_t)]=0;
            rejects([&]{buffer.value("encoder");},"Encoder driver overwrite beyond two bytes detected");
            GuardedDeviceRead<uint8_t> gpio(uint8_t(0xff));
            auto* gpioBytes=static_cast<uint8_t*>(gpio.data());gpioBytes[0]=1;
            check(gpio.value("gpio")==1,"Guarded read preserves legal one-byte GPIO");
            gpioBytes[-1]=0;rejects([&]{gpio.value("gpio");},"Device underwrite detected");
            GuardedDeviceRead<PwmInfo> pwm(pwmMetadataReadBuffer());
            const PwmInfo metadata{17000,0,10000,0,58823,1000000000};
            std::memcpy(pwm.data(),&metadata,sizeof(metadata));
            check(pwm.value("pwm").freq==17000,"Guarded read preserves full PWM struct");
            static_cast<uint8_t*>(pwm.data())[sizeof(PwmInfo)]=0;
            rejects([&]{pwm.value("pwm");},"PWM overwrite detected");
            for(uint8_t pattern : {uint8_t(0xa5),uint8_t(0x5a)}) {
                GuardedDeviceRead<int16_t> extended(int16_t(-32768),pattern);
                check(!extended.guardDamaged(),"Custom initial padding is intact");
                check(extended.paddingBefore()[0]==pattern && extended.paddingAfter()[31]==pattern,"Diagnostic snapshots preserve padding seed");
                const int32_t wideCount=-1;std::memcpy(extended.data(),&wideCount,sizeof(wideCount));
                const auto after=extended.paddingAfter();
                check(extended.guardDamaged(),"Four-byte write into encoder payload detected with both patterns");
                check(after[0]==0xff && after[1]==0xff && after[2]==pattern,"Diagnostic tail identifies first two overwritten bytes");
                rejects([&]{extended.value("encoder");},"Wider encoder copy is not silently truncated");
            }
            GuardedDeviceRead<uint8_t> extendedGpio(uint8_t(0xff));
            const uint32_t wideLevel=1;std::memcpy(extendedGpio.data(),&wideLevel,sizeof(wideLevel));
            check(extendedGpio.guardDamaged(),"Four-byte GPIO write detected");
            check(extendedGpio.paddingAfter()[0]==0 && extendedGpio.paddingAfter()[2]==0,"GPIO overwrite raw bytes available to probe");
        }
        {
            ZeroStatusInputIo io;io.populate(c);io.put(c.encoder_left,EncoderCount(633186));
            const auto before=c.motor_left_dir;std::ostringstream output;
            inspectHardware(io,c,output);const auto text=output.str();
            check(text.find(c.encoder_left+" count=633186\n")!=std::string::npos,"Full inspection preserves board four-byte sample without narrowing");
            check(text.find(c.motor_left_dir+" level=0\n")!=std::string::npos,"Full inspection keeps left direction path after encoder reads");
            check(text.find(c.motor_right_dir+" level=0\n")!=std::string::npos,"Full inspection keeps right direction path after encoder reads");
            check(text.find(c.key_prefix+"3 level=1\n")!=std::string::npos,"Full inspection includes all keys");
            check(text.find(c.switch_prefix+"1 level=1\n")!=std::string::npos,"Full inspection includes all switches");
            check(text.find("IIO model=IMU660RA")!=std::string::npos,"Full inspection reaches IIO");
            check(text.find("Camera: "+c.camera_device)!=std::string::npos,"Full inspection reaches camera metadata");
            check(io.writes.empty(),"Full hardware inspection does not write outputs");
            check(io.reads[c.encoder_left]==1 && io.reads[c.encoder_right]==1,"Full inspection reads each encoder only once");
            check(c.motor_left_dir==before,"Inspection does not mutate configured direction path");
            io.binary.erase(c.motor_left_dir);std::ostringstream failedOutput;
            rejects([&]{inspectHardware(io,c,failedOutput);},"Missing direction device fails inspection");
            check(failedOutput.str().find(" level=")==std::string::npos,"Failed GPIO read does not print an incomplete row");
            HardwareConfig empty=c;empty.motor_left_dir.clear();std::ostringstream emptyOutput;
            const auto readsBefore=io.reads;
            rejects([&]{inspectHardware(io,empty,emptyOutput);},"Empty direction path rejected before I/O");
            check(io.reads==readsBefore && emptyOutput.str().empty(),"Invalid config fails before any device read");
        }
        {
            for(EncoderCount count : {EncoderCount(-32768),EncoderCount(-1),EncoderCount(0),EncoderCount(1),EncoderCount(32767),EncoderCount(-23131),EncoderCount(633186),EncoderCount(-633186),std::numeric_limits<EncoderCount>::min(),std::numeric_limits<EncoderCount>::max()}) {
                check(decodeEncoderRead(count,0,"encoder")==count,"Zero-status encoder preserves every tested raw value");
                check(decodeEncoderRead(count,4,"encoder")==count,"Conventional four-byte encoder preserves raw value");
            }
            for(std::ptrdiff_t returned : {-1,1,2,3,5,8})
                rejects([&]{decodeEncoderRead(0,returned,"encoder");},"Negative/partial encoder transfers rejected");
            for(uint8_t raw : {uint8_t(0),uint8_t(1),uint8_t('0'),uint8_t('1')}) {
                const auto expected=raw=='0' || raw=='1' ? uint8_t(raw-'0') : raw;
                check(decodeGpioRead(raw,0,"gpio")==expected,"Zero-status binary/ASCII GPIO normalized");
                check(decodeGpioRead(raw,1,"gpio")==expected,"Conventional binary/ASCII GPIO normalized");
            }
            for(uint8_t raw : {uint8_t(2),uint8_t(7),uint8_t(0xff)}) {
                rejects([&]{decodeGpioRead(raw,0,"gpio");},"Invalid/unwritten zero-status GPIO rejected");
                rejects([&]{decodeGpioRead(raw,1,"gpio");},"Invalid conventional GPIO rejected");
            }
            for(std::ptrdiff_t returned : {-1,2})
                rejects([&]{decodeGpioRead(0,returned,"gpio");},"Negative/oversized GPIO transfers rejected");
            FakeIo ordinary;ordinary.put("encoder",EncoderCount(-1));
            check(ordinary.readEncoderCount("encoder")==-1,"Default typed encoder preserves negative count");
            check(ordinary.reads["encoder"]==1,"Default typed encoder reads once");
            for(size_t bytes : {size_t(0),size_t(1),size_t(2),size_t(3),size_t(5),size_t(8)}) {
                ordinary.binary["encoder"].resize(bytes);
                rejects([&]{ordinary.readEncoderCount("encoder");},"Default typed encoder rejects every non-four-byte fixture");
            }
            ordinary.put("gpio",uint8_t('1'));check(readGpio(ordinary,"gpio")==1,"Default typed GPIO routes and normalizes ASCII");
            ordinary.binary["gpio"].clear();rejects([&]{readGpio(ordinary,"gpio");},"Default typed GPIO rejects empty binary data");
        }
        {
            ZeroStatusInputIo io;io.populate(c);FactoryBoard board(io,c,p);
            check(io.reads[c.encoder_left]==1 && io.reads[c.encoder_right]==1,"Zero-status initialization reads each encoder once");
            board.sampleWheels(1);io.put(c.encoder_left,EncoderCount(-20));io.put(c.encoder_right,EncoderCount(20));
            const auto wheels=board.sampleWheels(1.02);
            near(wheels.leftRps,20/(1024*.02),"Zero-status encoder integrates real negative count with sign");
            near(wheels.rightRps,wheels.leftRps,"Zero-status right encoder speed matches");
            near(wheels.distanceM,20*.2/1024,"Zero-status encoder odometry preserved");
            check(io.reads[c.encoder_left]==3 && io.reads[c.encoder_right]==3,"Sampling never double-reads clearing encoder");
            near(board.sampleWheels(1.04).leftRps,0,"Zero-status legitimate stationary count remains zero");
            io.encoderStatus=1;rejects([&]{board.sampleWheels(1.06);},"Positive short status still aborts wheel sample");
            io.encoderStatus=0;io.put(c.key_prefix+"0",uint8_t(0xff));
            rejects([&]{board.stopInputActive();},"Unwritten/invalid GPIO does not become idle or pressed");
        }
        {
            HardwareConfig cumulative=c;cumulative.encoder_mode="cumulative16";
            ZeroStatusInputIo io;io.populate(c);io.clearCounts=false;FactoryBoard board(io,cumulative,p);
            io.put(c.encoder_left,EncoderCount(32767));io.put(c.encoder_right,EncoderCount(-32768));board.sampleWheels(2);
            io.put(c.encoder_left,EncoderCount(32766));io.put(c.encoder_right,EncoderCount(-32767));
            const auto wheels=board.sampleWheels(2.02);
            near(wheels.leftRps,1/(1024*.02),"Zero-status cumulative extrema are not reserved sentinels");
            near(wheels.rightRps,wheels.leftRps,"Zero-status cumulative extrema preserve motion");
        }
        {
            const std::array<uint8_t,4> boardRaw{{0x62,0xa9,0x09,0x00}};
            GuardedDeviceRead<EncoderCount> buffer(std::numeric_limits<EncoderCount>::min());
            std::memcpy(buffer.data(),boardRaw.data(),boardRaw.size());
            check(buffer.value("encoder")==633186,"Board little-endian sample retains high bytes");
            check(!buffer.guardDamaged(),"Four-byte board sample no longer overruns its payload");
            const int64_t oversized=0;std::memcpy(buffer.data(),&oversized,sizeof(oversized));
            rejects([&]{buffer.value("encoder");},"Eight-byte encoder copy still rejected");
            for(EncoderCount count : {EncoderCount(633186),EncoderCount(-633186),std::numeric_limits<EncoderCount>::min(),std::numeric_limits<EncoderCount>::max()})
                check(encoderCountDelta(count,0,"delta")==count,"Delta conversion retains full signed 32-bit value");
            check(encoderCountDelta(-32768,32767,"cumulative16")==1,"Legacy 16-bit forward wrap remains valid");
            check(encoderCountDelta(32767,-32768,"cumulative16")==-1,"Legacy 16-bit reverse wrap remains valid");
            rejects([&]{encoderCountDelta(633186,0,"cumulative16");},"Cumulative16 refuses to truncate wider counter");
            rejects([&]{encoderCountDelta(0,-633186,"cumulative16");},"Cumulative16 checks previous counter too");
            rejects([&]{encoderCountDelta(-32768,0,"cumulative16");},"Ambiguous 16-bit half-range jump rejected");
            const auto low=std::numeric_limits<EncoderCount>::min(),high=std::numeric_limits<EncoderCount>::max();
            check(encoderCountDelta(low,high,"cumulative32")==1,"32-bit forward wrap avoids signed overflow");
            check(encoderCountDelta(high,low,"cumulative32")==-1,"32-bit reverse wrap avoids signed overflow");
            check(encoderCountDelta(633186,633186,"cumulative32")==0,"Wide cumulative standstill stays zero");
            rejects([&]{encoderCountDelta(low,0,"cumulative32");},"Ambiguous 32-bit half-range jump rejected");
            rejects([&]{encoderCountDelta(0,0,"guess");},"Unknown count-delta semantics rejected");
        }
        {
            ZeroStatusInputIo io;io.populate(c);io.put(c.encoder_left,EncoderCount(633186));
            FactoryBoard board(io,c,p);board.sampleWheels(1);
            const auto stopped=board.sampleWheels(1.02);
            near(stopped.leftRps,0,"Stale wide startup count discarded rather than integrated");
            near(stopped.distanceM,0,"Stale startup count cannot become phantom distance");
            io.put(c.encoder_left,EncoderCount(633186));
            rejects([&]{board.sampleWheels(1.04);},"Wide implausible delta fails speed validation instead of narrowing");
            io.put(c.encoder_left,EncoderCount(-20));io.put(c.encoder_right,EncoderCount(20));
            const auto recovered=board.sampleWheels(1.06);
            near(recovered.leftRps,20/(1024*.04),"Rejected sample does not commit timestamp or baseline");
            io.put(c.encoder_right,std::numeric_limits<EncoderCount>::min());
            rejects([&]{board.sampleWheels(1.08);},"Minimum 32-bit delta saturation rejected on right wheel");
            io.put(c.encoder_left,std::numeric_limits<EncoderCount>::min());
            rejects([&]{board.sampleWheels(1.10);},"Minimum 32-bit delta saturation rejected on left wheel");
            io.encoderStatus=2;
            rejects([&]{board.sampleWheels(1.12);},"Old two-byte return not accepted as complete 32-bit data");
        }
        {
            ZeroStatusInputIo io;io.populate(c);FactoryBoard board(io,c,p);board.sampleWheels(1);
            io.put(c.encoder_left,EncoderCount(65536));
            rejects([&]{board.sampleWheels(1.02);},"Nonzero full-width delta with zero low word cannot look stationary");
            io.put(c.encoder_right,std::numeric_limits<EncoderCount>::max());
            rejects([&]{board.sampleWheels(1.04);},"Maximum 32-bit delta saturation rejected on right wheel");
        }
        {
            Params highResolution=p;highResolution.encoder_ppr=20000;highResolution.max_encoder_rps=100;highResolution.validate();
            ZeroStatusInputIo io;io.populate(c);FactoryBoard board(io,c,highResolution);board.sampleWheels(2);
            io.put(c.encoder_left,EncoderCount(-40000));io.put(c.encoder_right,EncoderCount(40000));
            const auto forward=board.sampleWheels(2.1);
            near(forward.leftRps,20,"Valid wide forward count is not a negative int16 value");
            near(forward.rightRps,20,"Valid wide right count reaches speed calculation intact");
            near(forward.distanceM,.4,"Wide counts preserve odometry units");
            io.put(c.encoder_left,EncoderCount(40000));io.put(c.encoder_right,EncoderCount(-40000));
            const auto reverse=board.sampleWheels(2.2);
            near(reverse.leftRps,-20,"Valid wide reverse count preserves sign");
            near(reverse.distanceM,.4,"Wide reverse distance remains magnitude of signed average");
            HardwareConfig narrow=c;narrow.encoder_mode="cumulative16";
            ZeroStatusInputIo narrowIo;narrowIo.populate(c);FactoryBoard narrowBoard(narrowIo,narrow,highResolution);narrowBoard.sampleWheels(2);
            rejects([&]{narrowBoard.sampleWheels(2.1);},"Cumulative16 still enforces representable sampling limit");
        }
        {
            HardwareConfig cumulative=c;cumulative.encoder_mode="cumulative32";cumulative.validate();
            ZeroStatusInputIo io;io.populate(c);io.clearCounts=false;
            const auto low=std::numeric_limits<EncoderCount>::min(),high=std::numeric_limits<EncoderCount>::max();
            io.put(c.encoder_left,high-9);io.put(c.encoder_right,low+9);
            FactoryBoard board(io,cumulative,p);board.sampleWheels(3);
            io.put(c.encoder_left,low+10);io.put(c.encoder_right,high-10);
            const auto backward=board.sampleWheels(3.02);
            near(backward.leftRps,-20/(1024*.02),"32-bit wrap reaches full controller with left sign");
            near(backward.rightRps,backward.leftRps,"Both wrap directions yield consistent physical speed");
            near(backward.distanceM,20*.2/1024,"32-bit wrapping odometry matches count difference");
            near(board.sampleWheels(3.04).leftRps,0,"Unchanged 32-bit cumulative count is stationary");
            check(io.reads[c.encoder_left]==4 && io.reads[c.encoder_right]==4,"32-bit cumulative mode still reads once per sample");
            HardwareConfig narrow=c;narrow.encoder_mode="cumulative16";
            ZeroStatusInputIo invalid;invalid.populate(c);invalid.put(c.encoder_left,EncoderCount(633186));
            rejects([&]{FactoryBoard rejected(invalid,narrow,p);},"Legacy cumulative mode refuses wider startup baseline");
            check(invalid.lastDuty(c.motor_left_pwm)==0 && invalid.lastDuty(c.motor_right_pwm)==0,"Invalid startup baseline keeps both motors zero");
        }
        {
            const PwmInfo motor{17000,0,10000,0,58823,1000000000};
            const PwmInfo servo{300,4470,10000,1489999,3333333,1000000000};
            validatePwmMetadataRead(motor,0,"motor");check(true,"Board zero-return motor metadata accepted");
            validatePwmMetadataRead(servo,0,"servo");check(true,"Board zero-return servo metadata accepted");
            validatePwmMetadataRead(servo,24,"servo");check(true,"Conventional complete PWM read accepted");
            rejects([&]{validatePwmMetadataRead(pwmMetadataReadBuffer(),0,"motor");},"Untouched zero-return PWM buffer rejected");
            for(std::ptrdiff_t count : {-1,1,2,8,23,25})
                rejects([&]{validatePwmMetadataRead(servo,count,"servo");},"Errors and partial PWM transfers rejected despite valid-looking data");
            const auto poison=pwmMetadataReadBuffer();
            for(size_t bytes=0;bytes<sizeof(servo);++bytes) {
                PwmInfo partial=poison;std::memcpy(&partial,&servo,bytes);
                rejects([&]{validatePwmMetadataRead(partial,0,"servo");},"Partial zero-return copy rejected");
            }
            for(int field=0;field<6;++field) {
                PwmInfo incomplete=servo;
                switch(field) {
                    case 0: incomplete.freq=poison.freq;break;
                    case 1: incomplete.duty=poison.duty;break;
                    case 2: incomplete.duty_max=poison.duty_max;break;
                    case 3: incomplete.duty_ns=poison.duty_ns;break;
                    case 4: incomplete.period_ns=poison.period_ns;break;
                    case 5: incomplete.clk_freq=poison.clk_freq;break;
                }
                rejects([&]{validatePwmMetadataRead(incomplete,0,"servo");},"Each untouched metadata field rejected");
            }
            PwmInfo bad=servo;bad.period_ns+=100;
            rejects([&]{validatePwmMetadataRead(bad,0,"servo");},"Inconsistent period rejected");
            bad=servo;bad.duty_ns+=100;
            rejects([&]{validatePwmMetadataRead(bad,0,"servo");},"Inconsistent pulse width rejected");
            bad=servo;bad.clk_freq=0;
            rejects([&]{validatePwmMetadataRead(bad,0,"servo");},"Missing clock rejected");
            bad=servo;bad.duty=bad.duty_max+1;
            rejects([&]{validatePwmMetadataRead(bad,0,"servo");},"Out-of-range duty rejected");
            FakeIo io;io.put("servo",servo);
            const auto ordinary=readPwmInfo(io,"servo");
            check(ordinary.freq==servo.freq && ordinary.duty==servo.duty,"Default DeviceIo typed PWM read preserves metadata");
            io.binary["servo"].resize(23);
            rejects([&]{readPwmInfo(io,"servo");},"Default DeviceIo still rejects short metadata");
        }
        const PwmInfo info{300,0,10000,0,3333333,100000000};
        check(motorDuty(12000,50000,info)==2400,"PID command retains physical 24 percent limit");
        check(motorDuty(-12000,50000,info)==2400,"Reverse uses positive magnitude");
        check(motorDuty(2000,50000,info)==400,"Controller command 2000 is not raw PWM 2000");
        check(motorDuty(10000,50000,info)==2000,"Controller command 10000 maps to raw PWM 2000");
        check(motorDuty(1e6,50000,info)==10000,"PWM saturation respects driver's range");
        check(servoDuty(0,c,info)==4470,"Factory center pulse");
        check(servoDuty(-22,c,info)==5170,"Factory left pulse");
        check(servoDuty(22,c,info)==3370,"Factory right pulse");
        PwmInfo slow=info;slow.freq=50;check(servoDuty(0,c,slow)==745,"Same pulse duration at 50 Hz");
        rejects([&]{motorDuty(std::numeric_limits<double>::quiet_NaN(),50000,info);},"NaN command rejected");
        HardwareConfig bad=c;bad.servo_right_us=bad.servo_center_us;rejects([&]{bad.validate();},"Misordered servo calibration rejected");
        bad=c;bad.motor_right_pwm=bad.motor_left_pwm;rejects([&]{bad.validate();},"Aliased motor outputs rejected");
        bad=c;bad.encoder_mode="guess";rejects([&]{bad.validate();},"Unknown encoder semantics rejected");
        bad=c;bad.stop_key_index=4;rejects([&]{bad.validate();},"Invalid stop key rejected");
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Startup zeros both motors");
            check(io.lastDuty(c.servo_pwm)==4470,"Startup straight pulse");
            io.writes.clear();board.setMotorCommands(-12000,12000);
            check(io.writes.size()==4 && io.writes[0].path==c.motor_left_pwm && io.writes[1].path==c.motor_left_dir && io.writes[1].bytes==std::vector<uint8_t>{'1'},"Reverse changes direction to ASCII 1 after zero PWM");
            check(io.lastDuty(c.motor_left_pwm)==2400,"Output scales to driver");
            board.sampleWheels(1);io.put(c.encoder_left,EncoderCount(-20));io.put(c.encoder_right,EncoderCount(20));
            auto s=board.sampleWheels(1.02);near(s.leftRps,20/(1024*.02),"Left sign corrected");
            near(s.rightRps,s.leftRps,"Wheel units match");near(s.distanceM,20*.2/1024,"Distance comes from counts");
            s=board.sampleWheels(1.04);near(s.leftRps,0,"Cleared counts produce stopped speed");near(s.distanceM,0,"No phantom stopped odometry");
            io.put(c.encoder_left,EncoderCount(10));io.put(c.encoder_right,EncoderCount(-10));s=board.sampleWheels(1.09);
            near(s.leftRps,-10/(1024*.05),"Reverse uses actual interval");near(s.distanceM,10*.2/1024,"Reverse travel is positive");
            rejects([&]{board.sampleWheels(1.3);},"Long sampling gap rejected");
            board.setBeep(true);check(io.writes.back().bytes==std::vector<uint8_t>{'1'},"Buzzer uses factory GPIO");
            check(board.stop(),"Shutdown succeeds");check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Shutdown zeros both sides");
        }
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);
            auto lastDirection=[&](const std::string& path) {
                for(auto i=io.writes.rbegin();i!=io.writes.rend();++i)
                    if(i->path==path) {check(i->bytes.size()==1,"Direction write is exactly one ASCII byte");return i->bytes[0];}
                throw std::runtime_error("Missing direction write");
            };
            board.setMotorCommands(10000,10000);
            check(lastDirection(c.motor_left_dir)=='0' && lastDirection(c.motor_right_dir)=='0',"Positive commands select physical forward mapping for BOTH wheels");
            check(io.lastDuty(c.motor_left_pwm)==2000 && io.lastDuty(c.motor_right_pwm)==2000,"Both wheels use identical raw-duty scaling");
            io.writes.clear();board.setMotorCommands(-10000,-10000);
            check(io.writes.size()==6,"Each reversing wheel gets zero, direction, duty");
            for(size_t start : {size_t(0),size_t(3)}) {
                check(io.writes[start].bytes==std::vector<uint8_t>({0,0}),"Direction change begins with zero PWM");
                check(io.writes[start+1].bytes==std::vector<uint8_t>{'1'},"Both reverse GPIO writes are ASCII 1");
            }
            board.setMotorCommands(0,0);
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Zero commands stop both wheels after reverse");
            HardwareConfig alternative=c;alternative.motor_left_forward_level=1;alternative.motor_right_forward_level=1;
            FakeIo other;other.populate(alternative);FactoryBoard configurable(other,alternative,p);
            configurable.setMotorCommands(-10000,-10000);
            bool leftLow=false,rightLow=false;
            for(const auto& write : other.writes) {
                if(write.path==alternative.motor_left_dir && write.bytes==std::vector<uint8_t>{'0'}) leftLow=true;
                if(write.path==alternative.motor_right_dir && write.bytes==std::vector<uint8_t>{'0'}) rightLow=true;
            }
            check(leftLow && rightLow,"Explicit alternative vehicle direction mapping remains configurable");
        }
        {
            FakeIo io;io.populate(c);io.binary.erase(c.motor_right_pwm);
            rejects([&]{FactoryBoard board(io,c,p);},"Partial initialization failure propagates");
            check(io.lastDuty(c.motor_left_pwm)==0,"Partially initialized motor stopped");
            check(io.lastDuty(c.motor_right_pwm)==0,"Unreadable metadata still receives zero command");
        }
        {
            FakeIo io;io.populate(c);io.binary.erase(c.motor_left_pwm);
            rejects([&]{FactoryBoard board(io,c,p);},"First metadata failure propagates");
            check(io.lastDuty(c.motor_right_pwm)==0,"First metadata failure cannot skip other motor stop");
        }
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);board.setMotorCommands(1000,1000);
            io.failWrite=c.motor_left_pwm;io.failOnce=true;
            rejects([&]{board.setMotorCommands(2000,2000);},"Write failure propagates");
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Fault rollback stops both motors");
            io.failWrite=c.motor_left_pwm;check(!board.stop(),"Failed shutdown reported");
            check(io.lastDuty(c.motor_right_pwm)==0,"One failure cannot block other shutdown");
        }
        {
            HardwareConfig stopConfig=c;stopConfig.stop_key_index=0;
            FakeIo io;io.populate(c);FactoryBoard board(io,stopConfig,p);
            check(!board.stopInputActive(),"Idle key not stopping");io.put(c.key_prefix+"0",uint8_t(0));
            check(board.stopInputActive(),"Assigned low-active key stops");
            io.put(c.key_prefix+"0",uint8_t(7));rejects([&]{board.stopInputActive();},"Invalid GPIO is not silently accepted");
        }
        {
            HardwareConfig cumulative=c;cumulative.encoder_mode="cumulative16";
            FakeIo io;io.populate(c);io.clearCounts=false;FactoryBoard board(io,cumulative,p);
            io.put(c.encoder_left,EncoderCount(-32760));io.put(c.encoder_right,EncoderCount(32760));board.sampleWheels(2);
            io.put(c.encoder_left,EncoderCount(32756));io.put(c.encoder_right,EncoderCount(-32756));auto s=board.sampleWheels(2.02);
            near(s.leftRps,20/(1024*.02),"Cumulative counter wraps backward");near(s.rightRps,s.leftRps,"Cumulative wraps forward");
            s=board.sampleWheels(2.04);near(s.leftRps,0,"Unchanged cumulative counter stopped");
        }
        {
            FakeIo io;io.populate(c);FactoryBoard board(io,c,p);board.sampleWheels(3);
            io.put(c.encoder_left,std::numeric_limits<EncoderCount>::max());rejects([&]{board.sampleWheels(3.02);},"Saturated 32-bit delta count rejected");
            io.binary[c.encoder_left].resize(1);rejects([&]{board.sampleWheels(3.04);},"Short encoder count is not fabricated as zero");
        }
        {
            FakeIo io;io.populate(c);io.binary[c.servo_pwm].resize(2);
            rejects([&]{FactoryBoard board(io,c,p);},"Short metadata read rejected");
            check(io.lastDuty(c.motor_left_pwm)==0 && io.lastDuty(c.motor_right_pwm)==0,"Metadata failure leaves motors zero");
        }
        {
            HardwareConfig gyro=c;gyro.imu_calibration_samples=20;gyro.imu_gyro_scale_deg_s=.1;
            FakeIo io;io.populate(c);const auto raw=c.imu_iio_device+"/in_anglvel_z_raw";
            io.text[raw]="10";IioYaw yaw(io,gyro,1);
            for(int i=0;i<20;++i) yaw.poll(i*.02,true);
            check(yaw.ready() && yaw.fresh(.38),"Stationary calibration completes");near(yaw.value(),0,"Calibration adds no yaw");
            io.text[raw]="110";yaw.poll(.40,false);yaw.poll(.42,false);near(yaw.value(),.3,"Bias-corrected integration uses actual time");
            check(!yaw.fresh(.6),"Stale yaw invalid");rejects([&]{yaw.poll(.7,false);},"Gyro gap stops continuity");
            rejects([&]{yaw.poll(.72,false);},"Gyro cannot silently restart");
            IioYaw moving(io,gyro,1);rejects([&]{moving.poll(1,false);},"Calibration requires stationary wheels");
        }
        {
            FakeIo io;io.populate(c);io.text[c.imu_iio_device+"/in_anglvel_scale"]="invalid";
            rejects([&]{IioYaw yaw(io,c,1);},"Scale is not guessed");
            io.text[c.imu_iio_device+"/in_anglvel_scale"]="0.001";
            io.text[c.imu_iio_device+"/in_anglvel_z_scale"]="bad";
            rejects([&]{IioYaw yaw(io,c,1);},"Corrupt per-axis scale is not masked by valid global scale");
            io.text[c.imu_iio_device+"/in_anglvel_z_scale"]="nan";
            rejects([&]{IioYaw yaw(io,c,1);},"Nonfinite per-axis scale is not silently replaced");
            io.text.erase(c.imu_iio_device+"/in_anglvel_z_scale");
            io.text[c.imu_iio_device+"/name"]="wrong_sensor";rejects([&]{IioYaw yaw(io,c,1);},"Unknown model rejected");
        }
        {
            HardwareConfig gyro=c;gyro.imu_calibration_samples=20;
            FakeIo io;io.populate(c);io.text[c.imu_iio_device+"/in_anglvel_scale"]="0.001";
            IioYaw yaw(io,gyro,-1);for(int i=0;i<20;++i) yaw.poll(i*.02,true);
            io.text[c.imu_iio_device+"/in_anglvel_z_raw"]="100";yaw.poll(.4,false);
            near(yaw.value(),-0.1*180/pi*.01,"Auto IIO scale converts radians to degrees with yaw sign");
            io.text[c.imu_iio_device+"/in_anglvel_z_raw"]="bad";
            rejects([&]{yaw.poll(.42,false);},"Malformed gyro sample fails");
            check(!yaw.fresh(.42),"Malformed gyro sample latches invalid yaw");
        }
        {
            HardwareConfig gyro=c;gyro.imu_calibration_samples=20;gyro.imu_gyro_scale_deg_s=1;
            FakeIo io;io.populate(c);const auto raw=c.imu_iio_device+"/in_anglvel_z_raw";
            IioYaw yaw(io,gyro,1);for(int i=0;i<19;++i) {io.text[raw]=(i%2?"10":"-10");yaw.poll(i*.02,true);}
            rejects([&]{yaw.poll(.38,true);},"Noisy/moving calibration rejected");
        }
        std::cout<<"PASS "<<checks<<" hardware checks using simulated devices (no board attached)\n";return 0;
    }catch(const std::exception& e) {std::cerr<<"FAIL after "<<checks<<" hardware checks: "<<e.what()<<'\n';return 1;}
}
