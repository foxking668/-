#include "ZebraRuntime.hpp"
#include "ZebraDetector.hpp"
#include "IoStatus.hpp"
#include "Experiment/VideoRecorder.hpp"
#include "Experiment/ProcessedRecording.hpp"
#include "Contral/Motor/Motor.hpp"
#include "Other/Voice/WonderEcho.hpp"
#include "Other/Global/Global.hpp"
#include <opencv2/imgproc.hpp>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <mutex>
#include <sstream>
#include <thread>
#include <ctime>
#include <map>

namespace ZebraTest {
namespace {
namespace fs=std::filesystem;
long long nowMs() {return std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
std::string json(const std::string& value) {
    std::ostringstream o;o<<'"';for(unsigned char c:value) {
        if(c=='"'||c=='\\')o<<'\\'<<c;
        else if(c<32)o<<"\\u"<<std::hex<<std::setw(4)<<std::setfill('0')<<int(c)<<std::dec;
        else o<<c;
    }o<<'"';return o.str();
}
std::string csvText(const std::string& value) {
    if(value.find_first_of(",\"\r\n")==std::string::npos)return value;
    std::string out="\"";for(char c:value){if(c=='\"')out+='\"';out+=c;}return out+'\"';
}
// All writes, flushes and storage probes belong to this worker, never to motor control.
class LogWriter {
    struct Item {int file;std::string line;};
    std::deque<Item> queue_;std::mutex mutex_;std::condition_variable wake_;
    std::thread thread_;bool closing_=false;Settings settings_;std::string directory_;
    std::string error_;
    std::ofstream files_[3];std::atomic<bool> failed_{false};std::atomic<unsigned long long> dropped_{0};
public:
    ~LogWriter(){close();}
    bool start(const std::string& directory,const Settings& s,std::string& error) {
        directory_=directory;settings_=s;
        const char* names[]={"frames.csv","control.csv","events.jsonl"};
        const char* heads[]={"frame_id,capture_ns,phase,event_count,lane_valid,input_valid,far_valid,far_hit,near_valid,near_hit,strong,far_rows,near_rows,stripes,components,black_ratio,contrast,nearest_y,steer_error,hold_remaining_ms,reason,far_reason,near_reason,far_confirm_frames,near_confirm_frames,strong_confirm_frames,illumination_band_rows,snapshot_latency_ms,raw_submit,diagnostic_submit",
            "time_ms,phase,event_count,target_rps,left_rps,right_rps,left_pwm_command,right_pwm_command,left_write_ok,right_write_ok,hold_remaining_ms,reason",""};
        for(int i=0;i<3;++i) {
            files_[i].open(directory+"/"+names[i],std::ios::out);
            if(heads[i][0])files_[i]<<heads[i]<<'\n';files_[i].flush();
            if(!files_[i]){error="LOG_OPEN: "+std::string(names[i]);return false;}
        }
        thread_=std::thread([this]{run();});return true;
    }
    bool put(int file,const std::string& line) {
        std::lock_guard<std::mutex> lock(mutex_);
        if(failed_||closing_)return false;
        if(queue_.size()>=std::size_t(settings_.log_queue)){++dropped_;failed_=true;if(error_.empty())error_="LOG_QUEUE_OVERFLOW";return false;}
        queue_.push_back({file,line});wake_.notify_one();return true;
    }
    bool bad() const {return failed_;}
    std::string error() {std::lock_guard<std::mutex> lock(mutex_);return error_;}
    unsigned long long dropped() const {return dropped_;}
    void close() {
        {std::lock_guard<std::mutex> lock(mutex_);closing_=true;}wake_.notify_one();
        if(thread_.joinable())thread_.join();
    }
private:
    void failLog(const std::string& reason) {
        failed_=true;std::lock_guard<std::mutex> lock(mutex_);
        if(error_.empty()){error_=reason;std::cerr<<"[ZebraTest] "<<reason<<std::endl;}
    }
    void run() {
        auto lastFlush=nowMs(),lastSpace=lastFlush;
        std::error_code initialError;auto parameterTime=fs::last_write_time("image_params.txt",initialError);unsigned parameterRevision=0;
        if(initialError)failLog("PARAM_SNAPSHOT_STAT_FAILED: "+initialError.message());
        for(;;) {
            std::deque<Item> items;bool done=false;
            {std::unique_lock<std::mutex> lock(mutex_);wake_.wait_for(lock,std::chrono::milliseconds(settings_.flush_ms),[this]{return closing_||!queue_.empty();});
             items.swap(queue_);done=closing_&&items.empty();}
            for(const auto& row:items){errno=0;files_[row.file]<<row.line<<'\n';if(!files_[row.file])failLog("LOG_WRITE_FAILED file="+std::to_string(row.file)+" errno="+std::to_string(errno));
                if(row.file==2)std::cout<<"[ZebraTest] "<<row.line<<std::endl;}
            const auto now=nowMs();
            if(now-lastFlush>=settings_.flush_ms||done){for(auto& f:files_){f.flush();if(!f)failLog("LOG_FLUSH_FAILED errno="+std::to_string(errno));}lastFlush=now;}
            if(now-lastSpace>=settings_.storage_check_ms) {
                std::error_code ec;const auto space=fs::space(directory_,ec);
                if(ec||space.available<static_cast<std::uintmax_t>(settings_.minimum_free_mb)*1024*1024) {
                    failLog("STORAGE_ERROR: "+(ec?ec.message():"free space below configured limit"));files_[2]<<"{\"time_ms\":"<<now<<",\"code\":\"STORAGE_ERROR\",\"detail\":"<<json(ec?ec.message():"free space below configured limit")<<"}\n";
                }
                const auto changed=fs::last_write_time("image_params.txt",ec);
                if(!ec&&changed!=parameterTime) {
                    parameterTime=changed;
                    const auto name="image_params.observed_"+std::to_string(++parameterRevision)+".txt";
                    fs::copy_file("image_params.txt",fs::path(directory_)/"config_snapshot"/name,ec);
                    files_[2]<<"{\"time_ms\":"<<now<<",\"code\":\"PARAM_FILE_CHANGE_OBSERVED\",\"detail\":"<<json(ec?ec.message():name)<<"}\n";
                    if(ec)failLog("PARAM_SNAPSHOT_FAILED: "+ec.message());
                }
                lastSpace=now;
            }
            if(done)break;
        }
        for(auto& f:files_)f.close();
    }
};
Settings settings;std::unique_ptr<Controller> controller;std::mutex stateMutex;
LogWriter logger;Experiment::VideoRecorder rawVideo,debugVideo;
std::atomic<bool> enabled{false},recordReady{false};std::atomic<double> speedLimit{0};
std::atomic<bool> finalFailure{false};
std::string directory,lastState,lastReason;std::uint64_t frameIndex=0;
long long lastControlLog=-1,lastSpeedTime=-1;double lastLeft=0,lastRight=0;
bool previousWrites=true;unsigned voicedCount=0;
void event(const std::string& code,const std::string& detail,long long time=nowMs()) {
    static std::mutex eventMutex;static std::map<std::string,std::pair<long long,unsigned>> repeated;
    unsigned count=1;
    if(code=="MOTOR_WRITE_FAILED"||code=="ENCODER_READ_FAILED") {
        std::lock_guard<std::mutex> lock(eventMutex);auto& seen=repeated[code+detail];++seen.second;
        if(seen.first>0&&time-seen.first<1000)return;
        count=seen.second;seen={time,0};
    }
    logger.put(2,"{\"time_ms\":"+std::to_string(time)+",\"code\":"+json(code)+",\"detail\":"+json(detail)+",\"repeat_count\":"+std::to_string(count)+"}");
}
void stateEvent() {
    const std::string state=phaseName(controller->phase()),reason=controller->reason();
    if(state!=lastState||reason!=lastReason) {
        event("STATE",lastState+" -> "+state+" reason="+reason);
        if(controller->phase()==Phase::Hold)event("YIELD_MESSAGE","人行道前停车礼让行人");
        lastState=state;lastReason=reason;
    }
}
void fail(const std::string& why) {
    std::lock_guard<std::mutex> lock(stateMutex);
    controller->fault(why);stateEvent();
}
void controlRow(Contral::Motor* motor,double target,double l,double r,long long now,bool force=false) {
    if(!force&&lastControlLog>=0&&now-lastControlLog<settings.control_log_ms)return;
    lastControlLog=now;std::ostringstream o;o<<std::setprecision(9)<<now<<','<<phaseName(controller->phase())<<','<<controller->count()<<','<<target<<','<<l<<','<<r<<','
        <<(motor?motor->getLeftPwm():0)<<','<<(motor?motor->getRightPwm():0)<<','
        <<(motor&&motor->leftWriteOk())<<','<<(motor&&motor->rightWriteOk())<<','<<controller->holdRemaining(now)<<','<<csvText(controller->reason());
    logger.put(1,o.str());
}
cv::Mat toColor(const cv::Mat& input) {
    cv::Mat out;if(input.channels()==1)cv::cvtColor(input,out,cv::COLOR_GRAY2BGR);else out=input;return out;
}
void panel(cv::Mat& canvas,const cv::Mat& image,int x,int y,const char* title) {
    if(!image.empty()) {
        cv::Mat color=toColor(image),scaled;const double k=std::min(480.0/color.cols,280.0/color.rows);
        cv::resize(color,scaled,{std::max(1,int(color.cols*k)),std::max(1,int(color.rows*k))},0,0,cv::INTER_NEAREST);
        scaled.copyTo(canvas(cv::Rect(x+(480-scaled.cols)/2,y+25,scaled.cols,scaled.rows)));
    }
    cv::putText(canvas,title,{x+8,y+18},cv::FONT_HERSHEY_SIMPLEX,.5,{0,220,255},1);
}
cv::Mat diagnostic(const cv::Mat& gray,const cv::Mat& raw,const std::vector<int>& left,const std::vector<int>& right,
                   const Observation& obs,const std::string& phase,const std::string& reason,unsigned count,
                   long long hold,double l,double r,float steer,std::uint64_t id,long long capture,int farCount,int nearCount,int strongCount) {
    cv::Mat canvas(800,960,CV_8UC3,cv::Scalar(12,12,12)),overlay=toColor(raw).clone();
    if(!overlay.empty()) {
        for(int y=0;y<overlay.rows&&y<int(left.size())&&y<int(right.size());++y) {
            if(left[y]>=0&&left[y]<overlay.cols)overlay.at<cv::Vec3b>(y,left[y])={0,255,0};
            if(right[y]>=0&&right[y]<overlay.cols)overlay.at<cv::Vec3b>(y,right[y])={255,0,0};
        }
        for(double v:{settings.far_top,settings.far_bottom,settings.near_top,settings.near_bottom,settings.trigger_y})
            cv::line(overlay,{0,std::min(overlay.rows-1,int(v*overlay.rows))},{overlay.cols-1,std::min(overlay.rows-1,int(v*overlay.rows))},{0,180,255},1);
        for(auto run:obs.near.segments)cv::line(overlay,{run.a,run.y},{run.b,run.y},{0,0,255},1);
    }
    panel(canvas,gray,0,0,"SAME FRAME GRAY");panel(canvas,raw,480,0,"RAW BINARY (unchanged)");
    panel(canvas,overlay,0,325,"ROAD + FAR/NEAR/TRIGGER ROWS");
    const auto& n=obs.near;
    std::vector<std::string> lines={"phase="+phase,"reason="+reason,"count="+std::to_string(count),
        "far="+obs.far.reason,"near="+n.reason,"rows="+std::to_string(n.hit_rows)+" stripes="+std::to_string(n.max_stripes),
        "black="+std::to_string(n.black_ratio)+" contrast="+std::to_string(n.contrast),
        "nearest_y="+std::to_string(n.nearest_y),"L/R measured RPS="+std::to_string(l)+" / "+std::to_string(r),
        "steer COMMAND error="+std::to_string(steer),"hold remaining ms="+std::to_string(hold),
        "confirm F/N/S="+std::to_string(farCount)+"/"+std::to_string(nearCount)+"/"+std::to_string(strongCount),
        "STOP: yield to pedestrians"};
    int y=350;for(const auto& line:lines){cv::putText(canvas,line,{488,y},cv::FONT_HERSHEY_SIMPLEX,.47,{230,230,230},1);y+=23;}
    cv::putText(canvas,"frame="+std::to_string(id)+" capture_ns="+std::to_string(capture),{12,690},cv::FONT_HERSHEY_SIMPLEX,.6,{0,230,230},1);
    cv::putText(canvas,"Video playback time is not capture time; use video_timestamps.csv",{12,725},cv::FONT_HERSHEY_SIMPLEX,.55,{230,230,230},1);
    cv::putText(canvas,"Brake: LEFT PWM 0 / RIGHT REVERSE. No barrier detection.",{12,760},cv::FONT_HERSHEY_SIMPLEX,.55,{230,230,230},1);
    return canvas;
}
}
bool active(){return enabled;}
int illuminationBandRows(){return settings.illumination_band_rows;}
bool initialize(const std::string& path,std::string& error) {
    try {
        settings=loadSettings(path);controller.reset(new Controller(settings));
        const auto wall=std::chrono::system_clock::now();const auto t=std::chrono::system_clock::to_time_t(wall);std::tm tm{};
#ifdef _WIN32
        localtime_s(&tm,&t);
#else
        localtime_r(&t,&tm);
#endif
        std::ostringstream name;name<<"run_"<<std::put_time(&tm,"%Y%m%d_%H%M%S")<<'_'<<nowMs();
        directory=Experiment::executableDirectory()+"/logs/"+name.str();
        if(!fs::create_directories(directory))throw std::runtime_error("RUN_DIRECTORY_EXISTS");
        fs::create_directories(directory+"/config_snapshot");fs::create_directories(directory+"/camera");fs::create_directories(directory+"/diagnostic");
        for(const auto& p:{path,std::string("image_params.txt")})fs::copy_file(p,fs::path(directory)/"config_snapshot"/fs::path(p).filename());
        if(fs::space(directory).available<static_cast<std::uintmax_t>(settings.minimum_free_mb)*1024*1024)throw std::runtime_error("DISK_SPACE_LOW");
        std::ofstream info(directory+"/run_info.json");
        info<<"{\"version\":\"VV17_ZEBRA_TEST_1\",\"time_source\":\"steady_clock nanoseconds\",\"encoder_units\":\"VV17 RPS convention\",\"brake\":\"right_only\",\"config_reload\":\"restart_required\",\"build\":"<<json(__DATE__ " " __TIME__)<<"}";info.flush();
        if(!info)throw std::runtime_error("RUN_INFO_WRITE_FAILED");
        if(!logger.start(directory,settings,error))return false;
        enabled=true;event("STARTUP","config="+path+" logs="+directory);return true;
    }catch(const std::exception& e){error=e.what();return false;}
}
void cameraFailure(const std::string& reason) {if(enabled)fail(reason);}
void onFrame(const cv::Mat& original,const cv::Mat& gray,const cv::Mat& raw,
             const std::vector<int>& left,const std::vector<int>& right,bool valid,bool quality,float steer,long long capture) {
    if(!enabled)return;
    try {
        if(original.empty()||capture<0){fail("ORIGINAL_FRAME_MISSING");return;}
        if(!recordReady) {
            Experiment::VideoOptions options;options.fps=settings.video_fps;options.queueCapacity=settings.video_queue;options.fileName="camera.avi";
            std::string error;
            if(!rawVideo.start(directory+"/camera",original.size(),options,error)){fail("RAW_RECORD_START: "+error);return;}
            options.fileName="diagnostic.avi";
            if(!debugVideo.start(directory+"/diagnostic",{960,800},options,error)){fail("DIAGNOSTIC_RECORD_START: "+error);return;}
            recordReady=true;event("RECORDERS_READY",directory);
        }
        const auto obs=detect(gray,raw,left,right,valid,quality,capture/1000000,settings);
        std::string phase,reason;unsigned count;long long remaining;double l,r;int farCount,nearCount,strongCount;
        {std::lock_guard<std::mutex> lock(stateMutex);controller->observe(obs);stateEvent();
         phase=phaseName(controller->phase());reason=controller->reason();count=controller->count();remaining=controller->holdRemaining(nowMs());l=lastLeft;r=lastRight;
         farCount=controller->farFrames();nearCount=controller->nearFrames();strongCount=controller->strongFrames();}
        const auto id=++frameIndex;
        const auto rawResult=rawVideo.submit(toColor(original),id,capture);
        Experiment::RecordSubmit debugResult=Experiment::RecordSubmit::RateSkipped;
        if(debugVideo.shouldSelect(capture))debugResult=debugVideo.submit(diagnostic(gray,raw,left,right,obs,phase,reason,count,remaining,l,r,steer,id,capture,farCount,nearCount,strongCount),id,capture);
        if(rawResult==Experiment::RecordSubmit::Failed||debugResult==Experiment::RecordSubmit::Failed)fail("VIDEO_WRITE: "+rawVideo.stats().error+" / "+debugVideo.stats().error);
        std::ostringstream row;row<<std::setprecision(9)<<id<<','<<capture<<','<<phase<<','<<count<<','<<valid<<','<<obs.input_valid<<','
            <<obs.far.valid<<','<<obs.far.hit<<','<<obs.near.valid<<','<<obs.near.hit<<','<<obs.near.strong<<','
            <<obs.far.hit_rows<<','<<obs.near.hit_rows<<','<<obs.near.max_stripes<<','<<obs.near.components<<','
            <<obs.near.black_ratio<<','<<obs.near.contrast<<','<<obs.near.nearest_y<<','<<steer<<','<<remaining<<','
            <<csvText(reason)<<','<<obs.far.reason<<','<<obs.near.reason<<','<<farCount<<','<<nearCount<<','<<strongCount<<','<<settings.illumination_band_rows<<','
            <<nowMs()-capture/1000000<<','<<int(rawResult)<<','<<int(debugResult);
        logger.put(0,row.str());
        if(settings.voice_enable&&count>voicedCount){voicedCount=count;const auto result=Other::WonderEchoSpeakZebra();event(result==0?"VOICE_SENT":"VOICE_FAILED","return="+std::to_string(result));}
    }catch(const std::exception& e){fail(std::string("FRAME_EXCEPTION: ")+e.what());}
}
bool motorStep(Contral::Motor* motor,double l,double r,bool encoderValid,bool externalStop) {
    if(!enabled)return false;
    std::lock_guard<std::mutex> lock(stateMutex);const auto now=nowMs();
    lastLeft=l;lastRight=r;
    if(lastSpeedTime>=0&&now-lastSpeedTime>settings.speed_timeout_ms)controller->fault("CONTROL_LOOP_STALE");
    lastSpeedTime=now;
    if(logger.bad())controller->fault(logger.error());
    if(!previousWrites)controller->fault("MOTOR_WRITE_FAILED");
    if(!encoderValid&&!IoStatus::lastError.empty()&&controller->phase()!=Phase::Fault)event("ENCODER_READ_FAILED",IoStatus::lastError);
    if(!motor)controller->fault("MOTOR_MISSING");
    Command cmd=controller->step({now,l,r,encoderValid,externalStop});
    if(!recordReady)cmd=Command{};
    speedLimit=cmd.speed_limit?cmd.max_speed:0;
    stateEvent();
    if(!cmd.override_pwm)return false;
    bool ok=false;
    if(motor){const bool a=motor->setLeftPwm(cmd.left_pwm);const bool b=motor->setRightPwm(cmd.right_pwm);ok=a&&b;}
    controller->acknowledge(now,ok);previousWrites=ok;
    if(!ok&&motor){event("MOTOR_WRITE_FAILED","left_cmd="+std::to_string(cmd.left_pwm)+" right_cmd="+std::to_string(cmd.right_pwm)+" "+motor->lastWriteError());motor->setLeftPwm(0);motor->setRightPwm(0);}
    Other::gMotorAimSpeed=0;stateEvent();controlRow(motor,0,l,r,now,true);return true;
}
double limitSpeed(double speed) {const double cap=speedLimit;return cap>0?std::min(speed,cap):speed;}
void motorApplied(Contral::Motor* motor,double target,double l,double r) {
    if(!enabled)return;std::lock_guard<std::mutex> lock(stateMutex);
    previousWrites=motor&&motor->leftWriteOk()&&motor->rightWriteOk();
    if(!previousWrites){controller->fault("MOTOR_WRITE_FAILED");event("MOTOR_WRITE_FAILED",motor?"left_cmd="+std::to_string(motor->getLeftPwm())+" right_cmd="+std::to_string(motor->getRightPwm())+" "+motor->lastWriteError():"motor missing");stateEvent();}
    controlRow(motor,target,l,r,nowMs());
}
bool failed(){if(!enabled)return false;std::lock_guard<std::mutex> lock(stateMutex);return controller->phase()==Phase::Fault||logger.bad()||finalFailure;}
void shutdown() {
    if(!enabled)return;
    event("SHUTDOWN","motor thread stopped before draining recordings");rawVideo.stop();debugVideo.stop();
    const auto a=rawVideo.stats(),b=debugVideo.stats();
    finalFailure=!a.verified||!b.verified||!a.error.empty()||!b.error.empty();
    event("RECORDING_CLOSED","raw="+std::to_string(a.written)+" diagnostic="+std::to_string(b.written)+" dropped="+std::to_string(a.dropped+b.dropped)+" errors="+a.error+" / "+b.error);
    logger.close();std::ofstream status(directory+"/recording_status.txt");
    status<<"state="<<((failed()||!a.verified||!b.verified)?"INCOMPLETE":"CLOSED")<<"\nraw_verified="<<a.verified<<"\ndiagnostic_verified="<<b.verified<<"\nraw_error="<<a.error<<"\ndiagnostic_error="<<b.error<<"\nlog_failed="<<logger.bad()<<"\nlog_error="<<logger.error()<<"\nlog_dropped="<<logger.dropped()<<"\n";
    std::cout<<"[ZebraTest] CLOSED "<<directory<<std::endl;
}
}
