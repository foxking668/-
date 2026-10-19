#include "ZebraTest/ZebraTest.hpp"
#include "ZebraTest/ZebraDetector.hpp"
#include <opencv2/imgproc.hpp>
#include <fstream>
#include <iostream>
#include <stdexcept>
using namespace ZebraTest;
int checks=0;
void check(bool b,const char* what){++checks;if(!b)throw std::runtime_error(what);}
Observation observation(long long t,bool hit=false) {
    Observation o;o.time_ms=t;o.input_valid=o.lane_valid=true;o.near.valid=o.far.valid=true;
    o.far.hit=o.near.hit=o.near.strong=hit;o.near.nearest_y=hit?.7:-1;return o;
}
void ready(Controller& c) {for(int t:{0,33,66})c.observe(observation(t));}
void trigger(Controller& c) {for(int t:{100,133,166,199})c.observe(observation(t,true));}
void controlTests() {
    Settings s;Controller c(s);ready(c);
    check(c.phase()==Phase::Cruise,"valid frames enable ordinary lane control");
    check(!c.step({66,7,7,true,false}).override_pwm,"cruise leaves original PID in control");
    trigger(c);check(c.count()==1&&c.phase()==Phase::Coast,"one confirmed zebra event enters coast");
    auto cmd=c.step({199,7,7,true,false});check(cmd.left_pwm==0&&cmd.right_pwm==0,"coast zeros both wheels");
    c.acknowledge(199,true);
    c.observe(observation(250,true));cmd=c.step({250,7,7,true,false});
    check(cmd.left_pwm==0&&cmd.right_pwm==-12000,"brake reverses ONLY right wheel");
    // No successful output ack: reverse timer must not start yet.
    c.observe(observation(300,true));c.step({300,7,7,true,false});check(c.phase()==Phase::Brake,"timer awaits motor write ack");c.acknowledge(300,true);
    for(int t=350;t<=1100;t+=50){c.observe(observation(t,true));c.step({t,7,7,true,false});c.acknowledge(t,true);}
    check(c.phase()==Phase::Settle,"reverse maximum ends pulse and zeros output");
    check(c.holdRemaining(1100)==3000,"brake duration does not consume hold");
    for(int t=1150;t<=1350;t+=50){c.observe(observation(t,true));c.step({t,0,0,true,false});c.acknowledge(t,true);}
    check(c.phase()==Phase::Hold,"stationary interval starts hold");
    Controller moved=c;moved.observe(observation(1400,true));moved.step({1400,2,0,true,false});
    check(moved.phase()==Phase::Settle&&moved.holdRemaining(1400)==3000,"movement during hold restarts full stationary confirmation");
    const auto holdAt=1350;
    for(int t=1400;t<holdAt+3000;t+=50){c.observe(observation(t,true));cmd=c.step({t,0,0,true,false});c.acknowledge(t,true);check(cmd.override_pwm,"must not resume before 3 seconds");}
    c.observe(observation(4350,true));cmd=c.step({4350,0,0,true,false});
    check(c.phase()==Phase::Depart&&!cmd.override_pwm,"resume after full stationary hold");
    Controller noMovement=c;
    for(int t=4400;t<=6500;t+=50){noMovement.observe(observation(t));noMovement.step({t,0,0,true,false});}
    check(noMovement.phase()==Phase::Depart,"elapsed cooldown alone cannot rearm a stationary car");
    for(int t=4400;t<=6000;t+=50){c.observe(observation(t,true));c.step({t,6,6,true,false});}
    check(c.count()==1&&c.phase()==Phase::Depart,"same visible zebra stays latched");
    // Invalid far ROI is unknown, not absence.
    for(int t=6050;t<=6600;t+=50){auto o=observation(t);o.far.valid=false;c.observe(o);c.step({t,6,6,true,false});}
    check(c.phase()==Phase::Depart,"invalid ROI never unlocks zebra");
    for(int t=6650;t<=7100;t+=50){c.observe(observation(t));c.step({t,6,6,true,false});}
    check(c.phase()==Phase::Cruise,"valid absence plus actual forward motion rearms");
    for(int t:{7150,7200,7250,7300})c.observe(observation(t,true));
    check(c.count()==2,"second crossing can trigger without competition lap state");
    Controller stale(s);ready(stale);stale.step({500,4,4,true,false});check(stale.phase()==Phase::Fault&&stale.reason()=="CAMERA_STALE","stale camera faults to zero PWM");
    Controller encoder(s);ready(encoder);encoder.step({66,0,0,false,false});check(encoder.reason()=="ENCODER_INVALID","failed encoder is not interpreted as zero speed");
    Controller write(s);ready(write);trigger(write);write.acknowledge(200,false);check(write.reason()=="MOTOR_WRITE_FAILED","write failure latches fault");
    Controller wait(s);ready(wait);trigger(wait);wait.acknowledge(199,true);wait.observe(observation(250,true));wait.step({250,7,7,true,false});wait.acknowledge(250,true);
    for(int t=300;t<=5200;t+=50){wait.observe(observation(t,true));wait.step({t,7,7,true,false});wait.acknowledge(t,true);}
    check(wait.reason()=="SETTLE_TIMEOUT","failure to stop cannot start 3-second countdown");
    Controller backwards(s);ready(backwards);backwards.observe(observation(60));check(backwards.phase()==Phase::Fault,"nonmonotonic image timestamps rejected");
    Controller lost(s);ready(lost);auto bad=observation(100);bad.lane_valid=false;lost.observe(bad);
    check(lost.step({100,5,5,true,false}).override_pwm&&lost.phase()==Phase::LanePause,"temporary lane loss stops immediately");
    for(int t:{133,166,199})lost.observe(observation(t));
    check(!lost.step({199,0,0,true,false}).override_pwm&&lost.phase()==Phase::Cruise,"stable valid lane resumes existing steering");
    Controller longLost(s);ready(longLost);
    for(int t=100;t<=1700;t+=50){auto b=observation(t);b.lane_valid=false;longLost.observe(b);longLost.step({t,0,0,true,false});}
    check(longLost.reason()=="LANE_LOSS_TIMEOUT","persistent lane loss latches fault");
    Controller startup(s);startup.step({0,0,0,true,false});startup.step({5001,0,0,true,false});
    check(startup.reason()=="STARTUP_IMAGE_TIMEOUT","missing startup images report a concrete timeout");
}
void detectionTests() {
    Settings s;std::vector<int> left(60,5),right(60,74);
    auto run=[&](const cv::Mat& raw){return detect(raw,raw,left,right,true,true,0,s);};
    cv::Mat road(60,80,CV_8UC1,cv::Scalar(255));cv::line(road,{40,0},{40,59},0,2);
    check(!run(road).near.hit,"center solid line rejected");
    cv::Mat dashes(60,80,CV_8UC1,cv::Scalar(255));for(int y=3;y<60;y+=12)cv::rectangle(dashes,{37,y,5,7},0,-1);
    check(!run(dashes).near.hit,"longitudinal dashed center line rejected");
    cv::Mat fork=road.clone();cv::line(fork,{40,38},{15,10},0,3);cv::line(fork,{40,38},{65,10},0,3);
    check(!run(fork).near.hit,"Y branches rejected");
    cv::Mat bars=road.clone();for(int x:{12,24,48,60})cv::rectangle(bars,{x,28,5,14},0,-1);
    const auto evidence=run(bars);check(evidence.near.hit&&evidence.near.strong,"multiple black bars recognized beside center guide");
    cv::Mat connected=bars.clone();cv::line(connected,{12,32},{64,32},0,1);
    check(run(connected).near.hit,"stripes touching one shared connector remain detectable");
    cv::Mat tilted=road.clone();for(int x:{12,24,48,60})cv::rectangle(tilted,{x,20+x/5,5,14},0,-1);
    check(run(tilted).near.hit,"slanted group recognized");
    cv::Mat black(60,80,CV_8UC1,cv::Scalar(0));check(!run(black).near.hit,"black background rejected");
    cv::Mat glare(60,80,CV_8UC1,cv::Scalar(255));check(!run(glare).near.hit,"blank overexposure rejected");
    auto badLeft=left;badLeft.assign(60,-1);check(!detect(bars,bars,badLeft,right,true,true,0,s).near.valid,"invalid road bounds produce unknown evidence");
    check(!detect(bars,bars,left,right,true,false,0,s).input_valid,"invalid binary quality cannot trigger");
    const auto before=bars.clone();run(bars);check(cv::countNonZero(bars!=before)==0,"detector never modifies control input");
}
int main(int argc,char** argv){try {controlTests();detectionTests();if(argc>1){auto s=loadSettings(argv[1]);check(s.hold_ms==3000,"delivered tuning file parses");}std::cout<<"PASS "<<checks<<" checks\n";return 0;}catch(const std::exception& e){std::cerr<<"FAIL "<<e.what()<<'\n';return 1;}}
