// Hardware substitutes test the recorder/runtime boundary, not real PWM or encoder accuracy.
#include "ZebraTest/ZebraRuntime.hpp"
#include "Contral/Motor/Motor.hpp"
#include <opencv2/imgproc.hpp>
#include <chrono>
#include <thread>
#include <iostream>
namespace Other {float gMotorAimSpeed=0;int WonderEchoSpeakZebra(){return 0;}}
namespace NewTrack {std::string diagnosticCsvHeader(){return "unused_legacy_recorder";}}
namespace Contral {
Motor::Motor(int,int,int,int,int,int,int){pwm_left_value=pwm_right_value=0;}
Motor::~Motor(){}
bool Motor::setLeftPwm(int pwm){pwm_left_value=pwm;left_write_ok_=true;return true;}
bool Motor::setRightPwm(int pwm){pwm_right_value=pwm;right_write_ok_=true;return true;}
int Motor::getLeftPwm(){return pwm_left_value;}
int Motor::getRightPwm(){return pwm_right_value;}
}
long long timestamp(){return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();}
int main(){
    std::string error;if(!ZebraTest::initialize("config/zebra_test.ini",error)){std::cerr<<error<<'\n';return 1;}
    Contral::Motor motor(0,0,0,0,0,0,0);std::vector<int> left(60,5),right(60,74);
    cv::Mat road(60,80,CV_8UC1,cv::Scalar(255));cv::line(road,{40,0},{40,59},0,2);
    cv::Mat zebra=road.clone();for(int x:{12,24,48,60})cv::rectangle(zebra,{x,28,5,14},0,-1);
    bool sawBrake=false,sawResume=false;const auto start=timestamp();
    while(timestamp()-start<5500000000LL){
        const auto elapsed=(timestamp()-start)/1000000;
        const cv::Mat& raw=elapsed>200?zebra:road;cv::Mat original;cv::cvtColor(raw,original,cv::COLOR_GRAY2BGR);
        ZebraTest::onFrame(original,raw,raw,left,right,true,true,0,timestamp());
        const double speed=elapsed<1000?7:0;
        const bool held=ZebraTest::motorStep(&motor,speed,speed,true,false);
        if(motor.getRightPwm()<0){sawBrake=true;if(motor.getLeftPwm()!=0){std::cerr<<"left reversed\n";return 1;}}
        if(!held){motor.setLeftPwm(1000);motor.setRightPwm(1000);ZebraTest::motorApplied(&motor,7,speed,speed);if(sawBrake)sawResume=true;}
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    const bool bad=ZebraTest::failed();motor.setLeftPwm(0);motor.setRightPwm(0);ZebraTest::shutdown();
    if(bad||!sawBrake||!sawResume){std::cerr<<"runtime failed bad="<<bad<<" brake="<<sawBrake<<" resume="<<sawResume<<'\n';return 1;}
    std::cout<<"PASS runtime recording and stop/resume (stub hardware)\n";return 0;
}
