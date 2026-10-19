#pragma once
#include <opencv2/core.hpp>
#include <cstdint>
#include <string>
#include <vector>
namespace Contral {class Motor;}
namespace ZebraTest {
bool initialize(const std::string& configPath,std::string& error);
void shutdown();
bool active();
int illuminationBandRows();
void cameraFailure(const std::string& reason);
void onFrame(const cv::Mat& original,const cv::Mat& gray,const cv::Mat& raw,
             const std::vector<int>& left,const std::vector<int>& right,bool valid,bool quality,
             float steerError,long long captureTimeNs);
bool motorStep(Contral::Motor* motor,double leftRps,double rightRps,bool encodersValid,bool externalStop);
double limitSpeed(double speed);
void motorApplied(Contral::Motor* motor,double target,double leftRps,double rightRps);
bool failed();
}
