/*
 * color_gray_test.cpp - 验证"彩色相机(3通道)"和"灰度相机(1通道)"喂进同一条管线，
 * 结果【完全一致】，且 1/3/4 通道都不会崩。
 */
#include "headfile.hpp"
#include "NewTrack/NewTrack.hpp"
#include <opencv2/imgcodecs.hpp>
#include <cassert>
#include <fstream>
#include <sstream>
#include <cmath>
#ifdef _WIN32
#include <direct.h>
#else
#include <unistd.h>
#endif

namespace Other {
double RuntimeConfig::getCenterPratio() { return 0.5; }
double RuntimeConfig::getErrorGain()   { return 4.0; }
double RuntimeConfig::getMotorAimSpeed() { return 9.0; }
}
namespace ImageProcess {
bool FixLine::checkZebraSimple() { throw std::runtime_error("unexpected legacy zebra detector in NewTrack"); }
FixLineState FixLine::handleZebraSimple() { throw std::runtime_error("unexpected legacy zebra handler in NewTrack"); }
}

static cv::Mat makeScene()
{
    cv::Mat img(480, 640, CV_8UC3, cv::Scalar(60, 60, 60));
    cv::rectangle(img, cv::Point(90,0),  cv::Point(550,479), cv::Scalar(205,205,205), -1);
    cv::rectangle(img, cv::Point(0,0),   cv::Point(89,479),  cv::Scalar(20,20,20), -1);
    cv::rectangle(img, cv::Point(551,0), cv::Point(639,479), cv::Scalar(20,20,20), -1);
    for (int y = 300; y < 470; y += 34) cv::rectangle(img, cv::Point(315,y), cv::Point(330,y+14), cv::Scalar(0,0,0), -1);
    for (int y = 150; y < 270; y += 45) cv::rectangle(img, cv::Point(318,y), cv::Point(332,y+12), cv::Scalar(0,0,0), -1);
    return img;
}

int main(int argc, char **argv)
{
    if (argc != 2) return 2;
#ifdef _WIN32
    if (_chdir(argv[1]) != 0) return 2;
#else
    if (chdir(argv[1]) != 0) return 2;
#endif

    int failures = 0;
    cv::Mat color = makeScene();
    cv::Mat gray;  cv::cvtColor(color, gray, cv::COLOR_BGR2GRAY);
    cv::Mat bgra;  cv::cvtColor(color, bgra, cv::COLOR_BGR2BGRA);
    std::cout << "输入通道: color=" << color.channels()
              << " gray=" << gray.channels()
              << " bgra=" << bgra.channels() << std::endl;

    NewTrack::FrameResult r3, r1;
    try { r3 = NewTrack::processFrame(color.clone(), true); }
    catch (const std::exception &e) { std::cout << "FAIL 3通道抛异常: " << e.what() << std::endl; ++failures; }
    try { r1 = NewTrack::processFrame(gray.clone(), true); }
    catch (const std::exception &e) { std::cout << "FAIL 1通道抛异常: " << e.what() << std::endl; ++failures; }

    std::cout << "3通道 -> valid=" << r3.valid << " steerError=" << r3.steerError << std::endl;
    std::cout << "1通道 -> valid=" << r1.valid << " steerError=" << r1.steerError << std::endl;
    if (r3.valid != r1.valid) { std::cout << "FAIL valid 不一致" << std::endl; ++failures; }
    const double dErr = std::fabs((double)r3.steerError - (double)r1.steerError);
    std::cout << "steerError 差值 = " << dErr << std::endl;
    if (dErr > 1e-4) { std::cout << "FAIL steerError 不一致" << std::endl; ++failures; }

    try { NewTrack::processFrame(bgra.clone(), false); std::cout << "4通道 -> OK" << std::endl; }
    catch (const std::exception &e) { std::cout << "FAIL 4通道抛异常: " << e.what() << std::endl; ++failures; }

    for (int k = 0; k < 40; ++k)
    {
        try { NewTrack::processFrame(color.clone(), (k % 7) == 0); }
        catch (const std::exception &e) { std::cout << "FAIL 第" << k << "帧 3通道抛异常: " << e.what() << std::endl; ++failures; break; }
    }
    std::cout << "连续 40 帧 3通道 -> OK" << std::endl;

    if (failures == 0) { std::cout << "PASS  color_gray_test" << std::endl; return 0; }
    std::cout << "FAIL  color_gray_test 共 " << failures << " 项" << std::endl;
    return 1;
}