#pragma once
#include <chrono>
#include <iostream>
#include <iomanip>
#include <mutex>
#include <thread>
#ifdef _WIN32
#include <process.h>
#define getpid _getpid
#else
#include <unistd.h>
#endif
namespace Contral {
class Motor {
public:
    int left=0,right=0;bool failRight=false;
    bool setLeftPwm(int x){left=x;return true;}
    bool setRightPwm(int x){right=x;return !failRight || x==0;}
};
class Steer {
public:
    int writes=0;float angle=0,offset=0;
    void setSteerOffset(float x){offset=x;}
    bool setAngle(float x){angle=x;++writes;return true;}
};
}
namespace Other {
inline bool gMotorStop=false;
inline float gMotorAimSpeed=0;
namespace RuntimeConfig {inline double getSteerOffsetDeg(){return 95;}}
}
