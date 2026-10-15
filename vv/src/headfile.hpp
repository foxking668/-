/*
 * 项目总头文件。
 *
 * 该文件集中包含：
 * 1. 标准库头文件；
 * 2. OpenCV 核心依赖；
 * 3. 本项目所有主要模块头文件。
 */

/*=======================标准库=======================*/
#include <iostream>
#include <list>
#include <vector>
#include <string>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <unistd.h>
#include <sys/mman.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <cstring>
#include <stdexcept>
#include <fstream>
#include <sstream>
#include <cstdint>
#include <fcntl.h>
#include <thread>
#include <chrono>
#include <mutex>
#include <signal.h>
#include <cstdlib>
#include <condition_variable>
#include <termios.h>
#include <unordered_map>
#include <cctype>
#include <variant>
#include <iomanip>

/*=======================核心库=======================*/
#include <opencv2/opencv.hpp>

/*=======================用户库=======================*/
#include "define.hpp"
#include "Other/Log/Log.hpp"
#include "Other/Global/Global.hpp"
#include "Other/Base/Base.hpp"
#include "Other/Config/Config.hpp"
#include "Other/RuntimeConfig/RuntimeConfig.hpp"
#include "Other/Voice/WonderEcho.hpp"
#include "Experiment/RealCarExperiment.hpp"
#include "Experiment/NaturalMissingVisibility.hpp"

#include "ImageProcess/Camera/Camera.hpp"
#include "ImageProcess/CutFrame/CutFrame.hpp"
#include "ImageProcess/HandleImage/HandleImage.hpp"
#include "ImageProcess/FindStartPos/FindStartPos.hpp"
#include "ImageProcess/FindLine/FindLine.hpp"
#include "ImageProcess/FixLine/Elements/Zebra.hpp"
#include "ImageProcess/FixLine/FixLine.hpp"
#include "ImageProcess/CalcError/CalcError.hpp"
#include "ImageProcess/TrafficLight/TrafficLight.hpp"
#include "ImageProcess/BlueObstacle/BlueObstacle.hpp"

#include "Contral/GPIO/GPIO.hpp"
#include "Contral/Motor/Motor.hpp"
#include "Contral/Register/Register.hpp"
#include "Contral/PWM/PwmController.hpp"
#include "Contral/Encoder/Encoder.hpp"
#include "Contral/PWM/PwmAtim.hpp"
#include "Contral/PWM/PwmGtim.hpp"
#include "Contral/PID/PID.h"
#include "Contral/Uart/Uart.hpp"
#include "Contral/IMU/JY62.hpp"
#include "Contral/Steer/Steer.hpp"

#include "HandleProcess/ImageHandle/ImageHandle.hpp"
#include "HandleProcess/MotorHandle/MotorHandle.hpp"
