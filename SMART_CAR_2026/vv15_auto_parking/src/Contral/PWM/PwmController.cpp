#include "PwmController.hpp"
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <thread>

namespace Contral
{
    /*
     * 记录 PWM 芯片号与通道号，并拼接 sysfs 路径。
     * 实际导出动作在 `initialize()` 中执行。
     */
    /**
     * @brief 构造 PwmController 对象
     *
     * @details
     * 创建 Contral / PWM 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * @param pwmChip PWM 芯片编号，对应 /sys/class/pwm/pwmchipN。
     * @param pwmNumber PWM 通道编号，对应 pwmchip 下的 pwmN。
     */
    PwmController::PwmController(int pwmChip, int pwmNumber, const std::string& sysfsRoot)
        : pwm_chip(pwmChip), pwm_number(pwmNumber), period(0), duty_cycle(0)
    {
        const std::string chip=sysfsRoot+"/pwmchip"+std::to_string(pwmChip);
        this->pwm_path=chip+"/pwm"+std::to_string(pwmNumber)+"/";
        this->export_path=chip+"/export";
    }

    /**
     * @brief 析构 PwmController 对象
     *
     * @details
     * 释放 Contral / PWM 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    PwmController::~PwmController()
    {
        this->disable();
    }

    /**
     * @brief 获取 readPeriod 对应数据
     *
     * @details
     * 从 Contral / PWM 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int PwmController::readPeriod()
    {
        return this->period;
    }

    /**
     * @brief 获取 readDutyCycle 对应数据
     *
     * @details
     * 从 Contral / PWM 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
     *
     * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
     */
    int PwmController::readDutyCycle()
    {
        return this->duty_cycle;
    }

    /**
     * @brief 初始化 initialize 相关资源
     *
     * @details
     * 完成 Contral / PWM 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool PwmController::initialize()
    {
        if(pwm_chip<0 || pwm_number<0) return false;
        std::string problem;
        const auto ready=[&]() {
            for(const char* item:{"period","duty_cycle","enable"}) {
                errno=0;
                std::fstream node(pwm_path+item,std::ios::in|std::ios::out);
                if(!node.is_open()) {
                    problem="open "+pwm_path+item+" reason="+std::strerror(errno?errno:EIO);
                    return false;
                }
            }
            int currentPeriod=0,currentDuty=0,currentEnable=-1;
            std::ifstream periodFile(pwm_path+"period"),dutyFile(pwm_path+"duty_cycle"),enableFile(pwm_path+"enable");
            if(!(periodFile>>currentPeriod) || !(dutyFile>>currentDuty) || !(enableFile>>currentEnable) ||
               currentPeriod<0 || currentDuty<0 || currentDuty>currentPeriod || (currentEnable!=0 && currentEnable!=1)) {
                problem="invalid period/duty_cycle/enable metadata under "+pwm_path;
                return false;
            }
            period=currentPeriod;duty_cycle=currentDuty;return true;
        };
        if(ready()) return true; // Reuse an already exported channel without writing export.
        std::error_code error;
        if(!std::filesystem::is_directory(pwm_path,error)) {
            const bool exported=writeToFile(export_path,std::to_string(pwm_number));
            if(!exported && ready()) return true; // Another initializer may have won the race.
        }
        // Export completion may be delayed by sysfs/udev. Never issue a second export.
        for(int attempt=0;attempt<50;++attempt) {
            if(ready()) return true;
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
        std::cerr<<"[PWM_INIT_FAILED] channel="<<pwm_path<<" reason="<<problem<<std::endl;
        return false;
    }

    // 启用PWM
    /**
     * @brief 使能 enable 功能
     *
     * @details
     * 打开 Contral / PWM 模块对应的硬件输出或软件功能，让控制信号开始生效。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool PwmController::enable()
    {
        return writeToFile(this->pwm_path + "enable", std::to_string(1));
    }

    // 禁用PWM
    /**
     * @brief 关闭 disable 功能
     *
     * @details
     * 关闭 Contral / PWM 模块对应的硬件输出或软件功能，用于停止输出或进入安全状态。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool PwmController::disable()
    {
        return writeToFile(this->pwm_path + "enable", std::to_string(0));
    }

    // 设置周期（以纳秒为单位）
    /**
     * @brief 设置 setPeriod 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param period_ns PWM 周期，单位为纳秒。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool PwmController::setPeriod(unsigned int period_ns)
    {
        if(!writeToFile(this->pwm_path+"period",std::to_string(period_ns))) return false;
        this->period=period_ns;return true;
    }

    // 设置低电平时间（以纳秒为单位）
    /**
     * @brief 设置 setDutyCycle 对应参数
     *
     * @details
     * 根据传入参数更新 Contral / PWM 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param duty_cycle_ns PWM 占空时间，单位为纳秒。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool PwmController::setDutyCycle(unsigned int duty_cycle_ns)
    {
        if(!writeToFile(this->pwm_path+"duty_cycle",std::to_string(duty_cycle_ns))) return false;
        this->duty_cycle=duty_cycle_ns;return true;
    }

    // 向文件写入值的辅助函数
    /**
     * @brief 写入 writeToFile 对应数据
     *
     * @details
     * 将调用方提供的数据写入 Contral / PWM 模块管理的文件、设备节点或硬件寄存器，并返回写入结果。
     *
     * @param path 目标文件路径或设备节点路径。
     * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool PwmController::writeToFile(const std::string &path, const std::string &value)
    {
        errno=0;
        FILE* file=std::fopen(path.c_str(),"w");
        if(!file) {
            const int error=errno;
            std::cerr<<"[PWM_IO_ERROR] open path="<<path<<" value="<<value
                     <<" reason="<<std::strerror(error)<<std::endl;
            return false;
        }
        bool written=std::fwrite(value.data(),1,value.size(),file)==value.size();
        if(std::fflush(file)!=0) written=false;
        int error=written?0:errno;
        if(std::fclose(file)!=0) {written=false;if(!error) error=errno;}
        if(!written) std::cerr<<"[PWM_IO_ERROR] write path="<<path<<" value="<<value
                              <<" reason="<<std::strerror(error?error:EIO)<<std::endl;
        return written;
    }
}
