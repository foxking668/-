#pragma once
#include "headfile.hpp"

namespace Other
{
    /*
     * 运行时日志工具类。
     *
     * 支持不同日志等级并使用 ANSI 颜色在控制台输出，
     * 便于快速区分调试信息、普通信息、警告和错误。
     */
    enum LOG_LEVEL
    {
        LOG_DEBUG = 0,
        LOG_INFO,
        LOG_WARN,
        LOG_ERROR,
        LOG_FATAL,
        LOG_NONE
    };
    
    class Log
    {
    private:
        LOG_LEVEL min_level;
        std::vector<LOG_LEVEL> level_list;
        std::vector<std::string> log_list;

    public:
        /**
         * @brief 构造 Log 对象
         *
         * @details
         * 创建 Other / Log 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
         *
         * @param level 日志等级，用于控制本条日志是否输出。
         */
        Log(LOG_LEVEL level = LOG_WARN);
        /**
         * @brief logOutputConsole 函数说明
         *
         * @details
         * 该函数属于 Other / Log 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param msg 日志消息内容。
         * @param level 日志等级，用于控制本条日志是否输出。
         */
        void logOutputConsole(const char *msg, LOG_LEVEL level);
        /**
         * @brief logOutputConsole 函数说明
         *
         * @details
         * 该函数属于 Other / Log 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
         *
         * @param msg 日志消息内容。
         * @param level 日志等级，用于控制本条日志是否输出。
         */
        void logOutputConsole(const std::string &msg, LOG_LEVEL level);
        /**
         * @brief 析构 Log 对象
         *
         * @details
         * 释放 Other / Log 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
         */
        ~Log();
    };
}
