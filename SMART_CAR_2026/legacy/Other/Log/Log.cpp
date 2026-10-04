#include "Log.hpp"

namespace Other
{
    /* 构造日志对象，并建立日志等级与显示标签之间的映射关系。 */
    /**
     * @brief 构造 Log 对象
     *
     * @details
     * 创建 Other / Log 模块对象并初始化成员变量，使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * @param level 日志等级，用于控制本条日志是否输出。
     */
    Log::Log(LOG_LEVEL level)
    {
        this->min_level = level;
        this->level_list = {LOG_FATAL, LOG_ERROR, LOG_WARN, LOG_INFO, LOG_DEBUG};
        this->log_list = {"FATAL", "ERROR", "WARN", "INFO", "DEBUG"};
    }
    /**
     * @brief logOutputConsole 函数说明
     *
     * @details
     * 该函数属于 Other / Log 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @param msg 日志消息内容。
     * @param level 日志等级，用于控制本条日志是否输出。
     */
    void Log::logOutputConsole(const char *msg, LOG_LEVEL level)
    {
        if (this->min_level == LOG_NONE || level < this->min_level)
        {
            return;
        }

        // 查找日志级别的位置
        auto it = std::find(this->level_list.begin(), this->level_list.end(), level);

        if (it != this->level_list.end())
        {
            // 计算日志级别的索引
            size_t index = std::distance(this->level_list.begin(), it);

            // 设置颜色
            std::string colorCode;
            switch (level)
            {
            case LOG_FATAL:
                colorCode = "\033[31m"; // 红色
                break;
            case LOG_ERROR:
                colorCode = "\033[31m"; // 红色
                break;
            case LOG_WARN:
                colorCode = "\033[33m"; // 黄色
                break;
            case LOG_INFO:
                colorCode = "\033[32m"; // 绿色
                break;
            case LOG_DEBUG:
                colorCode = "\033[34m"; // 蓝色
                break;
            default:
                colorCode = "\033[37m"; // 默认白色
                break;
            }

            // 输出日志信息并添加颜色
            std::cout << colorCode << "[" << this->log_list[index] << "] " << msg << "[0m" << std::endl;
        }
    }
    /**
     * @brief logOutputConsole 函数说明
     *
     * @details
     * 该函数属于 Other / Log 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
     *
     * @param msg 日志消息内容。
     * @param level 日志等级，用于控制本条日志是否输出。
     */
    void Log::logOutputConsole(const std::string &msg, LOG_LEVEL level)
    {
        if (this->min_level == LOG_NONE || level < this->min_level)
        {
            return;
        }

        // 查找日志级别的位置
        auto it = std::find(this->level_list.begin(), this->level_list.end(), level);

        if (it != this->level_list.end())
        {
            // 计算日志级别的索引
            size_t index = std::distance(this->level_list.begin(), it);

            // 设置颜色
            std::string colorCode;
            switch (level)
            {
            case LOG_FATAL:
                colorCode = "\033[31m"; // 红色
                break;
            case LOG_ERROR:
                colorCode = "\033[31m"; // 红色
                break;
            case LOG_WARN:
                colorCode = "\033[33m"; // 黄色
                break;
            case LOG_INFO:
                colorCode = "\033[32m"; // 绿色
                break;
            case LOG_DEBUG:
                colorCode = "\033[34m"; // 蓝色
                break;
            default:
                colorCode = "\033[37m"; // 默认白色
                break;
            }

            // 输出日志信息并添加颜色
            std::cout << colorCode << "[" << this->log_list[index] << "] " << msg << "[0m" << std::endl;
        }
    }
    /**
     * @brief 析构 Log 对象
     *
     * @details
     * 释放 Other / Log 模块在对象生命周期中申请或占用的文件描述符、映射内存、PWM/GPIO 句柄等资源，避免资源泄漏。
     */
    Log::~Log() = default;
}