#include "Base.hpp"

namespace Other
{
    /*
     * 设置当前进程调度策略和实时优先级。
     *
     * main 调用该函数的目的，是让小车控制程序尽量稳定地获得 CPU 时间片，
     * 从而减少图像处理、电机控制线程的调度延迟。
     */
    /**
     * @brief 设置 SetProcessPriorityRealTime 对应参数
     *
     * @details
     * 根据传入参数更新 Other / Base 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
     *
     * @param priority 要设置的实时调度优先级。
     * @param policy Linux 调度策略，例如 SCHED_FIFO、SCHED_RR 或 SCHED_OTHER。
     *
     * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
     */
    bool SetProcessPriorityRealTime(int priority, int policy)
    {
        Log log(LOG_LEVEL_GLOBAL);

        /*
         * 第一步：获取当前进程 PID，并准备 Linux 调度参数结构体。
         */
        bool success = false;

        pid_t pid = getpid();
        struct sched_param param;
        int ret;

        /*
         * 第二步：查询当前调度策略允许的优先级范围。
         * 不同策略允许的优先级范围不同，实时策略通常范围更高。
         */
        int min_prio = sched_get_priority_min(policy);
        int max_prio = sched_get_priority_max(policy);

        /*
         * 第三步：校验调用方传入的优先级是否落在合法范围内。
         * 如果优先级不合法，继续调用 `sched_setscheduler` 只会失败，
         * 因此这里提前返回并打印错误原因。
         */
        if (priority < min_prio || priority > max_prio)
        {
            std::string errorMsg = "Priority " + std::to_string(priority) +
                                   " is outside valid range [" + std::to_string(min_prio) +
                                   ", " + std::to_string(max_prio) + "] for policy " +
                                   std::to_string(policy);
            log.logOutputConsole(errorMsg, LOG_ERROR);
            return false;
        }

        /*
         * 第四步：设置调度参数并调用系统接口。
         * 如果程序不是 root 或没有 CAP_SYS_NICE 权限，实时调度设置通常会失败。
         */
        param.sched_priority = priority;
        ret = sched_setscheduler(pid, policy, &param);

        /*
         * 第五步：处理系统调用失败的情况。
         * EPERM 是最常见错误，表示当前用户权限不足。
         */
        if (ret == -1)
        {
            std::string errorMsg = "sched_setscheduler failed: ";
            errorMsg += strerror(errno);
            log.logOutputConsole(errorMsg, LOG_ERROR);

            if (errno == EPERM)
            {
                log.logOutputConsole("Requires root privileges or CAP_SYS_NICE capability", LOG_ERROR);
            }
            return false;
        }

        /*
         * 第六步：调度策略设置成功。
         */
        return true;
    }

} // namespace Other;
