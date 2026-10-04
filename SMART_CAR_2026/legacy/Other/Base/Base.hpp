#pragma once

#include "headfile.hpp"

namespace Other
{

    /*
     * main 调用的实时优先级设置接口。
     *
     * 参数说明：
     * - priority：调度优先级，例如 99；
     * - policy：调度策略，例如 SCHED_RR / SCHED_FIFO / SCHED_OTHER。
     *
     * 返回值：
     * - true：设置成功；
     * - false：设置失败，常见原因是权限不足或优先级超出策略范围。
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
    bool SetProcessPriorityRealTime(int priority, int policy);

}
