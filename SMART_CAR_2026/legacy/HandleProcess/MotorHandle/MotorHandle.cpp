#include "MotorHandle.hpp"

using namespace ImageProcess;
using namespace Contral;
using namespace Other;

/*
 * 电机控制线程主循环。
 *
 * 当前安全停车策略：
 * - 左右编码器均按引脚表独立读取；
 * - 左编码器：pwm[0] + GPIO75，右编码器：pwm[3] + GPIO72；
 * - 正常目标速度从 ./car_params/image_params.txt 读取；
 * - 检测到人行横道/红灯：先短暂断动力 emergency_coast_ms，
 *   再按右编码器累计反转 emergency_reverse_turns 圈，
 *   然后 PWM=0 锁止制动 emergency_brake_lock_ms；
 * - 人行横道：锁止制动结束后继续停车 zebra_after_reverse_stop_ms，
 *   再按 pid_recover_delay_ms 恢复；
 * - 红灯：锁止制动结束后保持 PWM=0 等待绿灯；绿灯放行后按 pid_recover_delay_ms 恢复；
 * - 左右编码器独立读取：左轮 PID 使用左编码器反馈，右轮 PID 使用右编码器反馈。
 */
void threadMotorHandle(void)
{
    Motor *motor = (Motor *)gMotor;
    Encoder *leftEncoder = (Encoder *)gLeftEncoder;
    Encoder *rightEncoder = (Encoder *)gRightEncoder;

    gMotorAimSpeed = static_cast<float>(RuntimeConfig::getMotorAimSpeed());

    constexpr int MOTOR_SPEED_PRINT_INTERVAL_MS = 10000;

    auto lastMotorSpeedPrintTime =
        std::chrono::steady_clock::now() -
        std::chrono::milliseconds(MOTOR_SPEED_PRINT_INTERVAL_MS);

    bool lastStopState = gMotorStop;

    /*
     * 红灯/普通停车期间，只在 stop 从 false 变 true 时清一次 PID。
     * 等 stop 变回 false 后，进入 PID 恢复延迟。
     */
    bool pidClearedForCurrentStop = false;

    /* 人行横道停车保持状态。 */
    bool zebraStopRunning = false;
    auto zebraStopStartTime = std::chrono::steady_clock::now();

    /* PID 恢复延迟状态。 */
    bool pidRecoverDelayRunning = false;
    const char *pidRecoverReason = "none";
    auto pidRecoverDelayStartTime = std::chrono::steady_clock::now();

    /* 红灯/人行横道急停反转状态。 */
    enum class EmergencyReverseReason
    {
        NONE,
        ZEBRA,
        STOP
    };

    bool emergencyCoastRunning = false;
    EmergencyReverseReason emergencyCoastReason = EmergencyReverseReason::NONE;
    auto emergencyCoastStartTime = std::chrono::steady_clock::now();

    bool emergencyReverseRunning = false;
    EmergencyReverseReason emergencyReverseReason = EmergencyReverseReason::NONE;
    double emergencyReverseTurns = 0.0;
    auto emergencyReverseStartTime = std::chrono::steady_clock::now();
    auto emergencyReverseLastTime = std::chrono::steady_clock::now();

    bool emergencyBrakeLockRunning = false;
    EmergencyReverseReason emergencyBrakeLockReason = EmergencyReverseReason::NONE;
    auto emergencyBrakeLockStartTime = std::chrono::steady_clock::now();

    /*
     * 人行横道停车结束后的速度序列。
     * Zebra 停车保持 + PID 恢复延迟结束后：
     *   1) 先按正常 motor_aim_speed 运行 zebra_post_normal_run_ms；
     *   2) 再按 zebra_post_slow_aim_speed 低速运行 zebra_post_slow_ms；
     *   3) 最后恢复正常速度闭环。
     */
    bool zebraPostSpeedSequenceRunning = false;
    bool zebraPostSpeedSlowPrinted = false;
    auto zebraPostSpeedStartTime = std::chrono::steady_clock::now();

    /*
     * 人行横道后“动态巡线上裁剪结束后的低速窗口”。
     * ImageHandle 在动态裁剪真正结束、巡线图恢复正常上裁剪后才发事件；
     * 电机线程收到事件后才开始低速计时，不与动态裁剪同时执行。
     */
    bool blueBoardLinePostCropSlowWindowArmed = false;
    bool blueBoardLinePostCropSlowActive = false;
    bool blueBoardLinePostCropSlowStartPrinted = false;
    bool blueBoardLinePostCropSlowEndPrinted = false;
    auto blueBoardLinePostCropSlowStartTime = std::chrono::steady_clock::now();

    float p = 0;
    float i = 0;
    float d = 0;

    auto clearMotorPid = [&](const char *reason)
    {
        PID_Incremental_Clear_TempData(&gLeftPidMotor);
        PID_Incremental_Clear_TempData(&gRightPidMotor);

        /* quiet: motor PID cleared */
    };

    auto maybeClearMotorPid = [&](const char *reason)
    {
        if (RuntimeConfig::getMotorPidClearOnStopEnable() != 0)
        {
            clearMotorPid(reason);
        }
    };

    auto emergencyReasonText = [&](EmergencyReverseReason reason) -> const char *
    {
        if (reason == EmergencyReverseReason::ZEBRA)
        {
            return "zebra";
        }
        if (reason == EmergencyReverseReason::STOP)
        {
            return "red_or_stop";
        }
        return "none";
    };

    auto setMotorPwmZero = [&]()
    {
        if (motor != nullptr)
        {
            motor->setLeftPwm(0);
            motor->setRightPwm(0);
        }
        gMotorAimSpeed = 0;
    };

    auto startPidRecoverDelay = [&](const char *reason)
    {
        pidRecoverDelayRunning = true;
        pidRecoverReason = reason;
        pidRecoverDelayStartTime = std::chrono::steady_clock::now();

        setMotorPwmZero();
        maybeClearMotorPid(reason);

        /* quiet: PID recover delay start */
    };

    auto startEmergencyReverseDirect = [&](EmergencyReverseReason reason, const char *reasonText) -> bool
    {
        const int reverseEnable = RuntimeConfig::getEmergencyReverseEnable();
        const int reversePwm = RuntimeConfig::getEmergencyReversePwm();
        const double targetTurns = RuntimeConfig::getEmergencyReverseTurns();

        if (reverseEnable == 0 || reversePwm <= 0 || targetTurns <= 0.0 || motor == nullptr)
        {
            return false;
        }

        emergencyReverseRunning = true;
        emergencyReverseReason = reason;
        emergencyReverseTurns = 0.0;
        emergencyReverseStartTime = std::chrono::steady_clock::now();
        emergencyReverseLastTime = emergencyReverseStartTime;

        maybeClearMotorPid(reasonText);
        gMotorAimSpeed = 0;
        motor->setLeftPwm(-reversePwm);
        motor->setRightPwm(-reversePwm);

        std::cout << "[EmergencyReverseStart] reason=" << reasonText
                  << " pwm=-" << reversePwm
                  << " target_turns=" << targetTurns
                  << " timeout_ms=" << RuntimeConfig::getEmergencyReverseTimeoutMs()
                  << std::endl;

        return true;
    };

    auto startEmergencyReverse = [&](EmergencyReverseReason reason, const char *reasonText) -> bool
    {
        const int reverseEnable = RuntimeConfig::getEmergencyReverseEnable();
        const int reversePwm = RuntimeConfig::getEmergencyReversePwm();
        const double targetTurns = RuntimeConfig::getEmergencyReverseTurns();

        if (reverseEnable == 0 || reversePwm <= 0 || targetTurns <= 0.0 || motor == nullptr)
        {
            return false;
        }

        const int coastMs = RuntimeConfig::getEmergencyCoastMs();
        if (coastMs <= 0)
        {
            return startEmergencyReverseDirect(reason, reasonText);
        }

        emergencyCoastRunning = true;
        emergencyCoastReason = reason;
        emergencyCoastStartTime = std::chrono::steady_clock::now();

        setMotorPwmZero();
        maybeClearMotorPid(reasonText);

        std::cout << "[EmergencyCoastStart] reason=" << reasonText
                  << " coast_ms=" << coastMs
                  << std::endl;

        return true;
    };

    auto printMotorSpeed = [&](const char *reason)
    {
        (void)reason;
        lastMotorSpeedPrintTime = std::chrono::steady_clock::now();
    };

    while (!gIsClose)
    {
        RuntimeConfig::reloadIfNeeded();

        const double normalAimSpeed = RuntimeConfig::getMotorAimSpeed();

        if (p != gMotorSpeedParamPvalue ||
            i != gMotorSpeedParamIvalue ||
            d != gMotorSpeedParamDvalue)
        {
            p = gMotorSpeedParamPvalue;
            i = gMotorSpeedParamIvalue;
            d = gMotorSpeedParamDvalue;

            PID_Incremental_Set_Kpid(&gLeftPidMotor, p, i, d);
            PID_Incremental_Set_Kpid(&gRightPidMotor, p, i, d);
        }

        /*
         * 左右编码器独立速度反馈。
         * 左编码器已经修好后，不能再让左轮速度映射右编码器；
         * 否则左轮遇到阻力变慢时，左 PID 不会单独加 PWM 补偿。
         */
        double leftEncoderSpeed = 0;
        if (leftEncoder != nullptr)
        {
            leftEncoderSpeed = leftEncoder->pulseConterUpdate();
        }

        double rightEncoderSpeed = 0;
        if (rightEncoder != nullptr)
        {
            rightEncoderSpeed = rightEncoder->pulseConterUpdate();
        }

        gMotorLeftTrueSpeed = static_cast<float>(leftEncoderSpeed);
        gMotorRightTrueSpeed = static_cast<float>(rightEncoderSpeed);

        auto now = std::chrono::steady_clock::now();

        bool stopStateChanged = (gMotorStop != lastStopState);
        bool timeToPrint =
            std::chrono::duration_cast<std::chrono::milliseconds>(
                now - lastMotorSpeedPrintTime)
                .count() >= MOTOR_SPEED_PRINT_INTERVAL_MS;

        bool shouldPrintSpeed = stopStateChanged || timeToPrint;

        /*
         * 红灯/人行横道急停断动力阶段。
         * 先撤掉前进动力，短暂停顿后再进入反转制动。
         */
        if (emergencyCoastRunning)
        {
            const auto coastNow = std::chrono::steady_clock::now();
            setMotorPwmZero();

            const int coastMs = RuntimeConfig::getEmergencyCoastMs();
            const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           coastNow - emergencyCoastStartTime)
                                           .count();

            if (elapsedMs >= coastMs)
            {
                const EmergencyReverseReason coastReason = emergencyCoastReason;
                emergencyCoastRunning = false;
                emergencyCoastReason = EmergencyReverseReason::NONE;

                std::cout << "[EmergencyCoastEnd] reason=" << emergencyReasonText(coastReason)
                          << " elapsed_ms=" << elapsedMs
                          << std::endl;

                startEmergencyReverseDirect(coastReason, emergencyReasonText(coastReason));
            }

            lastStopState = gMotorStop;
            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        /*
         * 红灯/人行横道急停反转过程。
         * 直接给左右轮负 PWM，按右编码器 RPS 累计反转圈数；
         * 到达目标圈数或超时后立即 PWM=0。
         */
        if (emergencyReverseRunning)
        {
            const auto emergencyNow = std::chrono::steady_clock::now();
            const double dtSeconds = std::chrono::duration_cast<std::chrono::milliseconds>(
                                         emergencyNow - emergencyReverseLastTime)
                                         .count() / 1000.0;
            emergencyReverseLastTime = emergencyNow;

            if (dtSeconds > 0.0 && dtSeconds < 1.0)
            {
                emergencyReverseTurns += std::abs(rightEncoderSpeed) * dtSeconds;
            }

            const int reversePwm = RuntimeConfig::getEmergencyReversePwm();
            if (motor != nullptr)
            {
                motor->setLeftPwm(-reversePwm);
                motor->setRightPwm(-reversePwm);
            }
            gMotorAimSpeed = 0;

            const double targetTurns = RuntimeConfig::getEmergencyReverseTurns();
            const int timeoutMs = RuntimeConfig::getEmergencyReverseTimeoutMs();
            const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           emergencyNow - emergencyReverseStartTime)
                                           .count();

            if (emergencyReverseTurns >= targetTurns || elapsedMs >= timeoutMs)
            {
                const EmergencyReverseReason finishedReason = emergencyReverseReason;
                emergencyReverseRunning = false;
                emergencyReverseReason = EmergencyReverseReason::NONE;
                setMotorPwmZero();
                maybeClearMotorPid("emergency_reverse_end");

                std::cout << "[EmergencyReverseEnd] reason="
                          << emergencyReasonText(finishedReason)
                          << " turns=" << emergencyReverseTurns
                          << " elapsed_ms=" << elapsedMs
                          << std::endl;

                const int lockMs = RuntimeConfig::getEmergencyBrakeLockMs();
                if (lockMs > 0)
                {
                    emergencyBrakeLockRunning = true;
                    emergencyBrakeLockReason = finishedReason;
                    emergencyBrakeLockStartTime = emergencyNow;

                    std::cout << "[EmergencyBrakeLockStart] reason="
                              << emergencyReasonText(finishedReason)
                              << " lock_ms=" << lockMs
                              << std::endl;
                }
                else if (finishedReason == EmergencyReverseReason::ZEBRA)
                {
                    zebraStopRunning = true;
                    zebraStopStartTime = emergencyNow;
                }
            }

            lastStopState = gMotorStop;
            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        /*
         * 急停反转结束后的锁止制动阶段。
         * 当前电机硬件接口没有独立刹车脚，这里保持 PWM=0 且禁止 PID 输出，
         * 用 0.5 秒左右的“锁止等待”吸收惯性，避免马上重新给动力。
         */
        if (emergencyBrakeLockRunning)
        {
            const auto lockNow = std::chrono::steady_clock::now();
            setMotorPwmZero();

            const int lockMs = RuntimeConfig::getEmergencyBrakeLockMs();
            const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                           lockNow - emergencyBrakeLockStartTime)
                                           .count();

            if (elapsedMs >= lockMs)
            {
                const EmergencyReverseReason lockReason = emergencyBrakeLockReason;
                emergencyBrakeLockRunning = false;
                emergencyBrakeLockReason = EmergencyReverseReason::NONE;

                std::cout << "[EmergencyBrakeLockEnd] reason="
                          << emergencyReasonText(lockReason)
                          << " elapsed_ms=" << elapsedMs
                          << std::endl;

                if (lockReason == EmergencyReverseReason::ZEBRA)
                {
                    zebraStopRunning = true;
                    zebraStopStartTime = lockNow;
                }
            }

            lastStopState = gMotorStop;
            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(20));
            continue;
        }

        /*
         * 人行横道停车请求。
         * 由 ImageHandle.cpp 的 zebraHandle 设置。
         * 如果此时正在红灯停车，则忽略 Zebra 请求，避免状态冲突。
         */
        if (gMotorReverseRequest &&
            !gMotorStop &&
            !zebraStopRunning &&
            !pidRecoverDelayRunning)
        {
            gMotorReverseRequest = false;

            if (startEmergencyReverse(EmergencyReverseReason::ZEBRA, "zebra"))
            {
                lastStopState = gMotorStop;
                std::this_thread::yield();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            zebraStopRunning = true;
            zebraStopStartTime = std::chrono::steady_clock::now();
            setMotorPwmZero();
            maybeClearMotorPid("zebra_safe_stop_start");

            /* quiet: zebra safe stop start */
        }

        /*
         * 红灯/普通停车刚开始：进入断动力 + 反转 + 锁止制动流程。
         */
        if (gMotorStop &&
            !pidClearedForCurrentStop &&
            !zebraStopRunning &&
            !pidRecoverDelayRunning)
        {
            pidClearedForCurrentStop = true;

            if (startEmergencyReverse(EmergencyReverseReason::STOP, "red_or_stop"))
            {
                lastStopState = gMotorStop;
                std::this_thread::yield();
                std::this_thread::sleep_for(std::chrono::milliseconds(50));
                continue;
            }

            setMotorPwmZero();
            maybeClearMotorPid("stop_start");

            /* quiet: traffic/red stop start */
        }

        /*
         * 红灯/普通停车解除：先延迟恢复 PID。
         */
        if (!gMotorStop &&
            pidClearedForCurrentStop &&
            !zebraStopRunning &&
            !pidRecoverDelayRunning)
        {
            pidClearedForCurrentStop = false;
            startPidRecoverDelay("stop_released");
        }

        /*
         * 人行横道停车保持状态：PWM=0，不执行 PID。
         */
        if (zebraStopRunning)
        {
            const int stopTimeMs = RuntimeConfig::getZebraAfterReverseStopMs();
            setMotorPwmZero();

            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 now - zebraStopStartTime)
                                 .count();

            if (elapsedMs >= stopTimeMs)
            {
                zebraStopRunning = false;

                /* quiet: zebra safe stop end */

                startPidRecoverDelay("zebra_stop_end");
            }

            if (shouldPrintSpeed)
            {
                printMotorSpeed("zebra_safe_stop");
            }

            lastStopState = gMotorStop;

            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        /*
         * PID 恢复延迟状态：PWM=0，不执行 PID。
         */
        if (pidRecoverDelayRunning)
        {
            const int delayMs = RuntimeConfig::getPidRecoverDelayMs();
            setMotorPwmZero();

            auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                 now - pidRecoverDelayStartTime)
                                 .count();

            if (elapsedMs >= delayMs)
            {
                pidRecoverDelayRunning = false;
                maybeClearMotorPid("recover_delay_end");
                gMotorAimSpeed = static_cast<float>(normalAimSpeed);

                if (std::strcmp(pidRecoverReason, "zebra_stop_end") == 0)
                {
                    /*
                     * 这里已经结束停车保持和 PID 恢复延迟，下一轮会真正给电机正常巡线速度。
                     * 用事件通知图像线程从这一刻开始“人行横道后动态巡线裁剪”计时，
                     * 不会把停车/倒退时间误算进扩大视野窗口。
                     */
                    gZebraResumeEvent = true;

                    /*
                     * 后置低速不要在这里开始：必须等 ImageHandle 报告“动态裁剪已经结束”后才开始。
                     * 同时关闭旧 zebra_post 速度序列，避免它和新后置低速窗口叠加、导致速度不可控。
                     */
                    blueBoardLinePostCropSlowWindowArmed = false;
                    blueBoardLinePostCropSlowActive = false;
                    blueBoardLinePostCropSlowStartPrinted = false;
                    blueBoardLinePostCropSlowEndPrinted = false;
                    zebraPostSpeedSequenceRunning = false;
                    zebraPostSpeedSlowPrinted = false;
                }

                /* quiet: PID recover delay end */
            }

            if (shouldPrintSpeed)
            {
                printMotorSpeed("pid_recover_delay");
            }

            lastStopState = gMotorStop;

            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        /*
         * 红灯或者其他普通停车状态：保持 PWM=0，等待绿灯把 gMotorStop 清掉。
         */
        if (gMotorStop)
        {
            setMotorPwmZero();

            if (shouldPrintSpeed)
            {
                printMotorSpeed(stopStateChanged ? "stop_changed" : "periodic_10s");
            }

            lastStopState = gMotorStop;

            std::this_thread::yield();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
            continue;
        }

        /*
         * 正常行驶。
         * 当前版本支持蓝色桶锥低速和人行横道远处预检测低速。
         */
        double currentAimSpeed = normalAimSpeed;

        if (zebraPostSpeedSequenceRunning)
        {
            const auto seqNow = std::chrono::steady_clock::now();
            const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            seqNow - zebraPostSpeedStartTime)
                                            .count();
            const int normalRunMs = RuntimeConfig::getZebraPostNormalRunMs();
            const int slowMs = RuntimeConfig::getZebraPostSlowMs();

            if (elapsedMs < normalRunMs)
            {
                currentAimSpeed = normalAimSpeed;
            }
            else if (elapsedMs < static_cast<long long>(normalRunMs) + slowMs)
            {
                currentAimSpeed = RuntimeConfig::getZebraPostSlowAimSpeed();
                if (!zebraPostSpeedSlowPrinted)
                {
                    std::cout << "[ZebraPostSpeedSlow] speed=" << currentAimSpeed
                              << " duration_ms=" << slowMs
                              << std::endl;
                    zebraPostSpeedSlowPrinted = true;
                }
            }
            else
            {
                zebraPostSpeedSequenceRunning = false;
                zebraPostSpeedSlowPrinted = false;
                currentAimSpeed = normalAimSpeed;
                std::cout << "[ZebraPostSpeedEnd] resume_normal_speed=" << normalAimSpeed
                          << std::endl;
            }
        }

        /*
         * ImageHandle 仅在动态裁剪窗口真正结束、并且巡线图已恢复 normal_top_crop 的那一帧置位该事件。
         * 所以下面的低速与动态裁剪严格串行：先裁剪，再恢复裁剪，最后才低速。
         */
        if (gBlueBoardLineCropFinishedEvent)
        {
            gBlueBoardLineCropFinishedEvent = false;
            blueBoardLinePostCropSlowWindowArmed = true;
            blueBoardLinePostCropSlowActive = false;
            blueBoardLinePostCropSlowStartPrinted = false;
            blueBoardLinePostCropSlowEndPrinted = false;
            blueBoardLinePostCropSlowStartTime = std::chrono::steady_clock::now();
        }

        if (blueBoardLinePostCropSlowWindowArmed)
        {
            const bool slowEnable = RuntimeConfig::getBlueBoardLinePostExpandSlowEnable() != 0;
            const int slowDurationMs = RuntimeConfig::getBlueBoardLinePostExpandSlowDurationMs();
            const long long elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(
                                            std::chrono::steady_clock::now() - blueBoardLinePostCropSlowStartTime)
                                            .count();

            if (!slowEnable || slowDurationMs <= 0)
            {
                if (blueBoardLinePostCropSlowActive && !blueBoardLinePostCropSlowEndPrinted)
                {
                    std::cout << "[BlueBoardLinePostCropSlowEnd] disabled_or_zero_duration resume_normal_speed="
                              << normalAimSpeed << std::endl;
                    blueBoardLinePostCropSlowEndPrinted = true;
                }
                blueBoardLinePostCropSlowWindowArmed = false;
                blueBoardLinePostCropSlowActive = false;
            }
            else if (elapsedMs < slowDurationMs)
            {
                const double configuredSpeed = RuntimeConfig::getBlueBoardLinePostExpandSlowAimSpeed();
                currentAimSpeed = std::min(currentAimSpeed, configuredSpeed);
                blueBoardLinePostCropSlowActive = true;

                if (!blueBoardLinePostCropSlowStartPrinted)
                {
                    std::cout << "[BlueBoardLinePostCropSlowStart] speed=" << configuredSpeed
                              << " duration_ms=" << slowDurationMs
                              << " crop_finished=1"
                              << std::endl;
                    blueBoardLinePostCropSlowStartPrinted = true;
                }
            }
            else
            {
                if (blueBoardLinePostCropSlowActive && !blueBoardLinePostCropSlowEndPrinted)
                {
                    std::cout << "[BlueBoardLinePostCropSlowEnd] resume_normal_speed="
                              << normalAimSpeed
                              << " elapsed_ms=" << elapsedMs
                              << std::endl;
                    blueBoardLinePostCropSlowEndPrinted = true;
                }
                blueBoardLinePostCropSlowWindowArmed = false;
                blueBoardLinePostCropSlowActive = false;
            }
        }

        if (gBlueReturnSlowMode)
        {
            /*
             * 蓝色挡板/蓝色避障状态机处于动作或保持阶段时限速。
             * 当前前方黑块兜底已暂停，所以这里使用蓝色避障速度参数。
             */
            currentAimSpeed = RuntimeConfig::getBlueObstacleAimSpeed();
        }
        if (gZebraSlowMode)
        {
            currentAimSpeed = RuntimeConfig::getZebraSlowAimSpeed();
        }

        gMotorAimSpeed = static_cast<float>(currentAimSpeed);

        if (motor != nullptr)
        {
            motor->submitLeftTrueSpeed(gMotorLeftTrueSpeed);
            motor->submitRightTrueSpeed(gMotorRightTrueSpeed);
            motor->setAimValue(gMotorAimSpeed);
            motor->updateCalcPidCtrl(0, 0);
        }

        if (shouldPrintSpeed)
        {
            printMotorSpeed(stopStateChanged ? "stop_changed" : "periodic_10s");
        }

        lastStopState = gMotorStop;

        std::this_thread::yield();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }

    if (motor != nullptr)
    {
        motor->setLeftPwm(0);
        motor->setRightPwm(0);
    }
}
