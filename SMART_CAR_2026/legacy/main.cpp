#include "headfile.hpp"

using namespace ImageProcess;
using namespace Contral;
using namespace Other;
using namespace std;

void SigintHandler(int sig);

namespace
{
    /*
     * 电机/编码器/反转测试模式。
     *
     * 使用方法：
     *   sudo ./SMART_CAR --motor-test
     *   sudo ./SMART_CAR --motor-test 3000 2000
     *
     * 参数：
     *   第一个数字：测试 PWM，默认 3000，正负方向由测试步骤自动切换；
     *   第二个数字：每个方向持续时间 ms，默认 2000。
     *
     * 注意：
     *   1. 测试前务必把小车架空，让左右轮离地；
     *   2. 本模式不会启动摄像头、舵机、图像线程和普通电机 PID 线程；
     *   3. 本模式直接对左右电机输出正/负 PWM，并打印左右编码器速度；
     *   4. 如果电机物理方向变了，但编码器符号不变，重点检查编码器方向 GPIO；
     *   5. 如果正负 PWM 下电机方向不变，重点检查电机方向 GPIO 或驱动板 DIR 引脚。
     */
    int parseTestIntArg(int argc, char **argv, int index, int defaultValue, int minValue, int maxValue)
    {
        if (argc <= index || argv[index] == nullptr)
        {
            return defaultValue;
        }

        try
        {
            int value = std::stoi(argv[index]);
            if (value < minValue)
            {
                value = minValue;
            }
            if (value > maxValue)
            {
                value = maxValue;
            }
            return value;
        }
        catch (...)
        {
            return defaultValue;
        }
    }

    void stopTestMotor(Motor &motor)
    {
        motor.setLeftPwm(0);
        motor.setRightPwm(0);
        usleep(300 * 1000);
    }


    enum class StartupDebugExitMode
    {
        EXIT_PROGRAM,
        CONTINUE_NORMAL
    };

    class TerminalRawNonblockGuard
    {
    private:
        bool valid_;
        termios old_term_{};
        int old_flags_;

    public:
        TerminalRawNonblockGuard() : valid_(false), old_flags_(-1)
        {
            if (!isatty(STDIN_FILENO))
            {
                return;
            }
            if (tcgetattr(STDIN_FILENO, &old_term_) != 0)
            {
                return;
            }

            termios new_term = old_term_;
            new_term.c_lflag &= static_cast<unsigned int>(~(ICANON | ECHO));
            new_term.c_cc[VMIN] = 0;
            new_term.c_cc[VTIME] = 0;

            old_flags_ = fcntl(STDIN_FILENO, F_GETFL, 0);
            if (old_flags_ < 0)
            {
                return;
            }

            if (tcsetattr(STDIN_FILENO, TCSANOW, &new_term) != 0)
            {
                return;
            }
            if (fcntl(STDIN_FILENO, F_SETFL, old_flags_ | O_NONBLOCK) != 0)
            {
                tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
                return;
            }

            valid_ = true;
        }

        ~TerminalRawNonblockGuard()
        {
            if (valid_)
            {
                tcsetattr(STDIN_FILENO, TCSANOW, &old_term_);
                fcntl(STDIN_FILENO, F_SETFL, old_flags_);
            }
        }

        bool valid() const
        {
            return valid_;
        }
    };

    int readCharNonblock()
    {
        char ch = 0;
        ssize_t n = read(STDIN_FILENO, &ch, 1);
        if (n == 1)
        {
            return static_cast<unsigned char>(ch);
        }
        return -1;
    }

    bool waitStartupDebugKey(int waitMs)
    {
        std::cout << "[StartupDebug] 3秒内按 C 进入电机/编码器调试；不按则正常启动。" << std::endl;

        TerminalRawNonblockGuard guard;
        if (!guard.valid())
        {
            // 如果不是交互终端，保留原来的 3 秒等待。
            usleep(waitMs * 1000);
            return false;
        }

        const auto start = std::chrono::steady_clock::now();
        while (!gIsClose)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if (elapsedMs >= waitMs)
            {
                break;
            }

            int ch = readCharNonblock();
            if (ch == 'c' || ch == 'C')
            {
                std::cout << "\n[StartupDebug] 按下 C，进入电机/编码器调试模式。" << std::endl;
                return true;
            }

            usleep(50 * 1000);
        }

        return false;
    }

    std::string trimLocal(const std::string &s)
    {
        size_t b = 0;
        while (b < s.size() && std::isspace(static_cast<unsigned char>(s[b])))
        {
            b++;
        }
        size_t e = s.size();
        while (e > b && std::isspace(static_cast<unsigned char>(s[e - 1])))
        {
            e--;
        }
        return s.substr(b, e - b);
    }

    std::string upperLocal(std::string s)
    {
        for (auto &c : s)
        {
            c = static_cast<char>(std::toupper(static_cast<unsigned char>(c)));
        }
        return s;
    }

    int clampIntLocal(int value, int minValue, int maxValue)
    {
        if (value < minValue)
        {
            return minValue;
        }
        if (value > maxValue)
        {
            return maxValue;
        }
        return value;
    }

    int askInt(const std::string &prompt, int defaultValue, int minValue, int maxValue)
    {
        std::cout << prompt << " [默认 " << defaultValue << "]: ";
        std::string line;
        std::getline(std::cin, line);
        line = trimLocal(line);
        if (line.empty())
        {
            return defaultValue;
        }
        try
        {
            return clampIntLocal(std::stoi(line), minValue, maxValue);
        }
        catch (...)
        {
            std::cout << "[Debug] 输入无效，使用默认值 " << defaultValue << std::endl;
            return defaultValue;
        }
    }

    double askDouble(const std::string &prompt, double defaultValue, double minValue, double maxValue)
    {
        std::cout << prompt << " [默认 " << defaultValue << "]: ";
        std::string line;
        std::getline(std::cin, line);
        line = trimLocal(line);
        if (line.empty())
        {
            return defaultValue;
        }
        try
        {
            double value = std::stod(line);
            if (value < minValue)
            {
                value = minValue;
            }
            if (value > maxValue)
            {
                value = maxValue;
            }
            return value;
        }
        catch (...)
        {
            std::cout << "[Debug] 输入无效，使用默认值 " << defaultValue << std::endl;
            return defaultValue;
        }
    }


    void recreateDebugEncoders(int leftPwmCounter,
                               int leftDirGpio,
                               int rightPwmCounter,
                               int rightDirGpio,
                               Encoder *&leftEncoder,
                               Encoder *&rightEncoder)
    {
        /*
         * 只在启动调试菜单里调用。此时图像线程和电机线程还没有启动，
         * 可以安全地释放并重新创建全局编码器对象，用来临时验证不同 PWM 计数通道。
         */
        if (gLeftEncoder != nullptr)
        {
            delete static_cast<Encoder *>(gLeftEncoder);
            gLeftEncoder = nullptr;
        }
        if (gRightEncoder != nullptr)
        {
            delete static_cast<Encoder *>(gRightEncoder);
            gRightEncoder = nullptr;
        }

        gLeftEncoder = new Encoder(leftPwmCounter, leftDirGpio);
        gRightEncoder = new Encoder(rightPwmCounter, rightDirGpio);

        leftEncoder = static_cast<Encoder *>(gLeftEncoder);
        rightEncoder = static_cast<Encoder *>(gRightEncoder);

        std::cout << "[StartupDebug] 已重建编码器对象："
                  << " left_pwm_counter=" << leftPwmCounter
                  << " left_dir_gpio=" << leftDirGpio
                  << " right_pwm_counter=" << rightPwmCounter
                  << " right_dir_gpio=" << rightDirGpio
                  << std::endl;
    }

    std::string askWheel()
    {
        while (true)
        {
            std::cout << "选择轮子：L=左轮，R=右轮，B=双轮 [默认 B]: ";
            std::string line;
            std::getline(std::cin, line);
            line = upperLocal(trimLocal(line));
            if (line.empty())
            {
                return "B";
            }
            if (line == "L" || line == "R" || line == "B")
            {
                return line;
            }
            std::cout << "[Debug] 只能输入 L / R / B。" << std::endl;
        }
    }

    int askDirectionSign()
    {
        while (true)
        {
            std::cout << "选择方向：F=正转，B=反转 [默认 F]: ";
            std::string line;
            std::getline(std::cin, line);
            line = upperLocal(trimLocal(line));
            if (line.empty() || line == "F")
            {
                return 1;
            }
            if (line == "B" || line == "R")
            {
                return -1;
            }
            std::cout << "[Debug] 只能输入 F / B。" << std::endl;
        }
    }

    const char *speedDirectionText(double value)
    {
        if (value > 0.02)
        {
            return "FORWARD/正";
        }
        if (value < -0.02)
        {
            return "REVERSE/反";
        }
        return "STOP/无脉冲";
    }

    void buildWheelPwm(const std::string &wheel, int signedPwm, int &leftPwm, int &rightPwm)
    {
        leftPwm = 0;
        rightPwm = 0;
        if (wheel == "L" || wheel == "B")
        {
            leftPwm = signedPwm;
        }
        if (wheel == "R" || wheel == "B")
        {
            rightPwm = signedPwm;
        }
    }

    void runManualEncoderMonitor(Motor &motor, Encoder &leftEncoder, Encoder &rightEncoder)
    {
        stopTestMotor(motor);
        std::cout << "\n[ManualEncoder] 手动编码器检测开始。" << std::endl;
        std::cout << "[ManualEncoder] 电机 PWM 已清零。用手转左/右轮，程序会实时打印左右编码器速度和方向。" << std::endl;
        std::cout << "[ManualEncoder] 按 Q 退出。" << std::endl;

        TerminalRawNonblockGuard guard;
        const auto start = std::chrono::steady_clock::now();
        auto last = start;
        double leftTurns = 0.0;
        double rightTurns = 0.0;
        int sample = 0;

        while (!gIsClose)
        {
            int ch = guard.valid() ? readCharNonblock() : -1;
            if (ch == 'q' || ch == 'Q')
            {
                break;
            }

            const auto now = std::chrono::steady_clock::now();
            const double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count() / 1000000.0;
            last = now;

            const double leftSpeed = leftEncoder.pulseConterUpdate();
            const double rightSpeed = rightEncoder.pulseConterUpdate();
            leftTurns += leftSpeed * dt;
            rightTurns += rightSpeed * dt;

            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            std::cout << "[ManualEncoder] sample=" << sample
                      << " elapsed_ms=" << elapsedMs
                      << " left_rps=" << leftSpeed
                      << " left_dir=" << speedDirectionText(leftSpeed)
                      << " left_turns=" << leftTurns
                      << " right_rps=" << rightSpeed
                      << " right_dir=" << speedDirectionText(rightSpeed)
                      << " right_turns=" << rightTurns
                      << std::endl;

            sample++;
            usleep(200 * 1000);
        }

        stopTestMotor(motor);
        std::cout << "[ManualEncoder] 退出。left_turns=" << leftTurns
                  << " right_turns=" << rightTurns << std::endl;
    }

    void runCustomTimedMotorTest(Motor &motor,
                                 Encoder &leftEncoder,
                                 Encoder &rightEncoder,
                                 const std::string &name,
                                 const std::string &wheel,
                                 int signedPwm,
                                 int durationMs,
                                 double targetTurns)
    {
        int leftPwm = 0;
        int rightPwm = 0;
        buildWheelPwm(wheel, signedPwm, leftPwm, rightPwm);

        std::cout << "\n[DebugMotor] STEP=" << name
                  << " wheel=" << wheel
                  << " left_pwm=" << leftPwm
                  << " right_pwm=" << rightPwm
                  << " duration_ms=" << durationMs;
        if (targetTurns > 0.0)
        {
            std::cout << " target_turns=" << targetTurns;
        }
        std::cout << std::endl;

        motor.setLeftPwm(leftPwm);
        motor.setRightPwm(rightPwm);

        const auto start = std::chrono::steady_clock::now();
        auto last = start;
        int sample = 0;
        double leftTurns = 0.0;
        double rightTurns = 0.0;
        double leftAbsTurns = 0.0;
        double rightAbsTurns = 0.0;
        double leftSum = 0.0;
        double rightSum = 0.0;
        int sampleCount = 0;

        while (!gIsClose)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - start).count();
            if (elapsedMs >= durationMs)
            {
                break;
            }

            const double dt = std::chrono::duration_cast<std::chrono::microseconds>(now - last).count() / 1000000.0;
            last = now;

            const double leftSpeed = leftEncoder.pulseConterUpdate();
            const double rightSpeed = rightEncoder.pulseConterUpdate();
            leftTurns += leftSpeed * dt;
            rightTurns += rightSpeed * dt;
            leftAbsTurns += std::abs(leftSpeed) * dt;
            rightAbsTurns += std::abs(rightSpeed) * dt;
            leftSum += leftSpeed;
            rightSum += rightSpeed;
            sampleCount++;

            std::cout << "[DebugMotor] sample=" << sample
                      << " elapsed_ms=" << elapsedMs
                      << " left_rps=" << leftSpeed
                      << " left_dir=" << speedDirectionText(leftSpeed)
                      << " left_turns=" << leftTurns
                      << " right_rps=" << rightSpeed
                      << " right_dir=" << speedDirectionText(rightSpeed)
                      << " right_turns=" << rightTurns
                      << std::endl;

            if (targetTurns > 0.0)
            {
                bool reached = false;
                if (wheel == "L")
                {
                    reached = leftAbsTurns >= targetTurns;
                }
                else if (wheel == "R")
                {
                    reached = rightAbsTurns >= targetTurns;
                }
                else
                {
                    reached = leftAbsTurns >= targetTurns && rightAbsTurns >= targetTurns;
                }

                if (reached)
                {
                    std::cout << "[DebugMotor] target turns reached." << std::endl;
                    break;
                }
            }

            sample++;
            usleep(100 * 1000);
        }

        stopTestMotor(motor);
        const double leftAvg = sampleCount > 0 ? leftSum / sampleCount : 0.0;
        const double rightAvg = sampleCount > 0 ? rightSum / sampleCount : 0.0;
        std::cout << "[DebugMotor] SUMMARY STEP=" << name
                  << " left_avg_rps=" << leftAvg
                  << " right_avg_rps=" << rightAvg
                  << " left_turns=" << leftTurns
                  << " right_turns=" << rightTurns
                  << " samples=" << sampleCount
                  << std::endl;
    }


    void runOneMotorEncoderTestStep(const char *stepName,
                                    Motor &motor,
                                    Encoder &leftEncoder,
                                    Encoder &rightEncoder,
                                    int leftPwm,
                                    int rightPwm,
                                    int durationMs)
    {
        std::cout << "\n[MotorEncoderTest] STEP=" << stepName
                  << " left_pwm=" << leftPwm
                  << " right_pwm=" << rightPwm
                  << " duration_ms=" << durationMs
                  << std::endl;

        motor.setLeftPwm(leftPwm);
        motor.setRightPwm(rightPwm);

        const auto startTime = std::chrono::steady_clock::now();
        int sampleIndex = 0;
        double leftSum = 0.0;
        double rightSum = 0.0;
        int sampleCount = 0;

        while (!gIsClose)
        {
            const auto now = std::chrono::steady_clock::now();
            const auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - startTime).count();
            if (elapsedMs >= durationMs)
            {
                break;
            }

            const double leftSpeed = leftEncoder.pulseConterUpdate();
            const double rightSpeed = rightEncoder.pulseConterUpdate();

            leftSum += leftSpeed;
            rightSum += rightSpeed;
            sampleCount++;

            std::cout << "[MotorEncoderTest] sample=" << sampleIndex
                      << " elapsed_ms=" << elapsedMs
                      << " left_encoder=" << leftSpeed
                      << " right_encoder=" << rightSpeed
                      << std::endl;

            sampleIndex++;
            usleep(100 * 1000);
        }

        stopTestMotor(motor);

        const double leftAvg = sampleCount > 0 ? leftSum / sampleCount : 0.0;
        const double rightAvg = sampleCount > 0 ? rightSum / sampleCount : 0.0;

        std::cout << "[MotorEncoderTest] SUMMARY STEP=" << stepName
                  << " left_avg=" << leftAvg
                  << " right_avg=" << rightAvg
                  << " samples=" << sampleCount
                  << std::endl;
    }

    int runMotorEncoderReverseTest(int argc, char **argv)
    {
        signal(SIGINT, SigintHandler);
        gIsClose = false;

        const int testPwm = parseTestIntArg(argc, argv, 2, 3000, MOTOR_MIN_PWM_VALUE, 15000);
        const int durationMs = parseTestIntArg(argc, argv, 3, 2000, 500, 10000);

        std::cout << "[MotorEncoderTest] START" << std::endl;
        std::cout << "[MotorEncoderTest] usage: sudo ./SMART_CAR --motor-test [pwm] [duration_ms]" << std::endl;
        std::cout << "[MotorEncoderTest] current pwm=" << testPwm
                  << " duration_ms=" << durationMs
                  << std::endl;
        std::cout << "[MotorEncoderTest] IMPORTANT: lift the car wheels before test." << std::endl;

        std::cout << "[MotorEncoderTest] motor config: "
                  << "left_pwm=pwmchip" << MOTOR_LEFT_PWM_CHIP << "/pwm" << MOTOR_LEFT_PWM_INDEX
                  << " left_dir_gpio=" << MOTOR_LEFT_DIR_NUMBER
                  << " right_pwm=pwmchip" << MOTOR_RIGHT_PWM_CHIP << "/pwm" << MOTOR_RIGHT_PWM_INDEX
                  << " right_dir_gpio=" << MOTOR_RIGHT_DIR_NUMBER
                  << " enable_gpio=" << MOTOR_ENABLE_PIN
                  << std::endl;

        std::cout << "[MotorEncoderTest] encoder config: "
                  << "left_pwm_counter=" << ENCODER_LEFT_PIN
                  << " left_dir_gpio=" << ENCODER_LEFT_DIR
                  << " right_pwm_counter=" << ENCODER_RIGHT_PIN
                  << " right_dir_gpio=" << ENCODER_RIGHT_DIR
                  << std::endl;

        Motor motor(MOTOR_LEFT_PWM_CHIP,
                    MOTOR_LEFT_PWM_INDEX,
                    MOTOR_LEFT_DIR_NUMBER,
                    MOTOR_RIGHT_PWM_CHIP,
                    MOTOR_RIGHT_PWM_INDEX,
                    MOTOR_RIGHT_DIR_NUMBER,
                    MOTOR_ENABLE_PIN);
        Encoder leftEncoder(ENCODER_LEFT_PIN, ENCODER_LEFT_DIR);
        Encoder rightEncoder(ENCODER_RIGHT_PIN, ENCODER_RIGHT_DIR);

        stopTestMotor(motor);

        runOneMotorEncoderTestStep("LEFT_FORWARD", motor, leftEncoder, rightEncoder, testPwm, 0, durationMs);
        runOneMotorEncoderTestStep("LEFT_REVERSE", motor, leftEncoder, rightEncoder, -testPwm, 0, durationMs);
        runOneMotorEncoderTestStep("RIGHT_FORWARD", motor, leftEncoder, rightEncoder, 0, testPwm, durationMs);
        runOneMotorEncoderTestStep("RIGHT_REVERSE", motor, leftEncoder, rightEncoder, 0, -testPwm, durationMs);
        runOneMotorEncoderTestStep("BOTH_FORWARD", motor, leftEncoder, rightEncoder, testPwm, testPwm, durationMs);
        runOneMotorEncoderTestStep("BOTH_REVERSE", motor, leftEncoder, rightEncoder, -testPwm, -testPwm, durationMs);

        stopTestMotor(motor);

        std::cout << "\n[MotorEncoderTest] END" << std::endl;
        std::cout << "[MotorEncoderTest] 判断方法：" << std::endl;
        std::cout << "[MotorEncoderTest] 1. LEFT_FORWARD 与 LEFT_REVERSE 时，左电机物理方向应相反。" << std::endl;
        std::cout << "[MotorEncoderTest] 2. RIGHT_FORWARD 与 RIGHT_REVERSE 时，右电机物理方向应相反。" << std::endl;
        std::cout << "[MotorEncoderTest] 3. 如果物理方向不反，优先检查 GPIO12/GPIO13 或驱动板 DIR。" << std::endl;
        std::cout << "[MotorEncoderTest] 4. 如果物理方向反了但编码器正负不反，优先检查编码器方向 GPIO75/GPIO72。" << std::endl;
        return 0;
    }

    StartupDebugExitMode runStartupMotorEncoderDebugMenu()
    {
        Motor *motor = static_cast<Motor *>(gMotor);
        Encoder *leftEncoder = static_cast<Encoder *>(gLeftEncoder);
        Encoder *rightEncoder = static_cast<Encoder *>(gRightEncoder);

        if (motor == nullptr || leftEncoder == nullptr || rightEncoder == nullptr)
        {
            std::cout << "[StartupDebug] 电机或编码器对象为空，无法进入调试。" << std::endl;
            return StartupDebugExitMode::EXIT_PROGRAM;
        }

        stopTestMotor(*motor);

        int currentLeftEncoderPwmCounter = ENCODER_LEFT_PIN;
        int currentLeftEncoderDirGpio = ENCODER_LEFT_DIR;
        int currentRightEncoderPwmCounter = ENCODER_RIGHT_PIN;
        int currentRightEncoderDirGpio = ENCODER_RIGHT_DIR;
        int defaultDebugPwm = 3000;
        int defaultDebugDurationMs = 2000;

        while (!gIsClose)
        {
            std::cout << "\n================ 电机/编码器调试菜单 ================" << std::endl;
            std::cout << "当前编码器：left_pwm_counter=" << currentLeftEncoderPwmCounter
                      << " left_dir_gpio=" << currentLeftEncoderDirGpio
                      << " right_pwm_counter=" << currentRightEncoderPwmCounter
                      << " right_dir_gpio=" << currentRightEncoderDirGpio
                      << " | 默认PWM=" << defaultDebugPwm
                      << " 默认时长=" << defaultDebugDurationMs << "ms" << std::endl;
            std::cout << "  1. 手动转轮编码器检测：电机停止，用手转轮，打印左右速度/正反/累计圈数" << std::endl;
            std::cout << "  2. 指定轮子定时正/反转：选择左/右/双轮，设置 PWM 和时间" << std::endl;
            std::cout << "  3. 指定轮子按目标圈数正/反转：设置 PWM 和目标圈数，按编码器估算停止" << std::endl;
            std::cout << "  4. 自动顺序测试：左正、左反、右正、右反、双正、双反" << std::endl;
            std::cout << "  5. 修改本次调试的编码器计数通道 / 方向GPIO / 默认测试PWM" << std::endl;
            std::cout << "  9. 退出调试并继续正常启动小车" << std::endl;
            std::cout << "  0. 退出调试并关闭程序" << std::endl;
            std::cout << "请选择：";

            std::string choice;
            std::getline(std::cin, choice);
            choice = trimLocal(choice);

            if (choice == "1")
            {
                runManualEncoderMonitor(*motor, *leftEncoder, *rightEncoder);
            }
            else if (choice == "2")
            {
                const std::string wheel = askWheel();
                const int directionSign = askDirectionSign();
                const int pwm = askInt("输入测试 PWM", defaultDebugPwm, MOTOR_MIN_PWM_VALUE, 15000);
                const int durationMs = askInt("输入持续时间 ms", defaultDebugDurationMs, 300, 20000);
                runCustomTimedMotorTest(*motor, *leftEncoder, *rightEncoder,
                                        directionSign > 0 ? "CUSTOM_FORWARD_TIME" : "CUSTOM_REVERSE_TIME",
                                        wheel, directionSign * pwm, durationMs, 0.0);
            }
            else if (choice == "3")
            {
                const std::string wheel = askWheel();
                const int directionSign = askDirectionSign();
                const int pwm = askInt("输入测试 PWM", defaultDebugPwm, MOTOR_MIN_PWM_VALUE, 15000);
                const double targetTurns = askDouble("输入目标圈数/转数", 2.0, 0.1, 100.0);
                const int maxDurationMs = askInt("输入最长保护时间 ms", 8000, 1000, 60000);
                runCustomTimedMotorTest(*motor, *leftEncoder, *rightEncoder,
                                        directionSign > 0 ? "CUSTOM_FORWARD_TURNS" : "CUSTOM_REVERSE_TURNS",
                                        wheel, directionSign * pwm, maxDurationMs, targetTurns);
            }
            else if (choice == "4")
            {
                const int pwm = askInt("输入自动测试 PWM", defaultDebugPwm, MOTOR_MIN_PWM_VALUE, 15000);
                const int durationMs = askInt("输入每个步骤持续时间 ms", defaultDebugDurationMs, 500, 10000);
                runOneMotorEncoderTestStep("LEFT_FORWARD", *motor, *leftEncoder, *rightEncoder, pwm, 0, durationMs);
                runOneMotorEncoderTestStep("LEFT_REVERSE", *motor, *leftEncoder, *rightEncoder, -pwm, 0, durationMs);
                runOneMotorEncoderTestStep("RIGHT_FORWARD", *motor, *leftEncoder, *rightEncoder, 0, pwm, durationMs);
                runOneMotorEncoderTestStep("RIGHT_REVERSE", *motor, *leftEncoder, *rightEncoder, 0, -pwm, durationMs);
                runOneMotorEncoderTestStep("BOTH_FORWARD", *motor, *leftEncoder, *rightEncoder, pwm, pwm, durationMs);
                runOneMotorEncoderTestStep("BOTH_REVERSE", *motor, *leftEncoder, *rightEncoder, -pwm, -pwm, durationMs);
            }
            else if (choice == "5")
            {
                stopTestMotor(*motor);
                std::cout << "\n[StartupDebug] 修改本次调试参数。" << std::endl;
                std::cout << "提示：根据你上传的引脚表，编码器1=GPIO64/pwm[0]+DIR75，编码器2=GPIO67/pwm[3]+DIR72。" << std::endl;
                std::cout << "如果左轮没动但 left_encoder 有大数，优先把左编码器计数通道设为 0。" << std::endl;

                currentLeftEncoderPwmCounter = askInt("左编码器 PWM 计数通道", currentLeftEncoderPwmCounter, 0, 8);
                currentLeftEncoderDirGpio = askInt("左编码器 DIR GPIO", currentLeftEncoderDirGpio, 0, 200);
                currentRightEncoderPwmCounter = askInt("右编码器 PWM 计数通道", currentRightEncoderPwmCounter, 0, 8);
                currentRightEncoderDirGpio = askInt("右编码器 DIR GPIO", currentRightEncoderDirGpio, 0, 200);
                defaultDebugPwm = askInt("默认测试 PWM/动力", defaultDebugPwm, MOTOR_MIN_PWM_VALUE, 15000);
                defaultDebugDurationMs = askInt("默认每步测试时长 ms", defaultDebugDurationMs, 500, 20000);

                recreateDebugEncoders(currentLeftEncoderPwmCounter,
                                      currentLeftEncoderDirGpio,
                                      currentRightEncoderPwmCounter,
                                      currentRightEncoderDirGpio,
                                      leftEncoder,
                                      rightEncoder);
            }
            else if (choice == "9")
            {
                stopTestMotor(*motor);
                std::cout << "[StartupDebug] 退出调试，继续正常启动。" << std::endl;
                return StartupDebugExitMode::CONTINUE_NORMAL;
            }
            else if (choice == "0" || upperLocal(choice) == "Q")
            {
                stopTestMotor(*motor);
                std::cout << "[StartupDebug] 退出调试并关闭程序。" << std::endl;
                return StartupDebugExitMode::EXIT_PROGRAM;
            }
            else
            {
                std::cout << "[StartupDebug] 输入无效。" << std::endl;
            }
        }

        stopTestMotor(*motor);
        return StartupDebugExitMode::EXIT_PROGRAM;
    }
}

/*
 * SIGINT 信号处理函数。
 *
 * 当用户按下 Ctrl + C 时，不直接粗暴退出程序，
 * 而是通过设置全局关闭标记 `gIsClose`，让各线程走正常退出流程，
 * 以确保摄像头、PWM、GPIO 等资源被安全释放。
 */
/**
 * @brief Ctrl+C 信号处理函数
 *
 * @details
 * 在收到 SIGINT 时设置全局关闭标志，避免进程被强制中断，从而让线程和硬件资源按既定流程安全释放。
 *
 * @param sig 收到的系统信号编号。
 */
void SigintHandler(int sig)
{
    if (sig == SIGINT)
    {
        if (!gIsClose)
        {
            // ctrl+c退出时执行的代码
            /* quiet: waiting close */
            gIsClose = true;
        }
    }
}
/**
 * @brief 程序主入口
 *
 * @details
 * 完成智能车程序的整体启动、线程创建、关闭等待与资源回收，是进程生命周期的总调度函数。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
int main(int argc, char **argv)
{
    if (argc >= 2 && argv[1] != nullptr && std::string(argv[1]) == "--motor-test")
    {
        return runMotorEncoderReverseTest(argc, argv);
    }

    /*
     * 程序主入口。
     *
     * 启动流程概述：
     * 1. 初始化日志系统与进程调度优先级；
     * 2. 初始化全局控制对象（电机、编码器、相机、舵机、PID 等）；
     * 3. 创建图像处理线程和电机控制线程；
     * 4. 等待关闭信号；
     * 5. 回收线程与硬件资源；
     * 6. 如有需要，触发系统服务重启。
     */

    /*
     * 第一步：创建日志对象。
     *
     * `LOG_LEVEL_GLOBAL` 是全局日志等级宏，定义在 `define.hpp` 中。
     * 后续 main 流程中的初始化、线程创建、线程关闭都会通过该对象输出状态。
     */
    Log log(LOG_LEVEL_GLOBAL);

    /*
     * 第二步：尝试把当前进程切换到实时调度策略。
     *
     * 可选策略说明：
     * - SCHED_FIFO：实时先进先出调度，优先级高但可能长期占用 CPU；
     * - SCHED_RR：实时轮转调度，当前代码使用它，实时性和公平性相对均衡；
     * - SCHED_OTHER：普通 Linux 分时调度。
     *
     * 这里传入优先级 99，通常需要 root 权限或 CAP_SYS_NICE 能力。
     */
    // bool isSetMaxPriority = SetProcessPriorityRealTime(99, SCHED_FIFO);
    bool isSetMaxPriority = SetProcessPriorityRealTime(99, SCHED_RR);
    // bool isSetMaxPriority = SetProcessPriorityRealTime(0, SCHED_OTHER);

    if (isSetMaxPriority)
    {
        log.logOutputConsole("The progress have been set RealTime Mode", LOG_INFO);
    }

    /*
     * 第三步：初始化 main 控制用全局标志。
     *
     * - gIsReload：退出后是否重启 systemd 服务；
     * - gIsClose：整个程序是否进入关闭流程。
     */
    gIsReload = false;
    gIsClose = false;

    /*
     * 第四步：注册 Ctrl+C 信号处理函数。
     *
     * 一旦用户按 Ctrl+C，`SigintHandler()` 会把 `gIsClose` 置为 true，
     * 图像线程和电机线程检测到该标志后会自然退出循环。
     */
    signal(SIGINT, SigintHandler);

    /*
     * 第五步：初始化全局资源。
     *
     * 这是 main 中最重要的初始化调用，会创建并配置：
     * - 左右电机对象；
     * - 左右编码器；
     * - 摄像头；
     * - 舵机；
     * - 左右轮 PID 控制器。
     */
    globalInit();

    /*
     * 第六步：设置启动标志。
     *
     * 当前代码初始化完成后直接开始运行；
     * 如果后续要接按键/串口启动，可以把这里改成由外部事件设置。
     */
    gIsStart = true;

    log.logOutputConsole("Init Success", LOG_INFO);

    /*
     * 第七步：初始化完成后的短暂缓冲。
     *
     * 这里循环 3 次，每次 100ms，总计约 300ms，
     * 给底层设备、日志输出或外设状态一点稳定时间。
     */
    for (size_t i = 0; i < 3; i++)
    {
        usleep(1000 * 100);
    }

    log.logOutputConsole("Wait Key", LOG_INFO);

    /*
     * 第八步：等待启动条件。
     *
     * 当前代码已经提前把 `gIsStart` 设置为 true，
     * 因此这里一般不会阻塞。
     * 这个结构保留了“等待按键/上位机命令启动”的扩展入口。
     */
    while (!gIsStart && !gIsClose)
    {
        sleep(1);
    }

    /*
     * 第九步：正式启动前等待约 3 秒。
     *
     * 适合给操作者留出撤离、放车或等待外设稳定的时间。
     */
    // Wait Three Senconds

    bool enterStartupDebug = waitStartupDebugKey(3000);
    if (enterStartupDebug && !gIsClose)
    {
        StartupDebugExitMode debugExitMode = runStartupMotorEncoderDebugMenu();
        if (debugExitMode == StartupDebugExitMode::EXIT_PROGRAM)
        {
            gIsClose = true;
        }
    }

    if (!gIsClose)
    {

        log.logOutputConsole("START!!!", LOG_INFO);

        /*
         * 第十步：创建图像处理线程。
         *
         * 线程入口是 `threadRunImageHandle()`，它会循环完成：
         * - 从摄像头取帧；
         * - 识别赛道线；
         * - 计算转向误差；
         * - 调用舵机对象修正方向。
         */
        std::thread threadRunImageHandleMission(threadRunImageHandle);
        log.logOutputConsole("Create Thread threadRunImageHandleMission", LOG_INFO);

        /*
         * 第十一步：创建电机控制线程。
         *
         * 线程入口是 `threadMotorHandle()`，它会循环完成：
         * - 读取左右编码器速度；
         * - 用 PID 计算左右轮 PWM；
         * - 控制电机维持目标速度。
         */
        std::thread threadMotorHandleMission(threadMotorHandle);
        log.logOutputConsole("Create Thread threadMotorHandleMission", LOG_INFO);

        /*
         * 第十二步：main 线程进入守护等待状态。
         *
         * main 本身不直接跑控制算法，只负责守住进程生命周期。
         * 只要 `gIsClose` 没有被 Ctrl+C 或其他逻辑置为 true，就每秒睡眠一次。
         */
        while (!gIsClose)
        {
            sleep(1);
        }

        /*
         * 第十三步：等待两个工作线程退出。
         *
         * `join()` 会阻塞 main，直到对应线程函数执行结束。
         * 这样可以确保线程已经停止访问全局对象后，再释放硬件资源。
         */
        threadRunImageHandleMission.join();
        log.logOutputConsole("Close Thread threadRunImageHandleMission", LOG_INFO);
        threadMotorHandleMission.join();
        log.logOutputConsole("Close Thread threadMotorHandleMission", LOG_INFO);
    }

    /*
     * 第十四步：线程退出后再稍等 1 秒。
     *
     * 这里给日志输出、系统调度和底层设备状态切换留一点缓冲时间。
     */
    sleep(1);

    /*
     * 第十五步：销毁全局硬件资源。
     *
     * 必须在线程 join 之后调用，避免线程还在使用对象时对象被释放。
     */
    globalDestroy();

    /* quiet: closed success */

    /*
     * 第十六步：可选重启 systemd 服务。
     *
     * 如果运行过程中有逻辑把 `gIsReload` 置为 true，
     * 程序退出前会调用 systemctl 重启 `RunCar.service`。
     */
    if (gIsReload)
    {
        system("systemctl restart RunCar.service");
    }

    return 0;
}
