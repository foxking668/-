#include "Motor.hpp"
#include "ZebraTest/IoStatus.hpp"
#include <memory>

using namespace Other;

namespace Contral
{
    /*
     * 对 PWM 输出值做统一约束。
     *
     * 关键修复：
     * pwm == 0 必须直接返回 0。
     *
     * 原来的逻辑会把 0 当成小于 MOTOR_MIN_PWM_VALUE，
     * 然后变成 -100，导致 setPwm(0) 不是停车。
     */
    int Motor::constrainPwmValue(int pwm)
    {
        if (pwm == 0)
        {
            return 0;
        }

        if (std::abs(pwm) > MOTOR_MAX_PWM_VALUE)
        {
            pwm = pwm > 0 ? MOTOR_MAX_PWM_VALUE : -MOTOR_MAX_PWM_VALUE;
        }
        else if (std::abs(pwm) < MOTOR_MIN_PWM_VALUE)
        {
            pwm = pwm > 0 ? MOTOR_MIN_PWM_VALUE : -MOTOR_MIN_PWM_VALUE;
        }

        if (std::abs(pwm) > MOTOR_PWM_PERIOD)
        {
            pwm = pwm > 0 ? MOTOR_PWM_PERIOD : -MOTOR_PWM_PERIOD;
        }

        return pwm;
    }

    Motor::Motor(int pwm_left_chip,
                 int pwm_left_index,
                 int dir_left_index,
                 int pwm_right_chip,
                 int pwm_right_index,
                 int dir_right_index,
                 int motor_enable_pin, const std::string& pwmRoot, const std::string& gpioRoot)
    {
        Log log(LOG_LEVEL_GLOBAL);

        log.logOutputConsole("Initialize Motor", LOG_INFO);

        using ZebraTest::IoStatus::require;
        // Keep nSLEEP low until BOTH zero-duty PWM channels and direction GPIOs are ready.
        auto enable=std::make_unique<GPIO>(motor_enable_pin,gpioRoot);
        require(enable->setDirection("low"),"Motor nSLEEP low");
        auto left=std::make_unique<PwmController>(pwm_left_chip,pwm_left_index,pwmRoot);
        auto right=std::make_unique<PwmController>(pwm_right_chip,pwm_right_index,pwmRoot);
        auto leftDir=std::make_unique<GPIO>(dir_left_index,gpioRoot);
        auto rightDir=std::make_unique<GPIO>(dir_right_index,gpioRoot);
        try {
            require(left->prepare(MOTOR_PWM_PERIOD),"Left motor PWM prepare");
            require(right->prepare(MOTOR_PWM_PERIOD),"Right motor PWM prepare");
            require(leftDir->setDirection("out"),"Left motor direction output");
            require(leftDir->setValue(true),"Left motor initial direction");
            require(rightDir->setDirection("out"),"Right motor direction output");
            require(rightDir->setValue(true),"Right motor initial direction");
            require(left->enable(),"Left motor PWM enable");
            require(right->enable(),"Right motor PWM enable");
            require(enable->setValue(true),"Motor nSLEEP enable");
        } catch(...) {
            const std::string cause=ZebraTest::IoStatus::lastError;
            enable->setValue(false);
            ZebraTest::IoStatus::lastError=cause;
            throw;
        }
        this->pwm_left=left.release(); this->pwm_right=right.release();
        this->dir_left=leftDir.release(); this->dir_right=rightDir.release();
        this->motor_enable_gpio=enable.release();
        log.logOutputConsole("Left Motor Initialize Success",LOG_INFO);
        log.logOutputConsole("Right Motor Initialize Success",LOG_INFO);

        this->aim_value = 0;
        this->left_true_speed = 0;
        this->right_true_speed = 0;
        this->pwm_left_value = 0;
        this->pwm_right_value = 0;
        this->left_pid_attr = nullptr;
        this->right_pid_attr = nullptr;
    }

    /*
     * 设置左电机 PWM。
     *
     * 新拓展板实测：原方向电平与车体前进方向相反。
     * 因此这里把方向 GPIO 逻辑反过来：
     * pwm > 0：代码语义为正转/前进，实际写反向电平；
     * pwm < 0：代码语义为反转/后退，实际写正向电平；
     * pwm = 0：真正停车，占空比写 0。
     */
    bool Motor::setLeftPwm(int pwm)
    {
        ZebraTest::IoStatus::lastError.clear();
        GPIO *gpio = static_cast<GPIO *>(this->dir_left);
        PwmController *pwmController = static_cast<PwmController *>(this->pwm_left);

        pwm = this->constrainPwmValue(pwm);
        this->pwm_left_value = pwm;

        bool result = false;
        bool directionOk = true;

        if (pwm > 0)
        {
            directionOk = gpio->setValue(false);
            result = pwmController->setDutyCycle(directionOk ? pwm : 0);
        }
        else if (pwm < 0)
        {
            directionOk = gpio->setValue(true);
            result = pwmController->setDutyCycle(directionOk ? -pwm : 0);
        }
        else
        {
            result = pwmController->setDutyCycle(0);
        }

        result = result && directionOk;
        left_write_ok_ = result;
        left_error_ = result ? "" : ZebraTest::IoStatus::lastError;
        Log log(LOG_LEVEL_GLOBAL);
        if (result)
        {
            log.logOutputConsole(
                "Left pwm set to: " + std::to_string(pwm) +
                    ", direction: " + std::string(pwm > 0 ? "FORWARD" : (pwm < 0 ? "REVERSE" : "STOP")),
                LOG_DEBUG);
        }
        else
        {
            log.logOutputConsole("Failed to set left pwm: " + std::to_string(pwm), LOG_ERROR);
        }
        return result;
    }

    /*
     * 设置右电机 PWM。
     *
     * 新拓展板实测：原方向电平与车体前进方向相反。
     * 因此这里把方向 GPIO 逻辑反过来：
     * pwm > 0：代码语义为正转/前进，实际写反向电平；
     * pwm < 0：代码语义为反转/后退，实际写正向电平；
     * pwm = 0：真正停车，占空比写 0。
     */
    bool Motor::setRightPwm(int pwm)
    {
        ZebraTest::IoStatus::lastError.clear();
        GPIO *gpio = static_cast<GPIO *>(this->dir_right);
        PwmController *pwmController = static_cast<PwmController *>(this->pwm_right);

        pwm = this->constrainPwmValue(pwm);
        this->pwm_right_value = pwm;

        bool result = false;
        bool directionOk = true;

        if (pwm > 0)
        {
            directionOk = gpio->setValue(false);
            result = pwmController->setDutyCycle(directionOk ? pwm : 0);
        }
        else if (pwm < 0)
        {
            directionOk = gpio->setValue(true);
            result = pwmController->setDutyCycle(directionOk ? -pwm : 0);
        }
        else
        {
            result = pwmController->setDutyCycle(0);
        }

        result = result && directionOk;
        right_write_ok_ = result;
        right_error_ = result ? "" : ZebraTest::IoStatus::lastError;
        Log log(LOG_LEVEL_GLOBAL);
        if (result)
        {
            log.logOutputConsole(
                "Right pwm set to: " + std::to_string(pwm) +
                    ", direction: " + std::string(pwm > 0 ? "FORWARD" : (pwm < 0 ? "REVERSE" : "STOP")),
                LOG_DEBUG);
        }
        else
        {
            log.logOutputConsole("Failed to set right pwm: " + std::to_string(pwm), LOG_ERROR);
        }
        return result;
    }

    void Motor::submitLeftPidCtrlAttr(st_PID_Attr *pidAttr)
    {
        this->left_pid_attr = pidAttr;
    }

    void Motor::submitRightPidCtrlAttr(st_PID_Attr *pidAttr)
    {
        this->right_pid_attr = pidAttr;
    }

    void Motor::setAimValue(float aimValue)
    {
        this->aim_value = aimValue;
    }

    void Motor::submitRightTrueSpeed(float trueSpeed)
    {
        this->right_true_speed = trueSpeed;
    }

    void Motor::submitLeftTrueSpeed(float trueSpeed)
    {
        this->left_true_speed = trueSpeed;
    }

    void Motor::submitLeftPwm(int pwm)
    {
        this->pwm_left_value = pwm;
    }

    void Motor::submitRightPwm(int pwm)
    {
        this->pwm_right_value = pwm;
    }

    int Motor::getLeftPwm()
    {
        return this->pwm_left_value;
    }

    int Motor::getRightPwm()
    {
        return this->pwm_right_value;
    }

    void Motor::updateCalcPidCtrl(int offsetLeft, int offsetRight)
    {
        if (this->left_pid_attr == nullptr || this->right_pid_attr == nullptr)
        {
            return;
        }

        float pwmLeftSpeed = PID_Incremental_CalcResult_ByNowTureValue(
            this->left_pid_attr,
            this->left_true_speed,
            this->aim_value);

        float pwmRightSpeed = PID_Incremental_CalcResult_ByNowTureValue(
            this->right_pid_attr,
            this->right_true_speed,
            this->aim_value);

        this->pwm_left_value = pwmLeftSpeed + offsetLeft;
        this->pwm_right_value = pwmRightSpeed + offsetRight;

        this->setLeftPwm(this->pwm_left_value);
        this->setRightPwm(this->pwm_right_value);
    }

    Motor::~Motor()
    {
        if(this->motor_enable_gpio!=nullptr)
            static_cast<GPIO*>(this->motor_enable_gpio)->setValue(false);
        if (this->pwm_left != nullptr)
        {
            PwmController *pwmController = (PwmController *)this->pwm_left;
            pwmController->disable();
            delete (PwmController *)this->pwm_left;
            this->pwm_left = nullptr;
        }

        if (this->pwm_right != nullptr)
        {
            PwmController *pwmController = (PwmController *)this->pwm_right;
            pwmController->disable();
            delete (PwmController *)this->pwm_right;
            this->pwm_right = nullptr;
        }

        if (this->dir_left != nullptr)
        {
            delete (GPIO *)this->dir_left;
            this->dir_left = nullptr;
        }

        if (this->dir_right != nullptr)
        {
            delete (GPIO *)this->dir_right;
            this->dir_right = nullptr;
        }

        if (this->motor_enable_gpio != nullptr)
        {
            delete (GPIO *)this->motor_enable_gpio;
            this->motor_enable_gpio = nullptr;
        }
    }
}