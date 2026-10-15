#include "Steer.hpp"

// 使用 Other 命名空间中的类或函数
// 例如：Log、PwmController 等可能定义在 Other 命名空间中
using namespace Other;

// 注意：这里保持原来的命名空间 Contral，不修改为 Control
namespace Contral
{
    /**
     * @brief 构造 Steer 对象
     *
     * @details
     * 创建 Contral / Steer 模块对象并初始化成员变量，
     * 使后续硬件访问、图像处理或控制算法调用具备有效上下文。
     *
     * 这里使用 default 默认构造函数，
     * 表示构造函数内部不做额外操作，
     * 成员变量的初始化依赖于类内默认值或头文件中的初始化方式。
     */
    Steer::Steer() = default;

    /**
     * @brief 初始化舵机相关资源
     *
     * @details
     * 该函数主要完成以下工作：
     *
     * 1. 创建日志对象，用于输出初始化状态；
     * 2. 创建 PWM 控制器对象；
     * 3. 初始化 PWM 控制器；
     * 4. 设置 PWM 周期；
     * 5. 设置舵机初始角度为 90 度，也就是中位角；
     * 6. 使能 PWM 输出。
     *
     * 调用该函数后，舵机 PWM 输出通道开始工作。
     */
    void Steer::init()
    {
        /*
         * 创建日志对象。
         *
         * LOG_LEVEL_GLOBAL 通常表示使用全局日志等级，
         * 后续可通过 logOutputConsole 输出初始化成功或失败的信息。
         */
        Log log(LOG_LEVEL_GLOBAL);

        /*
         * 下面这一行是旧的 PWM 初始化方式，目前被注释掉了。
         * 保留该注释，不影响当前程序逻辑。
         */
        // PWM_ATIM test(86, 0b11, 3, 200000, 50000, 1);

        /*
         * 动态创建一个 PWM 控制器对象。
         *
         * STEER_PWM_CHIP：
         *     表示使用哪个 PWM 芯片或 PWM 控制器编号。
         *
         * STEER_PWM_INDEX：
         *     表示该 PWM 芯片下的具体 PWM 通道编号。
         *
         * 这里使用 new 创建对象，因此后续需要在析构函数中 delete，
         * 否则会产生内存泄漏。
         */
        PwmController *pwmTemp = new PwmController(STEER_PWM_CHIP, STEER_PWM_INDEX);

        /*
         * 初始化 PWM 控制器。
         *
         * initialize() 返回 true 表示初始化成功，
         * 返回 false 表示初始化失败。
         */
        if (pwmTemp->initialize())
        {
            /*
             * PWM 初始化成功时，在控制台输出提示信息。
             */
            log.logOutputConsole("Steer Initialize Success", LOG_INFO);
        }
        else
        {
            /*
             * PWM 初始化失败时，在控制台输出错误信息。
             */
            log.logOutputConsole("Steer Initialize Failed", LOG_ERROR);
        }

        /*
         * 将 PwmController 指针保存到类成员 pwmCtrl 中。
         *
         * 这里 pwmCtrl 的类型应该是 void*，
         * 因此需要强制转换成 void* 保存。
         *
         * 后续使用时，再从 void* 转回 PwmController*。
         */
        this->pwmCtrl = (void *)pwmTemp;

        /*
         * 设置 PWM 周期。
         *
         * this->period 表示 PWM 的完整周期。
         *
         * 对普通舵机来说，常见 PWM 周期为 20ms，
         * 如果单位是纳秒，则 20ms = 20000000ns。
         */
        pwmTemp->setPeriod(this->period);

        /*
         * 下面这行是直接设置 50% 占空比的旧写法，
         * 当前没有使用。
         *
         * 对舵机来说，通常不直接使用 50% 占空比，
         * 而是使用 0.5ms ~ 2.5ms 之间的高电平脉宽控制角度。
         */
        // pwmTemp->setDutyCycle(this->period / 2);

        /*
         * 设置舵机初始角度为 90 度。
         *
         * 90 度通常表示舵机中位角。
         * setAngle(90) 内部会将角度转换为 PWM 高电平时间。
         */
        this->setAngle(90);

        /*
         * 使能 PWM 输出。
         *
         * 调用 enable() 后，PWM 信号才真正开始输出到舵机。
         */
        pwmTemp->enable();
    }

    /**
     * @brief 设置舵机目标角度
     *
     * @details
     * 该函数会将输入的舵机角度转换为 PWM 占空时间，
     * 然后通过 PWM 控制器输出给舵机。
     *
     * 计算过程如下：
     *
     * 1. 先将输入角度加上机械偏差补偿 steer_offset；
     * 2. 再根据角度计算对应的 PWM 高电平时间 duty；
     * 3. 最后调用 setDutyCycle(duty) 输出 PWM 信号。
     *
     * 当前代码没有对 angle 进行限幅，
     * 因此如果传入角度过大或过小，
     * duty 可能会超出舵机安全范围。
     *
     * @param angle 舵机目标角度。
     */
    void Steer::setAngle(float angle)
    {
        /*
         * 加上舵机角度偏移量。
         *
         * steer_offset 用于修正机械安装误差。
         *
         * 例如：
         *     如果舵机实际安装时中位偏左，
         *     可以通过设置 steer_offset 来补偿。
         *
         * 假设：
         *     angle = 90
         *     steer_offset = 5
         *
         * 那么实际参与 PWM 计算的角度为：
         *     angle = 95
         */
        angle += this->steer_offset;

        /*
         * 根据角度计算 PWM 高电平时间 duty。
         *
         * 原始公式：
         *
         *     duty = (((angle - 90.0f) / 90.0f) * 5.0f)
         *            * (float)(this->period / 100.0f)
         *            + 1520000.0f;
         *
         * 公式含义：
         *
         *     1. angle - 90.0f
         *        表示当前角度相对于中位角 90° 的偏移量。
         *
         *        angle = 90 时：
         *            angle - 90 = 0
         *
         *        angle = 0 时：
         *            angle - 90 = -90
         *
         *        angle = 180 时：
         *            angle - 90 = 90
         *
         *     2. (angle - 90.0f) / 90.0f
         *        将角度偏移量归一化到 -1 ~ 1 范围。
         *
         *        angle = 0   -> -1
         *        angle = 90  ->  0
         *        angle = 180 ->  1
         *
         *     3. 再乘以 5.0f
         *        表示相对于 PWM 周期的 ±5% 变化。
         *
         *     4. this->period / 100.0f
         *        表示 PWM 周期的 1%。
         *
         *     5. 乘起来之后：
         *        ((angle - 90) / 90) * 5% * period
         *
         *        表示在中位脉宽基础上的偏移量。
         *
         *     6. 最后加上 1520000.0f
         *        1520000 通常表示 1.52ms。
         *
         *        也就是说：
         *            90° 对应 1.52ms 高电平。
         *
         * 如果 period = 20000000ns，也就是 20ms：
         *
         *     angle = 0:
         *         duty = 1520000 - 1000000 = 520000ns
         *         即 0.52ms
         *
         *     angle = 90:
         *         duty = 1520000ns
         *         即 1.52ms
         *
         *     angle = 180:
         *         duty = 1520000 + 1000000 = 2520000ns
         *         即 2.52ms
         *
         * 所以该公式大致将：
         *
         *     0°   映射为 0.52ms
         *     90°  映射为 1.52ms
         *     180° 映射为 2.52ms
         *
         * 这符合常见舵机通过 PWM 脉宽控制角度的方式。
         */
         float duty = (((angle - 90.0f) / 90.0f) * 5.0f) * (float)(this->period / 100.0f) + 1520000.0f;

        /*
         * 下面这一行是另一种舵机占空比计算公式，
         * 当前被注释掉，没有参与程序运行。
         *
         * 由于用户要求不修改逻辑，因此保持原样。
         */
        //float duty = (0.5f + (angle / 90.0f)) * (float)(this->period / 20);

        /*
         * 将保存的 void* 类型 pwmCtrl 转换回 PwmController*。
         *
         * 因为 pwmCtrl 在 init() 中保存的是：
         *
         *     this->pwmCtrl = (void *)pwmTemp;
         *
         * 所以这里使用时需要转换回来。
         */
        PwmController *pwmTemp = (PwmController *)this->pwmCtrl;

        /*
         * 设置 PWM 占空时间。
         *
         * duty 表示一个 PWM 周期内的高电平持续时间。
         *
         * 对舵机来说，高电平时间越大，
         * 舵机通常会转向一个方向；
         * 高电平时间越小，
         * 舵机通常会转向另一个方向。
         */
        pwmTemp->setDutyCycle(duty);
    }

    /**
     * @brief 设置舵机机械安装偏差补偿角度
     *
     * @details
     * 该函数用于设置舵机的角度偏移量 steer_offset。
     *
     * 在实际机械安装中，舵机的物理中位可能并不完全等于程序中的 90°。
     * 例如程序设置 90° 时，小车轮子可能略微偏左或偏右。
     *
     * 这时可以通过 steer_offset 进行补偿。
     *
     * 例如：
     *
     *     setSteerOffset(5);
     *     setAngle(90);
     *
     * 实际参与计算的角度就是：
     *
     *     90 + 5 = 95
     *
     * @param offsetAngle 舵机机械安装偏差补偿角度。
     */
    void Steer::setSteerOffset(float offsetAngle)
    {
        /*
         * 保存舵机角度偏移量。
         *
         * 后续每次调用 setAngle(angle) 时，
         * 都会先执行：
         *
         *     angle += this->steer_offset;
         *
         * 因此该偏移量会持续影响舵机角度计算。
         */
        this->steer_offset = offsetAngle;
    }

    /**
     * @brief 析构 Steer 对象
     *
     * @details
     * 当 Steer 对象生命周期结束时，释放其占用的资源。
     *
     * 主要操作包括：
     *
     * 1. 判断 pwmCtrl 是否为空；
     * 2. 如果不为空，将 void* 转换回 PwmController*；
     * 3. 禁用 PWM 输出；
     * 4. 删除通过 new 创建的 PwmController 对象；
     * 5. 避免内存泄漏和 PWM 资源持续占用。
     */
    Steer::~Steer()
    {
        /*
         * 判断 pwmCtrl 是否已经指向有效的 PWM 控制器对象。
         *
         * 如果 pwmCtrl 为 nullptr，
         * 说明可能没有初始化成功或没有创建 PWM 对象，
         * 此时不能进行 disable 或 delete 操作。
         */
        if (this->pwmCtrl != nullptr)
        {
            /*
             * 将 void* 类型的 pwmCtrl 转换回 PwmController*。
             *
             * 这样才能调用 PwmController 类中的成员函数。
             */
            PwmController *pwmTemp = (PwmController *)this->pwmCtrl;

            /*
             * 禁用 PWM 输出。
             *
             * 这样可以停止舵机控制信号，
             * 避免对象销毁后 PWM 仍然保持输出。
             */
            pwmTemp->disable();

            /*
             * 释放 init() 中通过 new 创建的 PwmController 对象。
             *
             * init() 中：
             *
             *     PwmController *pwmTemp = new PwmController(...);
             *
             * 因此这里必须 delete。
             */
            delete (PwmController *)this->pwmCtrl;
        }
    };
}