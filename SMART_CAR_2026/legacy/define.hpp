#ifndef DEFINE_HPP
#define DEFINE_HPP

/*
 * 项目级宏定义配置文件。
 *
 * 这里集中放置：
 * - 日志等级默认值；
 * - PID 默认参数；
 * - 摄像头默认参数；
 * - 寄存器映射常量；
 * - 电机、编码器、舵机等硬件引脚与通道配置。
 *
 * 这些宏主要用于嵌入式控制与图像处理模块之间共享硬件参数。
 */
/*==================LOG_LEVEL==================*/
#define LOG_LEVEL_GLOBAL Other::LOG_INFO

/*=================PID_DEFAULT=================*/
#define PID_DEFAULT_KP 64
#define PID_DEFAULT_KI 32
#define PID_DEFAULT_KD 48//qidongsudu*2
#define PID_LIMLIT_PWM 12000

/*====================CAMERA===================*/
#define CAMERA_OPENCV_WIDTH 80
#define CAMERA_OPENCV_HEIGHT 60
// #define CAMERA_OPENCV_FPS 30
#define CAMERA_PATH_OR_INDEX "/dev/video0"

/*==================Register===================*/
#define ENCODER_MAP_BYTES 0x10000

#define REG_READ(addr) (*(volatile uint32_t *)(addr))
#define REG_WRITE(addr, val) (*(volatile uint32_t *)(addr) = (val))

/*=================Encoder=====================*/
#define Encoder_PPR 1024 // 编码器线数

#define PWM_BASE_ADDR 0x1611B000
#define PWM_OFFSET 0x10
#define LOW_BUFFER_OFFSET 0x4
#define FULL_BUFFER_OFFSET 0x8
#define CONTROL_REG_OFFSET 0xC

#define CNTR_ENABLE_BIT (1 << 0)       // 计数器使能
#define PULSE_OUT_ENABLE_BIT (1 << 3)  // 脉冲输出使能（低有效）
#define SINGLE_PULSE_BIT (1 << 4)      // 单脉冲控制位
#define INT_ENABLE_BIT (1 << 5)        // 中断使能
#define INT_STATUS_BIT (1 << 6)        // 中断状态
#define COUNTER_RESET_BIT (1 << 7)     // 计数器重置
#define MEASURE_PULSE_BIT (1 << 8)     // 测量脉冲使能
#define INVERT_OUTPUT_BIT (1 << 9)     // 输出翻转使能
#define DEAD_ZONE_ENABLE_BIT (1 << 10) // 防死区使能

/*===================PwmAtim===================*/
#define GPIO_MUX_BASE_ADDR 0x16000490

#define ATIM_BASE_ADDR 0x16118000
#define ATIM_CR1_OFFSET 0x00
#define ATIM_CR2_OFFSET 0x04
#define ATIM_SMCR_OFFSET 0x08
#define ATIM_DIER_OFFSET 0x0C
#define ATIM_SR_OFFSET 0x10
#define ATIM_EGR_OFFSET 0x14
#define ATIM_CCMR1_OFFSET 0x18
#define ATIM_CCMR2_OFFSET 0x1C
#define ATIM_CCER_OFFSET 0x20
#define ATIM_CNT_OFFSET 0x24
#define ATIM_PSC_OFFSET 0x28
#define ATIM_ARR_OFFSET 0x2C
#define ATIM_RCR_OFFSET 0x30
#define ATIM_CCR1_OFFSET 0x34
#define ATIM_CCR2_OFFSET 0x38
#define ATIM_CCR3_OFFSET 0x3C
#define ATIM_CCR4_OFFSET 0x40
#define ATIM_BDTR_OFFSET 0x44
#define ATIM_INSTA_OFFSET 0x50

/*===================PwmGtim===================*/

#define GPIO_MUX_BASE_ADDR 0x16000490

#define GTIM_BASE_ADDR 0x16119000
#define GTIM_CR1_OFFSET 0x00
#define GTIM_CR2_OFFSET 0x04
#define GTIM_SMCR_OFFSET 0x08
#define GTIM_DIER_OFFSET 0x0C
#define GTIM_SR_OFFSET 0x10
#define GTIM_EGR_OFFSET 0x14
#define GTIM_CCMR1_OFFSET 0x18
#define GTIM_CCMR2_OFFSET 0x1C
#define GTIM_CCER_OFFSET 0x20
#define GTIM_CNT_OFFSET 0x24
#define GTIM_PSC_OFFSET 0x28
#define GTIM_ARR_OFFSET 0x2C
#define GTIM_CCR1_OFFSET 0x34
#define GTIM_CCR2_OFFSET 0x38
#define GTIM_CCR3_OFFSET 0x3C
#define GTIM_CCR4_OFFSET 0x40
#define GTIM_INSTA_OFFSET 0x50

/*=================Motor=====================*/
/*
 * 按《引脚对照表.xlsx》设置：
 * 电机1：PH=GPIO12，EN=GPIO89/tim2_ch3。
 * 电机2：PH=GPIO13，EN=GPIO88/tim2_ch2。
 * 在当前 Linux sysfs PWM 映射中：pwmchip8/pwm2 对应 tim2_ch3，pwmchip8/pwm1 对应 tim2_ch2。
 */
#define MOTOR_LEFT_PWM_CHIP 8
#define MOTOR_LEFT_PWM_INDEX 2   // 电机1 EN：GPIO89 / tim2_ch3
#define MOTOR_LEFT_DIR_NUMBER 12 // 电机1 PH：GPIO12
#define MOTOR_RIGHT_PWM_CHIP 8
#define MOTOR_RIGHT_PWM_INDEX 1   // 电机2 EN：GPIO88 / tim2_ch2
#define MOTOR_RIGHT_DIR_NUMBER 13 // 电机2 PH：GPIO13
#define MOTOR_PWM_PERIOD 50000
#define MOTOR_MAX_PWM_VALUE 50000
#define MOTOR_MIN_PWM_VALUE 100
#define MOTOR_ENABLE_PIN 73      // 电机总使能 nSLEEP：GPIO73

/*================Encoder===================*/
/*
 * 按《引脚对照表.xlsx》设置：
 * 编码器1：LSB=GPIO64/pwm[0]，DIR=GPIO75。
 * 编码器2：LSB=GPIO67/pwm[3]，DIR=GPIO72。
 */
#define ENCODER_LEFT_PIN 0       // 编码器1 LSB：GPIO64 / pwm[0]
#define ENCODER_LEFT_DIR 75      // 编码器1 DIR：GPIO75
#define ENCODER_RIGHT_PIN 3      // 编码器2 LSB：GPIO67 / pwm[3]
#define ENCODER_RIGHT_DIR 72     // 编码器2 DIR：GPIO72

/*==================Steer===================*/
#define STEER_PWM_CHIP 1
#define STEER_PWM_INDEX 0

#endif
