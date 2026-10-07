#ifndef __PID_H
#define __PID_H

/**
 * @file PID.h
 * @brief 轻量级多模式 PID 控制器公共接口。
 *
 * @details
 * 本模块把常见 PID 控制方式统一封装为同一套接口，支持：
 * - 增量式 PID：输出为本次控制增量，可选择累加输出或单次输出；
 * - 位置式 PID：输出为与目标误差对应的绝对控制量；
 * - 积分分离 PID：误差较大时暂停积分项，减小积分饱和风险；
 * - 变积分 PID：根据误差大小动态调整积分权重，使大误差响应快、小误差稳态精度高。
 *
 * 模块内部通过 `st_PID_Temp_Node` 为每一种计算类型保存一组独立的 Kp/Ki/Kd、运行数据和配置数据。
 * 调用方通常只需要定义一个 `st_PID_Attr` 对象，然后按以下顺序使用：
 * 1. 调用 `PID_Initialize()` 初始化属性对象；
 * 2. 调用对应模式的 `PID_xxx_Set_Kpid()` 设置 Kp、Ki、Kd；
 * 3. 按需要开启输出限幅、死区、单位换算、积分限幅、回调等功能；
 * 4. 调用 `PID_xxx_CalcResult_ByDispersedValue()` 或 `PID_xxx_CalcResult_ByNowTureValue()` 获取输出。
 *
 * 输入值类型说明：
 * - `PID_DISPERSED`：传入的 value 已经是误差或离散量，模块会直接作为 bias 使用；
 * - `PID_NOW_VALUE`：传入的 value 是当前真实测量值，模块会用 aimValue - value 得到误差。
 *
 * 注意事项：
 * - 本文件中的 `accuracy` 类型由 `ACCURACY_LEVEL` 决定，默认使用 float，可配置为 int/float/double；
 * - 模块自带固定大小内存池，大小由 `PID_MEMORY_HEAP_SIZE` 决定，适合嵌入式环境避免频繁 malloc；
 * - 头文件保留了原项目中的 `NowTure` 拼写，以兼容已有调用代码。
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

// 此版本是有注释版本，注释由AI生成的，仅供参考。

// PID 模块专用固定内存池大小，单位为 Byte。数值越大，可同时保存的控制节点、配置和缓存越多。
#define PID_MEMORY_HEAP_SIZE 4096

// PID 计算精度等级：1=int，2=float，3=double。小车控制建议使用 float 或 double 保证调参精度。
#define ACCURACY_LEVEL 3

// PID 控制器 ID 类型等级：1=unsigned char，2=unsigned short，3=unsigned int。用于回调时区分不同控制器。
#define PID_ID_LEVEL 1

// #ifndef bool
// #define bool char
// #define true 1
// #define false 0
// #endif // !bool

// Standard integer/size types must not be macros: macros corrupt OpenCV/std headers.

#ifndef PID_MEMORY_HEAP_SIZE
#define PID_MEMORY_HEAP_SIZE 1024
#endif // !PID_MEMORY_HEAP_SIZE

#ifdef ACCURACY_LEVEL
#if ACCURACY_LEVEL == 1
#define accuracy int
#elif ACCURACY_LEVEL == 2
#define accuracy float
#elif ACCURACY_LEVEL == 3
#define accuracy double
#endif
#else
#define accuracy float
#endif // !ACCURACY_LEVEL

#ifdef PID_ID_LEVEL
#if PID_ID_LEVEL == 1
#define pid_id_t unsigned char
#elif PID_ID_LEVEL == 2
#define pid_id_t unsigned short
#elif PID_ID_LEVEL == 3
#define pid_id_t unsigned int
#endif
#else
#define pid_id_t unsigned char
#endif

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * @brief PID 回调触发阶段。
     *
     * 回调函数会收到该枚举值，用来判断当前是在计算前还是计算后触发。
     * 前置回调适合记录输入、修改外部状态；后置回调适合记录输出或做联动控制。
     */
    typedef enum eu_PID_Callback
    {
        PID_CALLBACK_BEFORE = 0, // PID 公式计算前触发。
        PID_CALLBACK_AFTER       // PID 公式计算后触发。
    } eu_PID_Callback;

    /**
     * @brief PID 接口返回状态。
     *
     * 用于区分参数设置、节点创建、数据提交、计算过程是否成功。
     * STOP 类状态主要用于死区等配置在计算阶段提前终止输出。
     */
    typedef enum eu_PID_Result
    {
        PID_RESULT_SUCCESS = 0,      // 操作成功，内部状态已按预期更新。
        PID_RESULT_FAIL,             // 操作失败，通常是参数非法、节点不存在或内存池不足。
        PID_RESULT_STOP,             // 本次计算被配置逻辑中止，输出通常清零。
        PID_RESULT_STOP_BUT_KEEP_OUT // 本次计算停止，但保留上一次输出，常用于死区保持输出。
    } eu_PID_Result;

    /**
     * @brief PID 计算模式。
     *
     * 每种模式在 `st_PID_Attr` 中会对应一个独立临时节点，互不覆盖参数和历史误差。
     * 可通过不同宏接口快速选择模式，例如 `PID_Incremental_*`、`PID_Position_*` 等。
     */
    typedef enum eu_PID_Calc_Type
    {
        PID_NULL = 0,          // 空类型，用于表示不使用自动计算或未指定模式。
        PID_INCREMENTAL,       // 增量式 PID：输出本次增量，适合速度、电机 PWM 微调。
        PID_POSITION,          // 位置式 PID：输出绝对控制量，适合位置、角度、舵机控制。
        PID_INTEGRAL_SEPARATE, // 积分分离 PID：误差超过阈值时削弱/关闭积分，防止积分饱和。
        PID_VARIABLE_INTEGRAL  // 变积分 PID：根据误差区间动态改变积分作用强度。
    } eu_PID_Calc_Type;

    /**
     * @brief 单个 PID 模式的运行节点。
     *
     * 一个 `st_PID_Attr` 可以挂多个该节点，例如同时保存增量式和位置式 PID 的参数。
     * `config` 指向该模式的功能配置，`data` 指向该模式的运行数据，具体结构在 `PID.c` 内部定义。
     */
    struct st_PID_Temp_Node
    {
        enum eu_PID_Calc_Type pid_calc_type; // 当前节点对应的 PID 计算模式。
        float kp;                            // 比例系数 Kp，决定当前误差对输出的影响。
        float ki;                            // 积分系数 Ki，决定历史累积误差对输出的影响。
        float kd;                            // 微分系数 Kd，决定误差变化趋势对输出的影响。

        void *config; // 模式专用配置区，例如限幅、死区、单位换算、回调开关等。
        void *data;   // 模式专用运行数据区，例如当前误差、上次误差、积分项、输出结果等。

        struct st_PID_Temp_Node *next; // 下一个 PID 临时节点，用于组成单向链表。
    };

    /**
     * @brief PID 临时节点链表。
     *
     * 保存当前控制器已经创建的所有 PID 模式节点，查找时按 `pid_calc_type` 匹配。
     */
    struct st_PID_Temp
    {
        struct st_PID_Temp_Node *head; // 链表头指针
        struct st_PID_Temp_Node *end;  // 链表尾指针
    };

    // 定义PID缓存结构体
    struct st_PID_Cache
    {
        uint16_t size;                  // 缓存大小
        uint16_t now_size;              // 当前缓存大小
        struct st_PID_Cache_Node *head; // 缓存节点头指针
        struct st_PID_Cache_Node *end;  // 缓存节点尾指针
    };

    // 定义PID缓存节点结构体
    struct st_PID_Cache_Node
    {
        eu_PID_Calc_Type type;          // PID计算类型
        struct st_PID_Cache_Node *last; // 上一个缓存节点指针
        struct st_PID_Cache_Node *next; // 下一个缓存节点指针
        struct st_PID_Temp_Node *node;  // 对应的PID临时节点指针
    };

    // 定义PID属性结构体
    typedef struct st_PID_Attr
    {
        uint8_t id;                  // PID属性ID
        struct st_PID_Temp pid_temp; // PID临时链表

        bool is_use_cache;          // 是否使用缓存
        struct st_PID_Cache *cache; // 缓存指针
    } st_PID_Attr;

    // 定义PID提交类型的枚举
    enum eu_PID_Submit_Type
    {
        PID_DISPERSED = 0, // 分散提交
        PID_NOW_VALUE      // 当前值提交
    };

    /**
     * @brief 初始化PID控制器
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @return eu_PID_Result 返回PID控制器的初始化结果
     */
    eu_PID_Result PID_Init(st_PID_Attr *pid_st_PID_Attr_);

    /**
     * @brief 创建PID控制器的缓存
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param size_uint16_t 缓存大小
     * @return eu_PID_Result 返回创建缓存的结果
     */
    eu_PID_Result PID_Create_Cache(st_PID_Attr *pid_st_PID_Attr_, uint16_t size_uint16_t);

    /**
     * @brief 重置PID控制器的缓存大小
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param size_uint16_t 新的缓存大小
     * @return eu_PID_Result 返回重置缓存大小的结果
     */
    eu_PID_Result PID_Reset_CacheSize(st_PID_Attr *pid_st_PID_Attr_, uint16_t size_uint16_t);

    /**
     * @brief 释放PID控制器的缓存
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @return eu_PID_Result 返回释放缓存的结果
     */
    eu_PID_Result PID_Free_Cache(st_PID_Attr *pid_st_PID_Attr_);

    /**
     * @brief 设置PID控制器的比例、积分、微分参数
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param kp_float 比例参数
     * @param ki_float 积分参数
     * @param kd_float 微分参数
     * @return eu_PID_Result 返回设置参数的结果
     */
    eu_PID_Result PID_Set_KpidParam(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, float kp_float, float ki_float, float kd_float);

    /**
     * @brief 清除PID控制器的临时数据
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @return eu_PID_Result 返回清除临时数据的结果
     */
    eu_PID_Result PID_Clear_TempDate(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type);

    /**
     * @brief 设置PID控制器的单次输出模式
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param isTrue_bool 是否启用单次输出模式
     * @return eu_PID_Result 返回设置单次输出模式的结果
     */
    eu_PID_Result PID_Set_SingleOut(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isTrue_bool);

    /**
     * @brief 设置PID控制器的积分限幅偏置值
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param isUseLimitAmplitudeIntegralBias_bool 是否启用积分限幅偏置
     * @param isUseAimValueToLimit_bool 是否使用目标值作为限幅值
     * @param limitAmplitudeIntegralBiasValue_accuracy 积分限幅偏置值
     * @param paramIndex_uint8_t 参数索引
     * @return eu_PID_Result 返回设置积分限幅偏置值的结果
     */
    eu_PID_Result PID_Set_LimitAmplitudeIntegralBiasValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseLimitAmplitudeIntegralBias_bool, bool isUseAimValueToLimit_bool, float limitAmplitudeIntegralBiasValue_accuracy, uint8_t paramIndex_uint8_t);

    /**
     * @brief 设置PID控制器的输出限幅
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param isUseLimitOut_bool 是否启用输出限幅
     * @param limitValueAutoCalcType_eu_PID_Calc_Type 限幅值自动计算类型
     * @param limitValue_float 限幅值
     * @param paramIndex_uint8_t 参数索引
     * @return eu_PID_Result 返回设置输出限幅的结果
     */
    eu_PID_Result PID_Set_OutLimit(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseLimitOut_bool, eu_PID_Calc_Type limitValueAutoCalcType_eu_PID_Calc_Type, float limitValue_float, uint8_t paramIndex_uint8_t);

    /**
     * @brief 设置PID控制器的死区
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param isUseDeadArea_bool 是否启用死区
     * @param isKeepOutInDead_bool 是否在死区内保持输出
     * @param deadAreaUp_float 死区上限
     * @param deadAreaDown_float 死区下限
     * @param paramIndex_uint8_t 参数索引
     * @return eu_PID_Result 返回设置死区的结果
     */
    eu_PID_Result PID_Set_DeadArea(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseDeadArea_bool, bool isKeepOutInDead_bool, float deadAreaUp_float, float deadAreaDown_float, uint8_t paramIndex_uint8_t);

    /**
     * @brief 设置PID控制器的单位
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param isUseUnitValue_bool 是否启用单位值
     * @param unitValueSize_float 单位值大小
     * @param unitValueOffset_float 单位值偏移
     * @param paramIndex_uint8_t 参数索引
     * @return eu_PID_Result 返回设置单位的结果
     */
    eu_PID_Result PID_Set_Unit(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseUnitValue_bool, float unitValueSize_float, float unitValueOffset_float, uint8_t paramIndex_uint8_t);

    /**
     * @brief 设置PID控制器的回调函数
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param isUseCallback_bool 是否启用回调函数
     * @param id_pid_id_t PID控制器的ID
     * @param before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback 回调函数，执行计算前调用
     * @param after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback 回调函数，执行计算后调用
     * @param paramIndex_uint8_t 参数索引
     * @return eu_PID_Result 返回设置回调函数的结果
     */
    eu_PID_Result PID_Set_Callback(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseCallback_bool, pid_id_t id_pid_id_t, void (*before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback)(pid_id_t id_pid_id_t, accuracy paramOne_accuracy, accuracy paramTwo_accuracy, eu_PID_Callback callbackType_eu_PID_Callback), void (*after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback)(pid_id_t id_pid_id_t, accuracy paramOne_accuracy, accuracy paramTwo_accuracy, eu_PID_Callback callbackType_eu_PID_Callback), uint8_t paramIndex_uint8_t);

    /**
     * @brief 设置PID控制器的阈值
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param thresholdValue_accuracy 阈值
     * @param thresholdValueTwo_accuracy 第二阈值
     * @return eu_PID_Result 返回设置阈值的结果
     */
    eu_PID_Result PID_Set_ThresholdValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy thresholdValue_accuracy, accuracy thresholdValueTwo_accuracy);

    /**
     * @brief 提交PID控制器的分散值
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param dispersedValue_accuracy 分散值
     * @param tmpNode_st_PID_Temp_Node_ 指向临时节点的指针
     * @return eu_PID_Result 返回提交分散值的结果
     */
    eu_PID_Result PID_Submit_DispersedValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy dispersedValue_accuracy, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);

    /**
     * @brief 提交PID控制器的当前真实值
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param nowTrueValue_accuracy 当前真实值
     * @param tmpNode_st_PID_Temp_Node_ 指向临时节点的指针
     * @return eu_PID_Result 返回提交当前真实值的结果
     */
    eu_PID_Result PID_Submit_NowTureValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy nowTrueValue_accuracy, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);

    /**
     * @brief 设置PID控制器的目标值
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param aimValue_accuracy 目标值
     * @param tmpNode_st_PID_Temp_Node_ 指向临时节点的指针
     * @return eu_PID_Result 返回设置目标值的结果
     */
    eu_PID_Result PID_Set_AimValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy aimValue_accuracy, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);

    /**
     * @brief 计算PID控制器的结果
     *
     * @param pid_st_PID_Attr_ 指向PID属性结构体的指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param tmpNode_st_PID_Temp_Node_ 指向临时节点的指针
     * @return accuracy 返回PID控制器的计算结果
     */
    accuracy PID_Calc_Result(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);

    /**
     * @brief 计算PID控制器的限幅值
     *
     * @param value_accuracy 输入值
     * @param amplitude 幅度
     * @return accuracy 返回限幅后的值
     */
    accuracy PID_Calc_LimitAmplitude(accuracy value, accuracy amplitude);

    /**
     * @brief 获取当前PID控制的准确性
     *
     * 该函数根据PID控制器的当前属性和计算类型，计算并返回当前的准确性指标。
     *
     * @param pid_st_PID_Attr_ PID控制器的属性结构体指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @return accuracy 当前的准确性指标
     */
    accuracy PID_Get_NowValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type);

    /**
     * @brief PID计算结果接口函数
     *
     * 该函数根据提供的PID控制器属性、计算类型、当前值、目标值等参数，计算并返回PID控制的准确性。
     * 它用于在不同的计算模式下，根据当前的输入值和目标值，计算出PID控制器的输出。
     *
     * @param pid_st_PID_Attr_ PID控制器的属性结构体指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param value_float 当前输入值
     * @param aimValue_float 目标值
     * @param valueType_eu_PID_Submit_Type 输入值类型
     * @param tmpNode_st_PID_Temp_Node_ 临时节点结构体指针，用于PID计算过程中的临时数据存储
     * @return accuracy PID控制的准确性
     */
    accuracy PID_CalcResult_Interface(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, float value_float, float aimValue_float, enum eu_PID_Submit_Type valueType_eu_PID_Submit_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);

    /**
     * @brief 设置并提交目标值和当前值的PID控制接口函数
     *
     * 该函数用于设置PID控制器的目标值和当前值，并根据提供的参数进行PID计算，返回计算结果。
     * 它是一个全面的PID控制接口，允许通过一个函数调用来完成目标值和当前值的设置以及PID计算。
     *
     * @param pid_st_PID_Attr_ PID控制器的属性结构体指针
     * @param type_eu_PID_Calc_Type PID计算类型
     * @param value_float 当前输入值
     * @param aimValue_float 目标值
     * @param valueType_eu_PID_Submit_Type 输入值类型
     * @param tmpNode_st_PID_Temp_Node_ 临时节点结构体指针，用于PID计算过程中的临时数据存储
     * @return eu_PID_Result PID计算结果
     */
    eu_PID_Result PID_SetAndSubmit_AimValueAndValue_Interface(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, float value_float, float aimValue_float, enum eu_PID_Submit_Type valueType_eu_PID_Submit_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);

/**
 * @brief 初始化PID控制器属性
 *
 * 该宏用于初始化PID控制器的属性。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Initialize(pidAttr) PID_Init(pidAttr)

/**
 * @brief 开启使用缓存池
 *
 * 该宏用于开启PID控制器的缓存池功能。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Open_CachePool(pidAttr) (pidAttr->is_use_cache = true)

/**
 * @brief 创建缓存池并指定大小
 *
 * 该宏用于创建PID控制器的缓存池，并指定缓存池的大小。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param size 缓存池的大小
 */
#define PID_Create_CachePool(pidAttr, size) PID_Create_Cache(pidAttr, size)

/**
 * @brief 重置缓存池大小
 *
 * 该宏用于重置PID控制器的缓存池大小。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param size 新的缓存池大小
 */
#define PID_Reset_CachePoolSize(pidAttr, size) PID_Reset_CacheSize(pidAttr, size)

/**
 * @brief 关闭使用缓存池
 *
 * 该宏用于关闭PID控制器的缓存池功能。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Close_CachePool(pidAttr) (pidAttr->is_use_cache = false)

/**
 * @brief 释放缓存池
 *
 * 该宏用于释放PID控制器的缓存池。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Free_CachePool(pidAttr) PID_Free_Cache(pidAttr)

/**
 * @brief 设置增量式PID控制器的参数
 *
 * 该宏定义用于设置增量式PID控制器的比例、积分、微分参数
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
#define PID_Incremental_Set_Kpid(pidAttr, kp, ki, kd) PID_Set_KpidParam(pidAttr, PID_INCREMENTAL, kp, ki, kd)

/**
 * @brief 清除增量式PID控制器的临时数据
 *
 * 该宏定义用于清除增量式PID控制器在运行过程中产生的临时数据
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Incremental_Clear_TempData(pidAttr) PID_Clear_TempDate(pidAttr, PID_INCREMENTAL)

/**
 * @brief 通过离散值计算增量式PID控制器的输出
 *
 * 该宏定义用于通过离散值输入来计算增量式PID控制器的输出结果
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param value 当前的离散输入值
 * @param aimValue 目标值
 */
#define PID_Incremental_CalcResult_ByDispersedValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_INCREMENTAL, value, aimValue, PID_DISPERSED, NULL)

/**
 * @brief 通过当前真实值计算增量式PID控制器的输出
 *
 * 该宏定义用于通过当前的真实值输入来计算增量式PID控制器的输出结果
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param value 当前的真实输入值
 * @param aimValue 目标值
 */
#define PID_Incremental_CalcResult_ByNowTureValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_INCREMENTAL, value, aimValue, PID_NOW_VALUE, NULL)

/**
 * @brief 设置PID控制器为增量模式，并开启输出限制
 *
 * 该宏定义用于设置PID控制器的工作模式为增量模式，并且开启输出限制功能。
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Incremental_Open_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_INCREMENTAL, true, PID_NULL, 0, 1)

/**
 * @brief 设置PID控制器为增量模式，并设置自动输出限制
 *
 * 该宏定义用于设置PID控制器的工作模式为增量模式，并且根据指定的计算类型进行自动输出限制。
 * @param pidAttr PID控制器的属性结构体指针
 * @param pidCalcType PID控制器的计算类型
 */
#define PID_Incremental_Set_AutoOutLimit(pidAttr, pidCalcType) PID_Set_OutLimit(pidAttr, PID_INCREMENTAL, false, pidCalcType, 0, 2)

/**
 * @brief 设置PID控制器为增量模式，并设置具体的输出限制值
 *
 * 该宏定义用于设置PID控制器的工作模式为增量模式，并且指定输出限制的具体值。
 * @param pidAttr PID控制器的属性结构体指针
 * @param value 输出限制的具体值
 */
#define PID_Incremental_Set_OutLimitValue(pidAttr, value) PID_Set_OutLimit(pidAttr, PID_INCREMENTAL, false, PID_NULL, value, 3)

/**
 * @brief 设置PID控制器为增量模式，并清除自动输出限制
 *
 * 该宏定义用于设置PID控制器的工作模式为增量模式，并且清除自动输出限制设置。
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Incremental_Clear_AutoOutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_INCREMENTAL, false, PID_NULL, 0, 2)

/**
 * @brief 设置PID控制器为增量模式，并关闭输出限制
 *
 * 该宏定义用于设置PID控制器的工作模式为增量模式，并且关闭输出限制功能。
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Incremental_Close_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_INCREMENTAL, false, PID_NULL, 0, 1)

/**
 * @brief 开启增量式PID控制器的死区功能
 *
 * 该宏定义用于开启增量式PID控制器的死区功能。
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Incremental_Open_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_INCREMENTAL, true, false, 0, 0, 1)

/**
 * @brief 设置增量式PID控制器的上下死区值
 *
 * 该宏定义用于设置增量式PID控制器的上下死区值。
 * @param pidAttr PID控制器的属性结构体指针
 * @param up 上死区值
 * @param down 下死区值
 */
#define PID_Incremental_Set_DeadAreaValueUpDown(pidAttr, up, down) PID_Set_DeadArea(pidAttr, PID_INCREMENTAL, false, false, up, down, 5)

/**
 * @brief 关闭增量式PID控制器的死区功能
 *
 * 该宏定义用于关闭增量式PID控制器的死区功能。
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Incremental_Close_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_INCREMENTAL, false, false, 0, 0, 1)

/**
 * 开启增量型PID控制单元
 * @param pidAttr PID控制的属性指针
 */
#define PID_Incremental_Open_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_INCREMENTAL, true, 0, 0, 1)

/**
 * 设置增量型PID控制单元的大小
 * @param pidAttr PID控制的属性指针
 * @param size 单元大小
 */
#define PID_Incremental_Set_UnitSize(pidAttr, size) PID_Set_Unit(pidAttr, PID_INCREMENTAL, false, size, 0, 2)

/**
 * 设置增量型PID控制单元的偏移量
 * @param pidAttr PID控制的属性指针
 * @param value 偏移量值
 */
#define PID_Incremental_Set_UnitOffset(pidAttr, value) PID_Set_Unit(pidAttr, PID_INCREMENTAL, false, 0, value, 3)

/**
 * 关闭增量型PID控制单元
 * @param pidAttr PID控制的属性指针
 */
#define PID_Incremental_Close_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_INCREMENTAL, false, 0, 0, 1)

/**
 * 设置增量型PID控制单元的输出为单值（不累加输出）
 * @param pidAttr PID控制的属性指针
 */
#define PID_Incremental_Open_SingleOut(pidAttr) PID_Set_SingleOut(pidAttr, PID_INCREMENTAL, true)

/**
 * 设置增量型PID控制单元的输出为差分值（本次输出值累加上次输出值）
 * @param pidAttr PID控制的属性指针
 */
#define PID_Incremental_Close_SingleOut(pidAttr) PID_Set_SingleOut(pidAttr, PID_INCREMENTAL, false)

/**
 * @brief 设置增量式PID的开启回调函数
 *
 * 该宏定义用于设置增量式PID控制器的开启回调函数，当PID控制器被开启时，会调用此回调函数。
 *
 * @param pidAttr PID控制器的属性指针
 */
#define PID_Incremental_Open_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_INCREMENTAL, true, 0, NULL, NULL, 1);

/**
 * @brief 设置增量式PID的前置回调函数
 *
 * 该宏定义用于设置增量式PID控制器的前置回调函数，该函数会在PID控制器执行控制算法之前被调用。
 *
 * @param pidAttr PID控制器的属性指针
 * @param beforeCallbackFunction 前置回调函数指针
 */
#define PID_Incremental_Set_BeforeCallback(pidAttr, beforeCallbackFunction) PID_Set_Callback(pidAttr, PID_INCREMENTAL, false, 0, beforeCallbackFunction, NULL, 3);

/**
 * @brief 设置增量式PID的回调ID
 *
 * 该宏定义用于设置增量式PID控制器的回调ID，用于标识特定的回调事件。
 *
 * @param pidAttr PID控制器的属性指针
 * @param id 回调事件的ID
 */
#define PID_Incremental_Set_CallbackID(pidAttr, id) PID_Set_Callback(pidAttr, PID_INCREMENTAL, false, id, NULL, NULL, 2);

/**
 * @brief 设置增量式PID的后置回调函数
 *
 * 该宏定义用于设置增量式PID控制器的后置回调函数，该函数会在PID控制器执行控制算法之后被调用。
 *
 * @param pidAttr PID控制器的属性指针
 * @param afterCallbackFunction 后置回调函数指针
 */
#define PID_Incremental_Set_AfterCallback(pidAttr, afterCallbackFunction) PID_Set_Callback(pidAttr, PID_INCREMENTAL, false, 0, NULL, afterCallbackFunction, 4);

/**
 * @brief 设置增量式PID的关闭回调函数
 *
 * 该宏定义用于设置增量式PID控制器的关闭回调函数，当PID控制器被关闭时，会调用此回调函数。
 *
 * @param pidAttr PID控制器的属性指针
 */
#define PID_Incremental_Close_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_INCREMENTAL, false, 0, NULL, NULL, 1);

/**
 * @brief 定义用于设置位置PID控制器参数的宏
 *
 * 该宏用于设置PID控制器的比例(Kp)、积分(Ki)和微分(Kd)参数
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
#define PID_Position_Set_Kpid(pidAttr, kp, ki, kd) PID_Set_KpidParam(pidAttr, PID_POSITION, kp, ki, kd)

/**
 * @brief 定义用于清除位置PID控制器临时数据的宏
 *
 * 该宏用于清除PID控制器在运行过程中产生的临时数据，以便重新开始计算
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Clear_TempData(pidAttr) PID_Clear_TempDate(pidAttr, PID_POSITION)

/**
 * @brief 定义用于通过分散值计算位置PID控制器结果的宏
 *
 * 该宏用于根据当前的分散值和目标值计算PID控制器的输出结果
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param value 当前的分散值
 * @param aimValue 目标值
 */
#define PID_Position_CalcResult_ByDispersedValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_POSITION, value, aimValue, PID_DISPERSED, NULL)

/**
 * @brief 定义用于通过当前真实值计算位置PID控制器结果的宏
 *
 * 该宏用于根据当前的真实值和目标值计算PID控制器的输出结果
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param value 当前的真实值
 * @param aimValue 目标值
 */
#define PID_Position_CalcResult_ByNowTureValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_POSITION, value, aimValue, PID_NOW_VALUE, NULL)

/**
 * @brief 设置PID位置模式下的输出限制为开启
 *
 * 此宏定义用于设置PID控制器在位置模式下输出限制为开启状态它还指定了输出限制的类型、是否启用限制、以及限制的参数
 *
 * @param pidAttr PID控制器的属性结构体指针，包含PID控制器的配置和状态信息
 */
#define PID_Position_Open_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_POSITION, true, PID_NULL, 0, 1)

/**
 * @brief 设置PID位置模式下的输出限制为关闭
 *
 * 此宏定义用于设置PID控制器在位置模式下输出限制为关闭状态它同样指定了输出限制的类型、是否启用限制、以及限制的参数
 *
 * @param pidAttr PID控制器的属性结构体指针，包含PID控制器的配置和状态信息
 */
#define PID_Position_Close_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_POSITION, false, PID_NULL, 0, 1)

/**
 * @brief 设置PID位置模式下的自动输出限制
 *
 * 此宏定义用于设置PID控制器在位置模式下自动输出限制的状态和计算类型自动输出限制根据实际输出值自动调整限制范围
 *
 * @param pidAttr PID控制器的属性结构体指针，包含PID控制器的配置和状态信息
 * @param pidCalcType PID控制器的计算类型，用于自动输出限制的计算
 */
#define PID_Position_Set_AutoOutLimit(pidAttr, pidCalcType) PID_Set_OutLimit(pidAttr, PID_POSITION, false, pidCalcType, 0, 2)

/**
 * @brief 设置PID位置模式下的输出限制值
 *
 * 此宏定义用于设置PID控制器在位置模式下的具体输出限制值
 *
 * @param pidAttr PID控制器的属性结构体指针，包含PID控制器的配置和状态信息
 * @param value 输出限制的具体数值
 */
#define PID_Position_Set_OutLimitValue(pidAttr, value) PID_Set_OutLimit(pidAttr, PID_POSITION, false, PID_NULL, value, 3)

/**
 * @brief 清除PID位置模式下的自动输出限制
 *
 * 此宏定义用于清除PID控制器在位置模式下的自动输出限制设置，恢复到默认状态
 *
 * @param pidAttr PID控制器的属性结构体指针，包含PID控制器的配置和状态信息
 */
#define PID_Position_Clear_AutoOutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_POSITION, false, PID_NULL, 0, 2)

/**
 * @brief 打开位置控制的死区功能
 *
 * 死区功能用于忽略小的输入信号，以防止控制系统对微小的干扰做出反应。
 * 本宏定义用于设置位置控制模式下的死区参数。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Open_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_POSITION, true, false, 0, 0, 1)

/**
 * @brief 关闭位置控制的死区功能
 *
 * 死区功能用于忽略小的输入信号，以防止控制系统对微小的干扰做出反应。
 * 本宏定义用于设置位置控制模式下的死区参数。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Close_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_POSITION, false, false, 0, 0, 1)

/**
 * @brief 打开位置控制的死区功能，并保持死区外的信号不变
 *
 * 死区功能用于忽略小的输入信号，以防止控制系统对微小的干扰做出反应。
 * 本宏定义用于设置位置控制模式下的死区参数，并确保死区外的信号不受影响。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Open_DeadAreaKeepOut(pidAttr) PID_Set_DeadArea(pidAttr, PID_POSITION, false, true, 0, 0, 2)

/**
 * @brief 关闭位置控制的死区功能，并保持死区外的信号不变
 *
 * 死区功能用于忽略小的输入信号，以防止控制系统对微小的干扰做出反应。
 * 本宏定义用于设置位置控制模式下的死区参数，并确保死区外的信号不受影响。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Close_DeadAreaKeepOut(pidAttr) PID_Set_DeadArea(pidAttr, PID_POSITION, false, false, 0, 0, 2)

/**
 * @brief 设置位置控制的死区上下限值
 *
 * 死区功能用于忽略小的输入信号，以防止控制系统对微小的干扰做出反应。
 * 本宏定义用于设置位置控制模式下的死区上下限值。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param up 死区上限值
 * @param down 死区下限值
 */
#define PID_Position_Set_DeadAreaValueUpDown(pidAttr, up, down) PID_Set_DeadArea(pidAttr, PID_POSITION, false, false, up, down, 5)

/**
 * @brief 设置PID控制器为位置模式并开启单位调整
 *
 * 该宏定义用于设置PID控制器的工作模式为位置模式，并且开启单位调整功能。
 * 位置模式意味着控制器的输出将直接对应于被控对象的目标位置。
 * 开启单位调整表示后续可以通过设置单位尺寸和偏移量来精细调整控制输出。
 *
 * @param pidAttr PID控制器的属性结构体指针，用于配置PID控制器。
 */
#define PID_Position_Open_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_POSITION, true, 0, 0, 1)

/**
 * @brief 设置PID控制器为位置模式并关闭单位调整
 *
 * 该宏定义用于设置PID控制器的工作模式为位置模式，但不进行单位调整。
 * 这通常用于不需要精细调整输出的场景，或者在初始化阶段关闭单位调整。
 *
 * @param pidAttr PID控制器的属性结构体指针，用于配置PID控制器。
 */
#define PID_Position_Close_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_POSITION, false, 0, 0, 1)

/**
 * @brief 设置PID控制器位置模式下的单位尺寸
 *
 * 该宏定义用于在位置模式下设置PID控制器的单位尺寸。
 * 单位尺寸决定了控制器输出的分辨率，即每次输出变化的最小步长。
 *
 * @param pidAttr PID控制器的属性结构体指针，用于配置PID控制器。
 * @param size 单位尺寸值，表示控制器输出的最小变化量。
 */
#define PID_Position_Set_UnitSize(pidAttr, size) PID_Set_Unit(pidAttr, PID_POSITION, false, size, 0, 2)

/**
 * @brief 设置PID控制器位置模式下的单位偏移量
 *
 * 该宏定义用于在位置模式下设置PID控制器的单位偏移量。
 * 单位偏移量用于调整控制器输出的起始点，可以用于校正零点误差。
 *
 * @param pidAttr PID控制器的属性结构体指针，用于配置PID控制器。
 * @param value 单位偏移量值，表示控制器输出的起始点偏移。
 */
#define PID_Position_Set_UnitOffset(pidAttr, value) PID_Set_Unit(pidAttr, PID_POSITION, false, 0, value, 3)

/**
 * @brief 设置PID位置控制的积分限幅偏置值
 *
 * 此宏定义用于开启PID位置控制，并设置积分限幅偏置的相关参数
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_Position_Open_LimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_POSITION, true, false, 0, 1);

/**
 * @brief 关闭PID位置控制的积分限幅偏置
 *
 * 此宏定义用于关闭PID位置控制的积分限幅偏置功能
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_Position_Close_LimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_POSITION, false, false, 0, 1);

/**
 * @brief 设置PID位置控制的目标值到积分限幅偏置
 *
 * 此宏定义用于设置PID位置控制的目标值到积分限幅偏置的参数
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_Position_Open_AimValueToLimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_POSITION, false, true, 0, 2);

/**
 * @brief 关闭PID位置控制的目标值到积分限幅偏置
 *
 * 此宏定义用于关闭PID位置控制的目标值到积分限幅偏置功能
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_Position_Close_AimValueToLimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_POSITION, false, false, 0, 2);

/**
 * @brief 设置PID位置控制的积分限幅偏置的具体数值
 *
 * 此宏定义用于设置PID位置控制的积分限幅偏置的具体数值
 *
 * @param pidAttr PID控制器的属性
 * @param value 要设置的积分限幅偏置的具体数值
 */
#define PID_Position_Set_LimitAmplitudeIntegralBiasValue(pidAttr, value) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_POSITION, false, false, value, 3);

/**
 * @brief 获取PID位置控制的当前值
 *
 * 此宏定义用于获取PID位置控制的当前值
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_Position_Get_NowValue(pidAttr) PID_Get_NowValue(pidAttr, PID_POSITION);

/**
 * @brief 设置PID位置控制的回调函数
 *
 * 此宏定义用于设置PID位置控制模式下的回调函数，开启回调功能但不指定具体的回调函数。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Open_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_POSITION, true, 0, NULL, NULL, 1);

/**
 * @brief 设置PID位置控制的前置回调函数
 *
 * 此宏定义用于设置PID位置控制模式下的前置回调函数，即在PID计算之前调用的函数。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param beforeCallbackFunction 前置回调函数指针
 */
#define PID_Position_Set_BeforeCallback(pidAttr, beforeCallbackFunction) PID_Set_Callback(pidAttr, PID_POSITION, false, 0, beforeCallbackFunction, NULL, 3);

/**
 * @brief 设置PID位置控制的回调ID
 *
 * 此宏定义用于设置PID位置控制模式下的回调ID，用于标识回调事件。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param id 回调事件的ID
 */
#define PID_Position_Set_CallbackID(pidAttr, id) PID_Set_Callback(pidAttr, PID_POSITION, false, id, NULL, NULL, 2);

/**
 * @brief 设置PID位置控制的后置回调函数
 *
 * 此宏定义用于设置PID位置控制模式下的后置回调函数，即在PID计算之后调用的函数。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param afterCallbackFunction 后置回调函数指针
 */
#define PID_Position_Set_AfterCallback(pidAttr, afterCallbackFunction) PID_Set_Callback(pidAttr, PID_POSITION, false, 0, NULL, afterCallbackFunction, 4);

/**
 * @brief 关闭PID位置控制的回调功能
 *
 * 此宏定义用于关闭PID位置控制模式下的回调功能，清除所有回调设置。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Close_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_POSITION, false, 0, NULL, NULL, 1);

/**
 * @brief 设置PID位置控制的增量式速度限制
 *
 * 此宏定义用于设置PID位置控制模式下的增量式速度限制，通过限制输出来控制速度。
 *
 * @param pidAttr PID控制器的属性结构体指针
 */
#define PID_Position_Set_IncrementalSpeed(pidAttr) (PID_Set_OutLimit(pidAttr, PID_POSITION, true, PID_NULL, 0, 1), PID_Set_OutLimit(pidAttr, PID_POSITION, false, PID_INCREMENTAL, 0, 2))

/**
 * @brief 通过分散值和速度计算PID位置控制的结果
 *
 * 此宏定义用于通过分散值和速度参数来计算PID位置控制的结果，适用于增量式控制。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param dispersedValue 分散值
 * @param aimValue 目标值
 * @param dispersedSpeed 分散速度
 * @param aimSpeed 目标速度
 */
#define PID_Position_CalcResult_DispersedValueIncrementalByDispersedValueAndSpeed(pidAttr, dispersedValue, aimValue, dispersedSpeed, aimSpeed) (PID_SetAndSubmit_AimValueAndValue_Interface(pidAttr, PID_DISPERSED, dispersedSpeed, aimSpeed, PID_DISPERSED, NULL), PID_Position_CalcResult_ByDispersedValue(pidAttr, dispersedValue, aimValue))

/**
 * @brief 通过当前真实值和速度计算PID位置控制的结果
 *
 * 此宏定义用于通过当前真实值和速度参数来计算PID位置控制的结果，适用于增量式控制。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param dispersedValue 分散值
 * @param aimValue 目标值
 * @param nowTureSpeed 当前真实速度
 * @param aimSpeed 目标速度
 */
#define PID_Position_CalcResult_DispersedValueIncrementalByNowTureValueAndSpeed(pidAttr, dispersedValue, aimValue, nowTureSpeed, aimSpeed) (PID_SetAndSubmit_AimValueAndValue_Interface(pidAttr, PID_DISPERSED, nowTureSpeed, aimSpeed, PID_NOW_VALUE, NULL), PID_Position_CalcResult_ByDispersedValue(pidAttr, dispersedValue, aimValue))

/**
 * @brief 通过分散值和速度计算PID位置控制的结果
 *
 * 此宏定义用于通过分散值和速度参数来计算PID位置控制的结果，适用于当前值控制。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param nowTrueValue 当前真实值
 * @param aimValue 目标值
 * @param dispersedSpeed 分散速度
 * @param aimSpeed 目标速度
 */
#define PID_Position_CalcResult_NowTureValueIncrementalByDispersedValueAndSpeed(pidAttr, nowTrueValue, aimValue, dispersedSpeed, aimSpeed) (PID_SetAndSubmit_AimValueAndValue_Interface(pidAttr, PID_DISPERSED, dispersedSpeed, aimSpeed, PID_DISPERSED, NULL), PID_Position_CalcResult_ByNowTureValue(pidAttr, nowTrueValue, aimValue))

/**
 * @brief 通过当前真实值和速度计算PID位置控制的结果
 *
 * 此宏定义用于通过当前真实值和速度参数来计算PID位置控制的结果，适用于当前值控制。
 *
 * @param pidAttr PID控制器的属性结构体指针
 * @param nowTrueValue 当前真实值
 * @param aimValue 目标值
 * @param nowTureSpeed 当前真实速度
 * @param aimSpeed 目标速度
 */
#define PID_Position_CalcResult_NowTureValueIncrementalByNowTureValueAndSpeed(pidAttr, nowTrueValue, aimValue, nowTureSpeed, aimSpeed) (PID_SetAndSubmit_AimValueAndValue_Interface(pidAttr, PID_DISPERSED, nowTureSpeed, aimSpeed, PID_NOW_VALUE, NULL), PID_Position_CalcResult_ByNowTureValue(pidAttr, nowTrueValue, aimValue))

/**
 * @brief 设置PID参数为积分分离型
 *
 * @param pidAttr PID属性结构体指针
 * @param kp 比例系数
 * @param ki 积分系数
 * @param kd 微分系数
 */
#define PID_IntegralSeparate_Set_Kpid(pidAttr, kp, ki, kd) PID_Set_KpidParam(pidAttr, PID_INTEGRAL_SEPARATE, kp, ki, kd)

/**
 * @brief 设置积分分离型PID的阈值
 *
 * @param pidAttr PID属性结构体指针
 * @param value 阈值
 */
#define PID_IntegralSeparate_Set_ThresholdValue(pidAttr, value) PID_Set_ThresholdValue(pidAttr, PID_INTEGRAL_SEPARATE, value, 0)

/**
 * @brief 清除积分分离型PID的临时数据
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Clear_TempData(pidAttr) PID_Clear_TempDate(pidAttr, PID_INTEGRAL_SEPARATE)

/**
 * @brief 通过分散值计算积分分离型PID的结果
 *
 * @param pidAttr PID属性结构体指针
 * @param value 当前输入值
 * @param aimValue 目标值
 */
#define PID_IntegralSeparate_CalcResult_ByDispersedValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_INTEGRAL_SEPARATE, value, aimValue, PID_DISPERSED, NULL)

/**
 * @brief 通过当前真实值计算积分分离型PID的结果
 *
 * @param pidAttr PID属性结构体指针
 * @param value 当前输入值
 * @param aimValue 目标值
 */
#define PID_IntegralSeparate_CalcResult_ByNowTureValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_INTEGRAL_SEPARATE, value, aimValue, PID_NOW_VALUE, NULL)
/**
 * @brief 开启积分分离型PID的输出限制
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_INTEGRAL_SEPARATE, true, PID_NULL, 0, 1)

/**
 * @brief 关闭积分分离型PID的输出限制
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_INTEGRAL_SEPARATE, false, PID_NULL, 0, 1)

/**
 * @brief 设置积分分离型PID的自动输出限制
 *
 * @param pidAttr PID属性结构体指针
 * @param pidCalcType 计算类型
 */
#define PID_IntegralSeparate_Set_AutoOutLimit(pidAttr, pidCalcType) PID_Set_OutLimit(pidAttr, PID_INTEGRAL_SEPARATE, false, pidCalcType, 0, 2)

/**
 * @brief 设置积分分离型PID的输出限制值
 *
 * @param pidAttr PID属性结构体指针
 * @param value 输出限制的值
 */
#define PID_IntegralSeparate_Set_OutLimitValue(pidAttr, value) PID_Set_OutLimit(pidAttr, PID_INTEGRAL_SEPARATE, false, PID_NULL, value, 3)

/**
 * @brief 清除积分分离型PID的自动输出限制
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Clear_AutoOutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_INTEGRAL_SEPARATE, false, PID_NULL, 0, 2)

/**
 * @brief 开启积分分离型PID的死区
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_INTEGRAL_SEPARATE, true, false, 0, 0, 1)

/**
 * @brief 关闭积分分离型PID的死区
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_INTEGRAL_SEPARATE, false, false, 0, 0, 1)

/**
 * @brief 开启积分分离型PID的死区保持输出
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_DeadAreaKeepOut(pidAttr) PID_Set_DeadArea(pidAttr, PID_INTEGRAL_SEPARATE, false, true, 0, 0, 2)

/**
 * @brief 关闭积分分离型PID的死区保持输出
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_DeadAreaKeepOut(pidAttr) PID_Set_DeadArea(pidAttr, PID_INTEGRAL_SEPARATE, false, false, 0, 0, 2)

/**
 * @brief 设置积分分离型PID的死区上下限值
 *
 * @param pidAttr PID属性结构体指针
 * @param up 死区上限值
 * @param down 死区下限值
 */
#define PID_IntegralSeparate_Set_DeadAreaValueUpDown(pidAttr, up, down) PID_Set_DeadArea(pidAttr, PID_INTEGRAL_SEPARATE, false, false, up, down, 5)

/**
 * @brief 开启积分分离型PID的单位
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_INTEGRAL_SEPARATE, true, 0, 0, 1)

/**
 * @brief 关闭积分分离型PID的单位
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_INTEGRAL_SEPARATE, false, 0, 0, 1)

/**
 * @brief 设置积分分离型PID的单位大小
 *
 * @param pidAttr PID属性结构体指针
 * @param size 单位大小
 */
#define PID_IntegralSeparate_Set_UnitSize(pidAttr, size) PID_Set_Unit(pidAttr, PID_INTEGRAL_SEPARATE, false, size, 0, 2)

/**
 * @brief 设置积分分离型PID的单位偏移
 *
 * @param pidAttr PID属性结构体指针
 * @param value 单位偏移值
 */
#define PID_IntegralSeparate_Set_UnitOffset(pidAttr, value) PID_Set_Unit(pidAttr, PID_INTEGRAL_SEPARATE, false, 0, value, 3)
/**
 * @brief 开启积分分离型PID的限制幅值积分偏置
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_LimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_INTEGRAL_SEPARATE, true, false, 0, 1);

/**
 * @brief 关闭积分分离型PID的限制幅值积分偏置
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_LimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_INTEGRAL_SEPARATE, false, false, 0, 1);

/**
 * @brief 开启积分分离型PID的目标值到限制幅值积分偏置
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_AimValueToLimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_INTEGRAL_SEPARATE, false, true, 0, 2);

/**
 * @brief 关闭积分分离型PID的目标值到限制幅值积分偏置
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_AimValueToLimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_INTEGRAL_SEPARATE, false, false, 0, 2);

/**
 * @brief 设置积分分离型PID的限制幅值积分偏置值
 *
 * @param pidAttr PID属性结构体指针
 * @param value 要设置的积分偏置值
 */
#define PID_IntegralSeparate_Set_LimitAmplitudeIntegralBiasValue(pidAttr, value) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_INTEGRAL_SEPARATE, false, false, value, 3);
/**
 * @brief 获取积分分离型PID的当前值
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Get_NowValue(pidAttr) PID_Get_NowValue(pidAttr, PID_INTEGRAL_SEPARATE);

/**
 * @brief 开启积分分离型PID的回调
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Open_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_INTEGRAL_SEPARATE, true, 0, NULL, NULL, 1);

/**
 * @brief 设置积分分离型PID的回调前函数
 *
 * @param pidAttr PID属性结构体指针
 * @param beforeCallbackFunction 回调前函数的函数指针
 */
#define PID_IntegralSeparate_Set_BeforeCallback(pidAttr, beforeCallbackFunction) PID_Set_Callback(pidAttr, PID_INTEGRAL_SEPARATE, false, 0, beforeCallbackFunction, NULL, 3);
/**
 * @brief 设置积分分离型PID的回调ID
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Set_CallbackID(pidAttr, id) PID_Set_Callback(pidAttr, PID_INTEGRAL_SEPARATE, false, id, NULL, NULL, 2);

/**
 * @brief 设置积分分离型PID的回调后函数
 *
 * @param pidAttr PID属性结构体指针
 * @param afterCallbackFunction 回调后函数指针
 */
#define PID_IntegralSeparate_Set_AfterCallback(pidAttr, afterCallbackFunction) PID_Set_Callback(pidAttr, PID_INTEGRAL_SEPARATE, false, 0, NULL, afterCallbackFunction, 4);
/**
 * @brief 关闭积分分离型PID的回调
 *
 * @param pidAttr PID属性结构体指针
 */
#define PID_IntegralSeparate_Close_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_INTEGRAL_SEPARATE, false, 0, NULL, NULL, 1);

/**
 * @brief 设置PID变量积分控制器的PID参数
 *
 * @param pidAttr PID控制器的属性
 * @param kp 比例增益
 * @param ki 积分增益
 * @param kd 微分增益
 */
#define PID_VariableIntegral_Set_Kpid(pidAttr, kp, ki, kd) PID_Set_KpidParam(pidAttr, PID_VARIABLE_INTEGRAL, kp, ki, kd)

/**
 * @brief 设置PID变量积分控制器的阈值
 *
 * @param pidAttr PID控制器的属性
 * @param valueA 阈值A
 * @param valueB 阈值B
 */
#define PID_VariableIntegral_Set_ThresholdValue(pidAttr, valueA, valueB) PID_Set_ThresholdValue(pidAttr, PID_VARIABLE_INTEGRAL, valueA, valueB)

/**
 * @brief 清除PID变量积分控制器的临时数据
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Clear_TempData(pidAttr) PID_Clear_TempDate(pidAttr, PID_VARIABLE_INTEGRAL)

/**
 * @brief 通过分散值计算PID变量积分控制器的结果
 *
 * @param pidAttr PID控制器的属性
 * @param value 分散值
 * @param aimValue 目标值
 */
#define PID_VariableIntegral_CalcResult_ByDispersedValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_VARIABLE_INTEGRAL, value, aimValue, PID_DISPERSED, NULL)

/**
 * @brief 通过当前真实值计算PID变量积分控制器的结果
 *
 * @param pidAttr PID控制器的属性
 * @param value 当前真实值
 * @param aimValue 目标值
 */
#define PID_VariableIntegral_CalcResult_ByNowTureValue(pidAttr, value, aimValue) PID_CalcResult_Interface(pidAttr, PID_VARIABLE_INTEGRAL, value, aimValue, PID_NOW_VALUE, NULL)

/**
 * @brief 打开PID变量积分控制器的输出限制
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Open_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_VARIABLE_INTEGRAL, true, PID_NULL, 0, 1)

/**
 * @brief 关闭PID变量积分控制器的输出限制
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Close_OutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_VARIABLE_INTEGRAL, false, PID_NULL, 0, 1)

/**
 * @brief 设置PID变量积分控制器的自动输出限制
 *
 * @param pidAttr PID控制器的属性
 * @param pidCalcType PID计算类型
 */
#define PID_VariableIntegral_Set_AutoOutLimit(pidAttr, pidCalcType) PID_Set_OutLimit(pidAttr, PID_VARIABLE_INTEGRAL, false, pidCalcType, 0, 2)

/**
 * @brief 设置PID变量积分控制器的输出限制值
 *
 * @param pidAttr PID控制器的属性
 * @param value 输出限制值
 */
#define PID_VariableIntegral_Set_OutLimitValue(pidAttr, value) PID_Set_OutLimit(pidAttr, PID_VARIABLE_INTEGRAL, false, PID_NULL, value, 3)

/**
 * @brief 清除PID变量积分控制器的自动输出限制
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Clear_AutoOutLimit(pidAttr) PID_Set_OutLimit(pidAttr, PID_VARIABLE_INTEGRAL, false, PID_NULL, 0, 2)

/**
 * @brief 打开PID变量积分控制器的死区
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Open_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_VARIABLE_INTEGRAL, true, false, 0, 0, 1)

/**
 * @brief 关闭PID变量积分控制器的死区
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Close_DeadArea(pidAttr) PID_Set_DeadArea(pidAttr, PID_VARIABLE_INTEGRAL, false, false, 0, 0, 1)

/**
 * @brief 打开PID变量积分控制器的死区保持输出
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Open_DeadAreaKeepOut(pidAttr) PID_Set_DeadArea(pidAttr, PID_VARIABLE_INTEGRAL, false, true, 0, 0, 2)

/**
 * @brief 关闭PID变量积分控制器的死区保持输出
 *
 * @param pidAttr PID控制器的属性
 */
#define PID_VariableIntegral_Close_DeadAreaKeepOut(pidAttr) PID_Set_DeadArea(pidAttr, PID_VARIABLE_INTEGRAL, false, false, 0, 0, 2)

/**
 * @brief 设置PID变量积分控制器的死区上下值
 *
 * @param pidAttr PID控制器的属性
 * @param up 死区上限值
 * @param down 死区下限值
 */
#define PID_VariableIntegral_Set_DeadAreaValueUpDown(pidAttr, up, down) PID_Set_DeadArea(pidAttr, PID_VARIABLE_INTEGRAL, false, false, up, down, 5)

/**
 * @brief 开启变量积分功能
 *
 * 该宏用于开启PID控制器中的变量积分功能。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Open_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_VARIABLE_INTEGRAL, true, 0, 0, 1)

/**
 * @brief 关闭变量积分功能
 *
 * 该宏用于关闭PID控制器中的变量积分功能。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Close_Unit(pidAttr) PID_Set_Unit(pidAttr, PID_VARIABLE_INTEGRAL, false, 0, 0, 1)

/**
 * @brief 设置变量积分的单元大小
 *
 * 该宏用于设置PID控制器中变量积分的单元大小。
 *
 * @param pidAttr PID控制器的属性参数
 * @param size 积分单元的大小
 */
#define PID_VariableIntegral_Set_UnitSize(pidAttr, size) PID_Set_Unit(pidAttr, PID_VARIABLE_INTEGRAL, false, size, 0, 2)

/**
 * @brief 设置变量积分的单元偏移
 *
 * 该宏用于设置PID控制器中变量积分的单元偏移。
 *
 * @param pidAttr PID控制器的属性参数
 * @param value 积分单元的偏移值
 */
#define PID_VariableIntegral_Set_UnitOffset(pidAttr, value) PID_Set_Unit(pidAttr, PID_VARIABLE_INTEGRAL, false, 0, value, 3)

/**
 * @brief 开启变量积分的幅值限制
 *
 * 该宏用于开启PID控制器中变量积分的幅值限制。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Open_LimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_VARIABLE_INTEGRAL, true, false, 0, 1);

/**
 * @brief 关闭变量积分的幅值限制
 *
 * 该宏用于关闭PID控制器中变量积分的幅值限制。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Close_LimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_VARIABLE_INTEGRAL, false, false, 0, 1);

/**
 * @brief 开启目标值到幅值限制的映射
 *
 * 该宏用于开启PID控制器中变量积分的目标值到幅值限制的映射。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Open_AimValueToLimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_VARIABLE_INTEGRAL, false, true, 0, 2);

/**
 * @brief 关闭目标值到幅值限制的映射
 *
 * 该宏用于关闭PID控制器中变量积分的目标值到幅值限制的映射。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Close_AimValueToLimitAmplitudeIntegralBias(pidAttr) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_VARIABLE_INTEGRAL, false, false, 0, 2);

/**
 * @brief 设置变量积分的幅值限制值
 *
 * 该宏用于设置PID控制器中变量积分的幅值限制值。
 *
 * @param pidAttr PID控制器的属性参数
 * @param value 幅值限制值
 */
#define PID_VariableIntegral_Set_LimitAmplitudeIntegralBiasValue(pidAttr, value) PID_Set_LimitAmplitudeIntegralBiasValue(pidAttr, PID_VARIABLE_INTEGRAL, false, false, value, 3);

/**
 * @brief 获取变量积分的当前值
 *
 * 该宏用于获取PID控制器中变量积分的当前值。
 *
 * @param pidAttr PID控制器的属性参数
 * @return 返回当前的积分值
 */
#define PID_VariableIntegral_Get_NowValue(pidAttr) PID_Get_NowValue(pidAttr, PID_VARIABLE_INTEGRAL);

/**
 * @brief 开启变量积分的回调函数
 *
 * 该宏用于开启PID控制器中变量积分的回调函数。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Open_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_VARIABLE_INTEGRAL, true, 0, NULL, NULL, 1);

/**
 * @brief 设置变量积分的积分前回调函数
 *
 * 该宏用于设置PID控制器中变量积分的积分前回调函数。
 *
 * @param pidAttr PID控制器的属性参数
 * @param beforeCallbackFunction 积分前的回调函数指针
 */
#define PID_VariableIntegral_Set_BeforeCallback(pidAttr, beforeCallbackFunction) PID_Set_Callback(pidAttr, PID_VARIABLE_INTEGRAL, false, 0, beforeCallbackFunction, NULL, 3);

/**
 * @brief 设置变量积分的回调ID
 *
 * 该宏用于设置PID控制器中变量积分的回调ID。
 *
 * @param pidAttr PID控制器的属性参数
 * @param id 回调函数的ID
 */
#define PID_VariableIntegral_Set_CallbackID(pidAttr, id) PID_Set_Callback(pidAttr, PID_VARIABLE_INTEGRAL, false, id, NULL, NULL, 2);

/**
 * @brief 设置变量积分的积分后回调函数
 *
 * 该宏用于设置PID控制器中变量积分的积分后回调函数。
 *
 * @param pidAttr PID控制器的属性参数
 * @param afterCallbackFunction 积分后的回调函数指针
 */
#define PID_VariableIntegral_Set_AfterCallback(pidAttr, afterCallbackFunction) PID_Set_Callback(pidAttr, PID_VARIABLE_INTEGRAL, false, 0, NULL, afterCallbackFunction, 4);

/**
 * @brief 关闭变量积分的回调函数
 *
 * 该宏用于关闭PID控制器中变量积分的回调函数。
 *
 * @param pidAttr PID控制器的属性参数
 */
#define PID_VariableIntegral_Close_Callback(pidAttr) PID_Set_Callback(pidAttr, PID_VARIABLE_INTEGRAL, false, 0, NULL, NULL, 1);

#ifdef __cplusplus
}
#endif

#endif /* __PID_H */
