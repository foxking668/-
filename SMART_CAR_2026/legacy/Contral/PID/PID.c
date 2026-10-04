#include "PID.h"

/**
 * @file PID.c
 * @brief PID 控制器实现文件。
 *
 * @details
 * 本文件实现 `PID.h` 中声明的所有公共接口，并定义各 PID 模式私有运行数据、配置数据和固定内存池。
 * 主要实现内容包括：
 * - 固定大小内存池：为临时节点、配置结构和运行数据分配空间，避免嵌入式环境频繁动态申请内存；
 * - 临时节点链表：每个 PID 模式对应一个节点，保存 Kp/Ki/Kd、配置和历史误差；
 * - 节点缓存：可选缓存常用 PID 模式节点，减少链表查找开销；
 * - 计算前配置处理：回调、死区判断、单位换算等；
 * - PID 公式计算：增量式、位置式、积分分离、变积分；
 * - 计算后配置处理：输出限幅、积分限幅、回调和结果保存。
 */

/**
 * @brief PID 模块固定内存池中的分配节点。
 *
 * 每次通过 `PID_Memory_Malloc_Heap_self()` 申请空间时，都会在用户数据前面放置一个该结构，
 * 用于记录用户可见指针、占用大小和下一个内存块位置。
 */
struct st_PID_Memory_Node
{
	void *ptr;                         // 返回给调用方使用的数据区起始地址。
	size_t use_size;                   // 本块用户数据区大小，不包含节点头本身。
	struct st_PID_Memory_Node *next;   // 下一个已分配内存块节点。
};

/**
 * @brief 增量式 PID 的运行数据。
 *
 * 增量式 PID 根据当前误差、上一次误差和上上次误差计算输出增量。
 */
struct st_PID_Temp_Data_Incremental_Data
{
	accuracy bias;              // 当前误差或离散输入值。
	accuracy lastOneBias;       // 上一次误差，用于微分项和增量项计算。
	accuracy lastTwoBias;       // 上上次误差，用于二阶误差变化计算。

	accuracy aim_value;         // 目标值，用于由当前真实值换算误差。

	accuracy accumulate_result; // 累加输出值，非单次输出模式下保存累计控制量。
	accuracy result;            // 本次公式计算得到的增量或最终输出。
};

/**
 * @brief 增量式 PID 的功能配置。
 *
 * 用于控制输出限幅、死区、单位换算、单次输出模式以及计算前后回调。
 */
struct st_PID_Temp_Data_Incremental_Config
{
	bool is_use_limit_out;                    // 是否启用输出限幅。
	float limit_value;                        // 手动限幅值，输出会被限制到 ±limit_value。
	eu_PID_Calc_Type limit_value_auto_calc_type; // 自动限幅来源模式，可用其他 PID 模式的输出作为限幅参考。

	bool is_use_dead_area;                    // 是否启用误差死区。
	float dead_area_up;                       // 死区上限。
	float dead_area_down;                     // 死区下限。

	bool is_use_unit_value;                   // 是否启用单位换算。
	float unit_value_size;                    // 输出量化单位大小。
	float unit_value_offset;                  // 输出单位偏移，用于零点或机械中位校正。

	bool is_single_out;                       // true 表示只输出本次增量，false 表示与历史输出累加。

	bool is_use_callback;                     // 是否启用计算前后回调。
	pid_id_t id_callback;                     // 回调 ID，用于外部区分不同 PID 控制器。
	void (*before_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy accumulateResult_accuracy, eu_PID_Callback type); // 计算前回调。
	void (*after_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy accumulateResult_accuracy, eu_PID_Callback type);  // 计算后回调。
};

/**
 * @brief 位置式 PID 的运行数据。
 *
 * 位置式 PID 直接根据目标值与当前值的误差计算绝对控制输出。
 */
struct st_PID_Temp_Data_Position_Data
{
	accuracy bias;          // 当前误差：目标值 - 当前值，或外部提交的离散误差。
	accuracy lastOneBias;   // 上一次误差，用于微分项。
	accuracy integralBias;  // 积分误差累加值，用于积分项。

	accuracy aim_value;     // 目标值。
	accuracy now_value;     // 当前真实值。

	accuracy result;        // 本次位置式 PID 输出。
};

/**
 * @brief 位置式 PID 的功能配置。
 *
 * 相比增量式，位置式额外支持积分项限幅，避免长时间误差导致积分累积过大。
 */
struct st_PID_Temp_Data_Position_Config
{
	bool is_use_limit_out;                    // 是否启用输出限幅。
	float limit_value;                        // 手动限幅幅值。
	eu_PID_Calc_Type limit_value_auto_calc_type; // 自动限幅参考的 PID 模式。

	bool is_use_dead_area;                    // 是否启用死区。
	bool is_keep_out_in_dead;                 // 进入死区后是否保持上一次输出。
	float dead_area_up;                       // 死区上限。
	float dead_area_down;                     // 死区下限。

	bool is_use_unit_value;                   // 是否启用单位换算或量化输出。
	float unit_value_size;                    // 单位大小。
	float unit_value_offset;                  // 单位偏移。

	bool is_use_limit_amplitude_integral_bias; // 是否启用积分误差限幅。
	bool is_use_aim_value_to_limit;            // 是否使用目标值作为积分限幅幅值。
	float limit_amplitude_integral_bias_value; // 手动积分限幅幅值。

	bool is_use_callback;                     // 是否启用回调。
	pid_id_t id_callback;                     // 回调 ID。
	void (*before_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy nowValue_accuracy, eu_PID_Callback type); // 计算前回调。
	void (*after_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy nowValue_accuracy, eu_PID_Callback type);  // 计算后回调。
};

struct st_PID_Temp_Data_Integral_Separate_Data
{
	accuracy bias;
	accuracy lastOneBias;
	accuracy integralBias;

	accuracy aim_value;
	accuracy threshold_value;
	accuracy now_value;

	accuracy result;
};

struct st_PID_Temp_Data_Integral_Separate_Config
{
	bool is_use_limit_out;
	float limit_value;
	eu_PID_Calc_Type limit_value_auto_calc_type;

	bool is_use_dead_area;
	bool is_keep_out_in_dead;
	float dead_area_up;
	float dead_area_down;

	bool is_use_unit_value;
	float unit_value_size;
	float unit_value_offset;

	bool is_use_limit_amplitude_integral_bias;
	bool is_use_aim_value_to_limit;
	float limit_amplitude_integral_bias_value;

	bool is_use_callback;
	pid_id_t id_callback;
	void (*before_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy nowValue_accuracy, eu_PID_Callback type);
	void (*after_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy nowValue_accuracy, eu_PID_Callback type);
};

struct st_PID_Temp_Data_Variable_Integral_Data
{
	accuracy bias;
	accuracy lastOneBias;
	accuracy integralBias;

	accuracy aim_value;
	accuracy threshold_a_value;
	accuracy threshold_b_value;
	accuracy now_value;

	accuracy result;
};

struct st_PID_Temp_Data_Variable_Integral_Config
{
	bool is_use_limit_out;
	float limit_value;
	eu_PID_Calc_Type limit_value_auto_calc_type;

	bool is_use_dead_area;
	bool is_keep_out_in_dead;
	float dead_area_up;
	float dead_area_down;

	bool is_use_unit_value;
	float unit_value_size;
	float unit_value_offset;

	bool is_use_limit_amplitude_integral_bias;
	bool is_use_aim_value_to_limit;
	float limit_amplitude_integral_bias_value;

	bool is_use_callback;
	pid_id_t id_callback;
	void (*before_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy nowValue_accuracy, eu_PID_Callback type);
	void (*after_callback)(pid_id_t id_pid_id_t, accuracy result_accuracy, accuracy nowValue_accuracy, eu_PID_Callback type);
};

const uint8_t __Memory_Node_Size___uint8_t_CONST = sizeof(struct st_PID_Memory_Node);
static size_t __Use_Heap_Size___size_t = 0;
static uint8_t __Heap_Size_Map___uint8_t_Arr[PID_MEMORY_HEAP_SIZE];
static struct st_PID_Memory_Node *__Head_Node___st_PID_Memory_Node_ = NULL;
static struct st_PID_Memory_Node *__End_Node___st_PID_Memory_Node_ = NULL;

/**
 * @brief PID_Memory_Add_Node_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param node_st_PID_Memory_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
 */
bool PID_Memory_Add_Node_self(struct st_PID_Memory_Node *node_st_PID_Memory_Node_);
/**
 * @brief PID_Memory_GetCanArea_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param size_size_t size_size_t 参数，参与 PID_Memory_GetCanArea_self 的业务处理或状态更新。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
uint8_t *PID_Memory_GetCanArea_self(size_t size_size_t);
/**
 * @brief 创建 PID_Memory_Malloc_Heap_self 对应资源
 *
 * @details
 * 为 Contral / PID 模块申请并初始化新的缓存、节点或对象，供后续流程复用。
 *
 * @param size_size_t size_size_t 参数，参与 PID_Memory_Malloc_Heap_self 的业务处理或状态更新。
 */
void *PID_Memory_Malloc_Heap_self(size_t size_size_t);
/**
 * @brief 释放 PID_Memory_Clear_Heap_self 相关资源
 *
 * @details
 * 关闭、清理或释放 Contral / PID 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
 *
 * @param ptr 指针参数，指向调用方提供的数据或内部管理的资源。
 *
 * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
 */
bool PID_Memory_Clear_Heap_self(void *ptr);
/**
 * @brief 释放 PID_Memory_Free_Heap_self 相关资源
 *
 * @details
 * 关闭、清理或释放 Contral / PID 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
 *
 * @param ptr_void_ 指针参数，指向调用方提供的数据或内部管理的资源。
 *
 * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
 */
bool PID_Memory_Free_Heap_self(void *ptr_void_);
/**
 * @brief 获取 PID_Get_Data_Size_self 对应数据
 *
 * @details
 * 从 Contral / PID 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
 *
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
size_t PID_Get_Data_Size_self(eu_PID_Calc_Type type_eu_PID_Calc_Type);
/**
 * @brief 获取 PID_Get_Config_Size_self 对应数据
 *
 * @details
 * 从 Contral / PID 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
 *
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
size_t PID_Get_Config_Size_self(eu_PID_Calc_Type type_eu_PID_Calc_Type);
struct st_PID_Temp_Node *PID_Find_TempNode_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type);
struct st_PID_Temp_Node *PID_Create_TempNode_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type);
struct st_PID_Temp_Node *PID_Find_Cache_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type);
/**
 * @brief 更新 PID_Update_Cache_self 状态
 *
 * @details
 * 刷新 Contral / PID 模块的缓存、控制结果或运行状态，使对象内部数据与当前传感器/算法结果保持一致。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param node_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Update_Cache_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *node_st_PID_Temp_Node_);
/**
 * @brief 计算 PID_Calc_Formula_self 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Calc_Formula_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);
/**
 * @brief PID_Hanlde_Start_Config_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Hanlde_Start_Config_self(struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_);
/**
 * @brief PID_Hanlde_End_Config_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param result_accuracy_ 指针参数，指向调用方提供的数据或内部管理的资源。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Hanlde_End_Config_self(struct st_PID_Attr *pid_st_PID_Attr_, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_, accuracy *result_accuracy_);
/**
 * @brief 计算 PID_Calc_LimitValue_self 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param trueValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Calc_LimitValue_self(struct st_PID_Attr *pid_st_PID_Attr_, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_, accuracy trueValue_accuracy);
/**
 * @brief PID_Ctrl_Result_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isSet 布尔开关参数，用于启用或关闭对应功能。
 * @param result 用于接收处理结果的输出参数指针。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Ctrl_Result_self(struct st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_, bool isSet, accuracy result);

#define _PID_Abs(number) (number < 0 ? -number : number)

#define _PID_INCREMENTAL_FORMULA(Kp, Ki, Kd, bias, lastOneBias, lastTwoBias) (Kp * (bias - lastOneBias) + Ki * bias + Kd * (bias + lastTwoBias - 2 * lastOneBias))
#define _PID_POSITION_FORMULA(Kp, Ki, Kd, bias, lastOneBias, integralBias) (Kp * bias + Ki * integralBias + Kd * (bias - lastOneBias))
#define _PID_INTEGRAL_SEPARATE_FORMULA(Kp, Ki, Kd, bias, lastOneBias, integralBias, threshold) (Kp * bias + (_PID_Abs(bias) <= threshold ? 1 : 0) * Ki * integralBias + Kd * (bias - lastOneBias))
#define _PID_VARIABLE_INTEGRAL_FORMULA(Kp, Ki, Kd, bias, lastOneBias, integralBias, thresholdA, thresholdB) (Kp * bias + Ki * integralBias + (_PID_Abs(bias) <= thresholdB ? 1 : (_PID_Abs(bias) <= (thresholdA + thresholdB) ? ((thresholdA - bias + thresholdB) / thresholdA) : 0)) + Kd * (bias - lastOneBias))

/**
 * @brief PID_Memory_Add_Node_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param node_st_PID_Memory_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
 */
bool PID_Memory_Add_Node_self(struct st_PID_Memory_Node *node_st_PID_Memory_Node_)
{
	if (__Head_Node___st_PID_Memory_Node_ == NULL)
	{
		__Head_Node___st_PID_Memory_Node_ = node_st_PID_Memory_Node_;
		__End_Node___st_PID_Memory_Node_ = node_st_PID_Memory_Node_;
	}
	else
	{
		__End_Node___st_PID_Memory_Node_->next = node_st_PID_Memory_Node_;
		__End_Node___st_PID_Memory_Node_ = node_st_PID_Memory_Node_;
	}
	return true;
}

/**
 * @brief PID_Memory_GetCanArea_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param size_size_t size_size_t 参数，参与 PID_Memory_GetCanArea_self 的业务处理或状态更新。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
uint8_t *PID_Memory_GetCanArea_self(size_t size_size_t)
{
	struct st_PID_Memory_Node *node = __Head_Node___st_PID_Memory_Node_;
	uint8_t *start = __Heap_Size_Map___uint8_t_Arr;
	uint8_t *result = NULL;

	size_t minDistense_size_t = PID_MEMORY_HEAP_SIZE;

	while (node != NULL)
	{
		uint8_t *node_end = (uint8_t *)node + node->use_size + __Memory_Node_Size___uint8_t_CONST;

		if (node_end > start)
		{
			size_t distense_size_t = (uint8_t *)node - start;

			if (distense_size_t >= size_size_t && distense_size_t < minDistense_size_t)
			{
				minDistense_size_t = distense_size_t;
				result = start;
			}

			start = node_end;
		}

		node = node->next;
	}

	// Check if there's space after the last node
	if (result == NULL && start + size_size_t <= __Heap_Size_Map___uint8_t_Arr + PID_MEMORY_HEAP_SIZE)
	{
		result = start;
	}

	return result;
}

/**
 * @brief 创建 PID_Memory_Malloc_Heap_self 对应资源
 *
 * @details
 * 为 Contral / PID 模块申请并初始化新的缓存、节点或对象，供后续流程复用。
 *
 * @param size_size_t size_size_t 参数，参与 PID_Memory_Malloc_Heap_self 的业务处理或状态更新。
 */
void *PID_Memory_Malloc_Heap_self(size_t size_size_t)
{
	if (__Use_Heap_Size___size_t + size_size_t + __Memory_Node_Size___uint8_t_CONST > PID_MEMORY_HEAP_SIZE)
	{
		return NULL;
	}

	uint8_t *ptr_uint8_t_ = PID_Memory_GetCanArea_self(size_size_t + __Memory_Node_Size___uint8_t_CONST);

	if (ptr_uint8_t_ == NULL)
	{
		return NULL;
	}

	struct st_PID_Memory_Node *newNode_st_PID_Memory_Node_ = (struct st_PID_Memory_Node *)ptr_uint8_t_;
	void *result = ptr_uint8_t_ + __Memory_Node_Size___uint8_t_CONST;

	newNode_st_PID_Memory_Node_->ptr = result;
	newNode_st_PID_Memory_Node_->use_size = size_size_t;
	newNode_st_PID_Memory_Node_->next = NULL;

	PID_Memory_Add_Node_self(newNode_st_PID_Memory_Node_);

	__Use_Heap_Size___size_t += size_size_t + __Memory_Node_Size___uint8_t_CONST;

	return result;
}

/**
 * @brief 释放 PID_Memory_Free_Heap_self 相关资源
 *
 * @details
 * 关闭、清理或释放 Contral / PID 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
 *
 * @param ptr_void_ 指针参数，指向调用方提供的数据或内部管理的资源。
 *
 * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
 */
bool PID_Memory_Free_Heap_self(void *ptr_void_)
{
	struct st_PID_Memory_Node *lastNode_st_PID_Memory_Node_ = NULL;
	struct st_PID_Memory_Node *node_st_PID_Memory_Node_ = __Head_Node___st_PID_Memory_Node_;

	while (node_st_PID_Memory_Node_ != NULL && node_st_PID_Memory_Node_->ptr != ptr_void_)
	{
		lastNode_st_PID_Memory_Node_ = node_st_PID_Memory_Node_;
		node_st_PID_Memory_Node_ = node_st_PID_Memory_Node_->next;
	}

	if (node_st_PID_Memory_Node_ != NULL)
	{
		__Use_Heap_Size___size_t -= node_st_PID_Memory_Node_->use_size + __Memory_Node_Size___uint8_t_CONST;

		if (node_st_PID_Memory_Node_ == __End_Node___st_PID_Memory_Node_)
		{
			__End_Node___st_PID_Memory_Node_ = lastNode_st_PID_Memory_Node_;
			if (lastNode_st_PID_Memory_Node_ != NULL)
			{
				lastNode_st_PID_Memory_Node_->next = NULL;
			}
			else
			{
				__Head_Node___st_PID_Memory_Node_ = NULL;
			}
		}
		else if (node_st_PID_Memory_Node_ == __Head_Node___st_PID_Memory_Node_)
		{
			__Head_Node___st_PID_Memory_Node_ = node_st_PID_Memory_Node_->next;
		}
		else if (lastNode_st_PID_Memory_Node_ != NULL)
		{
			lastNode_st_PID_Memory_Node_->next = node_st_PID_Memory_Node_->next;
		}
	}
	else
	{
		return false; // Ptr not found
	}

	return true;
}

/**
 * @brief 释放 PID_Memory_Clear_Heap_self 相关资源
 *
 * @details
 * 关闭、清理或释放 Contral / PID 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
 *
 * @param ptr 指针参数，指向调用方提供的数据或内部管理的资源。
 *
 * @return 返回布尔结果；true 表示操作成功或条件成立，false 表示失败或条件不成立。
 */
bool PID_Memory_Clear_Heap_self(void *ptr)
{
	struct st_PID_Memory_Node *node_st_PID_Memory_Node_ = __Head_Node___st_PID_Memory_Node_;
	size_t i;

	// 鏌ユ壘鎸囧畾鐨勫唴瀛樺潡
	while (node_st_PID_Memory_Node_ != NULL)
	{
		if (node_st_PID_Memory_Node_->ptr == ptr)
		{
			break;
		}
		node_st_PID_Memory_Node_ = node_st_PID_Memory_Node_->next;
	}

	// 濡傛灉鎵句笉鍒版寚瀹氱殑鍐呭瓨鍧楋紝杩斿洖 1
	if (node_st_PID_Memory_Node_ == NULL || node_st_PID_Memory_Node_->ptr == NULL)
	{
		return false;
	}

	// 娓呯┖鍐呭瓨鍧楋紝閫愬瓧鑺傝缃负 0
	uint8_t *tmp = (uint8_t *)node_st_PID_Memory_Node_->ptr;
	for (i = 0; i < node_st_PID_Memory_Node_->use_size; i++)
	{
		tmp[i] = 0;
	}

	return true;
}

/**
 * @brief 计算 PID_CalcResult_Interface 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param value_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param aimValue_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param valueType_eu_PID_Submit_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_CalcResult_Interface(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, float value_float, float aimValue_float, enum eu_PID_Submit_Type valueType_eu_PID_Submit_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	accuracy result = 0;

	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		return 0;
	}

	PID_Set_AimValue(pid_st_PID_Attr_, type_eu_PID_Calc_Type, aimValue_float, tmpNode_st_PID_Temp_Node_);

	switch (valueType_eu_PID_Submit_Type)
	{
	case PID_DISPERSED:
		PID_Submit_DispersedValue(pid_st_PID_Attr_, type_eu_PID_Calc_Type, value_float, tmpNode_st_PID_Temp_Node_);
		break;
	case PID_NOW_VALUE:
		PID_Submit_NowTureValue(pid_st_PID_Attr_, type_eu_PID_Calc_Type, value_float, tmpNode_st_PID_Temp_Node_);
		break;
	default:
		break;
	}

	result = PID_Calc_Result(pid_st_PID_Attr_, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_);

	return result;
}

/**
 * @brief 提交 PID_SetAndSubmit_AimValueAndValue_Interface 输入数据
 *
 * @details
 * 把调用方提供的实时数据写入 Contral / PID 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param value_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param aimValue_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param valueType_eu_PID_Submit_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_SetAndSubmit_AimValueAndValue_Interface(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, float value_float, float aimValue_float, enum eu_PID_Submit_Type valueType_eu_PID_Submit_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		return PID_RESULT_FAIL;
	}

	PID_Set_AimValue(pid_st_PID_Attr_, type_eu_PID_Calc_Type, aimValue_float, tmpNode_st_PID_Temp_Node_);

	switch (valueType_eu_PID_Submit_Type)
	{
	case PID_DISPERSED:
		PID_Submit_DispersedValue(pid_st_PID_Attr_, type_eu_PID_Calc_Type, value_float, tmpNode_st_PID_Temp_Node_);
		break;
	case PID_NOW_VALUE:
		PID_Submit_NowTureValue(pid_st_PID_Attr_, type_eu_PID_Calc_Type, value_float, tmpNode_st_PID_Temp_Node_);
		break;
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

struct st_PID_Temp_Node *PID_Find_Cache_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type)
{

	if (!pid_st_PID_Attr_->is_use_cache)
	{
		return NULL;
	}

	struct st_PID_Temp_Node *result = NULL;

	struct st_PID_Cache_Node *cacheNode_st_PID_Cache_Node_ = pid_st_PID_Attr_->cache->head;
	while (cacheNode_st_PID_Cache_Node_ != NULL)
	{
		if (cacheNode_st_PID_Cache_Node_->type == type_eu_PID_Calc_Type)
		{
			result = cacheNode_st_PID_Cache_Node_->node;
			goto _End;
		}
		cacheNode_st_PID_Cache_Node_ = cacheNode_st_PID_Cache_Node_->next;
	}

_End:
	return result;
}

/**
 * @brief 创建 PID_Create_Cache 对应资源
 *
 * @details
 * 为 Contral / PID 模块申请并初始化新的缓存、节点或对象，供后续流程复用。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param size_uint16_t size_uint16_t 参数，参与 PID_Create_Cache 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Create_Cache(st_PID_Attr *pid_st_PID_Attr_, uint16_t size_uint16_t)
{
	if (pid_st_PID_Attr_->is_use_cache)
	{
		return PID_RESULT_FAIL;
	}

	if (pid_st_PID_Attr_->cache != NULL)
	{
		PID_Free_Cache(pid_st_PID_Attr_);
	}
	pid_st_PID_Attr_->cache = (struct st_PID_Cache *)PID_Memory_Malloc_Heap_self(sizeof(struct st_PID_Cache));
	PID_Memory_Clear_Heap_self(pid_st_PID_Attr_->cache);

	pid_st_PID_Attr_->is_use_cache = true;
	pid_st_PID_Attr_->cache->size = size_uint16_t;

	return PID_RESULT_SUCCESS;
}

/**
 * @brief PID_Reset_CacheSize 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param size_uint16_t size_uint16_t 参数，参与 PID_Reset_CacheSize 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Reset_CacheSize(st_PID_Attr *pid_st_PID_Attr_, uint16_t size_uint16_t)
{
	if (!pid_st_PID_Attr_->is_use_cache)
	{
		return PID_RESULT_FAIL;
	}

	if (size_uint16_t == 0)
	{
		if (PID_Free_Cache(pid_st_PID_Attr_) == PID_RESULT_SUCCESS)
		{
			pid_st_PID_Attr_->is_use_cache = true;
		}
	}
	if (pid_st_PID_Attr_->cache->now_size > size_uint16_t)
	{

		uint16_t i = pid_st_PID_Attr_->cache->now_size - size_uint16_t;
		struct st_PID_Cache_Node *cacheNode_st_PID_Cache_Node_ = pid_st_PID_Attr_->cache->end;
		struct st_PID_Cache_Node *tmpNode_st_PID_Cache_Node_ = NULL;
		while (i > 0)
		{
			tmpNode_st_PID_Cache_Node_ = cacheNode_st_PID_Cache_Node_->last;
			PID_Memory_Free_Heap_self(cacheNode_st_PID_Cache_Node_);
			cacheNode_st_PID_Cache_Node_ = tmpNode_st_PID_Cache_Node_;
			i -= 1;
		}
		pid_st_PID_Attr_->cache->end = cacheNode_st_PID_Cache_Node_;
		cacheNode_st_PID_Cache_Node_->next = NULL;
	}
	pid_st_PID_Attr_->cache->size = size_uint16_t;

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 释放 PID_Free_Cache 相关资源
 *
 * @details
 * 关闭、清理或释放 Contral / PID 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Free_Cache(st_PID_Attr *pid_st_PID_Attr_)
{
	if (!pid_st_PID_Attr_->is_use_cache || pid_st_PID_Attr_->cache == NULL)
	{
		return PID_RESULT_FAIL;
	}
	struct st_PID_Cache_Node *cacheNode_st_PID_Cache_Node_ = pid_st_PID_Attr_->cache->head;
	struct st_PID_Cache_Node *cacheNodeLast_st_PID_Cache_Node_ = NULL;
	while (cacheNode_st_PID_Cache_Node_ != NULL)
	{
		cacheNodeLast_st_PID_Cache_Node_ = cacheNode_st_PID_Cache_Node_;
		cacheNode_st_PID_Cache_Node_ = cacheNode_st_PID_Cache_Node_->next;
		if (cacheNodeLast_st_PID_Cache_Node_->last != NULL)
		{
			PID_Memory_Free_Heap_self(cacheNodeLast_st_PID_Cache_Node_);
		}
	}
	PID_Memory_Free_Heap_self(cacheNodeLast_st_PID_Cache_Node_);

	PID_Memory_Free_Heap_self(pid_st_PID_Attr_->cache);

	pid_st_PID_Attr_->cache = NULL;
	pid_st_PID_Attr_->is_use_cache = false;

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 更新 PID_Update_Cache_self 状态
 *
 * @details
 * 刷新 Contral / PID 模块的缓存、控制结果或运行状态，使对象内部数据与当前传感器/算法结果保持一致。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param node_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Update_Cache_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *node_st_PID_Temp_Node_)
{
	if (!pid_st_PID_Attr_->is_use_cache)
	{
		return PID_RESULT_FAIL;
	}

	struct st_PID_Cache_Node *cacheNode_st_PID_Cache_Node_ = pid_st_PID_Attr_->cache->head;
	while (cacheNode_st_PID_Cache_Node_ != NULL)
	{
		if (cacheNode_st_PID_Cache_Node_->type == type_eu_PID_Calc_Type)
		{
			goto _End;
		}
		cacheNode_st_PID_Cache_Node_ = cacheNode_st_PID_Cache_Node_->next;
	}
	if (pid_st_PID_Attr_->cache->now_size < pid_st_PID_Attr_->cache->size)
	{
		struct st_PID_Cache_Node *cacheNode_st_PID_Cache_Node_ = (struct st_PID_Cache_Node *)PID_Memory_Malloc_Heap_self(sizeof(struct st_PID_Cache_Node));
		cacheNode_st_PID_Cache_Node_->next = NULL;
		cacheNode_st_PID_Cache_Node_->last = NULL;
		cacheNode_st_PID_Cache_Node_->type = type_eu_PID_Calc_Type;
		cacheNode_st_PID_Cache_Node_->node = node_st_PID_Temp_Node_;

		if (pid_st_PID_Attr_->cache->head == NULL)
		{
			pid_st_PID_Attr_->cache->head = cacheNode_st_PID_Cache_Node_;
			pid_st_PID_Attr_->cache->end = cacheNode_st_PID_Cache_Node_;
		}
		else
		{
			cacheNode_st_PID_Cache_Node_->next = pid_st_PID_Attr_->cache->head;
			pid_st_PID_Attr_->cache->head->last = cacheNode_st_PID_Cache_Node_;
			pid_st_PID_Attr_->cache->head = cacheNode_st_PID_Cache_Node_;
		}

		pid_st_PID_Attr_->cache->now_size += 1;
	}
	else if (pid_st_PID_Attr_->cache->size > 0)
	{
		pid_st_PID_Attr_->cache->end->next = pid_st_PID_Attr_->cache->head;
		pid_st_PID_Attr_->cache->end->node = node_st_PID_Temp_Node_;
		pid_st_PID_Attr_->cache->end->type = type_eu_PID_Calc_Type;

		if (pid_st_PID_Attr_->cache->size > 1)
		{
			pid_st_PID_Attr_->cache->head = pid_st_PID_Attr_->cache->end;
			pid_st_PID_Attr_->cache->end = pid_st_PID_Attr_->cache->end->last;

			pid_st_PID_Attr_->cache->end->next = NULL;
			pid_st_PID_Attr_->cache->head->last = NULL;

			if (pid_st_PID_Attr_->cache->head->next != NULL)
			{
				pid_st_PID_Attr_->cache->head->next->last = pid_st_PID_Attr_->cache->head;
			}
		}
		else
		{
			pid_st_PID_Attr_->cache->end->next = NULL;
		}
	}

_End:
	return PID_RESULT_SUCCESS;
}

/**
 * @brief 计算 PID_Calc_Formula_self 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Calc_Formula_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
		if (tmpNode_st_PID_Temp_Node_ == NULL)
		{
			return 0.0f;
		}
	}

	accuracy result = 0;

	// 杩欓噷娣诲姞绠楁硶鐨勮绠楁柟寮�
	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;
		accuracy unitValueOffset_accuracy = 1;
		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_offset != 0)
		{
			unitValueOffset_accuracy = tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_offset;
		}
		result = _PID_INCREMENTAL_FORMULA(tmpNode_st_PID_Temp_Node_->kp,
										  tmpNode_st_PID_Temp_Node_->ki,
										  tmpNode_st_PID_Temp_Node_->kd,
										  tmp_st_PID_Temp_Data_Incremental_Data_->bias * unitValueOffset_accuracy,
										  tmp_st_PID_Temp_Data_Incremental_Data_->lastOneBias * unitValueOffset_accuracy,
										  tmp_st_PID_Temp_Data_Incremental_Data_->lastTwoBias * unitValueOffset_accuracy) /
				 unitValueOffset_accuracy;
		tmp_st_PID_Temp_Data_Incremental_Data_->lastTwoBias = tmp_st_PID_Temp_Data_Incremental_Data_->lastOneBias;
		tmp_st_PID_Temp_Data_Incremental_Data_->lastOneBias = tmp_st_PID_Temp_Data_Incremental_Data_->bias;

		// 杩欐浠ｇ爜鏄笉闇�瑕佺殑锛屽悗闈㈢殑Handle浼氬鐞�
		/*tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result += result;*/
		/*tmp_st_PID_Temp_Data_Incremental_Data_->result = result;*/

		/*if (!tmp_st_PID_Temp_Data_Incremental_Config_->is_single_out) {
			result = tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result;
		}*/

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		accuracy unitValueOffset_accuracy = 1;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Position_Config_->unit_value_offset != 0)
		{
			unitValueOffset_accuracy = tmp_st_PID_Temp_Data_Position_Config_->unit_value_offset;
		}
		tmp_st_PID_Temp_Data_Position_Data_->integralBias += tmp_st_PID_Temp_Data_Position_Data_->bias;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_amplitude_integral_bias)
		{
			accuracy limitAmplitudeValue_accuracy = tmp_st_PID_Temp_Data_Position_Config_->limit_amplitude_integral_bias_value;
			if (tmp_st_PID_Temp_Data_Position_Config_->is_use_aim_value_to_limit)
			{
				limitAmplitudeValue_accuracy = tmp_st_PID_Temp_Data_Position_Data_->aim_value;
			}
			tmp_st_PID_Temp_Data_Position_Data_->integralBias = PID_Calc_LimitAmplitude(tmp_st_PID_Temp_Data_Position_Data_->integralBias, limitAmplitudeValue_accuracy);
		}
		result = _PID_POSITION_FORMULA(tmpNode_st_PID_Temp_Node_->kp,
									   tmpNode_st_PID_Temp_Node_->ki,
									   tmpNode_st_PID_Temp_Node_->kd,
									   tmp_st_PID_Temp_Data_Position_Data_->bias * unitValueOffset_accuracy,
									   tmp_st_PID_Temp_Data_Position_Data_->lastOneBias * unitValueOffset_accuracy,
									   tmp_st_PID_Temp_Data_Position_Data_->integralBias * unitValueOffset_accuracy) /
				 unitValueOffset_accuracy;
		tmp_st_PID_Temp_Data_Position_Data_->lastOneBias = tmp_st_PID_Temp_Data_Position_Data_->bias;
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		accuracy unitValueOffset_accuracy = 1;
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_offset != 0)
		{
			unitValueOffset_accuracy = tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_offset;
		}
		if (_PID_Abs(tmp_st_PID_Temp_Data_Integral_Separate_Data_->bias) <= tmp_st_PID_Temp_Data_Integral_Separate_Data_->threshold_value)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Data_->integralBias += tmp_st_PID_Temp_Data_Integral_Separate_Data_->bias;
			if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_amplitude_integral_bias)
			{
				accuracy limitAmplitudeValue_accuracy = tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_amplitude_integral_bias_value;
				if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_aim_value_to_limit)
				{
					limitAmplitudeValue_accuracy = tmp_st_PID_Temp_Data_Integral_Separate_Data_->aim_value;
				}
				tmp_st_PID_Temp_Data_Integral_Separate_Data_->integralBias = PID_Calc_LimitAmplitude(tmp_st_PID_Temp_Data_Integral_Separate_Data_->integralBias, limitAmplitudeValue_accuracy);
			}
		}
		result = _PID_INTEGRAL_SEPARATE_FORMULA(tmpNode_st_PID_Temp_Node_->kp,
												tmpNode_st_PID_Temp_Node_->ki,
												tmpNode_st_PID_Temp_Node_->kd,
												tmp_st_PID_Temp_Data_Integral_Separate_Data_->bias * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Integral_Separate_Data_->lastOneBias * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Integral_Separate_Data_->integralBias * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Integral_Separate_Data_->threshold_value * unitValueOffset_accuracy) /
				 unitValueOffset_accuracy;
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->lastOneBias = tmp_st_PID_Temp_Data_Integral_Separate_Data_->bias;
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		accuracy unitValueOffset_accuracy = 1;
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_offset != 0)
		{
			unitValueOffset_accuracy = tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_offset;
		}
		if (_PID_Abs(tmp_st_PID_Temp_Data_Variable_Integral_Data_->bias) <= tmp_st_PID_Temp_Data_Variable_Integral_Data_->threshold_a_value + tmp_st_PID_Temp_Data_Variable_Integral_Data_->threshold_b_value)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Data_->integralBias += tmp_st_PID_Temp_Data_Variable_Integral_Data_->bias;
			if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_amplitude_integral_bias)
			{
				accuracy limitAmplitudeValue_accuracy = tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_amplitude_integral_bias_value;
				if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_aim_value_to_limit)
				{
					limitAmplitudeValue_accuracy = tmp_st_PID_Temp_Data_Variable_Integral_Data_->aim_value;
				}
				tmp_st_PID_Temp_Data_Variable_Integral_Data_->integralBias = PID_Calc_LimitAmplitude(tmp_st_PID_Temp_Data_Variable_Integral_Data_->integralBias, limitAmplitudeValue_accuracy);
			}
		}
		result = _PID_VARIABLE_INTEGRAL_FORMULA(tmpNode_st_PID_Temp_Node_->kp,
												tmpNode_st_PID_Temp_Node_->ki,
												tmpNode_st_PID_Temp_Node_->kd,
												tmp_st_PID_Temp_Data_Variable_Integral_Data_->bias * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Variable_Integral_Data_->lastOneBias * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Variable_Integral_Data_->integralBias * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Variable_Integral_Data_->threshold_a_value * unitValueOffset_accuracy,
												tmp_st_PID_Temp_Data_Variable_Integral_Data_->threshold_b_value * unitValueOffset_accuracy) /
				 unitValueOffset_accuracy;
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->lastOneBias = tmp_st_PID_Temp_Data_Variable_Integral_Data_->bias;
		break;
	}
	default:
		return result;
	}

	return result;
}

/**
 * @brief 计算 PID_Calc_Result 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Calc_Result(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{

	if (pid_st_PID_Attr_ != NULL && tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}

	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		return 0.0f;
	}

	eu_PID_Result headType_eu_PID_Result = PID_Hanlde_Start_Config_self(tmpNode_st_PID_Temp_Node_);

	accuracy result = PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, false, 0);

	if (headType_eu_PID_Result == PID_RESULT_STOP)
	{
		return 0.0f;
	}
	else if (headType_eu_PID_Result == PID_RESULT_STOP_BUT_KEEP_OUT)
	{
		return result;
	}

	// 杩欓噷浼樺寲鎺変簡
	/*switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
		result += PID_Calc_Formula_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_);
		break;
	case PID_POSITION:
	case PID_INTEGRAL_SEPARATE:
	case PID_VARIABLE_INTEGRAL:
		result = PID_Calc_Formula_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_);
		break;
	default:
		break;
	}*/

	result = PID_Calc_Formula_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_);

	// 杩欒浠ｇ爜宸茬粡娌＄敤浜�
	/*PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, true, result);*/

	if (PID_Hanlde_End_Config_self(pid_st_PID_Attr_, tmpNode_st_PID_Temp_Node_, &result) == PID_RESULT_STOP)
	{
		return 0.0f;
	}

	return result;
}

/**
 * @brief 计算 PID_Calc_LimitValue_self 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param trueValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Calc_LimitValue_self(struct st_PID_Attr *pid_st_PID_Attr_, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_, accuracy trueValue_accuracy)
{
	accuracy result = trueValue_accuracy;

	eu_PID_Calc_Type type_eu_PID_Calc_Type = tmpNode_st_PID_Temp_Node_->pid_calc_type;

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		float limitValue_float = tmp_st_PID_Temp_Data_Incremental_Config_->limit_value;
		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_limit_out)
		{
			if (tmp_st_PID_Temp_Data_Incremental_Config_->limit_value_auto_calc_type != PID_NULL)
			{
				if (tmp_st_PID_Temp_Data_Incremental_Config_->limit_value_auto_calc_type == type_eu_PID_Calc_Type)
				{
					goto _End;
				}
				limitValue_float = PID_Calc_Result(pid_st_PID_Attr_, tmp_st_PID_Temp_Data_Incremental_Config_->limit_value_auto_calc_type, NULL);
			}
			result = PID_Calc_LimitAmplitude(result, limitValue_float);
		}
		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		float limitValue_float = tmp_st_PID_Temp_Data_Position_Config_->limit_value;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_out)
		{
			if (tmp_st_PID_Temp_Data_Position_Config_->limit_value_auto_calc_type != PID_NULL)
			{
				if (tmp_st_PID_Temp_Data_Position_Config_->limit_value_auto_calc_type == type_eu_PID_Calc_Type)
				{
					goto _End;
				}
				limitValue_float = PID_Calc_Result(pid_st_PID_Attr_, tmp_st_PID_Temp_Data_Position_Config_->limit_value_auto_calc_type, NULL);
			}
			result = PID_Calc_LimitAmplitude(result, limitValue_float);
		}
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		float limitValue_float = tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value;
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_out)
		{
			if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value_auto_calc_type != PID_NULL)
			{
				if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value_auto_calc_type == type_eu_PID_Calc_Type)
				{
					goto _End;
				}
				limitValue_float = PID_Calc_Result(pid_st_PID_Attr_, tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value_auto_calc_type, NULL);
			}
			result = PID_Calc_LimitAmplitude(result, limitValue_float);
		}
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		float limitValue_float = tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value;
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_out)
		{
			if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value_auto_calc_type != PID_NULL)
			{
				if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value_auto_calc_type == type_eu_PID_Calc_Type)
				{
					goto _End;
				}
				limitValue_float = PID_Calc_Result(pid_st_PID_Attr_, tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value_auto_calc_type, NULL);
			}
			result = PID_Calc_LimitAmplitude(result, limitValue_float);
		}
		break;
	}
	default:
		break;
	}

_End:
	return result;
}

/**
 * @brief 计算 PID_Calc_LimitAmplitude 结果
 *
 * @details
 * 按照 Contral / PID 模块的算法规则处理输入参数和内部状态，生成控制输出、误差值或中间计算结果。
 *
 * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
 * @param amplitude amplitude 参数，参与 PID_Calc_LimitAmplitude 的业务处理或状态更新。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Calc_LimitAmplitude(accuracy value, accuracy amplitude)
{
	accuracy abs_amplitude = amplitude < 0 ? -amplitude : amplitude;

	if (value > abs_amplitude)
	{
		return abs_amplitude;
	}
	else if (value < -abs_amplitude)
	{
		return -abs_amplitude;
	}
	return value;
}

/**
 * @brief 获取 PID_Get_Data_Size_self 对应数据
 *
 * @details
 * 从 Contral / PID 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
 *
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
size_t PID_Get_Data_Size_self(eu_PID_Calc_Type type_eu_PID_Calc_Type)
{
	size_t sizeResult = 0;
	// 杩欓噷娣诲姞绠楁硶鐨勪复鏃舵暟鎹被鍨�
	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
		sizeResult = sizeof(struct st_PID_Temp_Data_Incremental_Data);
		break;
	case PID_POSITION:
		sizeResult = sizeof(struct st_PID_Temp_Data_Position_Data);
		break;
	case PID_INTEGRAL_SEPARATE:
		sizeResult = sizeof(struct st_PID_Temp_Data_Integral_Separate_Data);
		break;
	case PID_VARIABLE_INTEGRAL:
		sizeResult = sizeof(struct st_PID_Temp_Data_Variable_Integral_Data);
		break;
	default:
		sizeResult = 0;
	}

	return sizeResult;
}

/**
 * @brief 获取 PID_Get_Config_Size_self 对应数据
 *
 * @details
 * 从 Contral / PID 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
 *
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回整型结果，具体含义由函数功能决定，例如数量、状态码、文件描述符或字节数。
 */
size_t PID_Get_Config_Size_self(eu_PID_Calc_Type type_eu_PID_Calc_Type)
{
	size_t sizeResult = 0;
	// 杩欓噷娣诲姞绠楁硶鐨勪复鏃舵暟鎹被鍨�
	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
		sizeResult = sizeof(struct st_PID_Temp_Data_Incremental_Config);
		break;
	case PID_POSITION:
		sizeResult = sizeof(struct st_PID_Temp_Data_Position_Config);
		break;
	case PID_INTEGRAL_SEPARATE:
		sizeResult = sizeof(struct st_PID_Temp_Data_Integral_Separate_Config);
		break;
	case PID_VARIABLE_INTEGRAL:
		sizeResult = sizeof(struct st_PID_Temp_Data_Variable_Integral_Config);
		break;
	default:
		sizeResult = 0;
	}

	return sizeResult;
}

/**
 * @brief 获取 PID_Get_NowValue 对应数据
 *
 * @details
 * 从 Contral / PID 模块的内部状态、硬件接口或缓存结构中读取数据，并按函数返回类型交给调用方继续使用。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Get_NowValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type)
{
	accuracy result = 0;
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);

	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		result = tmp_st_PID_Temp_Data_Position_Data_->now_value;
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		result = tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value;
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		result = tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value;
		break;
	}
	default:
		break;
	}

	return result;
}

/**
 * @brief PID_Hanlde_Start_Config_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Hanlde_Start_Config_self(struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	eu_PID_Calc_Type type_eu_PID_Calc_Type = tmpNode_st_PID_Temp_Node_->pid_calc_type;

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;

		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_dead_area)
		{
			if (tmp_st_PID_Temp_Data_Incremental_Data_->result >= tmp_st_PID_Temp_Data_Incremental_Data_->aim_value - tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_down &&
				tmp_st_PID_Temp_Data_Incremental_Data_->result <= tmp_st_PID_Temp_Data_Incremental_Data_->aim_value + tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_up)
			{
				return PID_RESULT_STOP;
			}
		}
		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Incremental_Config_->before_callback != NULL)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->before_callback(tmp_st_PID_Temp_Data_Incremental_Config_->id_callback, tmp_st_PID_Temp_Data_Incremental_Data_->result, tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result, PID_CALLBACK_BEFORE);
			}
		}
		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_dead_area)
		{
			if (tmp_st_PID_Temp_Data_Position_Data_->now_value >= tmp_st_PID_Temp_Data_Position_Data_->aim_value - tmp_st_PID_Temp_Data_Position_Config_->dead_area_down &&
				tmp_st_PID_Temp_Data_Position_Data_->now_value <= tmp_st_PID_Temp_Data_Position_Data_->aim_value + tmp_st_PID_Temp_Data_Position_Config_->dead_area_up)
			{
				if (tmp_st_PID_Temp_Data_Position_Config_->is_keep_out_in_dead)
				{
					return PID_RESULT_STOP_BUT_KEEP_OUT;
				}
				return PID_RESULT_STOP;
			}
		}
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Position_Config_->before_callback != NULL)
			{
				tmp_st_PID_Temp_Data_Position_Config_->before_callback(tmp_st_PID_Temp_Data_Position_Config_->id_callback, tmp_st_PID_Temp_Data_Position_Data_->result, tmp_st_PID_Temp_Data_Position_Data_->now_value, PID_CALLBACK_BEFORE);
			}
		}
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_dead_area)
		{
			if (tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value >= tmp_st_PID_Temp_Data_Integral_Separate_Data_->aim_value - tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_down &&
				tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value <= tmp_st_PID_Temp_Data_Integral_Separate_Data_->aim_value + tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_up)
			{
				if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_keep_out_in_dead)
				{
					return PID_RESULT_STOP_BUT_KEEP_OUT;
				}
				return PID_RESULT_STOP;
			}
		}
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->before_callback != NULL)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->before_callback(tmp_st_PID_Temp_Data_Integral_Separate_Config_->id_callback, tmp_st_PID_Temp_Data_Integral_Separate_Data_->result, tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value, PID_CALLBACK_BEFORE);
			}
		}
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_dead_area)
		{
			if (tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value >= tmp_st_PID_Temp_Data_Variable_Integral_Data_->aim_value - tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_down &&
				tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value <= tmp_st_PID_Temp_Data_Variable_Integral_Data_->aim_value + tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_up)
			{
				if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_keep_out_in_dead)
				{
					return PID_RESULT_STOP_BUT_KEEP_OUT;
				}
				return PID_RESULT_STOP;
			}
		}
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->before_callback != NULL)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->before_callback(tmp_st_PID_Temp_Data_Variable_Integral_Config_->id_callback, tmp_st_PID_Temp_Data_Variable_Integral_Data_->result, tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value, PID_CALLBACK_BEFORE);
			}
		}
		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief PID_Hanlde_End_Config_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param result_accuracy_ 指针参数，指向调用方提供的数据或内部管理的资源。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Hanlde_End_Config_self(struct st_PID_Attr *pid_st_PID_Attr_, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_, accuracy *result_accuracy_)
{
	eu_PID_Calc_Type type_eu_PID_Calc_Type = tmpNode_st_PID_Temp_Node_->pid_calc_type;

	accuracy result = *result_accuracy_;

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size != 0)
		{
			tmp_st_PID_Temp_Data_Incremental_Data_->result = result * (tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size);
		}
		else
		{
			tmp_st_PID_Temp_Data_Incremental_Data_->result = result;
		}

		tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result += result;

		if (!tmp_st_PID_Temp_Data_Incremental_Config_->is_single_out)
		{
			result = tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result;
		}

		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_limit_out)
		{
			result = PID_Calc_LimitValue_self(pid_st_PID_Attr_, tmpNode_st_PID_Temp_Node_, result);
		}

		*result_accuracy_ = result;

		PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, true, result);

		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Incremental_Config_->after_callback != NULL)
			{
				struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;
				tmp_st_PID_Temp_Data_Incremental_Config_->after_callback(tmp_st_PID_Temp_Data_Incremental_Config_->id_callback, tmp_st_PID_Temp_Data_Incremental_Data_->result, tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result, PID_CALLBACK_AFTER);
			}
		}

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Position_Config_->unit_value_size != 0)
		{
			result = result * tmp_st_PID_Temp_Data_Position_Config_->unit_value_size;
		}
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_out)
		{
			result = PID_Calc_LimitValue_self(pid_st_PID_Attr_, tmpNode_st_PID_Temp_Node_, result);
		}

		*result_accuracy_ = result;

		PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, true, result);

		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Position_Config_->after_callback != NULL)
			{
				struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
				tmp_st_PID_Temp_Data_Position_Config_->after_callback(tmp_st_PID_Temp_Data_Position_Config_->id_callback, result, tmp_st_PID_Temp_Data_Position_Data_->now_value, PID_CALLBACK_AFTER);
			}
		}

		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size != 0)
		{
			result = result * tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size;
		}
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_out)
		{
			result = PID_Calc_LimitValue_self(pid_st_PID_Attr_, tmpNode_st_PID_Temp_Node_, result);
		}

		*result_accuracy_ = result;

		PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, true, result);

		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->after_callback != NULL)
			{
				struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->after_callback(tmp_st_PID_Temp_Data_Integral_Separate_Config_->id_callback, result, tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value, PID_CALLBACK_AFTER);
			}
		}

		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_unit_value && tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size != 0)
		{
			result = result * tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size;
		}
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_out)
		{
			result = PID_Calc_LimitValue_self(pid_st_PID_Attr_, tmpNode_st_PID_Temp_Node_, result);
		}

		*result_accuracy_ = result;

		PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, true, result);

		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_callback)
		{
			if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->after_callback != NULL)
			{
				struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->after_callback(tmp_st_PID_Temp_Data_Variable_Integral_Config_->id_callback, result, tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value, PID_CALLBACK_AFTER);
			}
		}

		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 初始化 PID_Init 相关资源
 *
 * @details
 * 完成 Contral / PID 模块运行前所需的资源申请、参数写入和状态复位，确保后续调用处于可用状态。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Init(st_PID_Attr *pid_st_PID_Attr_)
{
	pid_st_PID_Attr_->id = 0;
	pid_st_PID_Attr_->pid_temp.end = NULL;
	pid_st_PID_Attr_->pid_temp.head = NULL;
	pid_st_PID_Attr_->is_use_cache = false;
	pid_st_PID_Attr_->cache = NULL;

	return PID_RESULT_SUCCESS;
}

/**
 * @brief PID_Ctrl_Result_self 函数说明
 *
 * @details
 * 该函数属于 Contral / PID 模块，用于完成与函数名对应的业务处理、状态维护或硬件访问，是模块对外或内部流程中的一个独立步骤。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isSet 布尔开关参数，用于启用或关闭对应功能。
 * @param value 待写入、提交或计算的数值，具体含义由调用场景决定。
 *
 * @return 返回 PID 计算使用的 accuracy 数值，通常表示控制输出、误差或限幅后的结果。
 */
accuracy PID_Ctrl_Result_self(struct st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_, bool isSet, accuracy value)
{
	if (tmpNode_st_PID_Temp_Node_ == NULL && pid_st_PID_Attr_ != NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}

	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		return 0.0f;
	}

	accuracy result = 0;

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;

		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_single_out)
		{
			if (isSet)
			{
				tmp_st_PID_Temp_Data_Incremental_Data_->result = value;
			}
			result = tmp_st_PID_Temp_Data_Incremental_Data_->result;
		}
		else
		{
			if (isSet)
			{
				tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result = value;
			}
			result = tmp_st_PID_Temp_Data_Incremental_Data_->accumulate_result;
		}

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (isSet)
		{
			tmp_st_PID_Temp_Data_Position_Data_->result = value;
		}
		result = tmp_st_PID_Temp_Data_Position_Data_->result;
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (isSet)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Data_->result = value;
		}
		result = tmp_st_PID_Temp_Data_Integral_Separate_Data_->result;
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		if (isSet)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Data_->result = value;
		}
		result = tmp_st_PID_Temp_Data_Variable_Integral_Data_->result;
		break;
	}
	default:
		break;
	}

	return result;
}

struct st_PID_Temp_Node *PID_Find_TempNode_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type)
{
	struct st_PID_Temp_Node *node = NULL;
	bool isFind_bool = false;

	node = PID_Find_Cache_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (node != NULL)
	{
		isFind_bool = true;
	}
	else
	{
		node = pid_st_PID_Attr_->pid_temp.head;
		while (node != NULL)
		{
			if (node->pid_calc_type == type_eu_PID_Calc_Type)
			{
				break;
			}
			node = node->next;
		}
	}
	if (!isFind_bool && node != NULL)
	{
		PID_Update_Cache_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type, node);
	}
	return node;
}

struct st_PID_Temp_Node *PID_Create_TempNode_self(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type)
{
	size_t nodeSize_size_t = 0;
	size_t dataSize_size_t = 0;
	size_t configSize_size_t = 0;
	nodeSize_size_t = sizeof(struct st_PID_Temp_Node);

	dataSize_size_t = PID_Get_Data_Size_self(type_eu_PID_Calc_Type);
	configSize_size_t = PID_Get_Config_Size_self(type_eu_PID_Calc_Type);

	struct st_PID_Temp_Node *newNode = (struct st_PID_Temp_Node *)PID_Memory_Malloc_Heap_self(nodeSize_size_t);
	if (newNode == NULL)
	{
		return NULL;
	}

	PID_Memory_Clear_Heap_self(newNode);

	void *newTempData_void_ = NULL;
	if (dataSize_size_t != 0)
	{
		newTempData_void_ = PID_Memory_Malloc_Heap_self(dataSize_size_t);
		if (newTempData_void_ == NULL)
		{
			PID_Memory_Free_Heap_self(newNode);
			return NULL;
		}
		PID_Memory_Clear_Heap_self(newTempData_void_);
	}

	void *newTempConfig_void_ = NULL;
	if (configSize_size_t != 0)
	{
		newTempConfig_void_ = PID_Memory_Malloc_Heap_self(configSize_size_t);
		if (newTempConfig_void_ == NULL)
		{
			PID_Memory_Free_Heap_self(newNode);
			return NULL;
		}
		PID_Memory_Clear_Heap_self(newTempConfig_void_);
	}

	newNode->data = newTempData_void_;
	newNode->config = newTempConfig_void_;
	newNode->pid_calc_type = type_eu_PID_Calc_Type;

	if (pid_st_PID_Attr_->pid_temp.head != NULL)
	{
		pid_st_PID_Attr_->pid_temp.end->next = newNode;
		pid_st_PID_Attr_->pid_temp.end = newNode;
	}
	else
	{
		pid_st_PID_Attr_->pid_temp.head = newNode;
		pid_st_PID_Attr_->pid_temp.end = newNode;
	}

	return newNode;
}

/**
 * @brief 释放 PID_Clear_TempDate 相关资源
 *
 * @details
 * 关闭、清理或释放 Contral / PID 模块占用的动态内存、文件描述符、缓存节点或硬件资源。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Clear_TempDate(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		return PID_RESULT_FAIL;
	}

	// This is one useless line of code
	// PID_Ctrl_Result_self(NULL, type_eu_PID_Calc_Type, tmpNode_st_PID_Temp_Node_, true, 0);

	PID_Memory_Clear_Heap_self(tmpNode_st_PID_Temp_Node_->data);

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_AimValue 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param aimValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_AimValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy aimValue_accuracy, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}
	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;
		tmp_st_PID_Temp_Data_Incremental_Data_->aim_value = aimValue_accuracy;
		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		tmp_st_PID_Temp_Data_Position_Data_->aim_value = aimValue_accuracy;
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->aim_value = aimValue_accuracy;
		break;
	}
	default:
		break;
	}
	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_Callback 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isUseCallback_bool 回调函数指针，在 PID 计算前后或特定处理阶段被调用。
 * @param id_pid_id_t PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param paramIndex_uint8_t paramIndex_uint8_t 参数，参与 PID_Set_Callback 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_Callback(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseCallback_bool, pid_id_t id_pid_id_t, void (*before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback)(pid_id_t id_pid_id_t, accuracy paramOne_accuracy, accuracy paramTwo_accuracy, eu_PID_Callback callbackType_eu_PID_Callback), void (*after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback)(pid_id_t id_pid_id_t, accuracy paramOne_accuracy, accuracy paramTwo_accuracy, eu_PID_Callback callbackType_eu_PID_Callback), uint8_t paramIndex_uint8_t)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Incremental_Config_->is_use_callback = isUseCallback_bool;
			tmp_st_PID_Temp_Data_Incremental_Config_->id_callback = id_pid_id_t;
			tmp_st_PID_Temp_Data_Incremental_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			tmp_st_PID_Temp_Data_Incremental_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->is_use_callback = isUseCallback_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->id_callback = id_pid_id_t;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
		}

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_callback = isUseCallback_bool;
			tmp_st_PID_Temp_Data_Position_Config_->id_callback = id_pid_id_t;
			tmp_st_PID_Temp_Data_Position_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			tmp_st_PID_Temp_Data_Position_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Position_Config_->is_use_callback = isUseCallback_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Position_Config_->id_callback = id_pid_id_t;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Position_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Position_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
		}

		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_callback = isUseCallback_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->id_callback = id_pid_id_t;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_callback = isUseCallback_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->id_callback = id_pid_id_t;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
		}

		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_callback = isUseCallback_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->id_callback = id_pid_id_t;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_callback = isUseCallback_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->id_callback = id_pid_id_t;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->before_callback = before_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->after_callback = after_Callback_void__pid_id_t_accuracy_accuracy_eu_PID_Callback;
			}
		}

		break;
	}
	default:
	{
		break;
	}
	}
	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_OutLimit 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isUseLimitOut_bool 限幅相关参数，用于约束控制输出或中间积分量的范围。
 * @param limitValueAutoCalcType_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param limitValue_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param paramIndex_uint8_t paramIndex_uint8_t 参数，参与 PID_Set_OutLimit 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_OutLimit(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseLimitOut_bool, eu_PID_Calc_Type limitValueAutoCalcType_eu_PID_Calc_Type, float limitValue_float, uint8_t paramIndex_uint8_t)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Incremental_Config_->is_use_limit_out = isUseLimitOut_bool;
			tmp_st_PID_Temp_Data_Incremental_Config_->limit_value = limitValue_float;
			tmp_st_PID_Temp_Data_Incremental_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->is_use_limit_out = isUseLimitOut_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->limit_value = limitValue_float;
			}
		}

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_out = isUseLimitOut_bool;
			tmp_st_PID_Temp_Data_Position_Config_->limit_value = limitValue_float;
			tmp_st_PID_Temp_Data_Position_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_out = isUseLimitOut_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Position_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Position_Config_->limit_value = limitValue_float;
			}
		}

		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_out = isUseLimitOut_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value = limitValue_float;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_out = isUseLimitOut_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_value = limitValue_float;
			}
		}

		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_out = isUseLimitOut_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value = limitValue_float;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_out = isUseLimitOut_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value_auto_calc_type = limitValueAutoCalcType_eu_PID_Calc_Type;
			}
			else if (paramIndex_uint8_t == 3)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_value = limitValue_float;
			}
		}

		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_DeadArea 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isUseDeadArea_bool 布尔开关参数，用于启用或关闭对应功能。
 * @param isKeepOutInDead_bool 布尔开关参数，用于启用或关闭对应功能。
 * @param deadAreaUp_float deadAreaUp_float 参数，参与 PID_Set_DeadArea 的业务处理或状态更新。
 * @param deadAreaDown_float deadAreaDown_float 参数，参与 PID_Set_DeadArea 的业务处理或状态更新。
 * @param paramIndex_uint8_t paramIndex_uint8_t 参数，参与 PID_Set_DeadArea 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_DeadArea(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseDeadArea_bool, bool isKeepOutInDead_bool, float deadAreaUp_float, float deadAreaDown_float, uint8_t paramIndex_uint8_t)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Incremental_Config_->is_use_dead_area = isUseDeadArea_bool;
			tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_up = deadAreaUp_float;
			tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_down = deadAreaDown_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->is_use_dead_area = isUseDeadArea_bool;
			}
			else if (paramIndex_uint8_t == 5)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_up = deadAreaUp_float;
				tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_down = deadAreaDown_float;
			}
			else
			{
				if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_up = deadAreaUp_float;
				}
				else if (paramIndex_uint8_t == 4)
				{
					tmp_st_PID_Temp_Data_Incremental_Config_->dead_area_down = deadAreaDown_float;
				}
			}
		}

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_dead_area = isUseDeadArea_bool;
			tmp_st_PID_Temp_Data_Position_Config_->is_keep_out_in_dead = isKeepOutInDead_bool;
			tmp_st_PID_Temp_Data_Position_Config_->dead_area_up = deadAreaUp_float;
			tmp_st_PID_Temp_Data_Position_Config_->dead_area_down = deadAreaDown_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Position_Config_->is_use_dead_area = isUseDeadArea_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Position_Config_->is_keep_out_in_dead = isKeepOutInDead_bool;
			}
			else if (paramIndex_uint8_t == 5)
			{
				tmp_st_PID_Temp_Data_Position_Config_->dead_area_up = deadAreaUp_float;
				tmp_st_PID_Temp_Data_Position_Config_->dead_area_down = deadAreaDown_float;
			}
			else
			{
				if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Position_Config_->dead_area_up = deadAreaUp_float;
				}
				else if (paramIndex_uint8_t == 4)
				{
					tmp_st_PID_Temp_Data_Position_Config_->dead_area_down = deadAreaDown_float;
				}
			}
		}

		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_dead_area = isUseDeadArea_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_keep_out_in_dead = isKeepOutInDead_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_up = deadAreaUp_float;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_down = deadAreaDown_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_dead_area = isUseDeadArea_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_keep_out_in_dead = isKeepOutInDead_bool;
			}
			else if (paramIndex_uint8_t == 5)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_up = deadAreaUp_float;
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_down = deadAreaDown_float;
			}
			else
			{
				if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_up = deadAreaUp_float;
				}
				else if (paramIndex_uint8_t == 4)
				{
					tmp_st_PID_Temp_Data_Integral_Separate_Config_->dead_area_down = deadAreaDown_float;
				}
			}
		}

		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_dead_area = isUseDeadArea_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_keep_out_in_dead = isKeepOutInDead_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_up = deadAreaUp_float;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_down = deadAreaDown_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_dead_area = isUseDeadArea_bool;
			}
			else if (paramIndex_uint8_t == 2)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_keep_out_in_dead = isKeepOutInDead_bool;
			}
			else if (paramIndex_uint8_t == 5)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_up = deadAreaUp_float;
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_down = deadAreaDown_float;
			}
			else
			{
				if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_up = deadAreaUp_float;
				}
				else if (paramIndex_uint8_t == 4)
				{
					tmp_st_PID_Temp_Data_Variable_Integral_Config_->dead_area_down = deadAreaDown_float;
				}
			}
		}

		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_Unit 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isUseUnitValue_bool 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param unitValueSize_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param unitValueOffset_float 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param paramIndex_uint8_t paramIndex_uint8_t 参数，参与 PID_Set_Unit 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_Unit(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseUnitValue_bool, float unitValueSize_float, float unitValueOffset_float, uint8_t paramIndex_uint8_t)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Incremental_Config_->is_use_unit_value = isUseUnitValue_bool;
			tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size = unitValueSize_float;
			tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_offset = unitValueOffset_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->is_use_unit_value = isUseUnitValue_bool;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size = unitValueSize_float;
				tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_offset = unitValueOffset_float;
			}
			else
			{
				if (paramIndex_uint8_t == 2)
				{
					tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size = unitValueSize_float;
				}
				else if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_offset = unitValueOffset_float;
				}
			}
		}

		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_unit_value = isUseUnitValue_bool;
			tmp_st_PID_Temp_Data_Position_Config_->unit_value_size = unitValueSize_float;
			tmp_st_PID_Temp_Data_Position_Config_->unit_value_offset = unitValueOffset_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Position_Config_->is_use_unit_value = isUseUnitValue_bool;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Position_Config_->unit_value_size = unitValueSize_float;
				tmp_st_PID_Temp_Data_Position_Config_->unit_value_offset = unitValueOffset_float;
			}
			else
			{
				if (paramIndex_uint8_t == 2)
				{
					tmp_st_PID_Temp_Data_Position_Config_->unit_value_size = unitValueSize_float;
				}
				else if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Position_Config_->unit_value_offset = unitValueOffset_float;
				}
			}
		}

		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_unit_value = isUseUnitValue_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size = unitValueSize_float;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_offset = unitValueOffset_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_unit_value = isUseUnitValue_bool;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size = unitValueSize_float;
				tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_offset = unitValueOffset_float;
			}
			else
			{
				if (paramIndex_uint8_t == 2)
				{
					tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size = unitValueSize_float;
				}
				else if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_offset = unitValueOffset_float;
				}
			}
		}

		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_unit_value = isUseUnitValue_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size = unitValueSize_float;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_offset = unitValueOffset_float;
		}
		else
		{
			if (paramIndex_uint8_t == 1)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_unit_value = isUseUnitValue_bool;
			}
			else if (paramIndex_uint8_t == 4)
			{
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size = unitValueSize_float;
				tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_offset = unitValueOffset_float;
			}
			else
			{
				if (paramIndex_uint8_t == 2)
				{
					tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size = unitValueSize_float;
				}
				else if (paramIndex_uint8_t == 3)
				{
					tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_offset = unitValueOffset_float;
				}
			}
		}

		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_ThresholdValue 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param thresholdValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param thresholdValueTwo_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_ThresholdValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy thresholdValue_accuracy, accuracy thresholdValueTwo_accuracy)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->threshold_value = thresholdValue_accuracy;
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->threshold_a_value = thresholdValue_accuracy;
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->threshold_b_value = thresholdValueTwo_accuracy;
		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

// 浠ヤ笅涓嶉渶瑕佸仛鍖哄垎

/**
 * @brief 设置 PID_Set_KpidParam 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param kp_float kp_float 参数，参与 PID_Set_KpidParam 的业务处理或状态更新。
 * @param ki_float ki_float 参数，参与 PID_Set_KpidParam 的业务处理或状态更新。
 * @param kd_float kd_float 参数，参与 PID_Set_KpidParam 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_KpidParam(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, float kp_float, float ki_float, float kd_float)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Create_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
		if (tmpNode_st_PID_Temp_Node_ == NULL)
		{
			return PID_RESULT_FAIL;
		}
	}

	tmpNode_st_PID_Temp_Node_->kp = kp_float;
	tmpNode_st_PID_Temp_Node_->ki = ki_float;
	tmpNode_st_PID_Temp_Node_->kd = kd_float;

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_SingleOut 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isTrue_bool 布尔开关参数，用于启用或关闭对应功能。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_SingleOut(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isTrue_bool)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Create_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
		if (tmpNode_st_PID_Temp_Node_ == NULL)
		{
			return PID_RESULT_FAIL;
		}
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		tmp_st_PID_Temp_Data_Incremental_Config_->is_single_out = isTrue_bool;
		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 设置 PID_Set_LimitAmplitudeIntegralBiasValue 对应参数
 *
 * @details
 * 根据传入参数更新 Contral / PID 模块的配置、控制目标或硬件寄存器，使后续控制/处理逻辑按新参数运行。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param isUseLimitAmplitudeIntegralBias_bool 限幅相关参数，用于约束控制输出或中间积分量的范围。
 * @param isUseAimValueToLimit_bool 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param limitAmplitudeIntegralBiasValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param paramIndex_uint8_t paramIndex_uint8_t 参数，参与 PID_Set_LimitAmplitudeIntegralBiasValue 的业务处理或状态更新。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Set_LimitAmplitudeIntegralBiasValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, bool isUseLimitAmplitudeIntegralBias_bool, bool isUseAimValueToLimit_bool, float limitAmplitudeIntegralBiasValue_accuracy, uint8_t paramIndex_uint8_t)
{
	struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_ = NULL;
	tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;

		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_amplitude_integral_bias = isUseLimitAmplitudeIntegralBias_bool;
			tmp_st_PID_Temp_Data_Position_Config_->is_use_aim_value_to_limit = isUseAimValueToLimit_bool;
			tmp_st_PID_Temp_Data_Position_Config_->limit_amplitude_integral_bias_value = limitAmplitudeIntegralBiasValue_accuracy;
		}
		else if (paramIndex_uint8_t == 1)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_limit_amplitude_integral_bias = isUseLimitAmplitudeIntegralBias_bool;
		}
		else if (paramIndex_uint8_t == 2)
		{
			tmp_st_PID_Temp_Data_Position_Config_->is_use_aim_value_to_limit = isUseAimValueToLimit_bool;
		}
		else if (paramIndex_uint8_t == 3)
		{
			tmp_st_PID_Temp_Data_Position_Config_->limit_amplitude_integral_bias_value = limitAmplitudeIntegralBiasValue_accuracy;
		}

		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;

		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_amplitude_integral_bias = isUseLimitAmplitudeIntegralBias_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_aim_value_to_limit = isUseAimValueToLimit_bool;
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_amplitude_integral_bias_value = limitAmplitudeIntegralBiasValue_accuracy;
		}
		else if (paramIndex_uint8_t == 1)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_limit_amplitude_integral_bias = isUseLimitAmplitudeIntegralBias_bool;
		}
		else if (paramIndex_uint8_t == 2)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_aim_value_to_limit = isUseAimValueToLimit_bool;
		}
		else if (paramIndex_uint8_t == 3)
		{
			tmp_st_PID_Temp_Data_Integral_Separate_Config_->limit_amplitude_integral_bias_value = limitAmplitudeIntegralBiasValue_accuracy;
		}

		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;

		if (paramIndex_uint8_t == 0)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_amplitude_integral_bias = isUseLimitAmplitudeIntegralBias_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_aim_value_to_limit = isUseAimValueToLimit_bool;
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_amplitude_integral_bias_value = limitAmplitudeIntegralBiasValue_accuracy;
		}
		else if (paramIndex_uint8_t == 1)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_limit_amplitude_integral_bias = isUseLimitAmplitudeIntegralBias_bool;
		}
		else if (paramIndex_uint8_t == 2)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_aim_value_to_limit = isUseAimValueToLimit_bool;
		}
		else if (paramIndex_uint8_t == 3)
		{
			tmp_st_PID_Temp_Data_Variable_Integral_Config_->limit_amplitude_integral_bias_value = limitAmplitudeIntegralBiasValue_accuracy;
		}

		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 提交 PID_Submit_DispersedValue 输入数据
 *
 * @details
 * 把调用方提供的实时数据写入 Contral / PID 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param dispersedValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Submit_DispersedValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy dispersedValue_accuracy, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy biasValue = dispersedValue_accuracy;
		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_unit_value)
		{
			biasValue = biasValue / tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Incremental_Data_->bias = biasValue;
		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy biasValue = dispersedValue_accuracy;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_unit_value)
		{
			biasValue = biasValue / tmp_st_PID_Temp_Data_Position_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Position_Data_->bias = tmp_st_PID_Temp_Data_Position_Data_->aim_value - tmp_st_PID_Temp_Data_Position_Data_->now_value;
		tmp_st_PID_Temp_Data_Position_Data_->now_value += biasValue;
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy biasValue = dispersedValue_accuracy;
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_unit_value)
		{
			biasValue = biasValue / tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->bias = tmp_st_PID_Temp_Data_Integral_Separate_Data_->aim_value - tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value;
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value += biasValue;
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{

		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy biasValue = dispersedValue_accuracy;
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_unit_value)
		{
			biasValue = biasValue / tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->bias = tmp_st_PID_Temp_Data_Variable_Integral_Data_->aim_value - tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value;
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value += biasValue;
		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}

/**
 * @brief 提交 PID_Submit_NowTureValue 输入数据
 *
 * @details
 * 把调用方提供的实时数据写入 Contral / PID 模块的内部状态，供后续 PID、图像处理或电机控制流程使用。
 *
 * @param pid_st_PID_Attr_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param type_eu_PID_Calc_Type PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 * @param nowTrueValue_accuracy 数值参数，用于提交当前值、目标值、限幅值或配置值。
 * @param tmpNode_st_PID_Temp_Node_ PID 控制器相关参数，用于读取或更新该控制器的状态、配置或临时计算节点。
 *
 * @return 返回 PID_RESULT_* 枚举值，用于表示本次 PID 配置、提交或计算是否成功。
 */
eu_PID_Result PID_Submit_NowTureValue(st_PID_Attr *pid_st_PID_Attr_, eu_PID_Calc_Type type_eu_PID_Calc_Type, accuracy nowTrueValue_accuracy, struct st_PID_Temp_Node *tmpNode_st_PID_Temp_Node_)
{
	if (tmpNode_st_PID_Temp_Node_ == NULL)
	{
		tmpNode_st_PID_Temp_Node_ = PID_Find_TempNode_self(pid_st_PID_Attr_, type_eu_PID_Calc_Type);
	}
	if (tmpNode_st_PID_Temp_Node_ == NULL || tmpNode_st_PID_Temp_Node_->config == NULL)
	{
		return PID_RESULT_FAIL;
	}

	switch (type_eu_PID_Calc_Type)
	{
	case PID_INCREMENTAL:
	{
		struct st_PID_Temp_Data_Incremental_Data *tmp_st_PID_Temp_Data_Incremental_Data_ = (struct st_PID_Temp_Data_Incremental_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Incremental_Config *tmp_st_PID_Temp_Data_Incremental_Config_ = (struct st_PID_Temp_Data_Incremental_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy nowValue = nowTrueValue_accuracy;
		if (tmp_st_PID_Temp_Data_Incremental_Config_->is_use_unit_value)
		{
			nowValue = nowValue / tmp_st_PID_Temp_Data_Incremental_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Incremental_Data_->bias = tmp_st_PID_Temp_Data_Incremental_Data_->aim_value - nowValue;
		break;
	}
	case PID_POSITION:
	{
		struct st_PID_Temp_Data_Position_Data *tmp_st_PID_Temp_Data_Position_Data_ = (struct st_PID_Temp_Data_Position_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Position_Config *tmp_st_PID_Temp_Data_Position_Config_ = (struct st_PID_Temp_Data_Position_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy nowValue = nowTrueValue_accuracy;
		if (tmp_st_PID_Temp_Data_Position_Config_->is_use_unit_value)
		{
			nowValue = nowValue / tmp_st_PID_Temp_Data_Position_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Position_Data_->bias = tmp_st_PID_Temp_Data_Position_Data_->aim_value - tmp_st_PID_Temp_Data_Position_Data_->now_value;
		tmp_st_PID_Temp_Data_Position_Data_->now_value = nowValue;
		break;
	}
	case PID_INTEGRAL_SEPARATE:
	{
		struct st_PID_Temp_Data_Integral_Separate_Data *tmp_st_PID_Temp_Data_Integral_Separate_Data_ = (struct st_PID_Temp_Data_Integral_Separate_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Integral_Separate_Config *tmp_st_PID_Temp_Data_Integral_Separate_Config_ = (struct st_PID_Temp_Data_Integral_Separate_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy nowValue = nowTrueValue_accuracy;
		if (tmp_st_PID_Temp_Data_Integral_Separate_Config_->is_use_unit_value)
		{
			nowValue = nowValue / tmp_st_PID_Temp_Data_Integral_Separate_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->bias = tmp_st_PID_Temp_Data_Integral_Separate_Data_->aim_value - tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value;
		tmp_st_PID_Temp_Data_Integral_Separate_Data_->now_value = nowValue;
		break;
	}
	case PID_VARIABLE_INTEGRAL:
	{

		struct st_PID_Temp_Data_Variable_Integral_Data *tmp_st_PID_Temp_Data_Variable_Integral_Data_ = (struct st_PID_Temp_Data_Variable_Integral_Data *)tmpNode_st_PID_Temp_Node_->data;
		struct st_PID_Temp_Data_Variable_Integral_Config *tmp_st_PID_Temp_Data_Variable_Integral_Config_ = (struct st_PID_Temp_Data_Variable_Integral_Config *)tmpNode_st_PID_Temp_Node_->config;
		accuracy nowValue = nowTrueValue_accuracy;
		if (tmp_st_PID_Temp_Data_Variable_Integral_Config_->is_use_unit_value)
		{
			nowValue = nowValue / tmp_st_PID_Temp_Data_Variable_Integral_Config_->unit_value_size;
		}
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->bias = tmp_st_PID_Temp_Data_Variable_Integral_Data_->aim_value - tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value;
		tmp_st_PID_Temp_Data_Variable_Integral_Data_->now_value = nowValue;
		break;
	}
	default:
		break;
	}

	return PID_RESULT_SUCCESS;
}
