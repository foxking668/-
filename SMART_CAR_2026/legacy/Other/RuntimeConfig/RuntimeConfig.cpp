#include "RuntimeConfig.hpp"

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <fstream>
#include <iostream>
#include <sstream>
#include <cstdlib>
#include <cctype>

namespace Other
{
    std::mutex RuntimeConfig::mutex_;
    RuntimeImageParams RuntimeConfig::params_;
    std::chrono::steady_clock::time_point RuntimeConfig::last_load_time_ = std::chrono::steady_clock::now();
    bool RuntimeConfig::initialized_ = false;

    static const char *CONFIG_DIR = "./car_params";
    static const char *CONFIG_FILE = "./car_params/image_params.txt";

    std::string localTrim(const std::string &text)
    {
        size_t start = 0;
        while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])))
        {
            start++;
        }

        size_t end = text.size();
        while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])))
        {
            end--;
        }

        return text.substr(start, end - start);
    }

    static const int CONFIG_TEMPLATE_VERSION = 2026070607;

    int readExistingConfigVersion()
    {
        std::ifstream input(CONFIG_FILE);
        if (!input.good())
        {
            return -1;
        }

        std::string line;
        while (std::getline(input, line))
        {
            size_t commentPos = line.find('#');
            if (commentPos != std::string::npos)
            {
                line = line.substr(0, commentPos);
            }

            line = localTrim(line);
            if (line.empty())
            {
                continue;
            }

            size_t eqPos = line.find('=');
            if (eqPos == std::string::npos)
            {
                continue;
            }

            std::string key = localTrim(line.substr(0, eqPos));
            std::string value = localTrim(line.substr(eqPos + 1));
            if (key == "config_version")
            {
                try
                {
                    return std::stoi(value);
                }
                catch (...)
                {
                    return -1;
                }
            }
        }

        return -1;
    }

    void backupOldConfigFile()
    {
        std::ifstream input(CONFIG_FILE, std::ios::binary);
        if (!input.good())
        {
            return;
        }

        std::ofstream output("./car_params/image_params.txt.bak", std::ios::binary);
        if (!output.good())
        {
            return;
        }

        output << input.rdbuf();
    }

    bool writeDefaultConfigFile()
    {
        std::ofstream output(CONFIG_FILE);
        if (!output.good())
        {
            /* quiet: failed to write config */
            return false;
        }

        output << R"CONFIG_TEMPLATE(
# 小车运行参数配置文件（以你本次参数为基准：斑马线单双远区循环 + 红灯启动诊断 + 蓝挡板状态机低速段）。
config_version=2026070607
#
# 使用方法：
# 1. 首次使用本包：替换完整源码后重新编译一次。
# 2. 程序运行时约每 500ms 自动重读本文件；之后只改 car_params/image_params.txt 并保存。
# 3. 除“本次新增”位置外，其余数值全部保持你这次贴出的数值。
# 4. 蓝色桶锥、蓝色 HSV 挡板识别、固定强制打方向均未启用；本文件不控制它们。
#
# 重要：每一行都写成 key=value；以 # 开头的行只是注释。

# ============================================================
# 一、基础巡线 / 正常速度（保持你的数值）
# ============================================================
# 巡线目标中心比例。小车整体偏右：略减小；整体偏左：略增大。
center_pratio=0.48
# 正常巡线目标速度。
motor_aim_speed=12

# ============================================================
# 二、新区域型人行横道：跳变 + 区域面积 + 连通白条
# ============================================================
# 说明：只有“多行黑白条纹跳变、白色面积足够、连续白条数量足够”三关都通过，
# 才会认为是人行横道；随机白点或少量反光不会只因跳变而触发。

# 0=关闭交替：每一次都只使用 zebra_region_far_center_y_ratio。
# 1=开启交替：第1、3、5...次使用第一高度；第2、4、6...次使用第二高度。
# 只有“最终确认并触发停车”的人行横道才计数；远处预检、冷却期不会计数。
# 想重新从第1次开始：先改为0保存，等待约1秒，再改回1保存；或重启程序。
zebra_region_far_cycle_enable=1

# 第1、3、5...次人行横道的远处带状检测区中心高度。
zebra_region_far_center_y_ratio=0.3
# 第2、4、6...次人行横道的远处带状检测区中心高度。
zebra_region_far_second_center_y_ratio=0.24

# 近处确认带中心高度。远处已经看见后，近处也确认才会触发停车。
zebra_region_near_center_y_ratio=0.38
# 单条扫描线上至少需要的黑白跳变次数。
zebra_region_jump_min_count=3
# 黑色间隔的最大值/最小值允许比值。越小越严格。
zebra_region_gap_ratio_max=2.0
# 每个远/近中心上下各取多高的带状区域，单位为图像高度比例。
zebra_region_half_height_ratio=0.06
# 每个带状区域横向抽样多少条扫描线。
zebra_region_sample_rows=9
# 上面的扫描线中，至少多少条满足“重复白条 + 间隔均匀”。
zebra_region_min_hit_rows=3
# 带状区域内、左右赛道线之间的白色总面积最小/最大占比。
zebra_region_white_area_ratio_min=0.15
zebra_region_white_area_ratio_max=0.75
# 单条合格白色连通条带的最小面积/最小连续高度/至少数量。
zebra_region_component_min_area_ratio=0.010
zebra_region_component_min_height_ratio=0.32
zebra_region_component_min_count=3

# 远处区域命中后是否先降速：0=不降速，只作为近处停车前置确认；1=先降速。
zebra_far_slow_enable=0
# 仅 zebra_far_slow_enable=1 时使用的预减速目标速度。
zebra_slow_aim_speed=15
# 远处命中状态保持时间；用于远近二次确认时间窗口，单位 ms。
zebra_slow_hold_ms=1000
# 同一个人行横道触发后，多久内不再重复触发，单位 ms。
zebra_ignore_after_detect_ms=10000
# 人行横道急停反转结束后，继续停车保持多久，单位 ms。
zebra_after_reverse_stop_ms=2200
# 停车结束后多久恢复 PID，0 表示立即恢复。
pid_recover_delay_ms=0
# 停车时是否清除 PID 历史：0=保留；1=清除。
motor_pid_clear_on_stop_enable=0

# ============================================================
# 三、新双 ROI 红灯：左侧上部 + 左侧下部 + 连续帧确认
# ============================================================
# 红灯图只给红灯检测使用，不影响巡线裁剪和左右边线。

# 0=本帧不运行红灯状态机；1=运行红灯状态机。
traffic_red_enable=1
# 0=关闭诊断；1=终端周期打印 [TrafficDebug]。
traffic_red_debug_enable=1
# 诊断打印间隔，单位 ms。
traffic_red_debug_interval_ms=800

# 红灯检测图顶部/左侧/右侧裁掉比例。
traffic_red_view_top_crop_ratio=0.00
traffic_red_view_left_crop_ratio=0.01
traffic_red_view_right_crop_ratio=0.10

# 上部红灯 ROI 的横向、纵向范围，基于“红灯检测图”。
traffic_red_upper_roi_x_start=0.05
traffic_red_upper_roi_x_end=0.95
traffic_red_upper_roi_y_start=0.20
traffic_red_upper_roi_y_end=0.56

# 关键：1=同时启用左下 ROI；0=只看上部。你的图中红灯在左下，因此保持 1。
traffic_red_dual_roi_enable=1
traffic_red_lower_roi_y_start=0.58
traffic_red_lower_roi_y_end=1.00
# 连续几帧检测到合格红灯才停车。当前 1 用于先验证识别是否启动。
traffic_red_confirm_frames=1

# HSV 中红色的两段色相范围。通常不要改。
traffic_red_h1_min=0
traffic_red_h1_max=12
traffic_red_h2_min=165
traffic_red_h2_max=179
# 红色饱和度/亮度下限。
traffic_red_s_min=60
traffic_red_v_min=55
# 合格红色连通区域最小面积、最大面积占比、圆度和外接矩形宽高比。
traffic_red_min_area=10.0
traffic_red_max_area_ratio=0.05
traffic_red_min_circle=0.1
traffic_red_aspect_min=0.45
traffic_red_aspect_max=2.20
# HSV 掩膜形态学核大小。
traffic_red_morph_kernel_size=1

# ============================================================
# 四、人行横道后：动态减小巡线图顶部裁剪 + 状态机低速段
# ============================================================
# 这一段不是蓝色识别，也不会强制转向。它只改变原巡线图上方裁剪和目标速度。

# 0=关闭动态裁剪状态机，永远使用 normal_top_crop；1=开启。
blue_board_line_expand_enable=1
# 人行横道停车完成并重新起步后，等多久才开始动态视野/低速，单位 ms。
blue_board_line_expand_delay_after_zebra_resume_ms=1000
# 动态视野（减小顶部裁剪）持续多久，单位 ms；到时间后裁剪自动恢复正常。
blue_board_line_expand_duration_ms=1500
# 正常巡线顶部裁剪比例。
blue_board_line_normal_top_crop_ratio=0.20
# 状态机期间顶部裁剪比例。数值越小，看回的上方画面越多。
blue_board_line_post_zebra_top_crop_ratio=0.01

# -------- 新增：动态裁剪结束后才开始的低速段 --------
# 0=关闭；1=开启。低速不会和动态裁剪同时发生。
# 只有动态裁剪窗口结束、巡线图恢复 normal_top_crop 后才开始低速。
blue_board_line_post_expand_slow_enable=1
# 低速目标速度。正常速度为 12 时，建议先试 6；挡板仍识别晚可降至 5。
blue_board_line_post_expand_slow_aim_speed=6.0
# 低速持续多久，单位 ms；从“动态裁剪真正结束”开始计时。
blue_board_line_post_expand_slow_duration_ms=3000

# ============================================================
# 五、红灯 / 人行横道急停反转（保持你的数值）
# ============================================================
# 0=不反转制动；1=先反转制动、再锁止。
emergency_reverse_enable=1
# 触发停车后，先撤掉前进动力多久，单位 ms。
emergency_coast_ms=50
# 反转制动 PWM。
emergency_reverse_pwm=14000
# 反转目标圈数。
emergency_reverse_turns=1.5
# 编码器异常时反转最长保护时间，单位 ms。
emergency_reverse_timeout_ms=500
# 反转结束后 PWM=0 锁止保持时间，单位 ms。
emergency_brake_lock_ms=500
)CONFIG_TEMPLATE";
        output.close();
        return true;
    }

    std::string RuntimeConfig::trim(const std::string &text)
    {
        size_t start = 0;
        while (start < text.size() && std::isspace(static_cast<unsigned char>(text[start])))
        {
            start++;
        }

        size_t end = text.size();
        while (end > start && std::isspace(static_cast<unsigned char>(text[end - 1])))
        {
            end--;
        }

        return text.substr(start, end - start);
    }

    double RuntimeConfig::clampDouble(double value, double minValue, double maxValue)
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

    int RuntimeConfig::clampInt(int value, int minValue, int maxValue)
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

    bool RuntimeConfig::isSameParams(const RuntimeImageParams &a, const RuntimeImageParams &b)
    {
        return a.center_pratio == b.center_pratio &&
               a.zebra_check_y_ratio == b.zebra_check_y_ratio &&
               a.zebra_check_row_count == b.zebra_check_row_count &&
               a.zebra_jump_min_count == b.zebra_jump_min_count &&
               a.zebra_jump_block_div == b.zebra_jump_block_div &&
               a.zebra_ignore_after_detect_ms == b.zebra_ignore_after_detect_ms &&
               a.zebra_far_y_ratio == b.zebra_far_y_ratio &&
               a.zebra_far_cycle_enable == b.zebra_far_cycle_enable &&
               a.zebra_far_second_y_ratio == b.zebra_far_second_y_ratio &&
               a.zebra_near_y_ratio == b.zebra_near_y_ratio &&
               a.zebra_gap_ratio_max == b.zebra_gap_ratio_max &&
               a.zebra_region_half_height_ratio == b.zebra_region_half_height_ratio &&
               a.zebra_region_sample_rows == b.zebra_region_sample_rows &&
               a.zebra_region_min_hit_rows == b.zebra_region_min_hit_rows &&
               a.zebra_region_white_area_ratio_min == b.zebra_region_white_area_ratio_min &&
               a.zebra_region_white_area_ratio_max == b.zebra_region_white_area_ratio_max &&
               a.zebra_component_min_area_ratio == b.zebra_component_min_area_ratio &&
               a.zebra_component_min_height_ratio == b.zebra_component_min_height_ratio &&
               a.zebra_component_min_count == b.zebra_component_min_count &&
               a.zebra_slow_aim_speed == b.zebra_slow_aim_speed &&
               a.zebra_slow_hold_ms == b.zebra_slow_hold_ms &&
               a.zebra_far_slow_enable == b.zebra_far_slow_enable &&
               a.motor_aim_speed == b.motor_aim_speed &&
               a.pid_recover_delay_ms == b.pid_recover_delay_ms &&
               a.motor_pid_clear_on_stop_enable == b.motor_pid_clear_on_stop_enable &&
               a.emergency_coast_ms == b.emergency_coast_ms &&
               a.emergency_brake_lock_ms == b.emergency_brake_lock_ms &&
               a.emergency_brake_pwm == b.emergency_brake_pwm &&
               a.emergency_brake_time_ms == b.emergency_brake_time_ms &&
               a.emergency_reverse_enable == b.emergency_reverse_enable &&
               a.emergency_reverse_pwm == b.emergency_reverse_pwm &&
               a.emergency_reverse_turns == b.emergency_reverse_turns &&
               a.emergency_reverse_timeout_ms == b.emergency_reverse_timeout_ms &&
               a.zebra_post_normal_run_ms == b.zebra_post_normal_run_ms &&
               a.zebra_post_slow_ms == b.zebra_post_slow_ms &&
               a.zebra_post_slow_aim_speed == b.zebra_post_slow_aim_speed &&
               a.zebra_after_reverse_stop_ms == b.zebra_after_reverse_stop_ms &&
               a.zebra_post_right_enable == b.zebra_post_right_enable &&
               a.zebra_post_right_delay_ms == b.zebra_post_right_delay_ms &&
               a.zebra_post_right_ms == b.zebra_post_right_ms &&
               a.zebra_post_right_error == b.zebra_post_right_error &&
               a.zebra_reverse_speed == b.zebra_reverse_speed &&
               a.zebra_reverse_time_ms == b.zebra_reverse_time_ms &&
               a.traffic_red_enable == b.traffic_red_enable &&
               a.traffic_red_debug_enable == b.traffic_red_debug_enable &&
               a.traffic_red_debug_interval_ms == b.traffic_red_debug_interval_ms &&
               a.traffic_cut_top_ratio == b.traffic_cut_top_ratio &&
               a.traffic_cut_left_ratio == b.traffic_cut_left_ratio &&
               a.traffic_cut_right_ratio == b.traffic_cut_right_ratio &&
               a.traffic_roi_x_start == b.traffic_roi_x_start &&
               a.traffic_roi_x_end == b.traffic_roi_x_end &&
               a.traffic_roi_y_start == b.traffic_roi_y_start &&
               a.traffic_roi_y_end == b.traffic_roi_y_end &&
               a.traffic_dual_roi_enable == b.traffic_dual_roi_enable &&
               a.traffic_lower_roi_y_start == b.traffic_lower_roi_y_start &&
               a.traffic_lower_roi_y_end == b.traffic_lower_roi_y_end &&
               a.traffic_red_confirm_frames == b.traffic_red_confirm_frames &&
               a.red1_h_min == b.red1_h_min &&
               a.red1_h_max == b.red1_h_max &&
               a.red2_h_min == b.red2_h_min &&
               a.red2_h_max == b.red2_h_max &&
               a.red_s_min == b.red_s_min &&
               a.red_v_min == b.red_v_min &&
               a.red_circle_min == b.red_circle_min &&
               a.red_area_min == b.red_area_min &&
               a.red_area_max_ratio == b.red_area_max_ratio &&
               a.red_aspect_min == b.red_aspect_min &&
               a.red_aspect_max == b.red_aspect_max &&
               a.green_h_min == b.green_h_min &&
               a.green_h_max == b.green_h_max &&
               a.green_s_min == b.green_s_min &&
               a.green_v_min == b.green_v_min &&
               a.green_pixel_min_ratio == b.green_pixel_min_ratio &&
               a.green_pixel_min_absolute == b.green_pixel_min_absolute &&
               a.red_ignore_after_green_ms == b.red_ignore_after_green_ms &&
               a.traffic_morph_kernel_size == b.traffic_morph_kernel_size &&
               a.blue_cone_disappear_ratio == b.blue_cone_disappear_ratio &&
               a.blue_return_bias == b.blue_return_bias &&
               a.blue_return_frames == b.blue_return_frames &&
               a.blue_obstacle_aim_speed == b.blue_obstacle_aim_speed &&
               a.blue_min_active_frames == b.blue_min_active_frames &&
               a.blue_lost_frames_trigger == b.blue_lost_frames_trigger &&
               a.blue_cone_min_area_ratio == b.blue_cone_min_area_ratio &&
               a.blue_cone_min_area_absolute == b.blue_cone_min_area_absolute &&
               a.blue_board_leave_ratio == b.blue_board_leave_ratio &&
               a.blue_board_black_min_area_ratio == b.blue_board_black_min_area_ratio &&
               a.blue_board_decrease_frames == b.blue_board_decrease_frames &&
               a.blue_board_roi_y_ratio_end == b.blue_board_roi_y_ratio_end &&
               a.blue_board_roi_x_ratio_width == b.blue_board_roi_x_ratio_width &&
               a.blue_board_black_v_max == b.blue_board_black_v_max &&
               a.blue_board_black_s_max == b.blue_board_black_s_max &&
               a.blue_board_monitor_frames == b.blue_board_monitor_frames &&
               a.blue_curve_run_after_leave_ms == b.blue_curve_run_after_leave_ms &&
               a.blue_board_primary_avoid_ms == b.blue_board_primary_avoid_ms &&
               a.blue_board_counter_turn_ms == b.blue_board_counter_turn_ms &&
               a.blue_board_counter_turn_error == b.blue_board_counter_turn_error &&
               a.blue_board_slow_hold_after_ms == b.blue_board_slow_hold_after_ms &&
               a.blue_board_enable == b.blue_board_enable &&
               a.blue_board_expand_after_zebra_ms == b.blue_board_expand_after_zebra_ms &&
               a.blue_board_normal_top_crop_ratio == b.blue_board_normal_top_crop_ratio &&
               a.blue_board_post_zebra_top_crop_ratio == b.blue_board_post_zebra_top_crop_ratio &&
               a.blue_board_top_min_area_ratio == b.blue_board_top_min_area_ratio &&
               a.blue_board_confirm_frames == b.blue_board_confirm_frames &&
               a.blue_board_primary_avoid_error == b.blue_board_primary_avoid_error &&
               a.blue_board_line_expand_enable == b.blue_board_line_expand_enable &&
               a.blue_board_line_expand_delay_after_zebra_resume_ms == b.blue_board_line_expand_delay_after_zebra_resume_ms &&
               a.blue_board_line_expand_duration_ms == b.blue_board_line_expand_duration_ms &&
               a.blue_board_line_normal_top_crop_ratio == b.blue_board_line_normal_top_crop_ratio &&
               a.blue_board_line_post_zebra_top_crop_ratio == b.blue_board_line_post_zebra_top_crop_ratio &&
               a.blue_board_line_post_expand_slow_enable == b.blue_board_line_post_expand_slow_enable &&
               a.blue_board_line_post_expand_slow_aim_speed == b.blue_board_line_post_expand_slow_aim_speed &&
               a.blue_board_line_post_expand_slow_duration_ms == b.blue_board_line_post_expand_slow_duration_ms &&
               a.front_black_enable == b.front_black_enable &&
               a.front_black_roi_y_start == b.front_black_roi_y_start &&
               a.front_black_roi_y_end == b.front_black_roi_y_end &&
               a.front_black_area_ratio_min == b.front_black_area_ratio_min &&
               a.front_black_side_ratio_min == b.front_black_side_ratio_min &&
               a.front_black_confirm_frames == b.front_black_confirm_frames &&
               a.front_black_primary_avoid_ms == b.front_black_primary_avoid_ms &&
               a.front_black_primary_avoid_error == b.front_black_primary_avoid_error &&
               a.front_black_counter_turn_ms == b.front_black_counter_turn_ms &&
               a.front_black_counter_turn_error == b.front_black_counter_turn_error &&
               a.front_black_slow_hold_after_ms == b.front_black_slow_hold_after_ms &&
               a.front_black_aim_speed == b.front_black_aim_speed;
    }

    void RuntimeConfig::printParams(const char *prefix, const RuntimeImageParams &params)
    {
        char cwd[512] = {0};
        const char *cwdText = getcwd(cwd, sizeof(cwd)) == nullptr ? "?" : cwd;
        std::cout << prefix
                  << " cwd=" << cwdText
                  << " file=./car_params/image_params.txt"
                  << " zebra_far_cycle=" << params.zebra_far_cycle_enable
                  << " far_odd=" << params.zebra_far_y_ratio
                  << " far_even=" << params.zebra_far_second_y_ratio
                  << " traffic_red_enable=" << params.traffic_red_enable
                  << " dual_roi=" << params.traffic_dual_roi_enable
                  << " debug=" << params.traffic_red_debug_enable
                  << " post_expand_slow=" << params.blue_board_line_post_expand_slow_enable
                  << "@" << params.blue_board_line_post_expand_slow_aim_speed
                  << " for_ms=" << params.blue_board_line_post_expand_slow_duration_ms
                  << std::endl;
    }

    void RuntimeConfig::ensureConfigFileExists()
    {
        /*
         * 只在程序启动后的第一次配置检查时执行模板创建/升级。
         * 这样新 zip 第一次运行可以覆盖旧模板，但你运行中现场调参时不会被反复覆盖。
         */
        static bool checked_once = false;
        if (checked_once)
        {
            return;
        }
        checked_once = true;

        mkdir(CONFIG_DIR, 0755);

        std::ifstream input(CONFIG_FILE);
        bool fileExists = input.good();
        input.close();

        if (!fileExists)
        {
            if (writeDefaultConfigFile())
            {
                /* quiet: default config created */
            }
            return;
        }

        int existingVersion = readExistingConfigVersion();
        if (existingVersion < CONFIG_TEMPLATE_VERSION)
        {
            backupOldConfigFile();
            if (writeDefaultConfigFile())
            {
/* quiet: config template upgraded */
            }
            return;
        }
    }

    bool RuntimeConfig::loadFromFile(bool printWhenChanged)
    {
        ensureConfigFileExists();

        RuntimeImageParams newParams = params_;

        std::ifstream input(CONFIG_FILE);
        if (!input.good())
        {
            /* quiet: failed to open config */
            return false;
        }

        std::string line;
        while (std::getline(input, line))
        {
            size_t commentPos = line.find('#');
            if (commentPos != std::string::npos)
            {
                line = line.substr(0, commentPos);
            }

            line = trim(line);
            if (line.empty())
            {
                continue;
            }

            size_t eqPos = line.find('=');
            if (eqPos == std::string::npos)
            {
                continue;
            }

            std::string key = trim(line.substr(0, eqPos));
            std::string value = trim(line.substr(eqPos + 1));

            try
            {
                if (key == "center_pratio")
                {
                    newParams.center_pratio = clampDouble(std::stod(value), 0.20, 0.80);
                }
                else if (key == "zebra_region_jump_min_count")
                {
                    newParams.zebra_jump_min_count = clampInt(std::stoi(value), 3, 12);
                }
                else if (key == "zebra_ignore_after_detect_ms")
                {
                    newParams.zebra_ignore_after_detect_ms = clampInt(std::stoi(value), 0, 30000);
                }
                else if (key == "zebra_region_far_center_y_ratio")
                {
                    /* 第1、3、5...次人行横道的远区中心高度。 */
                    newParams.zebra_far_y_ratio = clampDouble(std::stod(value), 0.05, 0.95);
                }
                else if (key == "zebra_region_far_cycle_enable")
                {
                    /* 0=永远用第一高度；1=第1/3...与第2/4...交替。 */
                    newParams.zebra_far_cycle_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "zebra_region_far_second_center_y_ratio")
                {
                    /* 第2、4、6...次人行横道的远区中心高度。 */
                    newParams.zebra_far_second_y_ratio = clampDouble(std::stod(value), 0.05, 0.95);
                }
                else if (key == "zebra_region_near_center_y_ratio")
                {
                    newParams.zebra_near_y_ratio = clampDouble(std::stod(value), 0.05, 0.95);
                }
                else if (key == "zebra_region_gap_ratio_max")
                {
                    newParams.zebra_gap_ratio_max = clampDouble(std::stod(value), 1.0, 10.0);
                }
                else if (key == "zebra_region_half_height_ratio")
                {
                    newParams.zebra_region_half_height_ratio = clampDouble(std::stod(value), 0.01, 0.20);
                }
                else if (key == "zebra_region_sample_rows")
                {
                    newParams.zebra_region_sample_rows = clampInt(std::stoi(value), 3, 21);
                }
                else if (key == "zebra_region_min_hit_rows")
                {
                    newParams.zebra_region_min_hit_rows = clampInt(std::stoi(value), 1, 21);
                }
                else if (key == "zebra_region_white_area_ratio_min")
                {
                    newParams.zebra_region_white_area_ratio_min = clampDouble(std::stod(value), 0.01, 0.95);
                }
                else if (key == "zebra_region_white_area_ratio_max")
                {
                    newParams.zebra_region_white_area_ratio_max = clampDouble(std::stod(value), 0.05, 0.99);
                }
                else if (key == "zebra_component_min_area_ratio" || key == "zebra_region_component_min_area_ratio")
                {
                    newParams.zebra_component_min_area_ratio = clampDouble(std::stod(value), 0.001, 0.30);
                }
                else if (key == "zebra_component_min_height_ratio" || key == "zebra_region_component_min_height_ratio")
                {
                    newParams.zebra_component_min_height_ratio = clampDouble(std::stod(value), 0.05, 1.00);
                }
                else if (key == "zebra_component_min_count" || key == "zebra_region_component_min_count")
                {
                    newParams.zebra_component_min_count = clampInt(std::stoi(value), 1, 12);
                }
                else if (key == "zebra_far_slow_enable")
                {
                    newParams.zebra_far_slow_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "zebra_slow_aim_speed")
                {
                    newParams.zebra_slow_aim_speed = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "zebra_slow_hold_ms")
                {
                    newParams.zebra_slow_hold_ms = clampInt(std::stoi(value), 0, 5000);
                }
                else if (key == "motor_aim_speed")
                {
                    newParams.motor_aim_speed = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "pid_recover_delay_ms")
                {
                    newParams.pid_recover_delay_ms = clampInt(std::stoi(value), 0, 2000);
                }
                else if (key == "motor_pid_clear_on_stop_enable")
                {
                    newParams.motor_pid_clear_on_stop_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "emergency_coast_ms")
                {
                    newParams.emergency_coast_ms = clampInt(std::stoi(value), 0, 1000);
                }
                else if (key == "emergency_brake_lock_ms")
                {
                    newParams.emergency_brake_lock_ms = clampInt(std::stoi(value), 0, 3000);
                }
                else if (key == "emergency_brake_pwm")
                {
                    /* 兼容旧配置，当前安全停车版本不再使用反向制动。 */
                    newParams.emergency_brake_pwm = clampInt(std::stoi(value), -30000, 30000);
                }
                else if (key == "emergency_brake_time_ms")
                {
                    /* 兼容旧配置。 */
                    newParams.emergency_brake_time_ms = clampInt(std::stoi(value), 0, 2000);
                }
                else if (key == "emergency_reverse_enable")
                {
                    newParams.emergency_reverse_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "emergency_reverse_pwm")
                {
                    newParams.emergency_reverse_pwm = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "emergency_reverse_turns")
                {
                    newParams.emergency_reverse_turns = clampDouble(std::stod(value), 0.0, 5.0);
                }
                else if (key == "emergency_reverse_timeout_ms")
                {
                    newParams.emergency_reverse_timeout_ms = clampInt(std::stoi(value), 100, 5000);
                }
                else if (key == "zebra_post_normal_run_ms")
                {
                    newParams.zebra_post_normal_run_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "zebra_post_slow_ms")
                {
                    newParams.zebra_post_slow_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "zebra_post_slow_aim_speed")
                {
                    newParams.zebra_post_slow_aim_speed = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "zebra_reverse_speed")
                {
                    /* 兼容旧配置，当前人行横道急停不再使用这个值。 */
                    newParams.zebra_reverse_speed = clampDouble(std::stod(value), -30.0, 30.0);
                }
                else if (key == "zebra_reverse_time_ms")
                {
                    /* 兼容旧配置，当前人行横道急停不再使用这个值。 */
                    newParams.zebra_reverse_time_ms = clampInt(std::stoi(value), 100, 5000);
                }
                else if (key == "zebra_after_reverse_stop_ms")
                {
                    newParams.zebra_after_reverse_stop_ms = clampInt(std::stoi(value), 0, 10000);
                }
                else if (key == "zebra_post_right_enable")
                {
                    newParams.zebra_post_right_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "zebra_post_right_delay_ms")
                {
                    newParams.zebra_post_right_delay_ms = clampInt(std::stoi(value), 0, 5000);
                }
                else if (key == "zebra_post_right_ms")
                {
                    newParams.zebra_post_right_ms = clampInt(std::stoi(value), 0, 5000);
                }
                else if (key == "zebra_post_right_error")
                {
                    newParams.zebra_post_right_error = clampDouble(std::stod(value), 0.0, 25.0);
                }
                else if (key == "traffic_red_enable")
                {
                    newParams.traffic_red_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "traffic_red_debug_enable")
                {
                    newParams.traffic_red_debug_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "traffic_red_debug_interval_ms")
                {
                    newParams.traffic_red_debug_interval_ms = clampInt(std::stoi(value), 200, 5000);
                }
                else if (key == "traffic_red_view_top_crop_ratio")
                {
                    newParams.traffic_cut_top_ratio = clampDouble(std::stod(value), 0.0, 0.50);
                }
                else if (key == "traffic_red_view_left_crop_ratio")
                {
                    newParams.traffic_cut_left_ratio = clampDouble(std::stod(value), 0.0, 0.45);
                }
                else if (key == "traffic_red_view_right_crop_ratio")
                {
                    newParams.traffic_cut_right_ratio = clampDouble(std::stod(value), 0.0, 0.45);
                }
                else if (key == "traffic_red_upper_roi_x_start")
                {
                    newParams.traffic_roi_x_start = clampDouble(std::stod(value), 0.0, 0.95);
                }
                else if (key == "traffic_red_upper_roi_x_end")
                {
                    newParams.traffic_roi_x_end = clampDouble(std::stod(value), 0.05, 1.0);
                }
                else if (key == "traffic_red_upper_roi_y_start")
                {
                    newParams.traffic_roi_y_start = clampDouble(std::stod(value), 0.0, 0.95);
                }
                else if (key == "traffic_red_upper_roi_y_end")
                {
                    newParams.traffic_roi_y_end = clampDouble(std::stod(value), 0.05, 1.0);
                }
                else if (key == "traffic_red_dual_roi_enable")
                {
                    newParams.traffic_dual_roi_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "traffic_red_lower_roi_y_start")
                {
                    newParams.traffic_lower_roi_y_start = clampDouble(std::stod(value), 0.00, 0.95);
                }
                else if (key == "traffic_red_lower_roi_y_end")
                {
                    newParams.traffic_lower_roi_y_end = clampDouble(std::stod(value), 0.05, 1.00);
                }
                else if (key == "traffic_red_confirm_frames")
                {
                    newParams.traffic_red_confirm_frames = clampInt(std::stoi(value), 1, 8);
                }
                else if (key == "traffic_red_h1_min")
                {
                    newParams.red1_h_min = clampInt(std::stoi(value), 0, 179);
                }
                else if (key == "traffic_red_h1_max")
                {
                    newParams.red1_h_max = clampInt(std::stoi(value), 0, 179);
                }
                else if (key == "traffic_red_h2_min")
                {
                    newParams.red2_h_min = clampInt(std::stoi(value), 0, 179);
                }
                else if (key == "traffic_red_h2_max")
                {
                    newParams.red2_h_max = clampInt(std::stoi(value), 0, 179);
                }
                else if (key == "traffic_red_s_min")
                {
                    newParams.red_s_min = clampInt(std::stoi(value), 0, 255);
                }
                else if (key == "traffic_red_v_min")
                {
                    newParams.red_v_min = clampInt(std::stoi(value), 0, 255);
                }
                else if (key == "traffic_red_min_circle")
                {
                    newParams.red_circle_min = clampDouble(std::stod(value), 0.05, 1.0);
                }
                else if (key == "traffic_red_min_area")
                {
                    newParams.red_area_min = clampDouble(std::stod(value), 1.0, 2000.0);
                }
                else if (key == "traffic_red_max_area_ratio")
                {
                    newParams.red_area_max_ratio = clampDouble(std::stod(value), 0.001, 1.0);
                }
                else if (key == "traffic_red_aspect_min")
                {
                    newParams.red_aspect_min = clampDouble(std::stod(value), 0.05, 10.0);
                }
                else if (key == "traffic_red_aspect_max")
                {
                    newParams.red_aspect_max = clampDouble(std::stod(value), 0.05, 10.0);
                }
                else if (key == "green_h_min")
                {
                    newParams.green_h_min = clampInt(std::stoi(value), 0, 179);
                }
                else if (key == "green_h_max")
                {
                    newParams.green_h_max = clampInt(std::stoi(value), 0, 179);
                }
                else if (key == "green_s_min")
                {
                    newParams.green_s_min = clampInt(std::stoi(value), 0, 255);
                }
                else if (key == "green_v_min")
                {
                    newParams.green_v_min = clampInt(std::stoi(value), 0, 255);
                }
                else if (key == "green_pixel_min_ratio")
                {
                    newParams.green_pixel_min_ratio = clampDouble(std::stod(value), 0.0, 0.20);
                }
                else if (key == "green_pixel_min_absolute")
                {
                    newParams.green_pixel_min_absolute = clampInt(std::stoi(value), 1, 5000);
                }
                else if (key == "red_ignore_after_green_ms")
                {
                    newParams.red_ignore_after_green_ms = clampInt(std::stoi(value), 0, 30000);
                }
                else if (key == "traffic_red_morph_kernel_size")
                {
                    newParams.traffic_morph_kernel_size = clampInt(std::stoi(value), 1, 15);
                }
                else if (key == "blue_cone_disappear_ratio")
                {
                    newParams.blue_cone_disappear_ratio = clampDouble(std::stod(value), 0.05, 0.95);
                }
                else if (key == "blue_return_bias")
                {
                    newParams.blue_return_bias = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "blue_return_frames")
                {
                    newParams.blue_return_frames = clampInt(std::stoi(value), 1, 300);
                }
                else if (key == "blue_obstacle_aim_speed")
                {
                    newParams.blue_obstacle_aim_speed = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "blue_return_aim_speed")
                {
                    /* 兼容旧配置名：旧配置里写 blue_return_aim_speed 时，也作为蓝色全流程低速读取。 */
                    newParams.blue_obstacle_aim_speed = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "blue_min_active_frames")
                {
                    newParams.blue_min_active_frames = clampInt(std::stoi(value), 1, 100);
                }
                else if (key == "blue_lost_frames_trigger")
                {
                    newParams.blue_lost_frames_trigger = clampInt(std::stoi(value), 1, 50);
                }
                else if (key == "blue_cone_min_area_ratio")
                {
                    newParams.blue_cone_min_area_ratio = clampDouble(std::stod(value), 0.0001, 0.05);
                }
                else if (key == "blue_cone_min_area_absolute")
                {
                    newParams.blue_cone_min_area_absolute = clampDouble(std::stod(value), 1.0, 500.0);
                }
                else if (key == "blue_board_leave_ratio")
                {
                    newParams.blue_board_leave_ratio = clampDouble(std::stod(value), 0.05, 0.95);
                }
                else if (key == "blue_board_black_min_area_ratio")
                {
                    newParams.blue_board_black_min_area_ratio = clampDouble(std::stod(value), 0.001, 0.50);
                }
                else if (key == "blue_board_decrease_frames")
                {
                    newParams.blue_board_decrease_frames = clampInt(std::stoi(value), 1, 30);
                }
                else if (key == "blue_board_roi_y_ratio_end")
                {
                    newParams.blue_board_roi_y_ratio_end = clampDouble(std::stod(value), 0.10, 1.00);
                }
                else if (key == "blue_board_roi_x_ratio_width")
                {
                    newParams.blue_board_roi_x_ratio_width = clampDouble(std::stod(value), 0.10, 1.00);
                }
                else if (key == "blue_board_black_v_max")
                {
                    newParams.blue_board_black_v_max = clampInt(std::stoi(value), 0, 255);
                }
                else if (key == "blue_board_black_s_max")
                {
                    newParams.blue_board_black_s_max = clampInt(std::stoi(value), 0, 255);
                }
                else if (key == "blue_board_monitor_frames")
                {
                    newParams.blue_board_monitor_frames = clampInt(std::stoi(value), 1, 1000);
                }
                else if (key == "blue_curve_run_after_leave_ms")
                {
                    newParams.blue_curve_run_after_leave_ms = clampInt(std::stoi(value), 0, 10000);
                }
                else if (key == "blue_board_primary_avoid_ms")
                {
                    newParams.blue_board_primary_avoid_ms = clampInt(std::stoi(value), 0, 5000);
                }
                else if (key == "blue_board_counter_turn_ms")
                {
                    newParams.blue_board_counter_turn_ms = clampInt(std::stoi(value), 0, 6000);
                }
                else if (key == "blue_board_counter_turn_error")
                {
                    newParams.blue_board_counter_turn_error = clampDouble(std::stod(value), 0.0, 35.0);
                }
                else if (key == "blue_board_slow_hold_after_ms")
                {
                    newParams.blue_board_slow_hold_after_ms = clampInt(std::stoi(value), 0, 10000);
                }
                else if (key == "blue_board_enable")
                {
                    newParams.blue_board_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_board_expand_after_zebra_ms")
                {
                    newParams.blue_board_expand_after_zebra_ms = clampInt(std::stoi(value), 0, 15000);
                }
                else if (key == "blue_board_normal_top_crop_ratio")
                {
                    newParams.blue_board_normal_top_crop_ratio = clampDouble(std::stod(value), 0.00, 0.45);
                }
                else if (key == "blue_board_post_zebra_top_crop_ratio")
                {
                    newParams.blue_board_post_zebra_top_crop_ratio = clampDouble(std::stod(value), 0.00, 0.45);
                }
                else if (key == "blue_board_top_min_area_ratio")
                {
                    newParams.blue_board_top_min_area_ratio = clampDouble(std::stod(value), 0.005, 0.60);
                }
                else if (key == "blue_board_confirm_frames")
                {
                    newParams.blue_board_confirm_frames = clampInt(std::stoi(value), 1, 10);
                }
                else if (key == "blue_board_primary_avoid_error")
                {
                    newParams.blue_board_primary_avoid_error = clampDouble(std::stod(value), 0.0, 35.0);
                }
                else if (key == "blue_board_line_expand_enable")
                {
                    newParams.blue_board_line_expand_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_board_line_expand_delay_after_zebra_resume_ms")
                {
                    newParams.blue_board_line_expand_delay_after_zebra_resume_ms = clampInt(std::stoi(value), 0, 15000);
                }
                else if (key == "blue_board_line_expand_duration_ms")
                {
                    newParams.blue_board_line_expand_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_board_line_normal_top_crop_ratio")
                {
                    newParams.blue_board_line_normal_top_crop_ratio = clampDouble(std::stod(value), 0.00, 0.45);
                }
                else if (key == "blue_board_line_post_zebra_top_crop_ratio")
                {
                    newParams.blue_board_line_post_zebra_top_crop_ratio = clampDouble(std::stod(value), 0.00, 0.45);
                }
                else if (key == "blue_board_line_post_expand_slow_enable")
                {
                    /* 0=关闭“动态裁剪结束后”的低速窗口；1=启用。 */
                    newParams.blue_board_line_post_expand_slow_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_board_line_post_expand_slow_aim_speed")
                {
                    /* 动态裁剪结束后低速的目标速度，避免配置为 0 导致车辆在赛道内停住。 */
                    newParams.blue_board_line_post_expand_slow_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_board_line_post_expand_slow_duration_ms")
                {
                    /* 低速从动态裁剪结束、恢复正常上裁剪后开始计时，最长 20 秒。 */
                    newParams.blue_board_line_post_expand_slow_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "front_black_enable")
                {
                    newParams.front_black_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "front_black_roi_y_start")
                {
                    newParams.front_black_roi_y_start = clampDouble(std::stod(value), 0.00, 0.95);
                }
                else if (key == "front_black_roi_y_end")
                {
                    newParams.front_black_roi_y_end = clampDouble(std::stod(value), 0.05, 1.00);
                }
                else if (key == "front_black_area_ratio_min")
                {
                    newParams.front_black_area_ratio_min = clampDouble(std::stod(value), 0.01, 0.80);
                }
                else if (key == "front_black_side_ratio_min")
                {
                    newParams.front_black_side_ratio_min = clampDouble(std::stod(value), 1.00, 5.00);
                }
                else if (key == "front_black_confirm_frames")
                {
                    newParams.front_black_confirm_frames = clampInt(std::stoi(value), 1, 20);
                }
                else if (key == "front_black_primary_avoid_ms")
                {
                    newParams.front_black_primary_avoid_ms = clampInt(std::stoi(value), 0, 8000);
                }
                else if (key == "front_black_primary_avoid_error")
                {
                    newParams.front_black_primary_avoid_error = clampDouble(std::stod(value), 0.0, 35.0);
                }
                else if (key == "front_black_counter_turn_ms")
                {
                    newParams.front_black_counter_turn_ms = clampInt(std::stoi(value), 0, 8000);
                }
                else if (key == "front_black_counter_turn_error")
                {
                    newParams.front_black_counter_turn_error = clampDouble(std::stod(value), 0.0, 35.0);
                }
                else if (key == "front_black_slow_hold_after_ms")
                {
                    newParams.front_black_slow_hold_after_ms = clampInt(std::stoi(value), 0, 15000);
                }
                else if (key == "front_black_aim_speed")
                {
                    newParams.front_black_aim_speed = clampDouble(std::stod(value), 0.0, 20.0);
                }
            }
            catch (...)
            {
                /* quiet: invalid runtime config value */
            }
        }

        bool changed = !isSameParams(params_, newParams);
        params_ = newParams;

        if (printWhenChanged && changed)
        {
            printParams("[RuntimeConfig] updated", params_);
        }

        return changed;
    }

    void RuntimeConfig::reloadIfNeeded()
    {
        std::lock_guard<std::mutex> lock(mutex_);

        auto now = std::chrono::steady_clock::now();

        if (!initialized_)
        {
            ensureConfigFileExists();
            loadFromFile(false);
            printParams("[RuntimeConfig] loaded", params_);
            last_load_time_ = now;
            initialized_ = true;
            return;
        }

        auto elapsedMs = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_load_time_).count();
        if (elapsedMs >= 500)
        {
            loadFromFile(true);
            last_load_time_ = now;
        }
    }

    RuntimeImageParams RuntimeConfig::getImageParams()
    {
        reloadIfNeeded();
        std::lock_guard<std::mutex> lock(mutex_);
        return params_;
    }

#define RUNTIME_DOUBLE_GETTER(NAME, FIELD) \
    double RuntimeConfig::NAME()          \
    {                                    \
        reloadIfNeeded();                \
        std::lock_guard<std::mutex> lock(mutex_); \
        return params_.FIELD;            \
    }

#define RUNTIME_INT_GETTER(NAME, FIELD)   \
    int RuntimeConfig::NAME()             \
    {                                    \
        reloadIfNeeded();                \
        std::lock_guard<std::mutex> lock(mutex_); \
        return params_.FIELD;            \
    }

    RUNTIME_DOUBLE_GETTER(getCenterPratio, center_pratio)
    RUNTIME_DOUBLE_GETTER(getZebraCheckYRatio, zebra_check_y_ratio)
    RUNTIME_INT_GETTER(getZebraCheckRowCount, zebra_check_row_count)
    RUNTIME_INT_GETTER(getZebraJumpMinCount, zebra_jump_min_count)
    RUNTIME_INT_GETTER(getZebraJumpBlockDiv, zebra_jump_block_div)
    RUNTIME_INT_GETTER(getZebraIgnoreAfterDetectMs, zebra_ignore_after_detect_ms)
    RUNTIME_DOUBLE_GETTER(getZebraFarYRatio, zebra_far_y_ratio)
    RUNTIME_INT_GETTER(getZebraRegionFarCycleEnable, zebra_far_cycle_enable)
    RUNTIME_DOUBLE_GETTER(getZebraRegionFarSecondCenterYRatio, zebra_far_second_y_ratio)
    RUNTIME_DOUBLE_GETTER(getZebraNearYRatio, zebra_near_y_ratio)
    RUNTIME_DOUBLE_GETTER(getZebraGapRatioMax, zebra_gap_ratio_max)
    RUNTIME_DOUBLE_GETTER(getZebraRegionHalfHeightRatio, zebra_region_half_height_ratio)
    RUNTIME_INT_GETTER(getZebraRegionSampleRows, zebra_region_sample_rows)
    RUNTIME_INT_GETTER(getZebraRegionMinHitRows, zebra_region_min_hit_rows)
    RUNTIME_DOUBLE_GETTER(getZebraRegionWhiteAreaRatioMin, zebra_region_white_area_ratio_min)
    RUNTIME_DOUBLE_GETTER(getZebraRegionWhiteAreaRatioMax, zebra_region_white_area_ratio_max)
    RUNTIME_DOUBLE_GETTER(getZebraComponentMinAreaRatio, zebra_component_min_area_ratio)
    RUNTIME_DOUBLE_GETTER(getZebraComponentMinHeightRatio, zebra_component_min_height_ratio)
    RUNTIME_INT_GETTER(getZebraComponentMinCount, zebra_component_min_count)
    RUNTIME_DOUBLE_GETTER(getZebraSlowAimSpeed, zebra_slow_aim_speed)
    RUNTIME_INT_GETTER(getZebraSlowHoldMs, zebra_slow_hold_ms)
    RUNTIME_INT_GETTER(getZebraFarSlowEnable, zebra_far_slow_enable)

    RUNTIME_DOUBLE_GETTER(getMotorAimSpeed, motor_aim_speed)
    RUNTIME_INT_GETTER(getEmergencyBrakePwm, emergency_brake_pwm)
    RUNTIME_INT_GETTER(getEmergencyBrakeTimeMs, emergency_brake_time_ms)
    RUNTIME_INT_GETTER(getEmergencyReverseEnable, emergency_reverse_enable)
    RUNTIME_INT_GETTER(getEmergencyReversePwm, emergency_reverse_pwm)
    RUNTIME_DOUBLE_GETTER(getEmergencyReverseTurns, emergency_reverse_turns)
    RUNTIME_INT_GETTER(getEmergencyReverseTimeoutMs, emergency_reverse_timeout_ms)
    RUNTIME_INT_GETTER(getZebraPostNormalRunMs, zebra_post_normal_run_ms)
    RUNTIME_INT_GETTER(getZebraPostSlowMs, zebra_post_slow_ms)
    RUNTIME_DOUBLE_GETTER(getZebraPostSlowAimSpeed, zebra_post_slow_aim_speed)
    RUNTIME_INT_GETTER(getPidRecoverDelayMs, pid_recover_delay_ms)
    RUNTIME_INT_GETTER(getMotorPidClearOnStopEnable, motor_pid_clear_on_stop_enable)
    RUNTIME_INT_GETTER(getEmergencyCoastMs, emergency_coast_ms)
    RUNTIME_INT_GETTER(getEmergencyBrakeLockMs, emergency_brake_lock_ms)
    RUNTIME_INT_GETTER(getZebraAfterReverseStopMs, zebra_after_reverse_stop_ms)
    RUNTIME_INT_GETTER(getZebraPostRightEnable, zebra_post_right_enable)
    RUNTIME_INT_GETTER(getZebraPostRightDelayMs, zebra_post_right_delay_ms)
    RUNTIME_INT_GETTER(getZebraPostRightMs, zebra_post_right_ms)
    RUNTIME_DOUBLE_GETTER(getZebraPostRightError, zebra_post_right_error)
    RUNTIME_DOUBLE_GETTER(getZebraReverseSpeed, zebra_reverse_speed)
    RUNTIME_INT_GETTER(getZebraReverseTimeMs, zebra_reverse_time_ms)

    RUNTIME_INT_GETTER(getTrafficRedEnable, traffic_red_enable)
    RUNTIME_INT_GETTER(getTrafficRedDebugEnable, traffic_red_debug_enable)
    RUNTIME_INT_GETTER(getTrafficRedDebugIntervalMs, traffic_red_debug_interval_ms)
    RUNTIME_DOUBLE_GETTER(getTrafficCutTopRatio, traffic_cut_top_ratio)
    RUNTIME_DOUBLE_GETTER(getTrafficCutLeftRatio, traffic_cut_left_ratio)
    RUNTIME_DOUBLE_GETTER(getTrafficCutRightRatio, traffic_cut_right_ratio)
    RUNTIME_DOUBLE_GETTER(getTrafficRoiXStart, traffic_roi_x_start)
    RUNTIME_DOUBLE_GETTER(getTrafficRoiXEnd, traffic_roi_x_end)
    RUNTIME_DOUBLE_GETTER(getTrafficRoiYStart, traffic_roi_y_start)
    RUNTIME_DOUBLE_GETTER(getTrafficRoiYEnd, traffic_roi_y_end)
    RUNTIME_INT_GETTER(getTrafficDualRoiEnable, traffic_dual_roi_enable)
    RUNTIME_DOUBLE_GETTER(getTrafficLowerRoiYStart, traffic_lower_roi_y_start)
    RUNTIME_DOUBLE_GETTER(getTrafficLowerRoiYEnd, traffic_lower_roi_y_end)
    RUNTIME_INT_GETTER(getTrafficRedConfirmFrames, traffic_red_confirm_frames)

    RUNTIME_INT_GETTER(getRed1HMin, red1_h_min)
    RUNTIME_INT_GETTER(getRed1HMax, red1_h_max)
    RUNTIME_INT_GETTER(getRed2HMin, red2_h_min)
    RUNTIME_INT_GETTER(getRed2HMax, red2_h_max)
    RUNTIME_INT_GETTER(getRedSMin, red_s_min)
    RUNTIME_INT_GETTER(getRedVMin, red_v_min)
    RUNTIME_DOUBLE_GETTER(getRedCircleMin, red_circle_min)
    RUNTIME_DOUBLE_GETTER(getRedAreaMin, red_area_min)
    RUNTIME_DOUBLE_GETTER(getRedAreaMaxRatio, red_area_max_ratio)
    RUNTIME_DOUBLE_GETTER(getRedAspectMin, red_aspect_min)
    RUNTIME_DOUBLE_GETTER(getRedAspectMax, red_aspect_max)

    RUNTIME_INT_GETTER(getGreenHMin, green_h_min)
    RUNTIME_INT_GETTER(getGreenHMax, green_h_max)
    RUNTIME_INT_GETTER(getGreenSMin, green_s_min)
    RUNTIME_INT_GETTER(getGreenVMin, green_v_min)
    RUNTIME_DOUBLE_GETTER(getGreenPixelMinRatio, green_pixel_min_ratio)
    RUNTIME_INT_GETTER(getGreenPixelMinAbsolute, green_pixel_min_absolute)

    RUNTIME_INT_GETTER(getRedIgnoreAfterGreenMs, red_ignore_after_green_ms)
    RUNTIME_INT_GETTER(getTrafficMorphKernelSize, traffic_morph_kernel_size)

    RUNTIME_DOUBLE_GETTER(getBlueConeDisappearRatio, blue_cone_disappear_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueReturnBias, blue_return_bias)
    RUNTIME_INT_GETTER(getBlueReturnFrames, blue_return_frames)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleAimSpeed, blue_obstacle_aim_speed)
    RUNTIME_INT_GETTER(getBlueMinActiveFrames, blue_min_active_frames)
    RUNTIME_INT_GETTER(getBlueLostFramesTrigger, blue_lost_frames_trigger)
    RUNTIME_DOUBLE_GETTER(getBlueConeMinAreaRatio, blue_cone_min_area_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueConeMinAreaAbsolute, blue_cone_min_area_absolute)

    RUNTIME_DOUBLE_GETTER(getBlueBoardLeaveRatio, blue_board_leave_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueBoardBlackMinAreaRatio, blue_board_black_min_area_ratio)
    RUNTIME_INT_GETTER(getBlueBoardDecreaseFrames, blue_board_decrease_frames)
    RUNTIME_DOUBLE_GETTER(getBlueBoardRoiYRatioEnd, blue_board_roi_y_ratio_end)
    RUNTIME_DOUBLE_GETTER(getBlueBoardRoiXRatioWidth, blue_board_roi_x_ratio_width)
    RUNTIME_INT_GETTER(getBlueBoardBlackVMax, blue_board_black_v_max)
    RUNTIME_INT_GETTER(getBlueBoardBlackSMax, blue_board_black_s_max)
    RUNTIME_INT_GETTER(getBlueBoardMonitorFrames, blue_board_monitor_frames)
    RUNTIME_INT_GETTER(getBlueCurveRunAfterLeaveMs, blue_curve_run_after_leave_ms)
    RUNTIME_INT_GETTER(getBlueBoardPrimaryAvoidMs, blue_board_primary_avoid_ms)
    RUNTIME_INT_GETTER(getBlueBoardCounterTurnMs, blue_board_counter_turn_ms)
    RUNTIME_DOUBLE_GETTER(getBlueBoardCounterTurnError, blue_board_counter_turn_error)
    RUNTIME_INT_GETTER(getBlueBoardSlowHoldAfterMs, blue_board_slow_hold_after_ms)
    RUNTIME_INT_GETTER(getBlueBoardEnable, blue_board_enable)
    RUNTIME_INT_GETTER(getBlueBoardExpandAfterZebraMs, blue_board_expand_after_zebra_ms)
    RUNTIME_DOUBLE_GETTER(getBlueBoardNormalTopCropRatio, blue_board_normal_top_crop_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueBoardPostZebraTopCropRatio, blue_board_post_zebra_top_crop_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueBoardTopMinAreaRatio, blue_board_top_min_area_ratio)
    RUNTIME_INT_GETTER(getBlueBoardConfirmFrames, blue_board_confirm_frames)
    RUNTIME_DOUBLE_GETTER(getBlueBoardPrimaryAvoidError, blue_board_primary_avoid_error)
    RUNTIME_INT_GETTER(getBlueBoardLineExpandEnable, blue_board_line_expand_enable)
    RUNTIME_INT_GETTER(getBlueBoardLineExpandDelayAfterZebraResumeMs, blue_board_line_expand_delay_after_zebra_resume_ms)
    RUNTIME_INT_GETTER(getBlueBoardLineExpandDurationMs, blue_board_line_expand_duration_ms)
    RUNTIME_DOUBLE_GETTER(getBlueBoardLineNormalTopCropRatio, blue_board_line_normal_top_crop_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueBoardLinePostZebraTopCropRatio, blue_board_line_post_zebra_top_crop_ratio)
    RUNTIME_INT_GETTER(getBlueBoardLinePostExpandSlowEnable, blue_board_line_post_expand_slow_enable)
    RUNTIME_DOUBLE_GETTER(getBlueBoardLinePostExpandSlowAimSpeed, blue_board_line_post_expand_slow_aim_speed)
    RUNTIME_INT_GETTER(getBlueBoardLinePostExpandSlowDurationMs, blue_board_line_post_expand_slow_duration_ms)

    RUNTIME_INT_GETTER(getFrontBlackEnable, front_black_enable)
    RUNTIME_DOUBLE_GETTER(getFrontBlackRoiYStart, front_black_roi_y_start)
    RUNTIME_DOUBLE_GETTER(getFrontBlackRoiYEnd, front_black_roi_y_end)
    RUNTIME_DOUBLE_GETTER(getFrontBlackAreaRatioMin, front_black_area_ratio_min)
    RUNTIME_DOUBLE_GETTER(getFrontBlackSideRatioMin, front_black_side_ratio_min)
    RUNTIME_INT_GETTER(getFrontBlackConfirmFrames, front_black_confirm_frames)
    RUNTIME_INT_GETTER(getFrontBlackPrimaryAvoidMs, front_black_primary_avoid_ms)
    RUNTIME_DOUBLE_GETTER(getFrontBlackPrimaryAvoidError, front_black_primary_avoid_error)
    RUNTIME_INT_GETTER(getFrontBlackCounterTurnMs, front_black_counter_turn_ms)
    RUNTIME_DOUBLE_GETTER(getFrontBlackCounterTurnError, front_black_counter_turn_error)
    RUNTIME_INT_GETTER(getFrontBlackSlowHoldAfterMs, front_black_slow_hold_after_ms)
    RUNTIME_DOUBLE_GETTER(getFrontBlackAimSpeed, front_black_aim_speed)

#undef RUNTIME_DOUBLE_GETTER
#undef RUNTIME_INT_GETTER
}
