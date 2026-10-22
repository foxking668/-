#include "RuntimeConfig.hpp"
#include "Experiment/RealCarExperiment.hpp"
#include "Experiment/ExperimentConfigValidation.hpp"
#include <set>

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
    static const char *CONFIG_FILE = "./image_params.txt";

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

    static const int CONFIG_TEMPLATE_VERSION = 2026083101;

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

        std::ofstream output("./image_params.txt.bak", std::ios::binary);
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
# 小车运行参数配置文件（A3/A4 实车实验版，原车功能默认不变）。
config_version=2026083101
#
# 使用方法：
# 1. 首次使用本包：替换完整源码后重新编译一次。
# 2. 程序运行时约每 500ms 自动重读本文件；之后只改 image_params.txt 并保存。
# 3. 除“人行横道后暂停红灯识别、红灯放行后关闭新红灯识别、绿灯起步后强制左偏、绿灯起步后临时暂停红绿灯识别”新增内容外，其余数值保持你本次发送的参数值。
# 4. 方案1/方案2参数完全独立；你本次给出的数值保持不变。
#
# 重要：每一行都写成 key=value；以 # 开头的行只是注释。

# ============================================================
# A3/A4 实车实验（0=原车；正式实验时按说明修改）
# ============================================================
experiment_enable=0
experiment_method=4
# 0=A3/A4 (legacy); 1=M1/M2/M3/M4 comparison. Frozen until restart.
experiment_method_group=0
# Original camera frames, bounded asynchronous MJPG/AVI recording.
# Requires experiment_enable=1. Set 0 to keep original operation.
experiment_video_enable=0
# Maximum recording rate, 1..30; does not set camera/control rate.
experiment_video_fps=15
experiment_task_features_enable=1
experiment_log_enable=0
experiment_run_id=DISABLED

# ============================================================
# 一、基础巡线 / 正常速度（保持你的数值）
# ============================================================
center_pratio=0.50
error_gain=4.0
steer_offset_deg=-15
motor_aim_speed=4

# ============================================================
# 二、新区域型人行横道：跳变 + 区域面积 + 连通白条
# ============================================================
zebra_region_far_cycle_enable=1
zebra_region_far_center_y_ratio=0.3
zebra_region_far_second_center_y_ratio=0.24
zebra_region_near_center_y_ratio=0.25
zebra_region_jump_min_count=3
zebra_region_gap_ratio_max=2.0
zebra_region_half_height_ratio=0.06
zebra_region_sample_rows=9
zebra_region_min_hit_rows=3
zebra_region_white_area_ratio_min=0.15
zebra_region_white_area_ratio_max=0.75
zebra_region_component_min_area_ratio=0.010
zebra_region_component_min_height_ratio=0.32
zebra_region_component_min_count=3
zebra_far_slow_enable=0
zebra_slow_aim_speed=10
zebra_slow_hold_ms=1000
zebra_ignore_after_detect_ms=10000

# 红灯停车并由黄/绿灯放行、真正恢复行驶后，暂停人行横道识别7秒。
zebra_ignore_after_traffic_resume_ms=7000

zebra_after_reverse_stop_ms=2200
pid_recover_delay_ms=0
motor_pid_clear_on_stop_enable=0

# ============================================================
# 三、新双 ROI 红灯（保持你的数值，并保留实时控制参数）
# ============================================================
traffic_red_enable=1
traffic_red_debug_enable=1
traffic_red_debug_interval_ms=800
traffic_red_startup_ignore_ms=5000

# 最终确认人行横道后，多少毫秒内暂停确认“新的红灯”。
# 6200=暂停6.2秒；0=关闭。等待绿灯状态不受影响，避免无法放行。
traffic_red_ignore_after_zebra_detect_ms=6200

# 新增：真实红灯放行且电机真正重新起步后，立即暂停红绿灯检测多久。
# 5000=起步后暂停5秒；0=关闭。该时间结束后会自动恢复本轮红灯检测资格。
traffic_red_ignore_after_resume_ms=5000

# 真实红灯放行且电机真正重新起步后，经过多少毫秒停止确认后续“新的红灯”。
# 8000=起步8秒后关闭本轮后续新红灯识别；下一次确认人行横道时重新开启下一轮。
# 0=关闭该功能，之后仍可继续识别新的红灯。
traffic_red_disable_after_resume_ms=8000

# 红灯放行并真正恢复行驶后，延迟多久开始短暂低速。
traffic_red_after_detect_slow_delay_ms=9200
traffic_red_after_detect_slow_aim_speed=8
traffic_red_after_detect_slow_duration_ms=5000

# -------- 绿灯放行并真正起步后的独立强制左偏 --------
# 0=关闭；1=开启。
traffic_red_after_resume_left_enable=1
# 左偏阶段使用的中心比例。
traffic_red_after_resume_left_pratio=0.65
# 左偏持续时间，单位ms。
traffic_red_after_resume_left_duration_ms=300
# 左偏期间目标速度。
traffic_red_after_resume_left_aim_speed=6
# 至少保持的左转误差幅度；不是舵机物理角度。
traffic_red_after_resume_left_min_error=30

traffic_red_view_top_crop_ratio=0.00
traffic_red_view_left_crop_ratio=0.01
traffic_red_view_right_crop_ratio=0.10
traffic_red_upper_roi_x_start=0.05
traffic_red_upper_roi_x_end=0.95
traffic_red_upper_roi_y_start=0.20
traffic_red_upper_roi_y_end=0.56
traffic_red_dual_roi_enable=1
traffic_red_lower_roi_y_start=0.58
traffic_red_lower_roi_y_end=1.00
traffic_red_confirm_frames=1
traffic_red_h1_min=0
traffic_red_h1_max=12
traffic_red_h2_min=165
traffic_red_h2_max=179
traffic_red_s_min=60
traffic_red_v_min=55
traffic_red_min_area=10.0
traffic_red_max_area_ratio=0.05
traffic_red_min_circle=0.1
traffic_red_aspect_min=0.45
traffic_red_aspect_max=2.20
traffic_red_morph_kernel_size=1

# ============================================================
# 四、蓝色障碍路线总选择
# ============================================================
# 1=方案1：蓝色桶锥在右、挡板在左。
# 2=方案2：蓝色桶锥在左、挡板在右。
# 建议停车前选择好，不要在一段绕障动作执行中途切换。
blue_obstacle_route_profile=2

# ============================================================
# 五、方案1独立参数：桶锥在右、挡板在左
# 流程：正常运行 -> 强制左偏 -> 正常等待 -> 强制右偏 -> 指定速度巡线
# ============================================================
# 人行横道恢复后，先正常巡线多久。
blue_obstacle_profile1_normal_run_ms=1800

# 强制左偏阶段。
blue_obstacle_profile1_left_enable=1
blue_obstacle_profile1_left_pratio=0.65
blue_obstacle_profile1_left_duration_ms=500
blue_obstacle_profile1_left_aim_speed=1
blue_obstacle_profile1_left_min_error=30

# 左偏结束后等待，再强制右偏。
blue_obstacle_profile1_right_enable=1
blue_obstacle_profile1_right_delay_ms=200
blue_obstacle_profile1_right_duration_ms=700
blue_obstacle_profile1_right_min_error=58
blue_obstacle_profile1_right_aim_speed=3

# 右偏结束后的指定速度正常巡线。
blue_obstacle_profile1_post_run_enable=1
blue_obstacle_profile1_post_run_aim_speed=9.4
blue_obstacle_profile1_post_run_duration_ms=5500

# 方案1正常巡线顶部裁剪比例。
blue_obstacle_profile1_normal_top_crop_ratio=0.20

# ============================================================
# 六、方案2独立试调参数：桶锥在左、挡板在右
# ============================================================
# 方案2动作选择：
# 0=使用扩大视野；1=使用独立强制左偏。
blue_obstacle_profile2_action_mode=1

# 人行横道恢复后，方案2先正常巡线多久。
blue_obstacle_profile2_normal_run_ms=2300

# -------- 方案2模式1：独立强制左偏 --------
# 仅 blue_obstacle_route_profile=2 且 blue_obstacle_profile2_action_mode=1 时使用。
blue_obstacle_profile2_left_pratio=0.55
blue_obstacle_profile2_left_duration_ms=400
blue_obstacle_profile2_left_aim_speed=5
blue_obstacle_profile2_left_min_error=20

# -------- 方案2模式0：扩大视野 --------
# 仅 blue_obstacle_route_profile=2 且 blue_obstacle_profile2_action_mode=0 时使用。
blue_obstacle_profile2_expand_duration_ms=1000
blue_obstacle_profile2_expand_aim_speed=6.0
blue_obstacle_profile2_normal_top_crop_ratio=0.20
blue_obstacle_profile2_expanded_top_crop_ratio=0.01

# -------- 方案2动作结束后的指定速度正常巡线 --------
# 无论方案2选择强制左偏还是扩大视野，结束后都使用这一组。
blue_obstacle_profile2_post_run_enable=1
blue_obstacle_profile2_post_run_aim_speed=10
blue_obstacle_profile2_post_run_duration_ms=1200

# ============================================================
# 七、红灯 / 人行横道急停反转（保持你的数值）
# ============================================================
emergency_reverse_enable=1
emergency_reverse_left_motor_enable=0
emergency_reverse_right_motor_enable=1
emergency_coast_ms=50
emergency_reverse_pwm=22000
emergency_reverse_turns=2.5
emergency_reverse_timeout_ms=500
emergency_brake_lock_ms=800
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
        return a.experiment_enable == b.experiment_enable &&
               a.experiment_method == b.experiment_method &&
               a.experiment_method_group == b.experiment_method_group &&
               a.experiment_video_enable == b.experiment_video_enable &&
               a.experiment_video_fps == b.experiment_video_fps &&
               a.experiment_config_error == b.experiment_config_error &&
               a.experiment_task_features_enable == b.experiment_task_features_enable &&
               a.experiment_log_enable == b.experiment_log_enable &&
               a.experiment_run_id == b.experiment_run_id &&
               a.center_pratio == b.center_pratio &&
               a.error_gain == b.error_gain &&
               a.steer_offset_deg == b.steer_offset_deg &&
               a.zebra_post_center_bias_enable == b.zebra_post_center_bias_enable &&
               a.zebra_post_center_bias_pratio == b.zebra_post_center_bias_pratio &&
               a.zebra_post_center_bias_duration_ms == b.zebra_post_center_bias_duration_ms &&
               a.zebra_post_center_bias_aim_speed == b.zebra_post_center_bias_aim_speed &&
               a.zebra_post_center_bias_min_left_error == b.zebra_post_center_bias_min_left_error &&
               a.zebra_check_y_ratio == b.zebra_check_y_ratio &&
               a.zebra_check_row_count == b.zebra_check_row_count &&
               a.zebra_jump_min_count == b.zebra_jump_min_count &&
               a.zebra_jump_block_div == b.zebra_jump_block_div &&
               a.zebra_ignore_after_detect_ms == b.zebra_ignore_after_detect_ms &&
               a.zebra_ignore_after_traffic_resume_ms == b.zebra_ignore_after_traffic_resume_ms &&
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
               a.emergency_reverse_left_motor_enable == b.emergency_reverse_left_motor_enable &&
               a.emergency_reverse_right_motor_enable == b.emergency_reverse_right_motor_enable &&
               a.emergency_reverse_pwm == b.emergency_reverse_pwm &&
               a.emergency_reverse_turns == b.emergency_reverse_turns &&
               a.emergency_reverse_timeout_ms == b.emergency_reverse_timeout_ms &&
               a.zebra_post_normal_run_ms == b.zebra_post_normal_run_ms &&
               a.zebra_post_slow_ms == b.zebra_post_slow_ms &&
               a.zebra_post_slow_aim_speed == b.zebra_post_slow_aim_speed &&
               a.zebra_after_reverse_stop_ms == b.zebra_after_reverse_stop_ms &&
               a.blue_obstacle_route_profile == b.blue_obstacle_route_profile &&
               a.blue_obstacle_profile1_normal_run_ms == b.blue_obstacle_profile1_normal_run_ms &&
               a.blue_obstacle_profile1_left_enable == b.blue_obstacle_profile1_left_enable &&
               a.blue_obstacle_profile1_left_pratio == b.blue_obstacle_profile1_left_pratio &&
               a.blue_obstacle_profile1_left_duration_ms == b.blue_obstacle_profile1_left_duration_ms &&
               a.blue_obstacle_profile1_left_aim_speed == b.blue_obstacle_profile1_left_aim_speed &&
               a.blue_obstacle_profile1_left_min_error == b.blue_obstacle_profile1_left_min_error &&
               a.blue_obstacle_profile1_right_enable == b.blue_obstacle_profile1_right_enable &&
               a.blue_obstacle_profile1_right_delay_ms == b.blue_obstacle_profile1_right_delay_ms &&
               a.blue_obstacle_profile1_right_duration_ms == b.blue_obstacle_profile1_right_duration_ms &&
               a.blue_obstacle_profile1_right_min_error == b.blue_obstacle_profile1_right_min_error &&
               a.blue_obstacle_profile1_right_aim_speed == b.blue_obstacle_profile1_right_aim_speed &&
               a.blue_obstacle_profile1_post_run_enable == b.blue_obstacle_profile1_post_run_enable &&
               a.blue_obstacle_profile1_post_run_aim_speed == b.blue_obstacle_profile1_post_run_aim_speed &&
               a.blue_obstacle_profile1_post_run_duration_ms == b.blue_obstacle_profile1_post_run_duration_ms &&
               a.blue_obstacle_profile1_normal_top_crop_ratio == b.blue_obstacle_profile1_normal_top_crop_ratio &&
               a.blue_obstacle_profile2_action_mode == b.blue_obstacle_profile2_action_mode &&
               a.blue_obstacle_profile2_normal_run_ms == b.blue_obstacle_profile2_normal_run_ms &&
               a.blue_obstacle_profile2_left_pratio == b.blue_obstacle_profile2_left_pratio &&
               a.blue_obstacle_profile2_left_duration_ms == b.blue_obstacle_profile2_left_duration_ms &&
               a.blue_obstacle_profile2_left_aim_speed == b.blue_obstacle_profile2_left_aim_speed &&
               a.blue_obstacle_profile2_left_min_error == b.blue_obstacle_profile2_left_min_error &&
               a.blue_obstacle_profile2_expand_duration_ms == b.blue_obstacle_profile2_expand_duration_ms &&
               a.blue_obstacle_profile2_expand_aim_speed == b.blue_obstacle_profile2_expand_aim_speed &&
               a.blue_obstacle_profile2_normal_top_crop_ratio == b.blue_obstacle_profile2_normal_top_crop_ratio &&
               a.blue_obstacle_profile2_expanded_top_crop_ratio == b.blue_obstacle_profile2_expanded_top_crop_ratio &&
               a.blue_obstacle_profile2_post_run_enable == b.blue_obstacle_profile2_post_run_enable &&
               a.blue_obstacle_profile2_post_run_aim_speed == b.blue_obstacle_profile2_post_run_aim_speed &&
               a.blue_obstacle_profile2_post_run_duration_ms == b.blue_obstacle_profile2_post_run_duration_ms &&
               a.zebra_post_right_enable == b.zebra_post_right_enable &&
               a.zebra_post_right_delay_ms == b.zebra_post_right_delay_ms &&
               a.zebra_post_right_ms == b.zebra_post_right_ms &&
               a.zebra_post_right_error == b.zebra_post_right_error &&
               a.zebra_post_right_aim_speed == b.zebra_post_right_aim_speed &&
               a.zebra_reverse_speed == b.zebra_reverse_speed &&
               a.zebra_reverse_time_ms == b.zebra_reverse_time_ms &&
               a.traffic_red_enable == b.traffic_red_enable &&
               a.traffic_red_debug_enable == b.traffic_red_debug_enable &&
               a.traffic_red_debug_interval_ms == b.traffic_red_debug_interval_ms &&
               a.traffic_red_startup_ignore_ms == b.traffic_red_startup_ignore_ms &&
               a.traffic_red_ignore_after_zebra_detect_ms == b.traffic_red_ignore_after_zebra_detect_ms &&
               a.traffic_red_ignore_after_resume_ms == b.traffic_red_ignore_after_resume_ms &&
               a.traffic_red_disable_after_resume_ms == b.traffic_red_disable_after_resume_ms &&
               a.traffic_red_after_resume_left_enable == b.traffic_red_after_resume_left_enable &&
               a.traffic_red_after_resume_left_pratio == b.traffic_red_after_resume_left_pratio &&
               a.traffic_red_after_resume_left_duration_ms == b.traffic_red_after_resume_left_duration_ms &&
               a.traffic_red_after_resume_left_aim_speed == b.traffic_red_after_resume_left_aim_speed &&
               a.traffic_red_after_resume_left_min_error == b.traffic_red_after_resume_left_min_error &&
               a.traffic_red_after_detect_slow_delay_ms == b.traffic_red_after_detect_slow_delay_ms &&
               a.traffic_red_after_detect_slow_aim_speed == b.traffic_red_after_detect_slow_aim_speed &&
               a.traffic_red_after_detect_slow_duration_ms == b.traffic_red_after_detect_slow_duration_ms &&
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
               a.blue_board_line_expand_aim_speed == b.blue_board_line_expand_aim_speed &&
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
                  << " file=./image_params.txt"
                  << " experiment=" << params.experiment_enable
                  << ":" << (params.experiment_method_group == 1 ? "M" : "A") << params.experiment_method
                  << " group=" << params.experiment_method_group
                  << " video=" << params.experiment_video_enable << "@" << params.experiment_video_fps
                  << " run_id=" << params.experiment_run_id
                  << " task_features=" << params.experiment_task_features_enable
                  << " experiment_log=" << params.experiment_log_enable
                  << " zebra_far_cycle=" << params.zebra_far_cycle_enable
                  << " far_odd=" << params.zebra_far_y_ratio
                  << " far_even=" << params.zebra_far_second_y_ratio
                  << " traffic_red_enable=" << params.traffic_red_enable
                  << " dual_roi=" << params.traffic_dual_roi_enable
                  << " debug=" << params.traffic_red_debug_enable
                  << " startup_ignore_ms=" << params.traffic_red_startup_ignore_ms
                  << " red_ignore_after_zebra_ms=" << params.traffic_red_ignore_after_zebra_detect_ms
                  << " red_ignore_after_resume_ms=" << params.traffic_red_ignore_after_resume_ms
                  << " red_disable_after_resume_ms=" << params.traffic_red_disable_after_resume_ms
                  << " green_left=" << params.traffic_red_after_resume_left_enable
                  << ":" << params.traffic_red_after_resume_left_duration_ms
                  << "ms@" << params.traffic_red_after_resume_left_aim_speed
                  << "/err" << params.traffic_red_after_resume_left_min_error
                  << " zebra_ignore_after_traffic_resume_ms=" << params.zebra_ignore_after_traffic_resume_ms
                  << " red_after_slow=" << params.traffic_red_after_detect_slow_delay_ms
                  << "ms@" << params.traffic_red_after_detect_slow_aim_speed
                  << " for_ms=" << params.traffic_red_after_detect_slow_duration_ms
                  << " obstacle_profile=" << params.blue_obstacle_route_profile
                  << " p1(normal/left/wait/right/post)="
                  << params.blue_obstacle_profile1_normal_run_ms << "/"
                  << params.blue_obstacle_profile1_left_duration_ms << "/"
                  << params.blue_obstacle_profile1_right_delay_ms << "/"
                  << params.blue_obstacle_profile1_right_duration_ms << "/"
                  << params.blue_obstacle_profile1_post_run_duration_ms
                  << " p2_mode=" << params.blue_obstacle_profile2_action_mode
                  << " p2(normal/left/expand/post)="
                  << params.blue_obstacle_profile2_normal_run_ms << "/"
                  << params.blue_obstacle_profile2_left_duration_ms << "/"
                  << params.blue_obstacle_profile2_expand_duration_ms << "/"
                  << params.blue_obstacle_profile2_post_run_duration_ms
                  << " reverse_motors=L" << params.emergency_reverse_left_motor_enable
                  << "/R" << params.emergency_reverse_right_motor_enable
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

        newParams.experiment_config_error.clear();
        std::set<std::string> seenExperimentKeys;
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
                if (key.rfind("experiment_", 0) == 0 && !seenExperimentKeys.insert(key).second)
                    throw std::invalid_argument("duplicate experiment key");
                if (key == "experiment_enable")
                {
                    newParams.experiment_enable = Experiment::parseExperimentSwitch(value);
                }
                else if (key == "experiment_method")
                {
                    newParams.experiment_method = Experiment::parseExperimentInteger(value);
                }
                else if (key == "experiment_method_group")
                {
                    newParams.experiment_method_group = Experiment::parseExperimentInteger(value);
                }
                else if (key == "experiment_video_enable")
                {
                    newParams.experiment_video_enable = Experiment::parseExperimentSwitch(value);
                }
                else if (key == "experiment_video_fps")
                {
                    newParams.experiment_video_fps = Experiment::parseExperimentInteger(value);
                }
                else if (key == "experiment_task_features_enable")
                {
                    newParams.experiment_task_features_enable = Experiment::parseExperimentSwitch(value);
                }
                else if (key == "experiment_log_enable")
                {
                    newParams.experiment_log_enable = Experiment::parseExperimentSwitch(value);
                }
                else if (key == "experiment_run_id")
                {
                    newParams.experiment_run_id = value;
                }
                else if (key.rfind("experiment_", 0) == 0)
                {
                    throw std::invalid_argument("unknown experiment key");
                }
                else if (key == "center_pratio")
                {
                    newParams.center_pratio = clampDouble(std::stod(value), 0.20, 0.80);
                }
                else if (key == "error_gain")
                {
                    newParams.error_gain = clampDouble(std::stod(value), 0.5, 8.0);
                }
                else if (key == "steer_offset_deg")
                {
                    // 换舵机后机械零点可能差很多（实测出现过需要 +95 的情况），
                    // 所以范围放到 ±150 而不是常见的 ±60。
                    newParams.steer_offset_deg = clampDouble(std::stod(value), -150.0, 150.0);
                }
                else if (key == "zebra_post_center_bias_enable")
                {
                    newParams.zebra_post_center_bias_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "zebra_post_center_bias_pratio")
                {
                    newParams.zebra_post_center_bias_pratio = clampDouble(std::stod(value), 0.20, 0.80);
                }
                else if (key == "zebra_post_center_bias_duration_ms")
                {
                    newParams.zebra_post_center_bias_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "zebra_post_center_bias_aim_speed")
                {
                    newParams.zebra_post_center_bias_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "zebra_post_center_bias_min_left_error")
                {
                    newParams.zebra_post_center_bias_min_left_error = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "zebra_region_jump_min_count")
                {
                    newParams.zebra_jump_min_count = clampInt(std::stoi(value), 3, 12);
                }
                else if (key == "zebra_ignore_after_detect_ms")
                {
                    newParams.zebra_ignore_after_detect_ms = clampInt(std::stoi(value), 0, 30000);
                }
                else if (key == "zebra_ignore_after_traffic_resume_ms")
                {
                    newParams.zebra_ignore_after_traffic_resume_ms = clampInt(std::stoi(value), 0, 60000);
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
                else if (key == "emergency_reverse_left_motor_enable")
                {
                    newParams.emergency_reverse_left_motor_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "emergency_reverse_right_motor_enable")
                {
                    newParams.emergency_reverse_right_motor_enable = clampInt(std::stoi(value), 0, 1);
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
                else if (key == "blue_obstacle_route_profile")
                {
                    newParams.blue_obstacle_route_profile = clampInt(std::stoi(value), 1, 2);
                }
                else if (key == "blue_obstacle_profile1_normal_run_ms")
                {
                    newParams.blue_obstacle_profile1_normal_run_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile1_left_enable")
                {
                    newParams.blue_obstacle_profile1_left_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_obstacle_profile1_left_pratio")
                {
                    newParams.blue_obstacle_profile1_left_pratio = clampDouble(std::stod(value), 0.20, 0.80);
                }
                else if (key == "blue_obstacle_profile1_left_duration_ms")
                {
                    newParams.blue_obstacle_profile1_left_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile1_left_aim_speed")
                {
                    newParams.blue_obstacle_profile1_left_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_obstacle_profile1_left_min_error")
                {
                    newParams.blue_obstacle_profile1_left_min_error = clampDouble(std::stod(value), 0.0, 100.0);
                }
                else if (key == "blue_obstacle_profile1_right_enable")
                {
                    newParams.blue_obstacle_profile1_right_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_obstacle_profile1_right_delay_ms")
                {
                    newParams.blue_obstacle_profile1_right_delay_ms = clampInt(std::stoi(value), 0, 10000);
                }
                else if (key == "blue_obstacle_profile1_right_duration_ms")
                {
                    newParams.blue_obstacle_profile1_right_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile1_right_min_error")
                {
                    newParams.blue_obstacle_profile1_right_min_error = clampDouble(std::stod(value), 0.0, 100.0);
                }
                else if (key == "blue_obstacle_profile1_right_aim_speed")
                {
                    newParams.blue_obstacle_profile1_right_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_obstacle_profile1_post_run_enable")
                {
                    newParams.blue_obstacle_profile1_post_run_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_obstacle_profile1_post_run_aim_speed")
                {
                    newParams.blue_obstacle_profile1_post_run_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_obstacle_profile1_post_run_duration_ms")
                {
                    newParams.blue_obstacle_profile1_post_run_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile1_normal_top_crop_ratio")
                {
                    newParams.blue_obstacle_profile1_normal_top_crop_ratio = clampDouble(std::stod(value), 0.0, 0.80);
                }
                else if (key == "blue_obstacle_profile2_action_mode")
                {
                    newParams.blue_obstacle_profile2_action_mode = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_obstacle_profile2_normal_run_ms")
                {
                    newParams.blue_obstacle_profile2_normal_run_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile2_left_pratio")
                {
                    newParams.blue_obstacle_profile2_left_pratio = clampDouble(std::stod(value), 0.20, 0.80);
                }
                else if (key == "blue_obstacle_profile2_left_duration_ms")
                {
                    newParams.blue_obstacle_profile2_left_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile2_left_aim_speed")
                {
                    newParams.blue_obstacle_profile2_left_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_obstacle_profile2_left_min_error")
                {
                    newParams.blue_obstacle_profile2_left_min_error = clampDouble(std::stod(value), 0.0, 100.0);
                }
                else if (key == "blue_obstacle_profile2_expand_duration_ms")
                {
                    newParams.blue_obstacle_profile2_expand_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "blue_obstacle_profile2_expand_aim_speed")
                {
                    newParams.blue_obstacle_profile2_expand_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_obstacle_profile2_normal_top_crop_ratio")
                {
                    newParams.blue_obstacle_profile2_normal_top_crop_ratio = clampDouble(std::stod(value), 0.0, 0.80);
                }
                else if (key == "blue_obstacle_profile2_expanded_top_crop_ratio")
                {
                    newParams.blue_obstacle_profile2_expanded_top_crop_ratio = clampDouble(std::stod(value), 0.0, 0.80);
                }
                else if (key == "blue_obstacle_profile2_post_run_enable")
                {
                    newParams.blue_obstacle_profile2_post_run_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "blue_obstacle_profile2_post_run_aim_speed")
                {
                    newParams.blue_obstacle_profile2_post_run_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "blue_obstacle_profile2_post_run_duration_ms")
                {
                    newParams.blue_obstacle_profile2_post_run_duration_ms = clampInt(std::stoi(value), 0, 20000);
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
                    newParams.zebra_post_right_error = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "zebra_post_right_aim_speed")
                {
                    newParams.zebra_post_right_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
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
                else if (key == "traffic_red_startup_ignore_ms")
                {
                    newParams.traffic_red_startup_ignore_ms = clampInt(std::stoi(value), 0, 60000);
                }
                else if (key == "traffic_red_ignore_after_zebra_detect_ms")
                {
                    newParams.traffic_red_ignore_after_zebra_detect_ms = clampInt(std::stoi(value), 0, 120000);
                }
                else if (key == "traffic_red_ignore_after_resume_ms")
                {
                    newParams.traffic_red_ignore_after_resume_ms = clampInt(std::stoi(value), 0, 120000);
                }
                else if (key == "traffic_red_disable_after_resume_ms")
                {
                    newParams.traffic_red_disable_after_resume_ms = clampInt(std::stoi(value), 0, 120000);
                }
                else if (key == "traffic_red_after_resume_left_enable")
                {
                    newParams.traffic_red_after_resume_left_enable = clampInt(std::stoi(value), 0, 1);
                }
                else if (key == "traffic_red_after_resume_left_pratio")
                {
                    newParams.traffic_red_after_resume_left_pratio = clampDouble(std::stod(value), 0.20, 0.80);
                }
                else if (key == "traffic_red_after_resume_left_duration_ms")
                {
                    newParams.traffic_red_after_resume_left_duration_ms = clampInt(std::stoi(value), 0, 20000);
                }
                else if (key == "traffic_red_after_resume_left_aim_speed")
                {
                    newParams.traffic_red_after_resume_left_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
                }
                else if (key == "traffic_red_after_resume_left_min_error")
                {
                    newParams.traffic_red_after_resume_left_min_error = clampDouble(std::stod(value), 0.0, 100.0);
                }
                else if (key == "traffic_red_after_detect_slow_delay_ms")
                {
                    newParams.traffic_red_after_detect_slow_delay_ms = clampInt(std::stoi(value), 0, 120000);
                }
                else if (key == "traffic_red_after_detect_slow_aim_speed")
                {
                    newParams.traffic_red_after_detect_slow_aim_speed = clampDouble(std::stod(value), 0.0, 30.0);
                }
                else if (key == "traffic_red_after_detect_slow_duration_ms")
                {
                    newParams.traffic_red_after_detect_slow_duration_ms = clampInt(std::stoi(value), 0, 30000);
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
                else if (key == "blue_board_line_expand_aim_speed")
                {
                    newParams.blue_board_line_expand_aim_speed = clampDouble(std::stod(value), 0.5, 30.0);
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
                if (key.rfind("experiment_", 0) == 0)
                {
                    newParams.experiment_config_error = "invalid or duplicate setting: " + key + "=" + value;
                    std::cerr << "[RuntimeConfig] " << newParams.experiment_config_error << std::endl;
                }
            }
        }

        bool changed = !isSameParams(params_, newParams);
        if (changed) Experiment::gRealCarExperiment.markConfigChanged();
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

    std::string RuntimeConfig::getExperimentRunId()
    {
        reloadIfNeeded();
        std::lock_guard<std::mutex> lock(mutex_);
        return params_.experiment_run_id;
    }

    std::string RuntimeConfig::getExperimentConfigError()
    {
        reloadIfNeeded();
        std::lock_guard<std::mutex> lock(mutex_);
        return params_.experiment_config_error;
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

    RUNTIME_INT_GETTER(getExperimentEnable, experiment_enable)
    RUNTIME_INT_GETTER(getExperimentMethod, experiment_method)
    RUNTIME_INT_GETTER(getExperimentMethodGroup, experiment_method_group)
    RUNTIME_INT_GETTER(getExperimentVideoEnable, experiment_video_enable)
    RUNTIME_INT_GETTER(getExperimentVideoFps, experiment_video_fps)
    RUNTIME_INT_GETTER(getExperimentTaskFeaturesEnable, experiment_task_features_enable)
    RUNTIME_INT_GETTER(getExperimentLogEnable, experiment_log_enable)
    RUNTIME_DOUBLE_GETTER(getCenterPratio, center_pratio)
    RUNTIME_DOUBLE_GETTER(getErrorGain, error_gain)
    RUNTIME_DOUBLE_GETTER(getSteerOffsetDeg, steer_offset_deg)
    RUNTIME_INT_GETTER(getZebraPostCenterBiasEnable, zebra_post_center_bias_enable)
    RUNTIME_DOUBLE_GETTER(getZebraPostCenterBiasPratio, zebra_post_center_bias_pratio)
    RUNTIME_INT_GETTER(getZebraPostCenterBiasDurationMs, zebra_post_center_bias_duration_ms)
    RUNTIME_DOUBLE_GETTER(getZebraPostCenterBiasAimSpeed, zebra_post_center_bias_aim_speed)
    RUNTIME_DOUBLE_GETTER(getZebraPostCenterBiasMinLeftError, zebra_post_center_bias_min_left_error)
    RUNTIME_DOUBLE_GETTER(getZebraCheckYRatio, zebra_check_y_ratio)
    RUNTIME_INT_GETTER(getZebraCheckRowCount, zebra_check_row_count)
    RUNTIME_INT_GETTER(getZebraJumpMinCount, zebra_jump_min_count)
    RUNTIME_INT_GETTER(getZebraJumpBlockDiv, zebra_jump_block_div)
    RUNTIME_INT_GETTER(getZebraIgnoreAfterDetectMs, zebra_ignore_after_detect_ms)
    RUNTIME_INT_GETTER(getZebraIgnoreAfterTrafficResumeMs, zebra_ignore_after_traffic_resume_ms)
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
    RUNTIME_INT_GETTER(getEmergencyReverseLeftMotorEnable, emergency_reverse_left_motor_enable)
    RUNTIME_INT_GETTER(getEmergencyReverseRightMotorEnable, emergency_reverse_right_motor_enable)
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
    RUNTIME_INT_GETTER(getBlueObstacleRouteProfile, blue_obstacle_route_profile)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1NormalRunMs, blue_obstacle_profile1_normal_run_ms)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1LeftEnable, blue_obstacle_profile1_left_enable)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1LeftPratio, blue_obstacle_profile1_left_pratio)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1LeftDurationMs, blue_obstacle_profile1_left_duration_ms)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1LeftAimSpeed, blue_obstacle_profile1_left_aim_speed)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1LeftMinError, blue_obstacle_profile1_left_min_error)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1RightEnable, blue_obstacle_profile1_right_enable)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1RightDelayMs, blue_obstacle_profile1_right_delay_ms)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1RightDurationMs, blue_obstacle_profile1_right_duration_ms)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1RightMinError, blue_obstacle_profile1_right_min_error)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1RightAimSpeed, blue_obstacle_profile1_right_aim_speed)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1PostRunEnable, blue_obstacle_profile1_post_run_enable)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1PostRunAimSpeed, blue_obstacle_profile1_post_run_aim_speed)
    RUNTIME_INT_GETTER(getBlueObstacleProfile1PostRunDurationMs, blue_obstacle_profile1_post_run_duration_ms)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile1NormalTopCropRatio, blue_obstacle_profile1_normal_top_crop_ratio)
    RUNTIME_INT_GETTER(getBlueObstacleProfile2ActionMode, blue_obstacle_profile2_action_mode)
    RUNTIME_INT_GETTER(getBlueObstacleProfile2NormalRunMs, blue_obstacle_profile2_normal_run_ms)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2LeftPratio, blue_obstacle_profile2_left_pratio)
    RUNTIME_INT_GETTER(getBlueObstacleProfile2LeftDurationMs, blue_obstacle_profile2_left_duration_ms)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2LeftAimSpeed, blue_obstacle_profile2_left_aim_speed)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2LeftMinError, blue_obstacle_profile2_left_min_error)
    RUNTIME_INT_GETTER(getBlueObstacleProfile2ExpandDurationMs, blue_obstacle_profile2_expand_duration_ms)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2ExpandAimSpeed, blue_obstacle_profile2_expand_aim_speed)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2NormalTopCropRatio, blue_obstacle_profile2_normal_top_crop_ratio)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2ExpandedTopCropRatio, blue_obstacle_profile2_expanded_top_crop_ratio)
    RUNTIME_INT_GETTER(getBlueObstacleProfile2PostRunEnable, blue_obstacle_profile2_post_run_enable)
    RUNTIME_DOUBLE_GETTER(getBlueObstacleProfile2PostRunAimSpeed, blue_obstacle_profile2_post_run_aim_speed)
    RUNTIME_INT_GETTER(getBlueObstacleProfile2PostRunDurationMs, blue_obstacle_profile2_post_run_duration_ms)
    RUNTIME_INT_GETTER(getZebraPostRightEnable, zebra_post_right_enable)
    RUNTIME_INT_GETTER(getZebraPostRightDelayMs, zebra_post_right_delay_ms)
    RUNTIME_INT_GETTER(getZebraPostRightMs, zebra_post_right_ms)
    RUNTIME_DOUBLE_GETTER(getZebraPostRightError, zebra_post_right_error)
    RUNTIME_DOUBLE_GETTER(getZebraPostRightAimSpeed, zebra_post_right_aim_speed)
    RUNTIME_DOUBLE_GETTER(getZebraReverseSpeed, zebra_reverse_speed)
    RUNTIME_INT_GETTER(getZebraReverseTimeMs, zebra_reverse_time_ms)

    RUNTIME_INT_GETTER(getTrafficRedEnable, traffic_red_enable)
    RUNTIME_INT_GETTER(getTrafficRedDebugEnable, traffic_red_debug_enable)
    RUNTIME_INT_GETTER(getTrafficRedDebugIntervalMs, traffic_red_debug_interval_ms)
    RUNTIME_INT_GETTER(getTrafficRedStartupIgnoreMs, traffic_red_startup_ignore_ms)
    RUNTIME_INT_GETTER(getTrafficRedIgnoreAfterZebraDetectMs, traffic_red_ignore_after_zebra_detect_ms)
    RUNTIME_INT_GETTER(getTrafficRedIgnoreAfterResumeMs, traffic_red_ignore_after_resume_ms)
    RUNTIME_INT_GETTER(getTrafficRedDisableAfterResumeMs, traffic_red_disable_after_resume_ms)
    RUNTIME_INT_GETTER(getTrafficRedAfterResumeLeftEnable, traffic_red_after_resume_left_enable)
    RUNTIME_DOUBLE_GETTER(getTrafficRedAfterResumeLeftPratio, traffic_red_after_resume_left_pratio)
    RUNTIME_INT_GETTER(getTrafficRedAfterResumeLeftDurationMs, traffic_red_after_resume_left_duration_ms)
    RUNTIME_DOUBLE_GETTER(getTrafficRedAfterResumeLeftAimSpeed, traffic_red_after_resume_left_aim_speed)
    RUNTIME_DOUBLE_GETTER(getTrafficRedAfterResumeLeftMinError, traffic_red_after_resume_left_min_error)
    RUNTIME_INT_GETTER(getTrafficRedAfterDetectSlowDelayMs, traffic_red_after_detect_slow_delay_ms)
    RUNTIME_DOUBLE_GETTER(getTrafficRedAfterDetectSlowAimSpeed, traffic_red_after_detect_slow_aim_speed)
    RUNTIME_INT_GETTER(getTrafficRedAfterDetectSlowDurationMs, traffic_red_after_detect_slow_duration_ms)
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
    RUNTIME_DOUBLE_GETTER(getBlueBoardLineExpandAimSpeed, blue_board_line_expand_aim_speed)
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
