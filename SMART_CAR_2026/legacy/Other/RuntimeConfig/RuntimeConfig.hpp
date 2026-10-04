#pragma once

#include <string>
#include <mutex>
#include <chrono>

namespace Other
{
    struct RuntimeImageParams
    {
        /* 巡线中心比例。原来写死在 ImageHandle.cpp 里的 0.41。 */
        double center_pratio = 0.41;

        /* 旧版单行人行横道字段已不再从参数文件读取；实际检测只使用下面的区域型参数。 */
        double zebra_check_y_ratio = 0.45;
        int zebra_check_row_count = 7;
        int zebra_jump_min_count = 6;
        int zebra_jump_block_div = 16;

        /* 新区域型人行横道检测的远/近带状中心与白条间隔约束。 */
        /*
         * 远处带状检测区支持“第一次/第二次”交替。
         * count=0 时先使用 zebra_far_y_ratio；每真正触发一次停车后计数加一，
         * 第2、4、6...次改用 zebra_far_second_y_ratio；第3、5...次回到 zebra_far_y_ratio。
         */
        double zebra_far_y_ratio = 0.25;
        int zebra_far_cycle_enable = 1;
        double zebra_far_second_y_ratio = 0.20;
        double zebra_near_y_ratio = 0.50;
        double zebra_gap_ratio_max = 3.0;

        /*
         * 区域型人行横道确认：
         * 不再只压一条扫描线，而是在远/近位置各扫描一个小带状区域。
         * 命中的扫描行数达到 zebra_region_min_hit_rows 才认为该区域有效。
         */
        double zebra_region_half_height_ratio = 0.06;
        int zebra_region_sample_rows = 9;
        int zebra_region_min_hit_rows = 3;

        /*
         * 人行横道二维面积确认：
         * 扫描行的跳变只负责证明“存在重复黑白条”；下面这些参数再确认它们
         * 在带状区域里确实是有面积、有连续高度的多条白色斑马条，而不是零散亮点。
         */
        double zebra_region_white_area_ratio_min = 0.15;
        double zebra_region_white_area_ratio_max = 0.75;
        double zebra_component_min_area_ratio = 0.010;
        double zebra_component_min_height_ratio = 0.30;
        int zebra_component_min_count = 3;

        double zebra_slow_aim_speed = 5.0;
        int zebra_slow_hold_ms = 1000;
        /* 远处预检测是否触发减速：0=只记录用于二次确认，不减速；1=远处命中后减速。 */
        int zebra_far_slow_enable = 0;

        /* 检测到人行横道后，多少 ms 内忽略新的 Zebra，避免重复触发。 */
        int zebra_ignore_after_detect_ms = 6000;

        /* 电机正常目标速度。 */
        double motor_aim_speed = 10.0;

        /*
         * 安全停车参数。
         * 当前不再反向制动、不锁死：红灯/人行横道触发后直接 PWM=0。
         * PID 恢复前延迟一小段时间，避免停车后突然冲速。
         */
        int pid_recover_delay_ms = 0;

        /*
         * 停车/急停时是否清除电机 PID 历史。
         * 0：不清，保留上一次 PWM 输出趋势，绿灯/人行横道恢复时起步更快；
         * 1：清，恢复更柔和但起步会慢。
         */
        int motor_pid_clear_on_stop_enable = 0;

        /*
         * 红灯/人行横道急停动作序列：
         *   先断动力 emergency_coast_ms，
         *   再按 emergency_reverse_turns 反转，
         *   最后 PWM=0 锁止保持 emergency_brake_lock_ms。
         */
        int emergency_coast_ms = 80;
        int emergency_brake_lock_ms = 500;

        /* 人行横道停车保持时间。 */
        int zebra_after_reverse_stop_ms = 3000;

        /*
         * 人行横道停车恢复后的轻微右偏修正参数。
         * 触发 Zebra 并完成停车保持 + PID 恢复延迟后，可短时间强制给舵机一个
         * 向右的小偏移，用于修正停车恢复后车身方向或接后续弯道。
         */
        int zebra_post_right_enable = 0;
        int zebra_post_right_delay_ms = 0;
        int zebra_post_right_ms = 800;
        double zebra_post_right_error = 5.0;

        /*
         * 兼容旧配置字段。
         * 当前安全停车版本不再使用 emergency_brake_pwm / emergency_brake_time_ms。
         */
        int emergency_brake_pwm = 0;
        int emergency_brake_time_ms = 0;

        /*
         * 红灯 / 人行横道急停反转参数。
         * 触发后直接给左右轮负 PWM，使用右编码器累计圈数，反转到目标圈数后停车。
         * 左编码器损坏时仍可工作，因为当前电机反馈已经让左侧跟随右编码器。
         */
        int emergency_reverse_enable = 1;
        int emergency_reverse_pwm = 3000;
        double emergency_reverse_turns = 1.5;
        int emergency_reverse_timeout_ms = 1500;

        /*
         * 人行横道停车完全结束后速度序列。
         * Zebra 停车保持 + PID 恢复延迟结束后，先按正常速度运行一段时间，
         * 再按指定低速运行一段时间，最后恢复正常巡线速度。
         */
        int zebra_post_normal_run_ms = 3000;
        int zebra_post_slow_ms = 3000;
        double zebra_post_slow_aim_speed = 5.0;

        /* 兼容旧配置，当前不再用于人行横道急停。 */
        double zebra_reverse_speed = -5.0;
        int zebra_reverse_time_ms = 1000;


        /*
         * 红灯显式开关与调试。
         * traffic_red_enable=1 时才执行红灯状态机；debug 打开时会周期打印
         * 当前裁剪、ROI、红色像素、轮廓和每一层过滤的通过数量。
         */
        int traffic_red_enable = 1;
        int traffic_red_debug_enable = 1;
        int traffic_red_debug_interval_ms = 800;

        /* 内部红灯参数：外部实时配置只接受 traffic_red_ 前缀的新参数名。 */
        double traffic_cut_top_ratio = 0.00;
        double traffic_cut_left_ratio = 0.01;
        double traffic_cut_right_ratio = 0.45;

        double traffic_roi_x_start = 0.05;
        double traffic_roi_x_end = 0.95;
        double traffic_roi_y_start = 0.20;
        double traffic_roi_y_end = 0.56;

        /*
         * 红灯采用“左侧上部 + 左侧下部”双 ROI。
         * 上部使用 traffic_roi_y_start/end；下部使用以下范围。
         * 这样排除画面最上方易误判的背景，同时保留弯道初段左下的红灯。
         */
        int traffic_dual_roi_enable = 1;
        double traffic_lower_roi_y_start = 0.58;
        double traffic_lower_roi_y_end = 1.00;
        int traffic_red_confirm_frames = 2;

        int red1_h_min = 0;
        int red1_h_max = 12;
        int red2_h_min = 165;
        int red2_h_max = 179;
        int red_s_min = 70;
        int red_v_min = 70;
        double red_circle_min = 0.55;
        double red_area_min = 40.0;
        double red_area_max_ratio = 0.12;
        double red_aspect_min = 0.45;
        double red_aspect_max = 2.20;

        int green_h_min = 40;
        int green_h_max = 90;
        int green_s_min = 55;
        int green_v_min = 50;
        double green_pixel_min_ratio = 0.003;
        int green_pixel_min_absolute = 80;

        int red_ignore_after_green_ms = 5000;
        int traffic_morph_kernel_size = 1;

        /* 蓝色桶锥避障参数。 */
        double blue_cone_disappear_ratio = 0.35;
        double blue_return_bias = 7.0;
        int blue_return_frames = 25;
        /* 蓝色桶锥/挡板流程低速目标速度。
         * 从检测到蓝色桶锥开始，到进入弯道停车等绿灯前都使用这个速度。
         */
        double blue_obstacle_aim_speed = 6.0;
        int blue_min_active_frames = 2;
        int blue_lost_frames_trigger = 5;
        double blue_cone_min_area_ratio = 0.0010;
        double blue_cone_min_area_absolute = 2.0;

        /*
         * 蓝色挡板/弯道分支参数。
         * 注意：挡板分支不再看桶锥面积，而是看桶锥相反上方 ROI 内的黑色区域。
         * 黑色区域先随靠近挡板而增大，再随挡板离开视野而减小。
         * 当 current_black_area / max_black_area <= blue_board_leave_ratio 时，
         * 认为即将进入弯道，继续跑 blue_curve_run_after_leave_ms 后停车等绿灯。
         */
        double blue_board_leave_ratio = 0.35;
        double blue_board_black_min_area_ratio = 0.20;
        int blue_board_decrease_frames = 3;
        double blue_board_roi_y_ratio_end = 0.30;
        double blue_board_roi_x_ratio_width = 0.50;
        int blue_board_black_v_max = 70;
        int blue_board_black_s_max = 255;
        int blue_board_monitor_frames = 120;

        /* 到达 blue_board_leave_ratio 后，继续运行多少 ms 再停车等待绿灯。 */
        int blue_curve_run_after_leave_ms = 1200;

        /*
         * 蓝色挡板二段接弯运行时参数。
         * blue_board_primary_avoid_ms：检测到挡板后，第一段向挡板反方向躲避的时间。
         * blue_board_counter_turn_ms：第一段结束后，强制向相反方向接弯的时间。
         * blue_board_counter_turn_error：强制接弯阶段的舵机误差幅度，单位为度。
         *   只填正数即可，代码会根据挡板在左/右自动决定正负方向。
         * blue_board_slow_hold_after_ms：二段动作结束后，继续保持蓝色低速的时间。
         * 低速速度本身使用 blue_obstacle_aim_speed。
         */
        int blue_board_primary_avoid_ms = 1000;
        int blue_board_counter_turn_ms = 1800;
        double blue_board_counter_turn_error = 12.0;
        int blue_board_slow_hold_after_ms = 3000;

        /*
         * 顶部蓝色挡板提前检测：
         * 巡线图保持原来的顶部裁剪，只有蓝色挡板检测图会在斑马线后的短窗口中扩大上方视野。
         */
        int blue_board_enable = 1;
        int blue_board_expand_after_zebra_ms = 5000;
        double blue_board_normal_top_crop_ratio = 0.20;
        double blue_board_post_zebra_top_crop_ratio = 0.00;
        double blue_board_top_min_area_ratio = 0.058;
        int blue_board_confirm_frames = 2;
        double blue_board_primary_avoid_error = 12.0;

        /*
         * 人行横道后动态巡线裁剪。
         * 这不是蓝色 HSV 检测，也不会强制舵机转向；它只在实际重新起步后，
         * 临时把“巡线输入图”的顶部裁剪减小，让原有左右线/中线算法更早看到挡板。
         * 窗口结束后自动恢复 normal_top_crop_ratio。
         */
        int blue_board_line_expand_enable = 1;
        int blue_board_line_expand_delay_after_zebra_resume_ms = 0;
        int blue_board_line_expand_duration_ms = 5000;
        double blue_board_line_normal_top_crop_ratio = 0.20;
        double blue_board_line_post_zebra_top_crop_ratio = 0.04;

        /*
         * 动态巡线上裁剪结束后的低速窗口。
         * 只有 ImageHandle 确认“动态裁剪窗口已经结束，且巡线图已恢复 normal_top_crop”后，
         * 才开始这段低速。它不识别蓝色、不改舵机，只临时降低原巡线闭环目标速度。
         */
        int blue_board_line_post_expand_slow_enable = 1;
        double blue_board_line_post_expand_slow_aim_speed = 6.0;
        int blue_board_line_post_expand_slow_duration_ms = 3000;

        /*
         * 前方黑块兜底避障参数。
         *
         * 该逻辑不依赖蓝色 HSV 识别，而是在巡线二值图的赛道内部前方 ROI
         * 检测突然出现的大块黑色区域。触发后执行：
         *   第一段：向黑块相反方向强制躲避；
         *   第二段：向第一段相反方向强制接弯；
         *   第三段：动作结束后继续低速保持。
         */
        int front_black_enable = 0;
        double front_black_roi_y_start = 0.10;
        double front_black_roi_y_end = 0.38;
        double front_black_area_ratio_min = 0.18;
        double front_black_side_ratio_min = 1.15;
        int front_black_confirm_frames = 2;
        int front_black_primary_avoid_ms = 1000;
        double front_black_primary_avoid_error = 12.0;
        int front_black_counter_turn_ms = 1800;
        double front_black_counter_turn_error = 12.0;
        int front_black_slow_hold_after_ms = 3000;
        double front_black_aim_speed = 5.0;
    };

    class RuntimeConfig
    {
    public:
        static void reloadIfNeeded();
        static RuntimeImageParams getImageParams();

        static double getCenterPratio();
        static double getZebraCheckYRatio();
        static int getZebraCheckRowCount();
        static int getZebraJumpMinCount();
        static int getZebraJumpBlockDiv();
        static int getZebraIgnoreAfterDetectMs();
        static double getZebraFarYRatio();
        static int getZebraRegionFarCycleEnable();
        static double getZebraRegionFarSecondCenterYRatio();
        static double getZebraNearYRatio();
        static double getZebraGapRatioMax();
        static double getZebraRegionHalfHeightRatio();
        static int getZebraRegionSampleRows();
        static int getZebraRegionMinHitRows();
        static double getZebraRegionWhiteAreaRatioMin();
        static double getZebraRegionWhiteAreaRatioMax();
        static double getZebraComponentMinAreaRatio();
        static double getZebraComponentMinHeightRatio();
        static int getZebraComponentMinCount();
        static double getZebraSlowAimSpeed();
        static int getZebraSlowHoldMs();
        static int getZebraFarSlowEnable();

        static double getMotorAimSpeed();
        static int getEmergencyBrakePwm();
        static int getEmergencyBrakeTimeMs();
        static int getEmergencyReverseEnable();
        static int getEmergencyReversePwm();
        static double getEmergencyReverseTurns();
        static int getEmergencyReverseTimeoutMs();
        static int getZebraPostNormalRunMs();
        static int getZebraPostSlowMs();
        static double getZebraPostSlowAimSpeed();
        static int getPidRecoverDelayMs();
        static int getMotorPidClearOnStopEnable();
        static int getEmergencyCoastMs();
        static int getEmergencyBrakeLockMs();
        static int getZebraAfterReverseStopMs();
        static int getZebraPostRightEnable();
        static int getZebraPostRightDelayMs();
        static int getZebraPostRightMs();
        static double getZebraPostRightError();
        static double getZebraReverseSpeed();
        static int getZebraReverseTimeMs();

        static int getTrafficRedEnable();
        static int getTrafficRedDebugEnable();
        static int getTrafficRedDebugIntervalMs();
        static double getTrafficCutTopRatio();
        static double getTrafficCutLeftRatio();
        static double getTrafficCutRightRatio();

        static double getTrafficRoiXStart();
        static double getTrafficRoiXEnd();
        static double getTrafficRoiYStart();
        static double getTrafficRoiYEnd();
        static int getTrafficDualRoiEnable();
        static double getTrafficLowerRoiYStart();
        static double getTrafficLowerRoiYEnd();
        static int getTrafficRedConfirmFrames();

        static int getRed1HMin();
        static int getRed1HMax();
        static int getRed2HMin();
        static int getRed2HMax();
        static int getRedSMin();
        static int getRedVMin();
        static double getRedCircleMin();
        static double getRedAreaMin();
        static double getRedAreaMaxRatio();
        static double getRedAspectMin();
        static double getRedAspectMax();

        static int getGreenHMin();
        static int getGreenHMax();
        static int getGreenSMin();
        static int getGreenVMin();
        static double getGreenPixelMinRatio();
        static int getGreenPixelMinAbsolute();

        static int getRedIgnoreAfterGreenMs();
        static int getTrafficMorphKernelSize();

        static double getBlueConeDisappearRatio();
        static double getBlueReturnBias();
        static int getBlueReturnFrames();
        static double getBlueObstacleAimSpeed();
        static int getBlueMinActiveFrames();
        static int getBlueLostFramesTrigger();
        static double getBlueConeMinAreaRatio();
        static double getBlueConeMinAreaAbsolute();

        static double getBlueBoardLeaveRatio();
        static double getBlueBoardBlackMinAreaRatio();
        static int getBlueBoardDecreaseFrames();
        static double getBlueBoardRoiYRatioEnd();
        static double getBlueBoardRoiXRatioWidth();
        static int getBlueBoardBlackVMax();
        static int getBlueBoardBlackSMax();
        static int getBlueBoardMonitorFrames();
        static int getBlueCurveRunAfterLeaveMs();
        static int getBlueBoardPrimaryAvoidMs();
        static int getBlueBoardCounterTurnMs();
        static double getBlueBoardCounterTurnError();
        static int getBlueBoardSlowHoldAfterMs();
        static int getBlueBoardEnable();
        static int getBlueBoardExpandAfterZebraMs();
        static double getBlueBoardNormalTopCropRatio();
        static double getBlueBoardPostZebraTopCropRatio();
        static double getBlueBoardTopMinAreaRatio();
        static int getBlueBoardConfirmFrames();
        static double getBlueBoardPrimaryAvoidError();
        static int getBlueBoardLineExpandEnable();
        static int getBlueBoardLineExpandDelayAfterZebraResumeMs();
        static int getBlueBoardLineExpandDurationMs();
        static double getBlueBoardLineNormalTopCropRatio();
        static double getBlueBoardLinePostZebraTopCropRatio();
        static int getBlueBoardLinePostExpandSlowEnable();
        static double getBlueBoardLinePostExpandSlowAimSpeed();
        static int getBlueBoardLinePostExpandSlowDurationMs();

        static int getFrontBlackEnable();
        static double getFrontBlackRoiYStart();
        static double getFrontBlackRoiYEnd();
        static double getFrontBlackAreaRatioMin();
        static double getFrontBlackSideRatioMin();
        static int getFrontBlackConfirmFrames();
        static int getFrontBlackPrimaryAvoidMs();
        static double getFrontBlackPrimaryAvoidError();
        static int getFrontBlackCounterTurnMs();
        static double getFrontBlackCounterTurnError();
        static int getFrontBlackSlowHoldAfterMs();
        static double getFrontBlackAimSpeed();

    private:
        static void ensureConfigFileExists();
        static bool loadFromFile(bool printWhenChanged);
        static std::string trim(const std::string &text);
        static double clampDouble(double value, double minValue, double maxValue);
        static int clampInt(int value, int minValue, int maxValue);
        static bool isSameParams(const RuntimeImageParams &a, const RuntimeImageParams &b);
        static void printParams(const char *prefix, const RuntimeImageParams &params);

    private:
        static std::mutex mutex_;
        static RuntimeImageParams params_;
        static std::chrono::steady_clock::time_point last_load_time_;
        static bool initialized_;
    };
}
