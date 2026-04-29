#include "ball_beam_control.h"

#include <math.h>
#include <stdio.h>
#include "Servo.h"
#include "gp2y0a41.h"
#include "Serial.h"

/*
 * 文件综述：
 * 本文件实现球梁系统闭环控制主逻辑。
 * 核心职责：
 * 1) 传感器数据有效性判定与滤波；
 * 2) PID/PD 控制输出计算与舵机角控制；
 * 3) 中心保持、远端回拉、安全保护和遥测输出。
 */
#define CONTROL_PERIOD_MS             10U
#define CONTROL_PERIOD_SEC            ((float)CONTROL_PERIOD_MS * 0.001f)
#define TELEMETRY_PERIOD_MS           100U
#define PID_ENABLE_DELAY_MS           400U
#define MAX_CONSEC_INVALID_SAMPLES    60U

#define SENSOR_VALID_MIN_MM           40U   //参数含义：最小有效距离，单位：毫米
#define SENSOR_VALID_MAX_MM           160U  // 最大有效距离，单位：毫米
#define SENSOR_WARNING_THRESHOLD_MM   155U  // 警告阈值，单位：毫米
#define STUCK_ERROR_THRESHOLD_MM      16.0f
#define STUCK_ERROR_CONFIRM_COUNT     60U
#define STUCK_SUSPECT_MIN_MM          72U
#define STUCK_SUSPECT_MAX_MM          98U
#define STUCK_FAR_END_SUSPECT_MIN_MM  145U
#define STUCK_RECOVERY_HOLD_TICKS     60U
#define STUCK_RECOVERY_COOLDOWN_TICKS 100U
#define STUCK_RECOVERY_RECOVERED_ERROR_MM 4.0f
#define STUCK_RECOVERY_RECOVERED_CONFIRM_COUNT 3U
#define STUCK_RECOVERY_MIN_ANGLE      SERVO_MIN_ANGLE
#define NEAR_END_GLITCH_LAST_MM_MAX   60U
#define NEAR_END_GLITCH_NEW_MM_MIN    95U
#define NEAR_END_GLITCH_DELTA_MM      18U
#define FAR_END_GLITCH_LAST_MM_MIN    145U
#define FAR_END_GLITCH_NEW_MM_MAX     110U
#define FAR_END_GLITCH_DELTA_MM       18U
#define SENSOR_MAX_STEP_MM            40U  // 最大步进距离，单位：毫米
#define SENSOR_FILTER_ALPHA           0.35f  // EMA 系数，数值越大跟随越快（滤波更轻）
#define SENSOR_READ_SAMPLE_COUNT      3U

#define TRACK_END_MM                  115.0f
#define PID_INPUT_DEADZONE_MIN_MM     110U
#define PID_INPUT_DEADZONE_MAX_MM     120U
#define DEFAULT_TARGET_OFFSET_MM      0.0f
#define SETPOINT_MIN_OFFSET_MM        ((float)SENSOR_VALID_MIN_MM - TRACK_END_MM)
#define SETPOINT_MAX_OFFSET_MM        ((float)SENSOR_VALID_MAX_MM - TRACK_END_MM)
#define SETPOINT_RAMP_RATE_MM_PER_S   50.0f
#define PID_ACTUATOR_FEEDBACK_GAIN    0.10f
#define PID_NORM_POS_SPAN_MM          55.0f
#define PID_NORM_NEG_SPAN_MM          55.0f

/* 归一化 PID：输入/输出都在 [-1, 1]，再映射回舵机角度偏移 */
#define PID_OUT_MIN_NORM              (-1.0f)
#define PID_OUT_MAX_NORM              (1.0f)
#define PID_INTEGRAL_ENTER_BAND_NORM  0.80f
#define PID_INTEGRAL_EXIT_BAND_NORM   0.90f
#define PID_KP                        0.24f
#define PID_KI                        0.0002f
#define PID_KD                        0.18f
#define PID_INTEGRAL_LIMIT_NORM       1.0f // 积分项限制
#define PID_HOLD_ENTER_BAND_MM        2.0f
#define PID_HOLD_EXIT_BAND_MM         4.0f
#define PID_HOLD_ENTER_CONFIRM_COUNT  8U
#define PID_HOLD_EXIT_CONFIRM_COUNT   4U
#define PID_HOLD_EXIT_STRONG_BAND_MM  6.0f
#define PID_HOLD_EXIT_RISE_MM         0.3f
#define PID_CENTER_LIMIT_ERROR_NORM   0.20f
#define PID_CENTER_OUTPUT_LIMIT_DEG   6.0f
#define PID_TRIM_MAX_DEG              28.0f
#define PID_OUTPUT_DEADBAND_DEG       0.35f
#define PID_FEEDFORWARD_FULL_MM       55.0f // 误差达到该值时，映射主通道全量生效
#define PID_FEEDFORWARD_GAIN          0.00f
#define PID_FEEDFORWARD_LIMIT_DEG     6.0f
#define PID_SOFT_END_MARGIN_DEG       6.0f  // PID 软限位，避免两端硬撞
#define PID_SOFT_END_MARGIN_EMERGENCY_DEG 0.5f
#define EDGE_RECOVERY_ERROR_MM        30.0f
#define FAR_SIDE_GENTLE_START_MM      5.0f
#define FAR_SIDE_GENTLE_FULL_MM       55.0f
#define FAR_SIDE_GENTLE_MIN_PULL_DEG  6.0f
#define FAR_SIDE_GENTLE_MAX_PULL_DEG  26.0f
#define INVALID_RECOVERY_ANGLE_DEG    SERVO_MIN_ANGLE
#define INVALID_TELEMETRY_MM          (-999.0f) // 无效测距值

static float clampf(float value, float min_val, float max_val)
{
    if (value < min_val)
    {
        return min_val;
    }
    if (value > max_val)
    {
        return max_val;
    }
    return value;
}

static float apply_slew_limit(float previous, float target, float max_step)
{
    if (max_step <= 0.0f)
    {
        return target;
    }

    if (target > previous + max_step)
    {
        return previous + max_step;
    }
    if (target < previous - max_step)
    {
        return previous - max_step;
    }
    return target;
}

static float quantize_servo_angle(float angle, float step_deg)
{
    if (step_deg <= 0.0f)
    {
        return angle;
    }

    {
        float relative = (angle - SERVO_CENTER_ANGLE) / step_deg;
        float quantized = SERVO_CENTER_ANGLE + roundf(relative) * step_deg;
        return clampf(quantized, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
    }
}

static float apply_center_output_limit(float output_deg, float measurement_norm)
{
    float abs_error = fabsf(measurement_norm);
    if (abs_error >= PID_CENTER_LIMIT_ERROR_NORM)
    {
        return output_deg;
    }

    {
        float full_limit_deg = SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE;
        float ratio = abs_error / PID_CENTER_LIMIT_ERROR_NORM;
        float dynamic_limit_deg = PID_CENTER_OUTPUT_LIMIT_DEG +
                                  ratio * (full_limit_deg - PID_CENTER_OUTPUT_LIMIT_DEG);
        return clampf(output_deg, -dynamic_limit_deg, dynamic_limit_deg);
    }
}

static float map_distance_to_servo_angle(uint16_t distance_mm)
{
    const float far_end_mm = (float)SENSOR_VALID_MAX_MM;
    const float deadzone_min_mm = (float)PID_INPUT_DEADZONE_MIN_MM;
    const float deadzone_max_mm = (float)PID_INPUT_DEADZONE_MAX_MM;
    const float near_end_mm = (float)SENSOR_VALID_MIN_MM; /* 4cm */
    float distance = (float)distance_mm;

    if (distance <= near_end_mm)
    {
        /* 近端危险区：角度指向“离开近端方向”。 */
        return SERVO_MAX_ANGLE;
    }

    if ((distance >= deadzone_min_mm) && (distance <= deadzone_max_mm))
    {
        return SERVO_CENTER_ANGLE;
    }

    if (distance > deadzone_max_mm)
    {
        float ratio = (distance - deadzone_max_mm) / (far_end_mm - deadzone_max_mm);
        ratio = clampf(ratio, 0.0f, 1.0f);
        return SERVO_CENTER_ANGLE + ratio * (SERVO_MIN_ANGLE - SERVO_CENTER_ANGLE);
    }

    {
        float ratio = (distance - near_end_mm) / (deadzone_min_mm - near_end_mm);
        return SERVO_MAX_ANGLE + ratio * (SERVO_CENTER_ANGLE - SERVO_MAX_ANGLE);
    }
}

static float distance_to_offset_mm(uint16_t distance_mm)
{
    return (float)distance_mm - TRACK_END_MM;
}

static float distance_to_pid_input_mm(uint16_t distance_mm)
{
    return distance_to_offset_mm(distance_mm);
}

static float measurement_to_pid_input_norm(float measurement_mm, float setpoint_mm)
{
    float delta = measurement_mm - setpoint_mm;

    /* 关键点：使用固定归一化跨度，避免目标点改变后控制力度不一致。 */
    if (delta >= 0.0f)
    {
        float span = PID_NORM_POS_SPAN_MM;
        return clampf(delta / span, 0.0f, 1.0f);
    }

    float span = PID_NORM_NEG_SPAN_MM;
    return clampf(delta / span, -1.0f, 0.0f);
}

static float pid_output_norm_to_deg(float output_norm)
{
    if (output_norm >= 0.0f)
    {
        return output_norm * (SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE);
    }

    return output_norm * (SERVO_CENTER_ANGLE - SERVO_MIN_ANGLE);
}

static float deg_to_pid_output_norm(float output_deg)
{
    float span_deg = SERVO_MAX_ANGLE - SERVO_CENTER_ANGLE;
    if (span_deg < 1.0f)
    {
        span_deg = 1.0f;
    }
    return clampf(output_deg / span_deg, -1.0f, 1.0f);
}

static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

static uint16_t low_pass_u16(uint16_t previous, uint16_t current, float alpha)
{
    float filtered = (float)previous + alpha * ((float)current - (float)previous);
    if (filtered < 0.0f)
    {
        filtered = 0.0f;
    }
    return (uint16_t)(filtered + 0.5f);
}

void BallBeamController_Init(BallBeamController_t *ctrl, uint32_t start_tick_ms)
{
    ctrl->last_adc_raw = 0xFFFFU;
    ctrl->distance_mm = 0xFFFFU;
    ctrl->last_valid_distance_mm = 0U;
    ctrl->last_valid_raw_distance_mm = 0U;
    ctrl->control_output_deg = 0.0f;
    ctrl->servo_angle = SERVO_CENTER_ANGLE;
    ctrl->target_servo_angle = SERVO_CENTER_ANGLE;
    ctrl->setpoint_offset_mm = DEFAULT_TARGET_OFFSET_MM;
    ctrl->setpoint_cmd_mm = DEFAULT_TARGET_OFFSET_MM;
    ctrl->setpoint_slew_rate_mm_per_s = SETPOINT_RAMP_RATE_MM_PER_S;
    ctrl->control_dt_sec = CONTROL_PERIOD_SEC;
    ctrl->pid_actuator_feedback_norm = 0.0f;
    ctrl->serial_last_tick = 0U;
    ctrl->control_start_tick = start_tick_ms;
    ctrl->last_control_tick = start_tick_ms;
    ctrl->consecutive_invalid_samples = 0U;
    ctrl->anti_stall_counter = 0U;
    ctrl->near_center_stuck_counter = 0U;
    ctrl->far_end_exit_counter = 0U;
    ctrl->far_end_stuck_counter = 0U;
    ctrl->far_end_recovery_cooldown = 0U;
    ctrl->far_end_recovery_phase = 0U;
    ctrl->far_end_recovery_success_counter = 0U;
    ctrl->prev_abs_error_mm = 0.0f;
    ctrl->center_hold_active = false;
    ctrl->far_end_recovery_active = false;
    ctrl->has_valid_distance = false;
    ctrl->mode = CONTROL_MODE_PD;
    ctrl->state = CONTROL_STATE_INIT;

    Servo_SetAngle(SERVO_CENTER_ANGLE);
    PID_Init(&ctrl->pid, PID_KP, PID_KI, PID_KD, CONTROL_PERIOD_SEC, PID_OUT_MIN_NORM, PID_OUT_MAX_NORM,
             PID_INTEGRAL_LIMIT_NORM, PID_INTEGRAL_ENTER_BAND_NORM, PID_INTEGRAL_EXIT_BAND_NORM);
    ctrl->pid.integral_enabled = false;
    PID_Reset(&ctrl->pid, 0.0f);
}

bool BallBeamController_ShouldRunControl(const BallBeamController_t *ctrl, uint32_t now_ms)
{
    return (uint32_t)(now_ms - ctrl->last_control_tick) >= CONTROL_PERIOD_MS;
}

uint16_t BallBeamController_ReadDistanceMm(void)
{
    uint16_t samples[SENSOR_READ_SAMPLE_COUNT];
    uint32_t valid_count = 0U;
    for (uint32_t attempt = 0U; attempt < SENSOR_READ_SAMPLE_COUNT; ++attempt)
    {
        uint16_t distance_mm = GP2Y0A41_ReadDistanceMm();
        if (distance_mm != GP2Y0A41_INVALID_DISTANCE_MM)
        {
            samples[valid_count] = distance_mm;
            valid_count++;
        }
    }

    if (valid_count == 0U)
    {
        return 0xFFFFU;
    }
    if (valid_count == 1U)
    {
        return samples[0];
    }
    if (valid_count == 2U)
    {
        return (uint16_t)((samples[0] + samples[1]) / 2U);
    }

    if (samples[0] > samples[1])
    {
        uint16_t tmp = samples[0];
        samples[0] = samples[1];
        samples[1] = tmp;
    }
    if (samples[1] > samples[2])
    {
        uint16_t tmp = samples[1];
        samples[1] = samples[2];
        samples[2] = tmp;
    }
    if (samples[0] > samples[1])
    {
        uint16_t tmp = samples[0];
        samples[0] = samples[1];
        samples[1] = tmp;
    }

    return samples[1];
}

bool BallBeamController_Step(BallBeamController_t *ctrl, uint32_t now_ms, uint16_t sampled_distance_mm)
{
    uint32_t elapsed_ms = (uint32_t)(now_ms - ctrl->last_control_tick);
    if (elapsed_ms == 0U)
    {
        elapsed_ms = CONTROL_PERIOD_MS;
    }

    ctrl->control_dt_sec = clampf((float)elapsed_ms * 0.001f, 0.001f, 0.100f);
    ctrl->pid.dt = ctrl->control_dt_sec;
    ctrl->last_control_tick = now_ms;
    ctrl->last_adc_raw = GP2Y0A41_GetLastRawAdc();
    ctrl->distance_mm = sampled_distance_mm;

    {
        float setpoint_max_step_mm = ctrl->setpoint_slew_rate_mm_per_s * ctrl->control_dt_sec;
        ctrl->setpoint_offset_mm = apply_slew_limit(ctrl->setpoint_offset_mm, ctrl->setpoint_cmd_mm, setpoint_max_step_mm);
        ctrl->setpoint_offset_mm = clampf(ctrl->setpoint_offset_mm, SETPOINT_MIN_OFFSET_MM, SETPOINT_MAX_OFFSET_MM);
    }

    bool valid_sample = false;
    if ((sampled_distance_mm >= SENSOR_VALID_MIN_MM) && (sampled_distance_mm <= SENSOR_VALID_MAX_MM))
    {
        if (!ctrl->has_valid_distance)
        {
            valid_sample = true;
        }
        else
        {
            uint16_t delta = abs_diff_u16(sampled_distance_mm, ctrl->last_valid_raw_distance_mm);
            valid_sample = (delta <= SENSOR_MAX_STEP_MM);
            if (valid_sample &&
                (ctrl->last_valid_distance_mm <= NEAR_END_GLITCH_LAST_MM_MAX) &&
                (sampled_distance_mm >= NEAR_END_GLITCH_NEW_MM_MIN) &&
                (delta >= NEAR_END_GLITCH_DELTA_MM))
            {
                /* 近端漏光常见伪跳变：球明明在近端，却瞬间读到约100+mm，直接判为无效点 */
                valid_sample = false;
            }
            if (valid_sample &&
                (ctrl->last_valid_distance_mm >= FAR_END_GLITCH_LAST_MM_MIN) &&
                (sampled_distance_mm <= FAR_END_GLITCH_NEW_MM_MAX) &&
                (delta >= FAR_END_GLITCH_DELTA_MM))
            {
                /* 远端虚假回跳：球在远端却瞬间被读成80~100mm，直接判为无效点。 */
                valid_sample = false;
            }
        }
    }

    bool can_control = false;
    if (valid_sample)
    {
        ctrl->last_valid_raw_distance_mm = sampled_distance_mm;
        if (!ctrl->has_valid_distance)
        {
            ctrl->last_valid_distance_mm = sampled_distance_mm;
        }
        else
        {
            ctrl->last_valid_distance_mm = low_pass_u16(ctrl->last_valid_distance_mm, sampled_distance_mm, SENSOR_FILTER_ALPHA);
        }
        ctrl->has_valid_distance = true;
        ctrl->consecutive_invalid_samples = 0U;
        can_control = true;
    }
    else if (ctrl->has_valid_distance)
    {
        /* 无新有效样本时不继续闭环，避免用陈旧数据把系统慢慢“带偏”。 */
        ctrl->consecutive_invalid_samples++;
        can_control = false;
    }
    else
    {
        ctrl->consecutive_invalid_samples++;
    }

    if (can_control)
    {
        float measurement_mm = distance_to_pid_input_mm(ctrl->last_valid_distance_mm);
        float measurement_norm = measurement_to_pid_input_norm(measurement_mm, ctrl->setpoint_offset_mm);
        float abs_error_mm = fabsf(measurement_mm - ctrl->setpoint_offset_mm);
        bool in_stuck_suspect_window = ((ctrl->last_valid_distance_mm >= STUCK_SUSPECT_MIN_MM) &&
                                        (ctrl->last_valid_distance_mm <= STUCK_SUSPECT_MAX_MM)) ||
                                       (ctrl->last_valid_distance_mm >= STUCK_FAR_END_SUSPECT_MIN_MM);

        if (ctrl->state != CONTROL_STATE_RUN)
        {
            ctrl->state = CONTROL_STATE_RUN;
            ctrl->mode = CONTROL_MODE_PD;
            ctrl->pid.integral_enabled = false;
            ctrl->pid_actuator_feedback_norm = 0.0f;
            PID_Reset(&ctrl->pid, measurement_norm);
        }

        float target_servo_angle = map_distance_to_servo_angle(ctrl->last_valid_distance_mm);
        bool pid_ready = (uint32_t)(now_ms - ctrl->control_start_tick) >= PID_ENABLE_DELAY_MS;

        if (!ctrl->far_end_recovery_active)
        {
            if (ctrl->far_end_recovery_cooldown > 0U)
            {
                ctrl->far_end_recovery_cooldown--;
            }

            if ((ctrl->far_end_recovery_cooldown == 0U) &&
                in_stuck_suspect_window &&
                (abs_error_mm >= STUCK_ERROR_THRESHOLD_MM))
            {
                if (ctrl->far_end_stuck_counter < 0xFFFFU)
                {
                    ctrl->far_end_stuck_counter++;
                }
            }
            else
            {
                ctrl->far_end_stuck_counter = 0U;
            }

            if (ctrl->far_end_stuck_counter >= STUCK_ERROR_CONFIRM_COUNT)
            {
                ctrl->far_end_recovery_active = true;
                ctrl->far_end_exit_counter = STUCK_RECOVERY_HOLD_TICKS;
                ctrl->far_end_recovery_phase = 0U;
                ctrl->far_end_recovery_success_counter = 0U;
                ctrl->far_end_stuck_counter = 0U;
                ctrl->center_hold_active = false;
                ctrl->anti_stall_counter = 0U;
                ctrl->near_center_stuck_counter = 0U;
                ctrl->pid.integral_enabled = false;
                ctrl->pid.integral = 0.0f;
                ctrl->pid_actuator_feedback_norm = 0.0f;
                PID_Reset(&ctrl->pid, measurement_norm);
            }
        }
        else
        {
            bool escaped_stuck_zone = !in_stuck_suspect_window;
            bool recovery_success = escaped_stuck_zone || (abs_error_mm <= STUCK_RECOVERY_RECOVERED_ERROR_MM);
            if (recovery_success)
            {
                if (ctrl->far_end_recovery_success_counter < 0xFFU)
                {
                    ctrl->far_end_recovery_success_counter++;
                }
            }
            else
            {
                ctrl->far_end_recovery_success_counter = 0U;
            }

            if (ctrl->far_end_recovery_success_counter >= STUCK_RECOVERY_RECOVERED_CONFIRM_COUNT)
            {
                ctrl->far_end_recovery_active = false;
                ctrl->far_end_recovery_phase = 0U;
                ctrl->far_end_exit_counter = 0U;
                ctrl->far_end_recovery_success_counter = 0U;
                ctrl->far_end_recovery_cooldown = STUCK_RECOVERY_COOLDOWN_TICKS;
            }
            else
            {
                if (ctrl->far_end_exit_counter > 0U)
                {
                    ctrl->far_end_exit_counter--;
                }
                if (ctrl->far_end_exit_counter == 0U)
                {
                    ctrl->far_end_recovery_active = false;
                    ctrl->far_end_recovery_phase = 0U;
                    ctrl->far_end_recovery_success_counter = 0U;
                    ctrl->far_end_recovery_cooldown = STUCK_RECOVERY_COOLDOWN_TICKS;
                }
            }
        }

        if (ctrl->far_end_recovery_active)
        {
            ctrl->mode = CONTROL_MODE_PD;
            ctrl->pid.integral_enabled = false;
            ctrl->pid.integral = 0.0f;
            ctrl->pid_actuator_feedback_norm = 0.0f;
            ctrl->anti_stall_counter = 0U;
            ctrl->near_center_stuck_counter = 0U;
            ctrl->center_hold_active = false;
            target_servo_angle = STUCK_RECOVERY_MIN_ANGLE;
            ctrl->control_output_deg = target_servo_angle - SERVO_CENTER_ANGLE;
            ctrl->prev_abs_error_mm = abs_error_mm;
        }
        else if (pid_ready)
        {
            if (ctrl->mode != CONTROL_MODE_PID)
            {
                ctrl->mode = CONTROL_MODE_PID;
                ctrl->pid.integral_enabled = true;
                ctrl->pid_actuator_feedback_norm = 0.0f;
                ctrl->anti_stall_counter = 0U;
                ctrl->near_center_stuck_counter = 0U;
                ctrl->prev_abs_error_mm = 0.0f;
                ctrl->center_hold_active = false;
                PID_Reset(&ctrl->pid, measurement_norm);
            }

            if (abs_error_mm < PID_HOLD_EXIT_BAND_MM)
            {
                /* 中心附近清积分，抑制“待久后慢慢变陡”的漂移问题。 */
                ctrl->pid.integral_enabled = false;
                ctrl->pid.integral = 0.0f;
            }
            else
            {
                ctrl->pid.integral_enabled = true;
            }
            bool enter_hold = false;
            bool exit_hold = false;
            if (abs_error_mm <= PID_HOLD_ENTER_BAND_MM)
            {
                if (ctrl->near_center_stuck_counter < 0xFFFFU)
                {
                    ctrl->near_center_stuck_counter++;
                }
            }
            else
            {
                ctrl->near_center_stuck_counter = 0U;
            }

            if (abs_error_mm >= PID_HOLD_EXIT_BAND_MM)
            {
                bool should_count_exit = true;
                if (ctrl->center_hold_active)
                {
                    should_count_exit =
                        (abs_error_mm >= PID_HOLD_EXIT_STRONG_BAND_MM) ||
                        (abs_error_mm > (ctrl->prev_abs_error_mm + PID_HOLD_EXIT_RISE_MM));
                }
                if (should_count_exit)
                {
                    if (ctrl->anti_stall_counter < 0xFFFFU)
                    {
                        ctrl->anti_stall_counter++;
                    }
                }
                else
                {
                    ctrl->anti_stall_counter = 0U;
                }
            }
            else
            {
                ctrl->anti_stall_counter = 0U;
            }

            if ((!ctrl->center_hold_active) && (ctrl->near_center_stuck_counter >= PID_HOLD_ENTER_CONFIRM_COUNT))
            {
                enter_hold = true;
            }
            if (ctrl->center_hold_active && (ctrl->anti_stall_counter >= PID_HOLD_EXIT_CONFIRM_COUNT))
            {
                exit_hold = true;
            }
            if (enter_hold)
            {
                ctrl->center_hold_active = true;
                ctrl->near_center_stuck_counter = 0U;
                ctrl->anti_stall_counter = 0U;
                PID_Reset(&ctrl->pid, measurement_norm);
            }
            else if (exit_hold)
            {
                ctrl->center_hold_active = false;
                ctrl->near_center_stuck_counter = 0U;
                ctrl->anti_stall_counter = 0U;
                PID_Reset(&ctrl->pid, measurement_norm);
            }

            if (ctrl->center_hold_active)
            {
                ctrl->control_output_deg = 0.0f;
                ctrl->anti_stall_counter = 0U;
                ctrl->pid.integral_enabled = false;
                ctrl->pid.integral = 0.0f;
                target_servo_angle = SERVO_CENTER_ANGLE;
            }
            else
            {
                float control_output_norm = PID_Update(&ctrl->pid, 0.0f, measurement_norm, ctrl->pid_actuator_feedback_norm);
                float pid_output_deg = pid_output_norm_to_deg(control_output_norm);
                float mapped_servo_angle = map_distance_to_servo_angle(ctrl->last_valid_distance_mm);
                float span_mm = PID_FEEDFORWARD_FULL_MM - PID_HOLD_EXIT_BAND_MM;
                float ff_ratio;
                float ff_assist_deg;

                pid_output_deg = apply_center_output_limit(pid_output_deg, measurement_norm);
                pid_output_deg = clampf(pid_output_deg, -PID_TRIM_MAX_DEG, PID_TRIM_MAX_DEG);
                if (fabsf(pid_output_deg) <= PID_OUTPUT_DEADBAND_DEG)
                {
                    pid_output_deg = 0.0f;
                }

                if (span_mm < 1.0f)
                {
                    span_mm = 1.0f;
                }
                ff_ratio = clampf((abs_error_mm - PID_HOLD_EXIT_BAND_MM) / span_mm, 0.0f, 1.0f);
                ff_ratio = ff_ratio * ff_ratio;
                ff_assist_deg = PID_FEEDFORWARD_GAIN * ff_ratio * (mapped_servo_angle - SERVO_CENTER_ANGLE);
                ff_assist_deg = clampf(ff_assist_deg, -PID_FEEDFORWARD_LIMIT_DEG, PID_FEEDFORWARD_LIMIT_DEG);
                ctrl->control_output_deg = pid_output_deg + ff_assist_deg;
                ctrl->anti_stall_counter = 0U;
                ctrl->near_center_stuck_counter = 0U;

                target_servo_angle = SERVO_CENTER_ANGLE + ctrl->control_output_deg;
            }

            ctrl->prev_abs_error_mm = abs_error_mm;
        }
        else
        {
            ctrl->mode = CONTROL_MODE_PD;
            ctrl->pid.integral_enabled = false;
            ctrl->pid_actuator_feedback_norm = 0.0f;
            ctrl->anti_stall_counter = 0U;
            ctrl->near_center_stuck_counter = 0U;
            ctrl->prev_abs_error_mm = 0.0f;
            ctrl->center_hold_active = false;
            ctrl->control_output_deg = target_servo_angle - SERVO_CENTER_ANGLE;
        }

        if (!ctrl->far_end_recovery_active)
        {
            float far_error_mm = measurement_mm - ctrl->setpoint_offset_mm;
            if (far_error_mm > FAR_SIDE_GENTLE_START_MM)
            {
                float ratio = clampf((far_error_mm - FAR_SIDE_GENTLE_START_MM) /
                                     (FAR_SIDE_GENTLE_FULL_MM - FAR_SIDE_GENTLE_START_MM), 0.0f, 1.0f);
                float max_pull_deg = FAR_SIDE_GENTLE_MIN_PULL_DEG +
                                     ratio * (FAR_SIDE_GENTLE_MAX_PULL_DEG - FAR_SIDE_GENTLE_MIN_PULL_DEG);
                float min_servo_angle = SERVO_CENTER_ANGLE - max_pull_deg;
                if (target_servo_angle < min_servo_angle)
                {
                    target_servo_angle = min_servo_angle;
                }
            }
        }

        if (ctrl->mode == CONTROL_MODE_PID)
        {
            float soft_end_margin_deg = PID_SOFT_END_MARGIN_DEG;
            if (fabsf(measurement_mm - ctrl->setpoint_offset_mm) >= EDGE_RECOVERY_ERROR_MM)
            {
                soft_end_margin_deg = PID_SOFT_END_MARGIN_EMERGENCY_DEG;
            }
            target_servo_angle = clampf(target_servo_angle,
                                        SERVO_MIN_ANGLE + soft_end_margin_deg,
                                        SERVO_MAX_ANGLE - soft_end_margin_deg);
        }
        target_servo_angle = clampf(target_servo_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        ctrl->target_servo_angle = target_servo_angle;
        float previous_servo_angle = ctrl->servo_angle;
        ctrl->servo_angle = apply_slew_limit(previous_servo_angle, target_servo_angle, SERVO_SLEW_MAX_STEP_DEG);
        ctrl->servo_angle = clampf(ctrl->servo_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        ctrl->servo_angle = quantize_servo_angle(ctrl->servo_angle, SERVO_MIN_STEP_DEG);
        ctrl->pid_actuator_feedback_norm =
            PID_ACTUATOR_FEEDBACK_GAIN * deg_to_pid_output_norm(ctrl->servo_angle - ctrl->target_servo_angle);
        if (ctrl->mode == CONTROL_MODE_PD)
        {
            ctrl->control_output_deg = ctrl->servo_angle - SERVO_CENTER_ANGLE;
        }
        Servo_SetAngle(ctrl->servo_angle);
    }

    if (ctrl->consecutive_invalid_samples > MAX_CONSEC_INVALID_SAMPLES)
    {
        ctrl->state = CONTROL_STATE_FAULT;
        ctrl->pid.integral_enabled = false;
        ctrl->pid_actuator_feedback_norm = 0.0f;
        ctrl->mode = CONTROL_MODE_PD;
        ctrl->control_output_deg = 0.0f;
        ctrl->anti_stall_counter = 0U;
        ctrl->near_center_stuck_counter = 0U;
        ctrl->far_end_stuck_counter = 0U;
        ctrl->far_end_exit_counter = 0U;
        ctrl->far_end_recovery_cooldown = 0U;
        ctrl->far_end_recovery_phase = 0U;
        ctrl->far_end_recovery_success_counter = 0U;
        ctrl->prev_abs_error_mm = 0.0f;
        ctrl->center_hold_active = false;
        ctrl->far_end_recovery_active = false;
        PID_Reset(&ctrl->pid, 0.0f);
        ctrl->target_servo_angle = INVALID_RECOVERY_ANGLE_DEG;
        float previous_servo_angle = ctrl->servo_angle;
        ctrl->servo_angle = apply_slew_limit(previous_servo_angle, ctrl->target_servo_angle, SERVO_SLEW_MAX_STEP_DEG);
        ctrl->servo_angle = clampf(ctrl->servo_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        ctrl->servo_angle = quantize_servo_angle(ctrl->servo_angle, SERVO_MIN_STEP_DEG);
        ctrl->pid_actuator_feedback_norm = 0.0f;
        Servo_SetAngle(ctrl->servo_angle);
    }

    return valid_sample;
}

void BallBeamController_FormatDistanceLine(bool valid_sample, uint16_t sampled_distance_mm, char *buf, uint32_t buf_len)
{
    if (!valid_sample)
    {
        (void)snprintf(buf, buf_len, "Dis: -- mm");
    }
    else
    {
        (void)snprintf(buf, buf_len, "Dis:%4u mm", sampled_distance_mm);
    }
}

int32_t BallBeamController_GetFilteredDistanceMm(const BallBeamController_t *ctrl)
{
    return ctrl->has_valid_distance ? (int32_t)ctrl->last_valid_distance_mm : -1;
}

int32_t BallBeamController_GetMeasurementMm(const BallBeamController_t *ctrl)
{
    if (!ctrl->has_valid_distance)
    {
        return (int32_t)INVALID_TELEMETRY_MM;
    }

    float measurement_mm = distance_to_pid_input_mm(ctrl->last_valid_distance_mm);
    return (int32_t)(measurement_mm + ((measurement_mm >= 0.0f) ? 0.5f : -0.5f));
}

void BallBeamController_SendTelemetry(BallBeamController_t *ctrl, uint32_t now_ms)
{
    int32_t measurement_mm = BallBeamController_GetMeasurementMm(ctrl);
    int32_t output_mm = (measurement_mm == (int32_t)INVALID_TELEMETRY_MM) ? -9999 : measurement_mm;

    if ((uint32_t)(now_ms - ctrl->serial_last_tick) < TELEMETRY_PERIOD_MS)
    {
        return;
    }

    ctrl->serial_last_tick = now_ms;
    char telemetry_line[20];
    int line_len;
    line_len = snprintf(telemetry_line, sizeof(telemetry_line), "[p,%ld]\r\n", (long)output_mm);

    if (line_len <= 0)
    {
        return;
    }

    {
        uint16_t tx_len = (line_len >= (int)sizeof(telemetry_line)) ? (uint16_t)(sizeof(telemetry_line) - 1U) : (uint16_t)line_len;
        if (Serial_AvailableForWrite() >= tx_len)
        {
            Serial_SendData((const uint8_t *)telemetry_line, tx_len);
        }
    }
}

void BallBeamController_SetSetpointMm(BallBeamController_t *ctrl, float setpoint_mm)
{
    ctrl->setpoint_cmd_mm = clampf(setpoint_mm, SETPOINT_MIN_OFFSET_MM, SETPOINT_MAX_OFFSET_MM);
}

void BallBeamController_SetTargetDistanceMm(BallBeamController_t *ctrl, float target_distance_mm)
{
    BallBeamController_SetSetpointMm(ctrl, target_distance_mm - TRACK_END_MM);
}
