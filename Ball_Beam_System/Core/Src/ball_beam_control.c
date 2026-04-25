#include "ball_beam_control.h"

#include <math.h>
#include <stdio.h>
#include "Servo.h"
#include "gp2y0a41.h"
#include "Serial.h"

#define CONTROL_PERIOD_MS             10U
#define CONTROL_PERIOD_SEC            ((float)CONTROL_PERIOD_MS * 0.001f)
#define TELEMETRY_PERIOD_MS           500U
#define PID_ENABLE_DELAY_MS           400U
#define MAX_CONSEC_INVALID_SAMPLES    60U

#define SENSOR_VALID_MIN_MM           40U   //参数含义：最小有效距离，单位：毫米
#define SENSOR_VALID_MAX_MM           160U  // 最大有效距离，单位：毫米
#define SENSOR_WARNING_THRESHOLD_MM   155U  // 警告阈值，单位：毫米
#define SENSOR_MAX_STEP_MM            40U  // 最大步进距离，单位：毫米
#define SENSOR_FILTER_ALPHA           0.50f  // EMA 系数，数值越大跟随越快（滤波更轻）
#define SENSOR_READ_RETRY_COUNT       2U

#define TRACK_END_MM                  115.0f
#define PID_INPUT_DEADZONE_MIN_MM     110U
#define PID_INPUT_DEADZONE_MAX_MM     120U
#define DEFAULT_TARGET_OFFSET_MM      0.0f
#define SETPOINT_MIN_OFFSET_MM        ((float)SENSOR_VALID_MIN_MM - TRACK_END_MM)
#define SETPOINT_MAX_OFFSET_MM        ((float)SENSOR_VALID_MAX_MM - TRACK_END_MM)
#define SETPOINT_RAMP_RATE_MM_PER_S   50.0f
#define PID_ACTUATOR_FEEDBACK_GAIN    0.35f

/* 归一化 PID：输入/输出都在 [-1, 1]，再映射回舵机角度偏移 */
#define PID_OUT_MIN_NORM              (-1.0f)
#define PID_OUT_MAX_NORM              (1.0f)
#define PID_INTEGRAL_ENTER_BAND_NORM  0.80f
#define PID_INTEGRAL_EXIT_BAND_NORM   0.90f
#define PID_KP                        0.22f
#define PID_KI                        0.0f
#define PID_KD                        2.4f
#define PID_INTEGRAL_LIMIT_NORM       1.0f // 积分项限制
#define PID_HOLD_ENTER_BAND_MM        1.0f  // 缩小保持带，避免“来回切换停在两点”
#define PID_HOLD_EXIT_BAND_MM         2.0f
#define PID_CENTER_LIMIT_ERROR_NORM   0.20f
#define PID_CENTER_OUTPUT_LIMIT_DEG   6.0f
#define PID_TRIM_MAX_DEG              20.0f  // 让 PID 成为主通道
#define PID_FEEDFORWARD_FULL_MM       55.0f // 误差达到该值时，映射主通道全量生效
#define PID_FEEDFORWARD_GAIN          0.06f // 降为辅助通道
#define PID_SOFT_END_MARGIN_DEG       6.0f  // PID 软限位，避免两端硬撞
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
    const float warning_mm = (float)SENSOR_WARNING_THRESHOLD_MM;
    const float deadzone_min_mm = (float)PID_INPUT_DEADZONE_MIN_MM;
    const float deadzone_max_mm = (float)PID_INPUT_DEADZONE_MAX_MM;
    const float near_end_mm = (float)SENSOR_VALID_MIN_MM; /* 4cm */
    float distance = (float)distance_mm;

    if (distance >= warning_mm)
    {
        return SERVO_MIN_ANGLE;
    }

    if (distance <= near_end_mm)
    {
        return SERVO_MAX_ANGLE;
    }

    if ((distance >= deadzone_min_mm) && (distance <= deadzone_max_mm))
    {
        return SERVO_CENTER_ANGLE;
    }

    if (distance > deadzone_max_mm)
    {
        float ratio = (distance - deadzone_max_mm) / (warning_mm - deadzone_max_mm);
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
    const float measurement_min_mm = (float)SENSOR_VALID_MIN_MM - TRACK_END_MM; /* -75mm */
    const float measurement_max_mm = (float)SENSOR_VALID_MAX_MM - TRACK_END_MM; /* +45mm */
    float delta = measurement_mm - setpoint_mm;

    if (delta >= 0.0f)
    {
        float span = measurement_max_mm - setpoint_mm;
        if (span < 1.0f)
        {
            span = 1.0f;
        }
        return clampf(delta / span, 0.0f, 1.0f);
    }

    float span = setpoint_mm - measurement_min_mm;
    if (span < 1.0f)
    {
        span = 1.0f;
    }
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
    ctrl->prev_abs_error_mm = 0.0f;
    ctrl->center_hold_active = false;
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
    for (uint32_t attempt = 0U; attempt < SENSOR_READ_RETRY_COUNT; ++attempt)
    {
        uint16_t distance_mm = GP2Y0A41_ReadDistanceMm();
        if (distance_mm != GP2Y0A41_INVALID_DISTANCE_MM)
        {
            return distance_mm;
        }
    }

    return 0xFFFFU;
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
        ctrl->consecutive_invalid_samples++;
        can_control = true;
    }
    else
    {
        ctrl->consecutive_invalid_samples++;
    }

    if (can_control)
    {
        float measurement_mm = distance_to_pid_input_mm(ctrl->last_valid_distance_mm);
        float measurement_norm = measurement_to_pid_input_norm(measurement_mm, ctrl->setpoint_offset_mm);

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

        if (pid_ready)
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

            float abs_error_mm = fabsf(measurement_mm - ctrl->setpoint_offset_mm);
            bool enter_hold = (!ctrl->center_hold_active) && (abs_error_mm <= PID_HOLD_ENTER_BAND_MM);
            bool exit_hold = ctrl->center_hold_active && (abs_error_mm >= PID_HOLD_EXIT_BAND_MM);
            if (enter_hold)
            {
                ctrl->center_hold_active = true;
                PID_Reset(&ctrl->pid, measurement_norm);
            }
            else if (exit_hold)
            {
                ctrl->center_hold_active = false;
                PID_Reset(&ctrl->pid, measurement_norm);
            }

            if (ctrl->center_hold_active)
            {
                ctrl->control_output_deg = 0.0f;
                ctrl->anti_stall_counter = 0U;
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

                if (span_mm < 1.0f)
                {
                    span_mm = 1.0f;
                }
                ff_ratio = clampf((abs_error_mm - PID_HOLD_EXIT_BAND_MM) / span_mm, 0.0f, 1.0f);
                ff_ratio = ff_ratio * ff_ratio;
                ff_assist_deg = PID_FEEDFORWARD_GAIN * ff_ratio * (mapped_servo_angle - SERVO_CENTER_ANGLE);
                ff_assist_deg = clampf(ff_assist_deg, -2.0f, 2.0f);
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

        if (ctrl->mode == CONTROL_MODE_PID)
        {
            target_servo_angle = clampf(target_servo_angle,
                                        SERVO_MIN_ANGLE + PID_SOFT_END_MARGIN_DEG,
                                        SERVO_MAX_ANGLE - PID_SOFT_END_MARGIN_DEG);
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
        ctrl->prev_abs_error_mm = 0.0f;
        ctrl->center_hold_active = false;
        PID_Reset(&ctrl->pid, 0.0f);
        ctrl->target_servo_angle = SERVO_CENTER_ANGLE;
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
    static const char *k_mode_names[] = {"PD", "PID"};
    static const char *k_state_names[] = {"INIT", "RUN", "FAULT"};

    int32_t setpoint_mm = (int32_t)(ctrl->setpoint_offset_mm + ((ctrl->setpoint_offset_mm >= 0.0f) ? 0.5f : -0.5f));
    int32_t setpoint_cmd_mm = (int32_t)(ctrl->setpoint_cmd_mm + ((ctrl->setpoint_cmd_mm >= 0.0f) ? 0.5f : -0.5f));
    int32_t measurement_mm = BallBeamController_GetMeasurementMm(ctrl);
    int32_t error_mm = (measurement_mm == (int32_t)INVALID_TELEMETRY_MM) ? (int32_t)INVALID_TELEMETRY_MM : (setpoint_mm - measurement_mm);
    float cmd_out_deg = ctrl->target_servo_angle - SERVO_CENTER_ANGLE;
    float act_out_deg = ctrl->servo_angle - SERVO_CENTER_ANGLE;
    int32_t cmd_out_x10 = (int32_t)(cmd_out_deg * 10.0f + ((cmd_out_deg >= 0.0f) ? 0.5f : -0.5f));
    int32_t act_out_x10 = (int32_t)(act_out_deg * 10.0f + ((act_out_deg >= 0.0f) ? 0.5f : -0.5f));
    int32_t cmd_angle_x10 = (int32_t)(ctrl->target_servo_angle * 10.0f + ((ctrl->target_servo_angle >= 0.0f) ? 0.5f : -0.5f));
    int32_t servo_angle_x10 = (int32_t)(ctrl->servo_angle * 10.0f + ((ctrl->servo_angle >= 0.0f) ? 0.5f : -0.5f));
    uint32_t mode_index = ((uint32_t)ctrl->mode <= (uint32_t)CONTROL_MODE_PID) ? (uint32_t)ctrl->mode : 0U;
    uint32_t state_index = ((uint32_t)ctrl->state <= (uint32_t)CONTROL_STATE_FAULT) ? (uint32_t)ctrl->state : 0U;
    int32_t cmd_out_frac_x10 = cmd_out_x10 % 10;
    int32_t act_out_frac_x10 = act_out_x10 % 10;
    int32_t cmd_angle_frac_x10 = cmd_angle_x10 % 10;
    int32_t servo_angle_frac_x10 = servo_angle_x10 % 10;
    if (cmd_out_frac_x10 < 0)
    {
        cmd_out_frac_x10 = -cmd_out_frac_x10;
    }
    if (act_out_frac_x10 < 0)
    {
        act_out_frac_x10 = -act_out_frac_x10;
    }
    if (cmd_angle_frac_x10 < 0)
    {
        cmd_angle_frac_x10 = -cmd_angle_frac_x10;
    }
    if (servo_angle_frac_x10 < 0)
    {
        servo_angle_frac_x10 = -servo_angle_frac_x10;
    }

    if ((uint32_t)(now_ms - ctrl->serial_last_tick) < TELEMETRY_PERIOD_MS)
    {
        return;
    }

    ctrl->serial_last_tick = now_ms;
    char telemetry_line[220];
    int line_len;
    if (measurement_mm != (int32_t)INVALID_TELEMETRY_MM)
    {
        line_len = snprintf(telemetry_line,
                            sizeof(telemetry_line),
                            "RAW_ADC:%u DIST_MM:%u MEAS_MM:%ld SETPOINT_MM:%ld SETPOINT_CMD_MM:%ld ERROR_MM:%ld DT_MS:%lu CMD_OUT:%ld.%01lddeg ACT_OUT:%ld.%01lddeg CMD_ANGLE:%ld.%01lddeg SERVO_ANGLE:%ld.%01lddeg MODE:%s STATE:%s\r\n",
                            (unsigned int)ctrl->last_adc_raw,
                            (unsigned int)ctrl->last_valid_distance_mm,
                            (long)measurement_mm,
                            (long)setpoint_mm,
                            (long)setpoint_cmd_mm,
                            (long)error_mm,
                            (unsigned long)(ctrl->control_dt_sec * 1000.0f + 0.5f),
                            (long)(cmd_out_x10 / 10),
                            (long)cmd_out_frac_x10,
                            (long)(act_out_x10 / 10),
                            (long)act_out_frac_x10,
                            (long)(cmd_angle_x10 / 10),
                            (long)cmd_angle_frac_x10,
                            (long)(servo_angle_x10 / 10),
                            (long)servo_angle_frac_x10,
                            k_mode_names[mode_index],
                            k_state_names[state_index]);
    }
    else
    {
        line_len = snprintf(telemetry_line,
                            sizeof(telemetry_line),
                            "RAW_ADC:%u DIST_MM:-- MEAS_MM:-- SETPOINT_MM:%ld SETPOINT_CMD_MM:%ld ERROR_MM:-- DT_MS:%lu CMD_OUT:%ld.%01lddeg ACT_OUT:%ld.%01lddeg CMD_ANGLE:%ld.%01lddeg SERVO_ANGLE:%ld.%01lddeg MODE:%s STATE:%s\r\n",
                            (unsigned int)ctrl->last_adc_raw,
                            (long)setpoint_mm,
                            (long)setpoint_cmd_mm,
                            (unsigned long)(ctrl->control_dt_sec * 1000.0f + 0.5f),
                            (long)(cmd_out_x10 / 10),
                            (long)cmd_out_frac_x10,
                            (long)(act_out_x10 / 10),
                            (long)act_out_frac_x10,
                            (long)(cmd_angle_x10 / 10),
                            (long)cmd_angle_frac_x10,
                            (long)(servo_angle_x10 / 10),
                            (long)servo_angle_frac_x10,
                            k_mode_names[mode_index],
                            k_state_names[state_index]);
    }

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
