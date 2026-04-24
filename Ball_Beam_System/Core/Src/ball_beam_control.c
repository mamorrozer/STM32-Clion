#include "ball_beam_control.h"

#include <math.h>
#include <stdio.h>
#include "Servo.h"
#include "gp2y0a41.h"
#include "Serial.h"

#define CONTROL_PERIOD_MS             10U
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

/* 归一化 PID：输入/输出都在 [-1, 1]，再映射回舵机角度偏移 */
#define PID_OUT_MIN_NORM              (-1.0f)
#define PID_OUT_MAX_NORM              (1.0f)
#define PID_INTEGRAL_ACTIVE_BAND_NORM 0.80f
#define PID_KP                        0.25f
#define PID_KI                        0.0f
#define PID_KD                        0.4f
#define CONTROL_PERIOD_SEC            0.01f // 控制周期，单位：秒
#define PID_INTEGRAL_LIMIT_NORM       1.0f // 积分项限制
#define PID_HOLD_ENTER_BAND_MM        5.0f  // 中心保持进入阈值
#define PID_HOLD_EXIT_BAND_MM         9.0f  // 中心保持退出阈值（迟滞，避免边界抖动）
#define PID_CENTER_LIMIT_ERROR_NORM   0.10f
#define PID_CENTER_OUTPUT_LIMIT_DEG   0.8f
#define PID_TRIM_MAX_DEG              2.0f  // PID 在映射主通道上的最大微调幅度
#define PID_FEEDFORWARD_FULL_MM       55.0f // 误差达到该值时，映射主通道全量生效
#define PID_FEEDFORWARD_GAIN          0.30f // 映射主通道增益
#define PID_SOFT_END_MARGIN_DEG       6.0f  // PID 软限位，避免两端硬撞
#define PID_ANTI_STALL_ACTIVE_MM      20.0f // 误差超过该值才启用防卡住推进
#define PID_ANTI_STALL_FULL_MM        45.0f // 误差达到该值时使用最大推进下限
#define PID_ANTI_STALL_MIN_DEG        0.8f
#define PID_ANTI_STALL_MAX_DEG        2.8f
#define PID_ANTI_STALL_DELTA_MM       0.6f  // 单周期误差变化小于该值视为“几乎无进展”
#define PID_ANTI_STALL_COUNT_MAX      40U   // 持续无进展约 400ms（10ms 控制周期）
#define PID_ANTI_STALL_BOOST_DEG      0.5f
#define NEAR_CENTER_STUCK_DIST_MIN_MM 106U  // 用户反馈的卡住窗口
#define NEAR_CENTER_STUCK_DIST_MAX_MM 112U
#define NEAR_CENTER_STUCK_MEAS_MIN_MM (-8.0f)
#define NEAR_CENTER_STUCK_MEAS_MAX_MM (-4.0f)
#define NEAR_CENTER_STUCK_ERR_MIN_MM  4.0f
#define NEAR_CENTER_STUCK_ERR_MAX_MM  9.0f
#define NEAR_CENTER_STUCK_CMD_MAX_DEG 3.2f
#define NEAR_CENTER_STUCK_COUNT_MAX   120U  // 持续约1.2s后触发回调
#define NEAR_CENTER_STUCK_RESCUE_DEG  2.5f
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
    PID_Init(&ctrl->pid, PID_KP, PID_KI, PID_KD, CONTROL_PERIOD_SEC, PID_OUT_MIN_NORM, PID_OUT_MAX_NORM, PID_INTEGRAL_LIMIT_NORM, PID_INTEGRAL_ACTIVE_BAND_NORM);
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
    ctrl->last_control_tick = now_ms;
    ctrl->last_adc_raw = GP2Y0A41_GetLastRawAdc();
    ctrl->distance_mm = sampled_distance_mm;

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
                float control_output_norm = PID_Update(&ctrl->pid, 0.0f, measurement_norm);
                float pid_trim_deg = pid_output_norm_to_deg(control_output_norm);
                float mapped_servo_angle = map_distance_to_servo_angle(ctrl->last_valid_distance_mm);
                float span_mm = PID_FEEDFORWARD_FULL_MM - PID_HOLD_EXIT_BAND_MM;
                float ff_ratio;
                float base_output_deg;
                float error_signed_mm = ctrl->setpoint_offset_mm - measurement_mm;
                float min_drive_ratio;
                float min_drive_deg;
                float error_progress_mm = fabsf(abs_error_mm - ctrl->prev_abs_error_mm);
                float min_drive_sign = (error_signed_mm >= 0.0f) ? 1.0f : -1.0f;

                pid_trim_deg = apply_center_output_limit(pid_trim_deg, measurement_norm);
                pid_trim_deg = clampf(pid_trim_deg, -PID_TRIM_MAX_DEG, PID_TRIM_MAX_DEG);

                if (span_mm < 1.0f)
                {
                    span_mm = 1.0f;
                }
                ff_ratio = clampf((abs_error_mm - PID_HOLD_EXIT_BAND_MM) / span_mm, 0.0f, 1.0f);
                ff_ratio = ff_ratio * ff_ratio;
                base_output_deg = PID_FEEDFORWARD_GAIN * ff_ratio * (mapped_servo_angle - SERVO_CENTER_ANGLE);

                ctrl->control_output_deg = base_output_deg + pid_trim_deg;

                if ((abs_error_mm >= PID_ANTI_STALL_ACTIVE_MM) && (error_progress_mm <= PID_ANTI_STALL_DELTA_MM))
                {
                    if (ctrl->anti_stall_counter < PID_ANTI_STALL_COUNT_MAX)
                    {
                        ctrl->anti_stall_counter++;
                    }
                }
                else
                {
                    ctrl->anti_stall_counter = 0U;
                }

                min_drive_ratio = clampf((abs_error_mm - PID_ANTI_STALL_ACTIVE_MM) /
                                         (PID_ANTI_STALL_FULL_MM - PID_ANTI_STALL_ACTIVE_MM), 0.0f, 1.0f);
                min_drive_deg = PID_ANTI_STALL_MIN_DEG +
                                min_drive_ratio * (PID_ANTI_STALL_MAX_DEG - PID_ANTI_STALL_MIN_DEG);
                if (ctrl->anti_stall_counter >= PID_ANTI_STALL_COUNT_MAX)
                {
                    min_drive_deg += PID_ANTI_STALL_BOOST_DEG;
                }

                if (abs_error_mm >= PID_ANTI_STALL_ACTIVE_MM)
                {
                    float signed_min_drive = min_drive_sign * min_drive_deg;
                    if (fabsf(ctrl->control_output_deg) < min_drive_deg)
                    {
                        ctrl->control_output_deg = signed_min_drive;
                    }
                }

                target_servo_angle = SERVO_CENTER_ANGLE + ctrl->control_output_deg;
            }

            bool in_near_center_stuck_window =
                (ctrl->last_valid_distance_mm >= NEAR_CENTER_STUCK_DIST_MIN_MM) &&
                (ctrl->last_valid_distance_mm <= NEAR_CENTER_STUCK_DIST_MAX_MM) &&
                (measurement_mm >= NEAR_CENTER_STUCK_MEAS_MIN_MM) &&
                (measurement_mm <= NEAR_CENTER_STUCK_MEAS_MAX_MM) &&
                (abs_error_mm >= NEAR_CENTER_STUCK_ERR_MIN_MM) &&
                (abs_error_mm <= NEAR_CENTER_STUCK_ERR_MAX_MM) &&
                (fabsf(ctrl->control_output_deg) <= NEAR_CENTER_STUCK_CMD_MAX_DEG);

            if (in_near_center_stuck_window)
            {
                if (ctrl->near_center_stuck_counter < NEAR_CENTER_STUCK_COUNT_MAX)
                {
                    ctrl->near_center_stuck_counter++;
                }
            }
            else
            {
                ctrl->near_center_stuck_counter = 0U;
            }

            if (ctrl->near_center_stuck_counter >= NEAR_CENTER_STUCK_COUNT_MAX)
            {
                float rescue_sign = (ctrl->control_output_deg >= 0.0f) ? 1.0f : -1.0f;
                target_servo_angle += rescue_sign * NEAR_CENTER_STUCK_RESCUE_DEG;
            }

            ctrl->prev_abs_error_mm = abs_error_mm;
        }
        else
        {
            ctrl->mode = CONTROL_MODE_PD;
            ctrl->pid.integral_enabled = false;
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
    if (measurement_mm != (int32_t)INVALID_TELEMETRY_MM)
    {
        Serial_Printf("RAW_ADC:%u DIST_MM:%u MEAS_MM:%ld SETPOINT_MM:%ld ERROR_MM:%ld CMD_OUT:%ld.%01lddeg ACT_OUT:%ld.%01lddeg CMD_ANGLE:%ld.%01lddeg SERVO_ANGLE:%ld.%01lddeg MODE:%s STATE:%s\r\n",
                      (unsigned int)ctrl->last_adc_raw,
                      (unsigned int)ctrl->last_valid_distance_mm,
                      (long)measurement_mm,
                      (long)setpoint_mm,
                      (long)error_mm,
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
        Serial_Printf("RAW_ADC:%u DIST_MM:-- MEAS_MM:-- SETPOINT_MM:%ld ERROR_MM:-- CMD_OUT:%ld.%01lddeg ACT_OUT:%ld.%01lddeg CMD_ANGLE:%ld.%01lddeg SERVO_ANGLE:%ld.%01lddeg MODE:%s STATE:%s\r\n",
                      (unsigned int)ctrl->last_adc_raw,
                      (long)setpoint_mm,
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
}
