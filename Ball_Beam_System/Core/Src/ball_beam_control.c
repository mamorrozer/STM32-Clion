#include "ball_beam_control.h"

#include <stdio.h>
#include "Servo.h"
#include "vl53l0x.h"
#include "Serial.h"

#define CONTROL_PERIOD_MS             10U
#define TELEMETRY_PERIOD_MS           100U
#define PID_ENABLE_DELAY_MS           1200U
#define MAX_CONSEC_INVALID_SAMPLES    30U

#define SENSOR_VALID_MIN_MM           40U
#define SENSOR_VALID_MAX_MM           360U
#define SENSOR_MAX_STEP_MM            60U

#define DISTANCE_CENTER_MM            160.0f
#define DEFAULT_TARGET_OFFSET_MM      0.0f

#define SERVO_CENTER_ANGLE            90.0f
#define SERVO_MIN_ANGLE               55.0f
#define SERVO_MAX_ANGLE               125.0f
#define SERVO_MAX_STEP_DEG            1.5f

#define PID_OUT_MIN_DEG               (-25.0f)
#define PID_OUT_MAX_DEG               (25.0f)
#define PID_INTEGRAL_ACTIVE_BAND_MM   45.0f
#define PID_KP                        0.20f
#define PID_KI                        0.015f
#define PID_KD                        0.12f
#define CONTROL_PERIOD_SEC            0.01f
#define PID_INTEGRAL_LIMIT            120.0f
#define INVALID_TELEMETRY_MM          (-999.0f)

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

static float distance_to_offset_mm(uint16_t distance_mm)
{
    return (float)distance_mm - DISTANCE_CENTER_MM;
}

static uint16_t abs_diff_u16(uint16_t a, uint16_t b)
{
    return (a > b) ? (a - b) : (b - a);
}

void BallBeamController_Init(BallBeamController_t *ctrl, uint32_t start_tick_ms)
{
    ctrl->distance_mm = 0xFFFFU;
    ctrl->last_valid_distance_mm = 0U;
    ctrl->servo_angle = SERVO_CENTER_ANGLE;
    ctrl->setpoint_offset_mm = DEFAULT_TARGET_OFFSET_MM;
    ctrl->serial_last_tick = 0U;
    ctrl->control_start_tick = start_tick_ms;
    ctrl->last_control_tick = start_tick_ms;
    ctrl->consecutive_invalid_samples = 0U;
    ctrl->has_valid_distance = false;
    ctrl->mode = CONTROL_MODE_PD;
    ctrl->state = CONTROL_STATE_INIT;

    Servo_SetAngle(SERVO_CENTER_ANGLE);
    PID_Init(&ctrl->pid, PID_KP, PID_KI, PID_KD, CONTROL_PERIOD_SEC, PID_OUT_MIN_DEG, PID_OUT_MAX_DEG, PID_INTEGRAL_LIMIT, PID_INTEGRAL_ACTIVE_BAND_MM);
    ctrl->pid.integral_enabled = false;
    PID_Reset(&ctrl->pid, 0.0f);
}

bool BallBeamController_ShouldRunControl(const BallBeamController_t *ctrl, uint32_t now_ms)
{
    return (uint32_t)(now_ms - ctrl->last_control_tick) >= CONTROL_PERIOD_MS;
}

uint16_t BallBeamController_ReadDistanceMm(void)
{
    VL53L0X_RangingMeasurementData_t ranging_data;
    static char range_status_buf[VL53L0X_MAX_STRING_LENGTH];

    if (vl53l0x_start_single_test(&vl53l0x_dev, &ranging_data, range_status_buf) != VL53L0X_ERROR_NONE)
    {
        return 0xFFFFU;
    }

    if (ranging_data.RangeStatus != 0U)
    {
        return 0xFFFFU;
    }

    return ranging_data.RangeMilliMeter;
}

bool BallBeamController_Step(BallBeamController_t *ctrl, uint32_t now_ms, uint16_t sampled_distance_mm)
{
    ctrl->last_control_tick = now_ms;
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
            uint16_t delta = abs_diff_u16(sampled_distance_mm, ctrl->last_valid_distance_mm);
            valid_sample = (delta <= SENSOR_MAX_STEP_MM);
        }
    }

    if (valid_sample)
    {
        ctrl->last_valid_distance_mm = sampled_distance_mm;
        ctrl->has_valid_distance = true;
        ctrl->consecutive_invalid_samples = 0U;

        float measurement_mm = distance_to_offset_mm(sampled_distance_mm);
        if (ctrl->state != CONTROL_STATE_RUN)
        {
            ctrl->state = CONTROL_STATE_RUN;
            PID_Reset(&ctrl->pid, measurement_mm);
        }

        if ((uint32_t)(now_ms - ctrl->control_start_tick) >= PID_ENABLE_DELAY_MS)
        {
            ctrl->pid.integral_enabled = true;
            ctrl->mode = CONTROL_MODE_PID;
        }
        else
        {
            ctrl->pid.integral_enabled = false;
            ctrl->mode = CONTROL_MODE_PD;
        }

        float control_output_deg = PID_Update(&ctrl->pid, ctrl->setpoint_offset_mm, measurement_mm);
        float target_servo_angle = SERVO_CENTER_ANGLE + control_output_deg;
        target_servo_angle = clampf(target_servo_angle, SERVO_MIN_ANGLE, SERVO_MAX_ANGLE);
        ctrl->servo_angle = apply_slew_limit(ctrl->servo_angle, target_servo_angle, SERVO_MAX_STEP_DEG);
        Servo_SetAngle(ctrl->servo_angle);
    }
    else
    {
        ctrl->consecutive_invalid_samples++;
        if (ctrl->consecutive_invalid_samples > MAX_CONSEC_INVALID_SAMPLES)
        {
            ctrl->state = CONTROL_STATE_FAULT;
            ctrl->pid.integral_enabled = false;
            ctrl->mode = CONTROL_MODE_PD;
            PID_Reset(&ctrl->pid, 0.0f);
            ctrl->servo_angle = apply_slew_limit(ctrl->servo_angle, SERVO_CENTER_ANGLE, SERVO_MAX_STEP_DEG);
            Servo_SetAngle(ctrl->servo_angle);
        }
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

void BallBeamController_SendTelemetry(BallBeamController_t *ctrl, uint32_t now_ms)
{
    if ((uint32_t)(now_ms - ctrl->serial_last_tick) < TELEMETRY_PERIOD_MS)
    {
        return;
    }

    ctrl->serial_last_tick = now_ms;
    float measurement_mm = ctrl->has_valid_distance ? distance_to_offset_mm(ctrl->last_valid_distance_mm) : INVALID_TELEMETRY_MM;
    float error_mm = ctrl->has_valid_distance ? (ctrl->setpoint_offset_mm - measurement_mm) : INVALID_TELEMETRY_MM;

    Serial_Printf("T=%.1f M=%.1f E=%.1f U=%.1f ANG=%.1f MODE=%u STATE=%u\r\n",
                  ctrl->setpoint_offset_mm,
                  measurement_mm,
                  error_mm,
                  ctrl->servo_angle - SERVO_CENTER_ANGLE,
                  ctrl->servo_angle,
                  (unsigned int)ctrl->mode,
                  (unsigned int)ctrl->state);
}
