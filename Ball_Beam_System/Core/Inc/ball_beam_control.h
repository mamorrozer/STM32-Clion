#ifndef BALL_BEAM_CONTROL_H
#define BALL_BEAM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

/*
 * 文件综述：
 * 小球平衡控制器的对外接口与状态定义。
 * 该控制器负责：
 * - 采样有效性管理；
 * - PID/PD状态切换；
 * - 远端强制回拉保护；
 * - 目标点切换与遥测输出。
 */
#define SERVO_CENTER_ANGLE            35.0f
#define SERVO_MIN_ANGLE               0.0f
#define SERVO_MAX_ANGLE               70.0f
#define SERVO_MIN_STEP_DEG            0.0f
#define SERVO_SLEW_MAX_STEP_DEG       4.0f

typedef enum {
    CONTROL_MODE_PD = 0,
    CONTROL_MODE_PID = 1
} BallBeamControlMode_t;

typedef enum {
    CONTROL_STATE_INIT = 0,
    CONTROL_STATE_RUN = 1,
    CONTROL_STATE_FAULT = 2
} BallBeamControlState_t;

typedef struct {
    PID_Handle pid;
    volatile uint16_t last_adc_raw;
    volatile uint16_t distance_mm;
    volatile uint16_t last_valid_distance_mm;
    volatile uint16_t last_valid_raw_distance_mm;
    volatile float control_output_deg;
    volatile float servo_angle;
    volatile float target_servo_angle;
    float setpoint_offset_mm;
    float setpoint_cmd_mm;
    float setpoint_slew_rate_mm_per_s;
    float control_dt_sec;
    float pid_actuator_feedback_norm;
    uint32_t serial_last_tick;
    uint32_t control_start_tick;
    uint32_t last_control_tick;
    uint16_t consecutive_invalid_samples;
    uint16_t anti_stall_counter;
    uint16_t near_center_stuck_counter;
    uint16_t far_end_exit_counter;
    uint16_t far_end_stuck_counter;
    uint16_t far_end_recovery_cooldown;
    uint8_t far_end_recovery_phase;
    uint8_t far_end_recovery_success_counter;
    float prev_abs_error_mm;
    bool center_hold_active;
    bool far_end_recovery_active;
    bool has_valid_distance;
    BallBeamControlMode_t mode;
    BallBeamControlState_t state;
} BallBeamController_t;

void BallBeamController_Init(BallBeamController_t *ctrl, uint32_t start_tick_ms);
bool BallBeamController_ShouldRunControl(const BallBeamController_t *ctrl, uint32_t now_ms);
uint16_t BallBeamController_ReadDistanceMm(void);
bool BallBeamController_Step(BallBeamController_t *ctrl, uint32_t now_ms, uint16_t sampled_distance_mm);
void BallBeamController_FormatDistanceLine(bool valid_sample, uint16_t sampled_distance_mm, char *buf, uint32_t buf_len);
int32_t BallBeamController_GetFilteredDistanceMm(const BallBeamController_t *ctrl);
int32_t BallBeamController_GetMeasurementMm(const BallBeamController_t *ctrl);
void BallBeamController_SendTelemetry(BallBeamController_t *ctrl, uint32_t now_ms);
void BallBeamController_SetSetpointMm(BallBeamController_t *ctrl, float setpoint_mm);
void BallBeamController_SetTargetDistanceMm(BallBeamController_t *ctrl, float target_distance_mm);

#endif
