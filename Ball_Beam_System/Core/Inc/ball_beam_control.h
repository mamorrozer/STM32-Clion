#ifndef BALL_BEAM_CONTROL_H
#define BALL_BEAM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

#define SERVO_CENTER_ANGLE            35.0f
#define SERVO_MIN_ANGLE               0.0f
#define SERVO_MAX_ANGLE               70.0f
#define SERVO_MIN_STEP_DEG            0.2f
#define SERVO_SLEW_MAX_STEP_DEG       0.8f

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
    uint32_t serial_last_tick;
    uint32_t control_start_tick;
    uint32_t last_control_tick;
    uint16_t consecutive_invalid_samples;
    uint16_t anti_stall_counter;
    uint16_t near_center_stuck_counter;
    float prev_abs_error_mm;
    bool center_hold_active;
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

#endif
