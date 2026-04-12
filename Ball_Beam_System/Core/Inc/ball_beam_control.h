#ifndef BALL_BEAM_CONTROL_H
#define BALL_BEAM_CONTROL_H

#include <stdbool.h>
#include <stdint.h>
#include "pid.h"

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
    volatile uint16_t distance_mm;
    volatile uint16_t last_valid_distance_mm;
    volatile float servo_angle;
    float setpoint_offset_mm;
    uint32_t serial_last_tick;
    uint32_t control_start_tick;
    uint32_t last_control_tick;
    uint16_t consecutive_invalid_samples;
    bool has_valid_distance;
    BallBeamControlMode_t mode;
    BallBeamControlState_t state;
} BallBeamController_t;

void BallBeamController_Init(BallBeamController_t *ctrl, uint32_t start_tick_ms);
bool BallBeamController_ShouldRunControl(const BallBeamController_t *ctrl, uint32_t now_ms);
uint16_t BallBeamController_ReadDistanceMm(void);
bool BallBeamController_Step(BallBeamController_t *ctrl, uint32_t now_ms, uint16_t sampled_distance_mm);
void BallBeamController_FormatDistanceLine(bool valid_sample, uint16_t sampled_distance_mm, char *buf, uint32_t buf_len);
void BallBeamController_SendTelemetry(BallBeamController_t *ctrl, uint32_t now_ms);

#endif
