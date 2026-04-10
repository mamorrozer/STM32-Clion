#ifndef PID_H
#define PID_H

#include "stdint.h"

typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;     // 积分累加和
    float prev_error;   // 上一次误差
    float out_min;      // 输出最小值（对应舵机最小角度）
    float out_max;      // 输出最大值（对应舵机最大角度）
    float integral_limit; // 积分限幅值（防止积分饱和）
    float dt;
    float prev_meas;
} PID_Handle;

void PID_Init(PID_Handle *pid, float Kp, float Ki, float Kd,float dt, float out_min, float out_max, float integral_limit);

float PID_Update(PID_Handle *pid, float setpoint, float measurement);

#endif