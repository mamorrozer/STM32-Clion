#ifndef PID_H
#define PID_H

#include "stdint.h"
#include <stdbool.h>

typedef struct {
    float Kp;           // 比例系数
    float Ki;           // 积分系数
    float Kd;           // 微分系数
    float integral;     // 积分累加和
    float out_min;      // 输出最小值（对应舵机最小角度）
    float out_max;      // 输出最大值（对应舵机最大角度）
    float integral_limit; // 积分限幅值（防止积分饱和）
    float integral_active_band; // 误差进入该范围才允许积分
    float dt;           // 控制周期，单位：秒
    float prev_meas; // 上一次测量值
    float d_lpf_alpha; // 微分项低通滤波系数（0~1）
    float d_filtered;  // 滤波后的微分项
    bool integral_enabled;  // 积分是否启用
} PID_Handle;

void PID_Init(PID_Handle *pid, float Kp, float Ki, float Kd,float dt, float out_min, float out_max, float integral_limit, float integral_active_band);

void PID_Reset(PID_Handle *pid, float measurement);

float PID_Update(PID_Handle *pid, float setpoint, float measurement);

#endif
