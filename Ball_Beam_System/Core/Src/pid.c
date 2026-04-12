#include "pid.h"
#include <math.h>

// 初始化：新增控制周期 dt
void PID_Init(PID_Handle *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max, float integral_limit)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;          // 控制周期（10ms=0.01s）
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = integral_limit;
    pid->integral_active_band = 1.0e6f;

    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_meas = 0;
    pid->integral_enabled = false;
}

void PID_Reset(PID_Handle *pid, float measurement)
{
    pid->integral = 0.0f;
    pid->prev_error = 0.0f;
    pid->prev_meas = measurement;
}

float PID_Update(PID_Handle *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // 1. 比例项
    float Pout = pid->Kp * error;

    // 2. 微分项
    float Dout = -pid->Kd * (measurement - pid->prev_meas) / pid->dt;
    pid->prev_meas = measurement;

    float Iout = pid->Ki * pid->integral;
    float output_unsat = Pout + Iout + Dout;
    float output = output_unsat;

    // 输出限幅（保护舵机）
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;

    // 3. 条件积分（积分使能 + 积分分离 + 抗饱和）
    if (pid->integral_enabled && (fabsf(error) <= pid->integral_active_band))
    {
        bool saturated_high = (output_unsat > pid->out_max);
        bool saturated_low = (output_unsat < pid->out_min);
        bool drives_back_from_sat = (saturated_high && (error < 0.0f)) || (saturated_low && (error > 0.0f));

        if ((!saturated_high && !saturated_low) || drives_back_from_sat)
        {
            pid->integral += error * pid->dt;
            if(pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
            if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
        }
    }
    
    return output;
}
