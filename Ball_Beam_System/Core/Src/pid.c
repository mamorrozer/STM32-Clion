#include "pid.h"

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

    pid->integral = 0;
    pid->prev_error = 0;
    pid->prev_meas = 0;
}

float PID_Update(PID_Handle *pid, float setpoint, float measurement)
{
    float error = setpoint - measurement;

    // 1. 比例项
    float Pout = pid->Kp * error;

    // 2. 积分项（带限幅，抗饱和）
    pid->integral += error * pid->dt;
    if(pid->integral > pid->integral_limit)  pid->integral = pid->integral_limit;
    if(pid->integral < -pid->integral_limit) pid->integral = -pid->integral_limit;
    float Iout = pid->Ki * pid->integral;

    // 3. 微分项 
    float Dout = -pid->Kd * (measurement - pid->prev_meas) / pid->dt;
    pid->prev_meas = measurement;

    // 总输出
    float output = Pout + Iout + Dout;

    // 输出限幅（保护舵机）
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;
    
    return output;
}