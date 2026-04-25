#include "pid.h"
#include <math.h>

#define PID_INTEGRAL_BAND_UNLIMITED  (1.0e6f)
#define PID_DERIVATIVE_FILTER_ALPHA  (0.80f)
#define PID_ANTI_WINDUP_GAIN_DEFAULT (0.30f)

// 初始化：新增控制周期 dt
void PID_Init(PID_Handle *pid, float Kp, float Ki, float Kd, float dt, float out_min, float out_max,
              float integral_limit, float integral_enter_band, float integral_exit_band)
{
    pid->Kp = Kp;
    pid->Ki = Ki;
    pid->Kd = Kd;
    pid->dt = dt;          // 控制周期（10ms=0.01s）
    pid->out_min = out_min;
    pid->out_max = out_max;
    pid->integral_limit = integral_limit;
    pid->integral_enter_band = (integral_enter_band > 0.0f) ? integral_enter_band : PID_INTEGRAL_BAND_UNLIMITED;
    pid->integral_exit_band = (integral_exit_band > pid->integral_enter_band) ? integral_exit_band : pid->integral_enter_band;

    pid->integral = 0;
    pid->prev_meas = 0;
    pid->d_lpf_alpha = PID_DERIVATIVE_FILTER_ALPHA;
    if (pid->d_lpf_alpha < 0.0f)
    {
        pid->d_lpf_alpha = 0.0f;
    }
    else if (pid->d_lpf_alpha > 1.0f)
    {
        pid->d_lpf_alpha = 1.0f;
    }
    pid->d_filtered = 0.0f;
    pid->integral_enabled = false;
    pid->integral_band_active = false;
    pid->anti_windup_gain = PID_ANTI_WINDUP_GAIN_DEFAULT;
}

void PID_Reset(PID_Handle *pid, float measurement)
{
    pid->integral = 0.0f;
    pid->prev_meas = measurement;
    pid->d_filtered = 0.0f;
    pid->integral_band_active = false;
}

float PID_Update(PID_Handle *pid, float setpoint, float measurement, float actuator_feedback)
{
    float error = setpoint - measurement;
    float abs_error = fabsf(error);

    // 1. 比例项
    float Pout = pid->Kp * error;

    // 2. 微分项
    float d_raw = (measurement - pid->prev_meas) / pid->dt;
    pid->d_filtered = pid->d_lpf_alpha * pid->d_filtered + (1.0f - pid->d_lpf_alpha) * d_raw;
    float Dout = -pid->Kd * pid->d_filtered;
    pid->prev_meas = measurement;

    // 3. 条件积分（积分使能 + 进入/退出滞回）
    if (!pid->integral_enabled)
    {
        pid->integral_band_active = false;
    }
    else if (!pid->integral_band_active)
    {
        pid->integral_band_active = (abs_error <= pid->integral_enter_band);
    }
    else
    {
        pid->integral_band_active = (abs_error < pid->integral_exit_band);
    }

    bool within_integral_band = pid->integral_enabled && pid->integral_band_active;
    float integral_candidate = pid->integral;
    if (within_integral_band)
    {
        integral_candidate += error * pid->dt;
        if(integral_candidate > pid->integral_limit)  integral_candidate = pid->integral_limit;
        if(integral_candidate < -pid->integral_limit) integral_candidate = -pid->integral_limit;
    }

    float Iout = pid->Ki * integral_candidate;
    float output_unsat = Pout + Iout + Dout;
    float output = output_unsat;

    // 输出限幅（保护舵机）
    if(output > pid->out_max) output = pid->out_max;
    if(output < pid->out_min) output = pid->out_min;

    // 抗饱和：条件积分 + back-calculation
    bool saturated_high = (output_unsat > pid->out_max);
    bool saturated_low = (output_unsat < pid->out_min);
    // 饱和且误差继续推动同方向饱和时，不做前向积分，避免 windup
    bool drives_further_into_saturation = (saturated_high && (error > 0.0f)) || (saturated_low && (error < 0.0f));
    if (within_integral_band)
    {
        float backcalc = (output - output_unsat) + actuator_feedback;
        if (drives_further_into_saturation)
        {
            integral_candidate = pid->integral;
        }

        integral_candidate += pid->anti_windup_gain * backcalc * pid->dt;
        if(integral_candidate > pid->integral_limit)  integral_candidate = pid->integral_limit;
        if(integral_candidate < -pid->integral_limit) integral_candidate = -pid->integral_limit;
        pid->integral = integral_candidate;
    }
    
    return output;
}
