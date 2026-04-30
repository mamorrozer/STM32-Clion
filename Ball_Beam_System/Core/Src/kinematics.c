#include "kinematics.h"

/*
 * 文件综述：
 * 麦轮底盘运动学解算实现。
 * 无任何HAL/STM32依赖，纯数学计算，可在任意平台直接编译。
 */

/*
 * 逆运动学矩阵展开（v 单位：rad/s，R=轮半径，k=L+W）：
 *   v_FL = (1/R)(  Vx + Vy - k*Wz )
 *   v_FR = (1/R)(  Vx - Vy + k*Wz )
 *   v_RL = (1/R)(  Vx - Vy - k*Wz )
 *   v_RR = (1/R)(  Vx + Vy + k*Wz )
 *
 * 方向自检（以下均满足）：
 *   向前(Vy+): FL+, FR-, RL-, RR+
 *   向右(Vx+): FL+, FR+, RL+, RR+
 *   顺时针(Wz-): FL+, FR-, RL+, RR-
 */
void Kinematics_Inverse(const ChassisParams_t *params,
                        const ChassisVelocity_t *cmd,
                        WheelSpeed_t *out)
{
    float k     = params->half_track_width + params->half_wheel_base;  /* L + W */
    float inv_r = 1.0f / params->wheel_radius;

    out->v[0] = inv_r * ( cmd->vx + cmd->vy - k * cmd->wz );  /* FL */
    out->v[1] = inv_r * ( cmd->vx - cmd->vy + k * cmd->wz );  /* FR */
    out->v[2] = inv_r * ( cmd->vx - cmd->vy - k * cmd->wz );  /* RL */
    out->v[3] = inv_r * ( cmd->vx + cmd->vy + k * cmd->wz );  /* RR */
}

/*
 * 正运动学（伪逆，4方程3未知数，最小二乘平均）：
 *   Vx = R/4 * ( v_FL + v_FR + v_RL + v_RR )
 *   Vy = R/4 * ( v_FL - v_FR - v_RL + v_RR )
 *   Wz = R/(4*(L+W)) * ( -v_FL + v_FR - v_RL + v_RR )
 */
void Kinematics_Forward(const ChassisParams_t *params,
                        const WheelSpeed_t *actual,
                        ChassisVelocity_t *out)
{
    float k  = params->half_track_width + params->half_wheel_base;
    float r4 = params->wheel_radius * 0.25f;

    out->vx = r4 * ( actual->v[0] + actual->v[1] + actual->v[2] + actual->v[3] );
    out->vy = r4 * ( actual->v[0] - actual->v[1] - actual->v[2] + actual->v[3] );
    out->wz = (params->wheel_radius / (4.0f * k)) *
              ( -actual->v[0] + actual->v[1] - actual->v[2] + actual->v[3] );
}
