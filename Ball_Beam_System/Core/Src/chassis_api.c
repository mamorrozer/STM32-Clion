#include "chassis_api.h"

/*
 * 文件综述：
 * 麦轮底盘控制系统对外统一接口实现（单例模式）。
 *
 * 内部维护三个静态句柄：
 *   s_params  — 底盘物理参数（供运动学解算使用）
 *   s_ctrl    — 4路电机闭环控制句柄
 *   s_fsm     — 运行模式状态机句柄
 *
 * 线程/中断安全说明：
 *   Chassis_Set_Speed() 在主循环中调用，会更新 s_ctrl.target（4个float）。
 *   Chassis_Control_IRQHandler() 在定时器中断中读取 s_ctrl.target。
 *   在 Cortex-M 上单个对齐 float 读写为原子操作，但4个 float 的批量
 *   更新非原子。若需严格保证一致性，可在调用 Chassis_Set_Speed() 前后
 *   自行添加临界区（__disable_irq / __enable_irq）。
 *   对于竞赛级5ms控制周期，偶发的一帧数据不一致会被下一帧PID纠正，
 *   实际影响极小。
 */

/* ─── 内部单例句柄 ─────────────────────────────────────────── */
static ChassisParams_t   s_params;
static ChassisCtrl_Handle s_ctrl;
static ChassisFSM_Handle  s_fsm;

/* ─── 初始化 ───────────────────────────────────────────────── */

void Chassis_Init(const ChassisParams_t *params,
                  float dt,
                  uint32_t encoder_cpr,
                  MotorPWM_SetFn pwm_fn,
                  EncoderReadFn enc_fn)
{
    s_params = *params;
    ChassisCtrl_Init(&s_ctrl, dt, encoder_cpr, pwm_fn, enc_fn);
    ChassisFSM_Init(&s_fsm);
}

/* ─── 启停控制 ─────────────────────────────────────────────── */

void Chassis_Start(void)
{
    /* 仅从 STOP 状态自动进入 RC_CTRL；其他状态（已由 Set_Mode 设置）保持 */
    if (s_fsm.state == CHASSIS_STATE_STOP) {
        ChassisFSM_SetMode(&s_fsm, CHASSIS_STATE_RC_CTRL, &s_ctrl);
    }
}

void Chassis_Stop(void)
{
    ChassisFSM_SetMode(&s_fsm, CHASSIS_STATE_STOP, &s_ctrl);
}

/* ─── 速度指令 ─────────────────────────────────────────────── */

void Chassis_Set_Speed(float vx, float vy, float wz)
{
    ChassisMode_t mode = s_fsm.state;

    /* 停止或急停状态下不接受速度指令 */
    if ((mode == CHASSIS_STATE_STOP) || (mode == CHASSIS_STATE_EMERGENCY)) {
        return;
    }

    ChassisVelocity_t cmd;
    cmd.vx = vx;
    cmd.vy = vy;
    cmd.wz = wz;

    WheelSpeed_t wheel_target;
    Kinematics_Inverse(&s_params, &cmd, &wheel_target);
    ChassisCtrl_SetTarget(&s_ctrl, &wheel_target);
}

/* ─── 模式管理 ─────────────────────────────────────────────── */

void Chassis_Set_Mode(ChassisMode_t mode)
{
    ChassisFSM_SetMode(&s_fsm, mode, &s_ctrl);
}

ChassisMode_t Chassis_Get_Mode(void)
{
    return ChassisFSM_GetMode(&s_fsm);
}

/* ─── 状态查询 ─────────────────────────────────────────────── */

void Chassis_Get_WheelSpeed(WheelSpeed_t *actual)
{
    ChassisCtrl_GetActual(&s_ctrl, actual);
}

void Chassis_Get_Velocity(ChassisVelocity_t *vel)
{
    WheelSpeed_t actual;
    ChassisCtrl_GetActual(&s_ctrl, &actual);
    Kinematics_Forward(&s_params, &actual, vel);
}

/* ─── PID 在线整定 ─────────────────────────────────────────── */

void Chassis_Set_PID(uint8_t motor_id, float kp, float ki, float kd)
{
    ChassisCtrl_SetPID(&s_ctrl, motor_id, kp, ki, kd);
}

/* ─── 紧急制动 ─────────────────────────────────────────────── */

void Chassis_Emergency_Stop(void)
{
    ChassisFSM_EmergencyStop(&s_fsm, &s_ctrl);
}

void Chassis_Emergency_Reset(void)
{
    ChassisFSM_Reset(&s_fsm, &s_ctrl);
}

/* ─── 控制节拍（在定时器中断中调用） ──────────────────────── */

void Chassis_Control_IRQHandler(void)
{
    ChassisMode_t mode = s_fsm.state;

    /* STOP / EMERGENCY 状态保持 PWM 为零，直接返回 */
    if ((mode == CHASSIS_STATE_STOP) || (mode == CHASSIS_STATE_EMERGENCY)) {
        return;
    }

    ChassisCtrl_Update(&s_ctrl);
}
