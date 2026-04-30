#include "chassis_ctrl.h"
#include <string.h>

/*
 * 文件综述：
 * 4路电机闭环速度控制实现。
 *
 * 同步策略：
 *   编码器采样同步：关中断后连续读取4路CNT寄存器，4条LDR指令在
 *   Cortex-M3/M4 上约 4~8 个时钟周期（< 0.1μs @72MHz），采样
 *   不同步误差工程上可忽略。
 *
 *   PWM输出同步：本模块在循环内依次写入4路 duty，BSP侧需为 TIM1
 *   配置 CCR 预加载使能（影子寄存器），使4路新值在下一个 Update
 *   Event 同时生效，保证4路PWM在同一计数周期内完成切换。
 *
 * 临界区宏可由用户在包含此文件前重定义，以适配RTOS或其他平台：
 *   #define CHASSIS_ENTER_CRITICAL()  taskENTER_CRITICAL()
 *   #define CHASSIS_EXIT_CRITICAL()   taskEXIT_CRITICAL()
 */

#ifndef CHASSIS_ENTER_CRITICAL
  #ifdef __arm__
    /* Cortex-M 裸机：通过 CPS 指令关闭/开启全局中断 */
    #define CHASSIS_ENTER_CRITICAL()  __asm volatile ("cpsid i" ::: "memory")
    #define CHASSIS_EXIT_CRITICAL()   __asm volatile ("cpsie i" ::: "memory")
  #else
    /* PC 仿真 / 单元测试：空实现，方便在非ARM平台验证算法 */
    #define CHASSIS_ENTER_CRITICAL()  do {} while (0)
    #define CHASSIS_EXIT_CRITICAL()   do {} while (0)
  #endif
#endif

/* PID输出限幅（归一化 PWM 占空比） */
#define CTRL_OUTPUT_LIMIT    1.0f
#define CTRL_INTEGRAL_LIMIT  0.5f

/* 上电默认PID参数（建议通过 ChassisCtrl_SetPID 在线整定） */
#define CTRL_DEFAULT_KP  0.05f
#define CTRL_DEFAULT_KI  0.01f
#define CTRL_DEFAULT_KD  0.001f

#define CHASSIS_PI  3.14159265358979f

void ChassisCtrl_Init(ChassisCtrl_Handle *ctrl,
                      float dt,
                      uint32_t encoder_cpr,
                      MotorPWM_SetFn pwm_fn,
                      EncoderReadFn enc_fn)
{
    memset(ctrl, 0, sizeof(ChassisCtrl_Handle));

    ctrl->dt               = dt;
    ctrl->encoder_to_rad   = (2.0f * CHASSIS_PI) / (float)encoder_cpr;
    ctrl->motor_set_pwm    = pwm_fn;
    ctrl->encoder_read     = enc_fn;

    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        /*
         * integral_enter_band = 0.0f → PID_Init 内部替换为 1e6f（无限制）
         * integral_exit_band  = 0.0f → 同上，速度闭环全范围积分
         */
        PID_Init(&ctrl->pid[i],
                 CTRL_DEFAULT_KP, CTRL_DEFAULT_KI, CTRL_DEFAULT_KD,
                 dt,
                 -CTRL_OUTPUT_LIMIT, CTRL_OUTPUT_LIMIT,
                 CTRL_INTEGRAL_LIMIT,
                 0.0f, 0.0f);
        ctrl->pid[i].integral_enabled = true;
    }
}

void ChassisCtrl_SetPID(ChassisCtrl_Handle *ctrl,
                        uint8_t motor_id,
                        float kp, float ki, float kd)
{
    if (motor_id >= CHASSIS_MOTOR_COUNT) {
        return;
    }
    ctrl->pid[motor_id].Kp = kp;
    ctrl->pid[motor_id].Ki = ki;
    ctrl->pid[motor_id].Kd = kd;
}

void ChassisCtrl_SetTarget(ChassisCtrl_Handle *ctrl,
                           const WheelSpeed_t *target)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        ctrl->target.v[i] = target->v[i];
    }
}

void ChassisCtrl_Update(ChassisCtrl_Handle *ctrl)
{
    int32_t counts[CHASSIS_MOTOR_COUNT];

    /* Step 1: 关中断，原子批量读取4路编码器差值（< 0.1μs @72MHz） */
    CHASSIS_ENTER_CRITICAL();
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        counts[i] = ctrl->encoder_read(i);
    }
    CHASSIS_EXIT_CRITICAL();

    /* Step 2: 差值计数 → 轮速 (rad/s)
     *   actual = delta_counts × (2π / CPR) / dt  */
    float inv_dt = 1.0f / ctrl->dt;
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        ctrl->actual.v[i] = (float)counts[i] * ctrl->encoder_to_rad * inv_dt;
    }

    /* Step 3 & 4: PID计算 + 写入PWM影子寄存器（BSP侧预加载使能后同步生效） */
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        float output = PID_Update(&ctrl->pid[i],
                                  ctrl->target.v[i],
                                  ctrl->actual.v[i],
                                  0.0f);  /* actuator_feedback 保留扩展，当前传 0 */
        ctrl->motor_set_pwm(i, output);
    }
}

void ChassisCtrl_Stop(ChassisCtrl_Handle *ctrl)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        ctrl->target.v[i] = 0.0f;
        ctrl->motor_set_pwm(i, 0.0f);
        PID_Reset(&ctrl->pid[i], ctrl->actual.v[i]);
    }
}

void ChassisCtrl_GetActual(const ChassisCtrl_Handle *ctrl,
                           WheelSpeed_t *out)
{
    for (uint8_t i = 0U; i < CHASSIS_MOTOR_COUNT; i++) {
        out->v[i] = ctrl->actual.v[i];
    }
}
