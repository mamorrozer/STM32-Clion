#ifndef CHASSIS_CTRL_H
#define CHASSIS_CTRL_H

/*
 * 文件综述：
 * 4路电机闭环速度控制模块头文件。
 *
 * 通过函数指针与底层BSP完全解耦，无直接HAL依赖。
 * 在定时器中断中调用 ChassisCtrl_Update() 完成完整控制周期：
 *   Step 1: 原子读取4路编码器计数（临界区保护）
 *   Step 2: 计数差值换算为轮速 (rad/s)
 *   Step 3: 各路独立运行PID计算
 *   Step 4: 输出PWM（-1.0~+1.0归一化占空比）
 *
 * PWM同步说明：
 *   BSP侧应为TIM1配置CCR预加载使能（影子寄存器），使4路CCR在同一
 *   Update Event 同时生效，保证4路PWM严格同步切换。
 *
 * 电机编号约定（与 kinematics.h 一致）：
 *   0=FL(左前), 1=FR(右前), 2=RL(左后), 3=RR(右后)
 */

#include <stdint.h>
#include "pid.h"
#include "kinematics.h"

#define CHASSIS_MOTOR_COUNT  4U

/**
 * 电机PWM设置回调
 * @param motor_id  电机编号 0~3
 * @param duty      归一化占空比，范围 -1.0~+1.0
 *                  正值=正转（轮子前滚），负值=反转
 * 由用户提供BSP实现（例如通过 HAL_TIM_PWM 或直接写 TIM->CCR）。
 * 建议在此回调内配置方向引脚与PWM幅值，TIM1需开启CCR预加载以保证同步。
 */
typedef void (*MotorPWM_SetFn)(uint8_t motor_id, float duty);

/**
 * 编码器读取回调：返回自上次调用以来的有符号计数差值（delta counts）。
 * @param motor_id  电机编号 0~3
 * @return  有符号32位差值（正=正转方向）
 *
 * BSP实现建议（以16位定时器为例，防溢出最优方案）：
 *   int32_t BSP_Encoder_ReadReset(uint8_t id) {
 *       int32_t delta = (int32_t)(int16_t)TIMx->CNT;  // 带符号扩展
 *       TIMx->CNT = 0;
 *       return delta;
 *   }
 * 注意：此回调由 ChassisCtrl_Update 在临界区内批量调用，实现须轻量。
 */
typedef int32_t (*EncoderReadFn)(uint8_t motor_id);

/* 底盘闭环控制句柄（每个底盘实例独立一个） */
typedef struct {
    PID_Handle       pid[CHASSIS_MOTOR_COUNT];   /* 4路独立速度PID        */
    WheelSpeed_t     target;                      /* 目标轮速 (rad/s)      */
    WheelSpeed_t     actual;                      /* 实测轮速 (rad/s)      */
    float            dt;                          /* 控制周期 (s)          */
    float            encoder_to_rad;             /* 计数/周期 → rad/s 系数 */
    MotorPWM_SetFn   motor_set_pwm;               /* BSP: PWM输出回调      */
    EncoderReadFn    encoder_read;                /* BSP: 编码器读取回调   */
} ChassisCtrl_Handle;

/**
 * @brief 初始化底盘闭环控制句柄
 * @param ctrl          控制句柄（非空）
 * @param dt            控制周期，秒（例如 0.005f 对应 5ms）
 * @param encoder_cpr   编码器每圈计数（四倍频后，例如 330线*4=1320）
 * @param pwm_fn        PWM设置回调（非空）
 * @param enc_fn        编码器读取回调（非空）
 */
void ChassisCtrl_Init(ChassisCtrl_Handle *ctrl,
                      float dt,
                      uint32_t encoder_cpr,
                      MotorPWM_SetFn pwm_fn,
                      EncoderReadFn enc_fn);

/**
 * @brief 在线整定单路电机PID参数（可在运行时调用）
 * @param motor_id  电机编号 0~3
 */
void ChassisCtrl_SetPID(ChassisCtrl_Handle *ctrl,
                        uint8_t motor_id,
                        float kp, float ki, float kd);

/**
 * @brief 设置4轮目标转速（由上层在主循环中调用，非阻塞）
 * @param target  各轮目标转速 (rad/s)，正值=正转
 */
void ChassisCtrl_SetTarget(ChassisCtrl_Handle *ctrl,
                           const WheelSpeed_t *target);

/**
 * @brief 核心控制更新：在定时器中断中调用
 *        完整执行：编码器采样 → 速度计算 → 4路PID → PWM输出
 */
void ChassisCtrl_Update(ChassisCtrl_Handle *ctrl);

/**
 * @brief 强制停止：清零PWM输出并复位全部PID状态
 */
void ChassisCtrl_Stop(ChassisCtrl_Handle *ctrl);

/**
 * @brief 读取4轮实测转速（供上层查询/调试）
 */
void ChassisCtrl_GetActual(const ChassisCtrl_Handle *ctrl,
                           WheelSpeed_t *out);

#endif /* CHASSIS_CTRL_H */
