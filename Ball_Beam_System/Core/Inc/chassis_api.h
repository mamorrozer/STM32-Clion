#ifndef CHASSIS_API_H
#define CHASSIS_API_H

/*
 * 文件综述：
 * 麦轮底盘控制系统对外统一接口（门面模式，Facade Pattern）。
 *
 * 上层应用只需包含此单一头文件，无需了解 kinematics/chassis_ctrl/
 * chassis_fsm 等内部模块的细节。
 *
 * ═══════════════════════════════════════════════════════════════
 *  典型使用流程（裸机）
 * ═══════════════════════════════════════════════════════════════
 *  1. 在 BSP 初始化完成后调用 Chassis_Init()：
 *
 *     ChassisParams_t params = {
 *         .wheel_radius     = 0.05f,   // 轮半径 5cm
 *         .half_track_width = 0.15f,   // 左右轮距一半 15cm
 *         .half_wheel_base  = 0.15f,   // 前后轮距一半 15cm
 *     };
 *     Chassis_Init(&params, 0.005f, 1320,
 *                  BSP_Motor_SetPWM,
 *                  BSP_Encoder_ReadReset);
 *
 *  2. 选择模式并启动：
 *
 *     Chassis_Set_Mode(CHASSIS_STATE_RC_CTRL);
 *     Chassis_Start();
 *     // BSP 侧需另行启动 TIM6/7 产生 5ms 中断
 *
 *  3. 主循环中下发速度指令（非阻塞）：
 *
 *     Chassis_Set_Speed(vx, vy, wz);
 *
 *  4. 在 TIM6/7 Update IRQ 中调用控制节拍：
 *
 *     void TIM6_DAC_IRQHandler(void) {
 *         if (__HAL_TIM_GET_FLAG(&htim6, TIM_FLAG_UPDATE)) {
 *             __HAL_TIM_CLEAR_FLAG(&htim6, TIM_FLAG_UPDATE);
 *             Chassis_Control_IRQHandler();
 *         }
 *     }
 *
 * ═══════════════════════════════════════════════════════════════
 *  BSP 回调签名
 * ═══════════════════════════════════════════════════════════════
 *  void BSP_Motor_SetPWM(uint8_t motor_id, float duty);
 *      motor_id : 0=FL, 1=FR, 2=RL, 3=RR
 *      duty     : -1.0~+1.0，正值=正转（轮子前滚）
 *
 *  int32_t BSP_Encoder_ReadReset(uint8_t motor_id);
 *      返回自上次调用以来的有符号计数差值（delta counts）。
 *      实现示例（16位定时器）：
 *          int32_t delta = (int32_t)(int16_t)TIMx->CNT;
 *          TIMx->CNT = 0;
 *          return delta;
 *
 *  TIM1 PWM同步配置要点（BSP侧，保证4路PWM同步切换）：
 *      TIM1->CCMR1 |= TIM_CCMR1_OC1PE | TIM_CCMR1_OC2PE;
 *      TIM1->CCMR2 |= TIM_CCMR2_OC3PE | TIM_CCMR2_OC4PE;
 *      TIM1->CR1   |= TIM_CR1_ARPE;
 * ═══════════════════════════════════════════════════════════════
 */

#include <stdint.h>
#include "kinematics.h"
#include "chassis_ctrl.h"
#include "chassis_fsm.h"

/**
 * @brief 底盘系统初始化（上电后必须首先调用）
 * @param params        底盘物理参数（非空）
 * @param dt            控制周期 (s)，推荐 0.005f (5ms)
 * @param encoder_cpr   编码器每圈计数，四倍频后（如 330线×4=1320）
 * @param pwm_fn        BSP: 电机 PWM 设置回调（非空）
 * @param enc_fn        BSP: 编码器读取回调（非空）
 */
void Chassis_Init(const ChassisParams_t *params,
                  float dt,
                  uint32_t encoder_cpr,
                  MotorPWM_SetFn pwm_fn,
                  EncoderReadFn enc_fn);

/**
 * @brief 使能控制输出
 *        若当前为 STOP 状态则自动切换到 RC_CTRL 模式，
 *        其余状态（含已设置的模式）保持不变。
 *        注意：BSP 侧的定时器需另行启动（HAL_TIM_Base_Start_IT 等）。
 */
void Chassis_Start(void);

/**
 * @brief 安全停止：清零 PWM，复位 PID，进入 STOP 状态
 */
void Chassis_Stop(void);

/**
 * @brief 设置底盘速度指令（非阻塞，仅更新目标值，由定时器中断执行闭环）
 *        STOP 和 EMERGENCY 状态下调用此函数无效。
 * @param vx  X轴线速度 (m/s)，向右为正
 * @param vy  Y轴线速度 (m/s)，向前为正
 * @param wz  绕Z轴角速度 (rad/s)，逆时针为正
 */
void Chassis_Set_Speed(float vx, float vy, float wz);

/**
 * @brief 切换运行模式（切换前自动安全清零）
 * @param mode  目标模式
 */
void Chassis_Set_Mode(ChassisMode_t mode);

/**
 * @brief 读取当前运行模式
 */
ChassisMode_t Chassis_Get_Mode(void);

/**
 * @brief 读取4轮实测转速（调试/监控用）
 * @param actual  输出缓冲区（非空），单位 rad/s
 */
void Chassis_Get_WheelSpeed(WheelSpeed_t *actual);

/**
 * @brief 读取正解算后的底盘估算速度（里程计/调试用）
 * @param vel  输出缓冲区（非空）
 */
void Chassis_Get_Velocity(ChassisVelocity_t *vel);

/**
 * @brief 在线整定单路电机 PID 参数（调试接口，发布版可移除）
 * @param motor_id  0=FL, 1=FR, 2=RL, 3=RR
 */
void Chassis_Set_PID(uint8_t motor_id, float kp, float ki, float kd);

/**
 * @brief 触发紧急制动（最高优先级，可在任意中断/任务中调用）
 *        立即清零 PWM 并锁定状态为 EMERGENCY，须通过
 *        Chassis_Emergency_Reset() 解除。
 */
void Chassis_Emergency_Stop(void);

/**
 * @brief 从 EMERGENCY 状态复位到 STOP（须外部确认安全后调用）
 */
void Chassis_Emergency_Reset(void);

/**
 * @brief 控制节拍中断处理函数
 *        在 TIM6/7 Update IRQ 中调用，执行完整的采样→PID→输出周期。
 *        STOP 和 EMERGENCY 状态下自动跳过，不输出任何 PWM。
 */
void Chassis_Control_IRQHandler(void);

#endif /* CHASSIS_API_H */
