#ifndef CHASSIS_FSM_H
#define CHASSIS_FSM_H

/*
 * 文件综述：
 * 底盘运行模式状态机头文件。
 *
 * 负责模式切换时的安全清零与状态合法性校验。
 * 上层应用只需调用 ChassisFSM_SetMode()，底层细节由本模块屏蔽。
 *
 * 状态转移规则（优先级从高到低）：
 *
 *   任意状态 ──[急停信号]──→ EMERGENCY（不可被其他模式覆盖）
 *   EMERGENCY ──[显式复位]──→ STOP
 *   STOP ↔ RC_CTRL ↔ AUTO_TRACK  （正常模式互相切换）
 *
 * 安全保证：
 *   每次模式切换前必调用 ChassisCtrl_Stop() 清零输出，防止速度突变。
 */

#include "chassis_ctrl.h"

/* 底盘运行模式枚举 */
typedef enum {
    CHASSIS_STATE_STOP       = 0,  /* 停止态：输出清零，PID 复位              */
    CHASSIS_STATE_RC_CTRL    = 1,  /* 遥控模式：接收摇杆/串口速度指令        */
    CHASSIS_STATE_AUTO_TRACK = 2,  /* 自动模式：接收路径规划/循迹速度指令    */
    CHASSIS_STATE_EMERGENCY  = 3   /* 紧急制动：最高优先级，锁定输出为零     */
} ChassisMode_t;

/* 状态机句柄 */
typedef struct {
    ChassisMode_t state;
} ChassisFSM_Handle;

/**
 * @brief 初始化状态机，进入 STOP 状态
 */
void ChassisFSM_Init(ChassisFSM_Handle *fsm);

/**
 * @brief 请求切换运行模式
 *        切换前自动调用 ChassisCtrl_Stop() 进行安全清零。
 *        EMERGENCY 状态下拒绝切换到其他模式，需先调用 ChassisFSM_Reset()。
 * @return  0  = 切换成功
 *         -1  = 当前为 EMERGENCY，切换被拒绝
 */
int ChassisFSM_SetMode(ChassisFSM_Handle *fsm,
                       ChassisMode_t mode,
                       ChassisCtrl_Handle *ctrl);

/**
 * @brief 触发紧急制动（任何时刻均可调用，最高优先级）
 *        立即清零 PWM 输出，并将状态锁定为 EMERGENCY。
 */
void ChassisFSM_EmergencyStop(ChassisFSM_Handle *fsm,
                              ChassisCtrl_Handle *ctrl);

/**
 * @brief 从 EMERGENCY 状态复位回 STOP
 *        须在外部确认安全后显式调用，防止自动恢复导致意外运动。
 */
void ChassisFSM_Reset(ChassisFSM_Handle *fsm,
                      ChassisCtrl_Handle *ctrl);

/**
 * @brief 读取当前运行模式（可在中断/主循环中安全调用）
 */
ChassisMode_t ChassisFSM_GetMode(const ChassisFSM_Handle *fsm);

#endif /* CHASSIS_FSM_H */
