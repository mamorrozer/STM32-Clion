#include "chassis_fsm.h"

/*
 * 文件综述：
 * 底盘运行模式状态机实现。
 *
 * 所有模式切换统一经过此模块，保证：
 *   1. 切换前安全清零（防速度突变）
 *   2. EMERGENCY 为最高优先级，不可被常规模式覆盖
 *   3. 状态转移逻辑集中管理，便于调试与扩展
 */

void ChassisFSM_Init(ChassisFSM_Handle *fsm)
{
    fsm->state = CHASSIS_STATE_STOP;
}

int ChassisFSM_SetMode(ChassisFSM_Handle *fsm,
                       ChassisMode_t mode,
                       ChassisCtrl_Handle *ctrl)
{
    /* EMERGENCY 状态由专用接口解除，普通模式切换请求一律拒绝 */
    if (fsm->state == CHASSIS_STATE_EMERGENCY) {
        return -1;
    }

    /* 若请求切换到 EMERGENCY，走专用路径（保证原子性语义一致） */
    if (mode == CHASSIS_STATE_EMERGENCY) {
        ChassisFSM_EmergencyStop(fsm, ctrl);
        return 0;
    }

    /* 切换前安全清零输出，防止速度突变造成机械冲击 */
    ChassisCtrl_Stop(ctrl);
    fsm->state = mode;
    return 0;
}

void ChassisFSM_EmergencyStop(ChassisFSM_Handle *fsm,
                              ChassisCtrl_Handle *ctrl)
{
    ChassisCtrl_Stop(ctrl);
    fsm->state = CHASSIS_STATE_EMERGENCY;
}

void ChassisFSM_Reset(ChassisFSM_Handle *fsm,
                      ChassisCtrl_Handle *ctrl)
{
    ChassisCtrl_Stop(ctrl);
    fsm->state = CHASSIS_STATE_STOP;
}

ChassisMode_t ChassisFSM_GetMode(const ChassisFSM_Handle *fsm)
{
    return fsm->state;
}
