#ifndef KINEMATICS_H
#define KINEMATICS_H

/*
 * 文件综述：
 * 麦轮底盘运动学解算模块头文件。
 * 无任何HAL/STM32依赖，可在任意平台编译。
 *
 * 坐标系约定：
 *   +X : 车体右侧
 *   +Y : 车头前进方向
 *   +Wz: 逆时针旋转（右手定则，Z轴朝上）
 *
 * 轮子索引 0~3：
 *   [0] FL (左前)  辊子 +45°（正斜 / 形）
 *   [1] FR (右前)  辊子 -45°（反斜 \ 形）
 *   [2] RL (左后)  辊子 -45°（反斜 \ 形）
 *   [3] RR (右后)  辊子 +45°（正斜 / 形）
 *
 * 逆运动学公式（v 单位：rad/s）：
 *   v_FL = (1/R)(  Vx + Vy - (L+W)*Wz )
 *   v_FR = (1/R)(  Vx - Vy + (L+W)*Wz )
 *   v_RL = (1/R)(  Vx - Vy - (L+W)*Wz )
 *   v_RR = (1/R)(  Vx + Vy + (L+W)*Wz )
 */

/* 底盘速度指令（底盘坐标系） */
typedef struct {
    float vx;  /* m/s，向右为正  */
    float vy;  /* m/s，向前为正  */
    float wz;  /* rad/s，逆时针为正 */
} ChassisVelocity_t;

/* 4轮转速（rad/s，正值=轮子前滚=电机正转） */
typedef struct {
    float v[4];  /* 索引: 0=FL, 1=FR, 2=RL, 3=RR */
} WheelSpeed_t;

/* 底盘物理参数 */
typedef struct {
    float wheel_radius;      /* R：车轮半径 (m)              */
    float half_track_width;  /* L：左右轮距的一半 (m)        */
    float half_wheel_base;   /* W：前后轮距的一半 (m)        */
} ChassisParams_t;

/**
 * @brief 逆运动学解算：底盘速度 → 各轮目标转速
 * @param params  底盘物理参数（非空）
 * @param cmd     底盘速度指令（非空）
 * @param out     各轮目标转速输出，rad/s（非空）
 */
void Kinematics_Inverse(const ChassisParams_t *params,
                        const ChassisVelocity_t *cmd,
                        WheelSpeed_t *out);

/**
 * @brief 正运动学解算：各轮实测转速 → 底盘估算速度（里程计/调试用）
 *        采用 Moore-Penrose 伪逆（4轮过定约束取平均）。
 * @param params  底盘物理参数（非空）
 * @param actual  各轮实测转速，rad/s（非空）
 * @param out     底盘估算速度输出（非空）
 */
void Kinematics_Forward(const ChassisParams_t *params,
                        const WheelSpeed_t *actual,
                        ChassisVelocity_t *out);

#endif /* KINEMATICS_H */
