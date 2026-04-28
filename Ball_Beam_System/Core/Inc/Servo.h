#ifndef INC_SERVO_H_
#define INC_SERVO_H_

#include <stdint.h>

/*
 * 文件综述：
 * 对 TIM3_CH3 的舵机PWM输出做轻量封装：
 * - Servo_Init：启动PWM；
 * - Servo_SetAngle：角度到CCR映射，并做范围保护。
 */
void Servo_Init(void);

void Servo_SetAngle(float Angle);


#endif /* INC_SERVO_H_ */
