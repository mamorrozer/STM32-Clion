#include "Servo.h"
#include"main.h"
#include "stm32f1xx_hal_tim.h"
extern TIM_HandleTypeDef htim3;

/*
 * 文件综述：
 * 本文件负责把“角度命令”映射为定时器比较值(CCR)，输出标准舵机PWM脉宽。
 */
void Servo_Init(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
void Servo_SetAngle(float Angle){
	if(Angle < 0) Angle = 0;
	if(Angle > 180) Angle = 180;
	uint32_t arr = __HAL_TIM_GET_AUTORELOAD(&htim3) + 1U;
	/* 20ms周期下，0.5ms~2.5ms 对应 2.5%~12.5% 占空比。 */
	float min_ccr = (float)arr * 0.025f; /* 0.5ms / 20ms = 2.5% */
	float max_ccr = (float)arr * 0.125f; /* 2.5ms / 20ms = 12.5% */
	float ccr_f = min_ccr + (Angle / 180.0f) * (max_ccr - min_ccr);
	if (ccr_f < 0.0f)
	{
		ccr_f = 0.0f;
	}
	if (ccr_f > (float)(arr - 1U))
	{
		ccr_f = (float)(arr - 1U);
	}
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, (uint16_t)(ccr_f + 0.5f));
}
