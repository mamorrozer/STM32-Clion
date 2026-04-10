#include "Servo.h"
#include"main.h"
#include "stm32f1xx_hal_tim.h"
extern TIM_HandleTypeDef htim3;

void Servo_Init(void){
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
}
// void Servo_SetAngle(float Angle){
// 	if(Angle < 0) Angle = 0;
// 	if(Angle > 180) Angle = 180;
// 	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,Angle / 180  * 2000 + 500);
// }
void Servo_SetAngle(float Angle)
{
	if(Angle < 0) Angle = 0;
	if(Angle > 180) Angle = 180;
	// 映射：0° -> CCR=5, 180° -> CCR=25
	uint16_t ccr = 5 + (uint16_t)(Angle * 20.0f / 180.0f);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, ccr);
}