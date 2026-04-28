#ifndef GP2Y0A41_H
#define GP2Y0A41_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

/*
 * 文件综述：
 * 封装 GP2Y0A41 红外测距传感器的采样流程：
 * ADC初始化/校准、一次采样读取、以及最近一次ADC原始值回读。
 */
#define GP2Y0A41_INVALID_DISTANCE_MM 0xFFFFU

HAL_StatusTypeDef GP2Y0A41_Init(ADC_HandleTypeDef *hadc);
uint16_t GP2Y0A41_ReadDistanceMm(void);
uint16_t GP2Y0A41_GetLastRawAdc(void);

#endif
