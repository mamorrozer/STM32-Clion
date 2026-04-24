#ifndef GP2Y0A41_H
#define GP2Y0A41_H

#include <stdint.h>
#include "stm32f1xx_hal.h"

#define GP2Y0A41_INVALID_DISTANCE_MM 0xFFFFU

HAL_StatusTypeDef GP2Y0A41_Init(ADC_HandleTypeDef *hadc);
uint16_t GP2Y0A41_ReadDistanceMm(void);
uint16_t GP2Y0A41_GetLastRawAdc(void);

#endif
