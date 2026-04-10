#ifndef __VL53L0X_H
#define __VL53L0X_H

#include "vl53l0x_api.h"
#include "platform/vl53l0x_platform.h"
#include "main.h"

/* Default VL53L0X I2C address after power-on/reset. */
#define VL53L0X_Addr 0x52

/* XSHUT pin mapping comes from CubeMX-generated main.h definitions. */
#define VL53L0X_XshutPort PIN_VL53L0X_XSHUT_GPIO_Port
#define VL53L0X_XshutPin  PIN_VL53L0X_XSHUT_Pin
#define VL53L0X_Xshut_HIGH()  HAL_GPIO_WritePin(VL53L0X_XshutPort, VL53L0X_XshutPin, GPIO_PIN_SET)
#define VL53L0X_Xshut_LOW()   HAL_GPIO_WritePin(VL53L0X_XshutPort, VL53L0X_XshutPin, GPIO_PIN_RESET)
#define VL53L0X_Xshut         VL53L0X_Xshut_LOW

/* Use 2.8V IO mode. */
#define USE_I2C_2V8  1

/* Ranging modes. */
#define Default_Mode   0
#define HIGH_ACCURACY  1
#define LONG_RANGE     2
#define HIGH_SPEED     3

/* VL53L0X mode parameters. */
typedef struct
{
	FixPoint1616_t signalLimit;
	FixPoint1616_t sigmaLimit;
	uint32_t timingBudget;
	uint8_t preRangeVcselPeriod;
	uint8_t finalRangeVcselPeriod;

}mode_data;

extern mode_data Mode_data[];

/* VL53L0X calibration data. */
typedef struct
{
	uint8_t  adjustok;
	uint8_t  isApertureSpads;
	uint8_t  VhvSettings;
	uint8_t  PhaseCal;
	uint32_t XTalkCalDistance;
	uint32_t XTalkCompensationRateMegaCps;
	uint32_t CalDistanceMilliMeter;
	int32_t  OffsetMicroMeter;
	uint32_t refSpadCount;

}_vl53l0x_adjust;

extern _vl53l0x_adjust Vl53l0x_data;

extern uint8_t AjustOK;
extern VL53L0X_Dev_t vl53l0x_dev;

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev);
VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, uint8_t mode);
VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
                                        VL53L0X_RangingMeasurementData_t *pdata,
                                        char *buf);
VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr);
void print_pal_error(VL53L0X_Error Status);
void vl53l0x_reset(VL53L0X_Dev_t *dev);

void vl53l0x_info(void);
#endif
