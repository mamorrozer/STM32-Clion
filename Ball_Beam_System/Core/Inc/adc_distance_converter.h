#ifndef ADC_DISTANCE_CONVERTER_H
#define ADC_DISTANCE_CONVERTER_H

#include <stdint.h>

/*
 * 文件综述：
 * 提供 GP2Y0A41 的 ADC 原始值到物理距离(mm)转换接口。
 * 采用分段线性插值，并对传感器两端死区做硬限幅保护。
 */
#define ADC_DISTANCE_INVALID_MM 0xFFFFU

typedef struct
{
    uint16_t adc_raw;
    uint16_t distance_mm;
} AdcDistancePoint_t;

uint16_t AdcDistance_ConvertRawToMm(uint16_t adc_raw);

#endif
