#ifndef ADC_DISTANCE_CONVERTER_H
#define ADC_DISTANCE_CONVERTER_H

#include <stdint.h>

#define ADC_DISTANCE_INVALID_MM 0xFFFFU

typedef struct
{
    uint16_t adc_raw;
    uint16_t distance_mm;
} AdcDistancePoint_t;

uint16_t AdcDistance_ConvertRawToMm(uint16_t adc_raw);

#endif
