#include "gp2y0a41.h"
#include "adc_distance_converter.h"

/*
 * 文件综述：
 * 该文件实现 GP2Y0A41 的底层采样时序：
 * 触发ADC -> 轮询完成 -> 读取原始值 -> 转换为距离。
 */
static ADC_HandleTypeDef *s_hadc = NULL;
static uint16_t s_last_adc_raw = 0xFFFFU;

HAL_StatusTypeDef GP2Y0A41_Init(ADC_HandleTypeDef *hadc)
{
    if (hadc == NULL)
    {
        return HAL_ERROR;
    }

    s_hadc = hadc;
    for (uint32_t attempt = 0U; attempt < 3U; ++attempt)
    {
        (void)HAL_ADC_Stop(s_hadc);
        if (HAL_ADCEx_Calibration_Start(s_hadc) == HAL_OK)
        {
            return HAL_OK;
        }
        HAL_Delay(2U);
    }

    return HAL_ERROR;
}

uint16_t GP2Y0A41_ReadDistanceMm(void)
{
    if (s_hadc == NULL)
    {
        s_last_adc_raw = 0xFFFFU;
        return GP2Y0A41_INVALID_DISTANCE_MM;
    }

    if (HAL_ADC_Start(s_hadc) != HAL_OK)
    {
        s_last_adc_raw = 0xFFFFU;
        return GP2Y0A41_INVALID_DISTANCE_MM;
    }

    if (HAL_ADC_PollForConversion(s_hadc, 5U) != HAL_OK)
    {
        /* 采样超时直接判无效，避免主控制使用不确定值。 */
        (void)HAL_ADC_Stop(s_hadc);
        s_last_adc_raw = 0xFFFFU;
        return GP2Y0A41_INVALID_DISTANCE_MM;
    }

    uint32_t adc_raw = HAL_ADC_GetValue(s_hadc);
    (void)HAL_ADC_Stop(s_hadc);

    if (adc_raw > 4095U)
    {
        s_last_adc_raw = 0xFFFFU;
        return GP2Y0A41_INVALID_DISTANCE_MM;
    }

    s_last_adc_raw = (uint16_t)adc_raw;
    return AdcDistance_ConvertRawToMm(s_last_adc_raw);
}

uint16_t GP2Y0A41_GetLastRawAdc(void)
{
    return s_last_adc_raw;
}
