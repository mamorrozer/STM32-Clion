#include "adc_distance_converter.h"

/*
 * 文件综述：
 * 本文件保存距离标定表，并执行“ADC值 -> 距离(mm)”转换。
 * 设计要点：
 * 1) 标定区用线性插值；
 * 2) 近端/远端超界做硬限幅，降低漏光和极限区误读带来的控制风险。
 */
/*
 * 在这里维护 ADC 原始值与距离(mm)的标定点。
 * 要求 adc_raw 按从大到小排列。
 *
 * 坐标系说明：
 * 这里的距离是“距离传感器探头”的绝对物理距离(mm)。
 * 导轨近端(传感器端)为 40mm，导轨控制终点为 140mm。
 * 160mm 为传感器检测极限/警告阈值，不作为常规运行目标点。
 *
 * 边界保护说明：
 * 首点(3550): 4cm近端死区限幅，过滤漏光导致的乱跳
 * 尾点(716):  16cm远端死区限幅，防止ADC反转导致行程跑飞
 */
static const AdcDistancePoint_t k_adc_distance_points[] = {
    /* 近端硬限幅区 (4.0cm) */
    {3550U,  40U},

    /* 精确标定区 (5.0cm ~ 16.0cm, 每5mm一个点, IQR过滤中位数) */
    {2747U,  50U},
    {2549U,  55U},
    {2227U,  60U},
    {2060U,  65U},
    {1891U,  70U},
    {1796U,  75U},
    {1678U,  80U},
    {1552U,  85U},
    {1450U,  90U},
    {1377U,  95U},
    {1248U, 100U},
    {1175U, 105U},
    {1127U, 110U},
    {1074U, 115U},
    {1019U, 120U},
    { 976U, 125U},
    { 910U, 130U},
    { 887U, 135U},
    { 845U, 140U},  /* <-- 导轨控制终点在这里 */
    { 804U, 145U},
    { 764U, 150U},
    { 745U, 155U},

    /* 远端硬限幅区 (16.0cm) */
    { 716U, 160U}
};

uint16_t AdcDistance_ConvertRawToMm(uint16_t adc_raw)
{
    const uint32_t point_count = (uint32_t)(sizeof(k_adc_distance_points) / sizeof(k_adc_distance_points[0]));
    if (point_count < 2U)
    {
        return ADC_DISTANCE_INVALID_MM;
    }

    /* 球贴脸或近端漏光：死死卡在40mm，防止PID误动作 */
    if (adc_raw >= k_adc_distance_points[0].adc_raw)
    {
        return k_adc_distance_points[0].distance_mm;
    }

    const uint32_t last_index = point_count - 1U;
    /* 球冲到16cm极限区：限幅在160mm，交由控制器触发回拉逻辑 */
    if (adc_raw <= k_adc_distance_points[last_index].adc_raw)
    {
        return k_adc_distance_points[last_index].distance_mm;
    }

    /* 正常区间：在相邻标定点之间线性插值（并做四舍五入）。 */
    for (uint32_t i = 0U; i < last_index; ++i)
    {
        const uint16_t raw1 = k_adc_distance_points[i].adc_raw;
        const uint16_t raw2 = k_adc_distance_points[i + 1U].adc_raw;
        if ((adc_raw <= raw1) && (adc_raw >= raw2))
        {
            const uint16_t d1 = k_adc_distance_points[i].distance_mm;
            const uint16_t d2 = k_adc_distance_points[i + 1U].distance_mm;
            const uint32_t numerator = (uint32_t)(d2 - d1) * (uint32_t)(raw1 - adc_raw);
            const uint32_t denominator = (uint32_t)(raw1 - raw2);
            return (uint16_t)(d1 + (numerator + (denominator / 2U)) / denominator);
        }
    }

    return ADC_DISTANCE_INVALID_MM;
}
