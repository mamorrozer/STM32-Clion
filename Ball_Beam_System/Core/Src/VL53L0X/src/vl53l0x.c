#include "vl53l0x.h"
#include <stdio.h>
#include <string.h>
#include "vl53l0x_api_strings.h"

VL53L0X_Dev_t vl53l0x_dev;
VL53L0X_DeviceInfo_t vl53l0x_dev_info;
_vl53l0x_adjust Vl53l0x_data;
uint8_t AjustOK = 0;

/* VL53L0X ranging mode table:
 * 0: default, 1: high accuracy, 2: long range, 3: high speed
 */
mode_data Mode_data[] =
{
    {(FixPoint1616_t)(0.25 * 65536), (FixPoint1616_t)(18 * 65536), 33000, 14, 10},
    {(FixPoint1616_t)(0.25 * 65536), (FixPoint1616_t)(18 * 65536), 200000, 14, 10},
    {(FixPoint1616_t)(0.10 * 65536), (FixPoint1616_t)(60 * 65536), 33000, 18, 14},
    {(FixPoint1616_t)(0.25 * 65536), (FixPoint1616_t)(32 * 65536), 20000, 14, 10},
};

void print_pal_error(VL53L0X_Error Status)
{
    char buf[VL53L0X_MAX_STRING_LENGTH];
    VL53L0X_GetPalErrorString(Status, buf);
    printf("API Status: %d : %s\r\n", Status, buf);
}

VL53L0X_Error vl53l0x_Addr_set(VL53L0X_Dev_t *dev, uint8_t newaddr)
{
    uint16_t id = 0;
    uint8_t step = 0;
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    if (newaddr == dev->I2cDevAddr)
    {
        return VL53L0X_ERROR_NONE;
    }

    /* Put I2C into standard mode before first register access. */
    status = VL53L0X_WrByte(dev, 0x88, 0x00);
    if (status != VL53L0X_ERROR_NONE)
    {
        step = 0x01;
        goto set_error;
    }

    /* Verify sensor model ID on current address. */
    status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
    if (status != VL53L0X_ERROR_NONE)
    {
        step = 0x02;
        goto set_error;
    }

    if (id == 0xEEAA)
    {
        status = VL53L0X_SetDeviceAddress(dev, newaddr);
        if (status != VL53L0X_ERROR_NONE)
        {
            step = 0x03;
            goto set_error;
        }

        dev->I2cDevAddr = newaddr;
        status = VL53L0X_RdWord(dev, VL53L0X_REG_IDENTIFICATION_MODEL_ID, &id);
        if (status != VL53L0X_ERROR_NONE)
        {
            step = 0x04;
            goto set_error;
        }
    }

set_error:
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status);
    }
    if (step != 0U)
    {
        printf("addr_set step: 0x%02X\r\n", step);
    }
    return status;
}

void vl53l0x_reset(VL53L0X_Dev_t *dev)
{
    const uint8_t original_addr = dev->I2cDevAddr;

    VL53L0X_Xshut_LOW();
    HAL_Delay(30);
    VL53L0X_Xshut_HIGH();
    HAL_Delay(30);

    dev->I2cDevAddr = VL53L0X_Addr;
    (void)vl53l0x_Addr_set(dev, original_addr);
    (void)VL53L0X_DataInit(dev);
}

VL53L0X_Error vl53l0x_init(VL53L0X_Dev_t *dev)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    dev->I2cDevAddr = VL53L0X_Addr;
    dev->comms_type = 1;
    dev->comms_speed_khz = 400;

    VL53L0X_Xshut_LOW();
    HAL_Delay(30);
    VL53L0X_Xshut_HIGH();
    HAL_Delay(30);

    status = vl53l0x_Addr_set(dev, 0x54);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }

    status = VL53L0X_DataInit(dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }

    HAL_Delay(2);
    status = VL53L0X_GetDeviceInfo(dev, &vl53l0x_dev_info);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }

    AjustOK = (Vl53l0x_data.adjustok == 0xAAU) ? 1U : 0U;

error:
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status);
    }
    return status;
}

VL53L0X_Error vl53l0x_set_mode(VL53L0X_Dev_t *dev, uint8_t mode)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;
    uint8_t vhv_settings = 0;
    uint8_t phase_cal = 0;
    uint32_t ref_spad_count = 0;
    uint8_t is_aperture_spads = 0;

    if (mode > HIGH_SPEED)
    {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    /* Reset before changing profile to keep ranging data stable. */
    vl53l0x_reset(dev);

    status = VL53L0X_StaticInit(dev);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }

    if (AjustOK != 0U)
    {
        status = VL53L0X_SetReferenceSpads(dev, Vl53l0x_data.refSpadCount, Vl53l0x_data.isApertureSpads);
        if (status != VL53L0X_ERROR_NONE)
        {
            goto error;
        }
        HAL_Delay(2);

        status = VL53L0X_SetRefCalibration(dev, Vl53l0x_data.VhvSettings, Vl53l0x_data.PhaseCal);
        if (status != VL53L0X_ERROR_NONE)
        {
            goto error;
        }
        HAL_Delay(2);

        status = VL53L0X_SetOffsetCalibrationDataMicroMeter(dev, Vl53l0x_data.OffsetMicroMeter);
        if (status != VL53L0X_ERROR_NONE)
        {
            goto error;
        }
        HAL_Delay(2);

        status = VL53L0X_SetXTalkCompensationRateMegaCps(dev, Vl53l0x_data.XTalkCompensationRateMegaCps);
        if (status != VL53L0X_ERROR_NONE)
        {
            goto error;
        }
        HAL_Delay(2);
    }
    else
    {
        status = VL53L0X_PerformRefCalibration(dev, &vhv_settings, &phase_cal);
        if (status != VL53L0X_ERROR_NONE)
        {
            goto error;
        }
        HAL_Delay(2);

        status = VL53L0X_PerformRefSpadManagement(dev, &ref_spad_count, &is_aperture_spads);
        if (status != VL53L0X_ERROR_NONE)
        {
            goto error;
        }
        HAL_Delay(2);
    }

    status = VL53L0X_SetDeviceMode(dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetLimitCheckEnable(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, Mode_data[mode].sigmaLimit);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetLimitCheckValue(dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, Mode_data[mode].signalLimit);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetMeasurementTimingBudgetMicroSeconds(dev, Mode_data[mode].timingBudget);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_PRE_RANGE, Mode_data[mode].preRangeVcselPeriod);
    if (status != VL53L0X_ERROR_NONE)
    {
        goto error;
    }
    HAL_Delay(2);

    status = VL53L0X_SetVcselPulsePeriod(dev, VL53L0X_VCSEL_PERIOD_FINAL_RANGE, Mode_data[mode].finalRangeVcselPeriod);

error:
    if (status != VL53L0X_ERROR_NONE)
    {
        print_pal_error(status);
    }
    return status;
}

VL53L0X_Error vl53l0x_start_single_test(VL53L0X_Dev_t *dev,
                                        VL53L0X_RangingMeasurementData_t *pdata,
                                        char *buf)
{
    VL53L0X_Error status = VL53L0X_ERROR_NONE;

    status = VL53L0X_PerformSingleRangingMeasurement(dev, pdata);
    if (status != VL53L0X_ERROR_NONE)
    {
        return status;
    }

    memset(buf, 0, VL53L0X_MAX_STRING_LENGTH);
    VL53L0X_GetRangeStatusString(pdata->RangeStatus, buf);
    return status;
}

void vl53l0x_info(void)
{
    printf("\r\n------- VL53L0X Device Info -------\r\n\r\n");
    printf("  Name: %s\r\n", vl53l0x_dev_info.Name);
    printf("  Addr: 0x%02X\r\n", vl53l0x_dev.I2cDevAddr);
    printf("  ProductId: %s\r\n", vl53l0x_dev_info.ProductId);
    printf("  RevisionMajor: 0x%02X\r\n", vl53l0x_dev_info.ProductRevisionMajor);
    printf("  RevisionMinor: 0x%02X\r\n", vl53l0x_dev_info.ProductRevisionMinor);
    printf("\r\n-----------------------------------\r\n");
}
