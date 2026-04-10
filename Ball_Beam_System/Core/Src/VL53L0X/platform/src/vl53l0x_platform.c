#include "../../../../Inc/VL53L0X/platform/vl53l0x_platform.h"
#include "vl53l0x_def.h"
#include "i2c.h"
#include "main.h"

extern I2C_HandleTypeDef hi2c2;

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    if (HAL_I2C_Mem_Write(&hi2c2, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 1000) != HAL_OK)
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    if (HAL_I2C_Mem_Read(&hi2c2, Dev->I2cDevAddr, index, I2C_MEMADD_SIZE_8BIT, pdata, count, 1000) != HAL_OK)
    {
        Status = VL53L0X_ERROR_CONTROL_INTERFACE;
    }
    return Status;
}

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    return VL53L0X_WriteMulti(Dev, index, &data, 1);
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    uint8_t buf[2] = {data >> 8, data & 0xFF};
    return VL53L0X_WriteMulti(Dev, index, buf, 2);
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    uint8_t buf[4] = {data >> 24, (data >> 16) & 0xFF, (data >> 8) & 0xFF, data & 0xFF};
    return VL53L0X_WriteMulti(Dev, index, buf, 4);
}

VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    VL53L0X_Error Status = VL53L0X_ERROR_NONE;
    uint8_t data;
    Status = VL53L0X_RdByte(Dev, index, &data);
    if (Status == VL53L0X_ERROR_NONE)
    {
        data = (data & AndData) | OrData;
        Status = VL53L0X_WrByte(Dev, index, data);
    }
    return Status;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    return VL53L0X_ReadMulti(Dev, index, data, 1);
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    uint8_t buf[2];
    VL53L0X_Error Status = VL53L0X_ReadMulti(Dev, index, buf, 2);
    *data = (buf[0] << 8) | buf[1];
    return Status;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    uint8_t buf[4];
    VL53L0X_Error Status = VL53L0X_ReadMulti(Dev, index, buf, 4);
    *data = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    return Status;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}