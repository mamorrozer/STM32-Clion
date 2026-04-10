#include "../Inc/Serial.h"
#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "main.h"

extern UART_HandleTypeDef huart1;

void Serial_SendByte(uint8_t byte)
{
    (void)HAL_UART_Transmit(&huart1, &byte, 1U, HAL_MAX_DELAY);
}

void Serial_SendData(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }
    (void)HAL_UART_Transmit(&huart1, (uint8_t *)data, length, HAL_MAX_DELAY);
}

void Serial_SendString(const char *str)
{
    if (str == NULL)
    {
        return;
    }
    Serial_SendData((const uint8_t *)str, (uint16_t)strlen(str));
}

void Serial_SendNumber(uint32_t number)
{
    char buf[11];
    (void)snprintf(buf, sizeof(buf), "%lu", (unsigned long)number);
    Serial_SendString(buf);
}

void Serial_Printf(const char *format, ...)
{
    char buf[128];
    va_list args;

    va_start(args, format);
    (void)vsnprintf(buf, sizeof(buf), format, args);
    va_end(args);

    Serial_SendString(buf);
}
