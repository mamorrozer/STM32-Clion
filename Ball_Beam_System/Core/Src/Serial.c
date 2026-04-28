#include "../Inc/Serial.h"
#include <stdarg.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>

#include "main.h"

extern UART_HandleTypeDef huart1;

#define SERIAL_TX_BUFFER_SIZE 1024U

/*
 * 文件综述：
 * 1) 使用环形缓冲区缓存待发串口数据；
 * 2) 使用 USART1 发送完成中断串行推进下一段发送；
 * 3) 主循环只负责“塞数据”，不直接阻塞等待串口硬件。
 */
static uint8_t serial_tx_buffer[SERIAL_TX_BUFFER_SIZE];
static volatile uint16_t serial_tx_head = 0U;
static volatile uint16_t serial_tx_tail = 0U;
static volatile uint16_t serial_tx_inflight_len = 0U;
static volatile bool serial_tx_busy = false;

static uint16_t serial_next_index(uint16_t index)
{
    index++;
    if (index >= SERIAL_TX_BUFFER_SIZE)
    {
        index = 0U;
    }
    return index;
}

static void serial_start_next_transfer(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    /* 关键点：head/tail/busy 是并发共享状态，需要临界区保护。 */
    if ((!serial_tx_busy) && (serial_tx_head != serial_tx_tail))
    {
        uint16_t tx_len = (serial_tx_head > serial_tx_tail) ? (serial_tx_head - serial_tx_tail) : (SERIAL_TX_BUFFER_SIZE - serial_tx_tail);
        serial_tx_busy = true;
        serial_tx_inflight_len = tx_len;
        if (HAL_UART_Transmit_IT(&huart1, &serial_tx_buffer[serial_tx_tail], tx_len) != HAL_OK)
        {
            serial_tx_busy = false;
            serial_tx_inflight_len = 0U;
        }
    }

    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Serial_SendByte(uint8_t byte)
{
    Serial_SendData(&byte, 1U);
}

void Serial_SendData(const uint8_t *data, uint16_t length)
{
    if ((data == NULL) || (length == 0U))
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    for (uint16_t i = 0U; i < length; ++i)
    {
        uint16_t next_head = serial_next_index(serial_tx_head);
        if (next_head == serial_tx_tail)
        {
            /* 缓冲区满：新数据丢弃，保持系统实时性，不在这里阻塞。 */
            break;
        }
        serial_tx_buffer[serial_tx_head] = data[i];
        serial_tx_head = next_head;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    serial_start_next_transfer();
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

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance != USART1)
    {
        return;
    }

    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t completed_len = serial_tx_inflight_len;
    serial_tx_inflight_len = 0U;
    serial_tx_busy = false;

    while (completed_len > 0U)
    {
        serial_tx_tail = serial_next_index(serial_tx_tail);
        completed_len--;
    }

    if (primask == 0U)
    {
        __enable_irq();
    }

    serial_start_next_transfer();
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

uint16_t Serial_AvailableForWrite(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();

    uint16_t used = (serial_tx_head >= serial_tx_tail)
                        ? (serial_tx_head - serial_tx_tail)
                        : (SERIAL_TX_BUFFER_SIZE - (serial_tx_tail - serial_tx_head));
    uint16_t free_space = (uint16_t)(SERIAL_TX_BUFFER_SIZE - 1U - used);

    if (primask == 0U)
    {
        __enable_irq();
    }

    return free_space;
}
