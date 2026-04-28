#ifndef BALL_BEAM_SYSTEM_SERIAL_H
#define BALL_BEAM_SYSTEM_SERIAL_H

#include <stdint.h>

/*
 * 文件综述：
 * 该模块提供串口发送的统一接口，底层使用环形缓冲区 + USART中断发送，
 * 用于遥测输出和调参日志，避免主控制循环被阻塞式串口发送拖慢。
 */
void Serial_SendByte(uint8_t byte);
void Serial_SendData(const uint8_t *data, uint16_t length);
void Serial_SendString(const char *str);
void Serial_SendNumber(uint32_t number);
void Serial_Printf(const char *format, ...);
uint16_t Serial_AvailableForWrite(void);

#endif //BALL_BEAM_SYSTEM_SERIAL_H
