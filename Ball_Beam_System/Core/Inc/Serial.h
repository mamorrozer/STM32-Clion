#ifndef BALL_BEAM_SYSTEM_SERIAL_H
#define BALL_BEAM_SYSTEM_SERIAL_H

#include <stdint.h>

void Serial_SendByte(uint8_t byte);
void Serial_SendData(const uint8_t *data, uint16_t length);
void Serial_SendString(const char *str);
void Serial_SendNumber(uint32_t number);
void Serial_Printf(const char *format, ...);
uint16_t Serial_AvailableForWrite(void);

#endif //BALL_BEAM_SYSTEM_SERIAL_H
