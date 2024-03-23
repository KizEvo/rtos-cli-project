#ifndef UART_H
#define UART_H

#include <stdint.h>

void UART_Config(void);
void UART_WriteByte(uint8_t data);
uint8_t UART_ReadByte(void);

#endif
