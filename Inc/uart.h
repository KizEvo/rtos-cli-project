#ifndef UART_H
#define UART_H

#include <stdint.h>

#define RECEIVE_BUFF_LEN 200

extern volatile uint8_t receiveBuff[RECEIVE_BUFF_LEN];
extern volatile uint8_t receiveBuffIdx;

void UART_Config(void);
void UART_WriteByte(uint8_t data);
uint8_t UART_ReadByte(void);
void UART_WriteString(char *string);
void UART_WriteNString(char *string, uint8_t N);
void UART_WriteReceiveBuffer(void);

#endif
