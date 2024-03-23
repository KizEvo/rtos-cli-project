#include <stdint.h>
#include "stm32f4xx.h"                  // Device header
#include "uart.h"

/* 
 * ==========================================
 * Config USART2 to transmit data
 * PA2 = TX, PA3 = RX
 * USART2 is on the APB1 bus, previously set Fpclk_max = 50MHz
 * ==========================================
 */
void UART_Config(void)
{
	uint32_t configRegCR1 = USART_CR1_UE | (0x1 << USART_CR1_M_Pos);
	USART2->CR1 = configRegCR1;
	/* Write desired baud rate = Fpclk / (8 * (2 - OVER8) * USARTDIV)
	 * OVER8 = 0 by default */
	float baudRate = 9600.0;
	float usartDivTemp = (SystemCoreClock / 2.0) / (16.0 * baudRate);
	uint8_t fraction = 16 * ((uint32_t)(usartDivTemp * 100) % 100);
	uint16_t mantissa = (uint16_t)usartDivTemp;
	USART2->BRR = (mantissa << 4) | fraction;
	/*Enable transmit*/
	USART2->CR1 |= USART_CR1_TE;
}

/* 
 * ==========================================
 * Write data to the USART transmit register
 * ==========================================
 */
void UART_WriteByte(uint8_t data)
{
	USART2->DR = data;
	while(!((USART2->SR & USART_SR_TC) != 0)); /*Wait till data finished transmitting*/
}

/* 
 * ==========================================
 * Read data from the USART reception register
 * ==========================================
 */
uint8_t UART_ReadByte(void)
{
	while(!((USART2->SR & USART_SR_RXNE) != 0)); /*Wait till data available in the register*/
	return USART2->DR & 0xFF;
}
