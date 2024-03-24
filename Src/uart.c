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
	/*Enable GPIOA clock and USART2*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	RCC->APB1ENR |= RCC_APB1ENR_USART2EN;
	
	/*Config I/O pin, alt func - push pull and high speed*/
	GPIOA->MODER |= (0x2 << GPIO_MODER_MODE2_Pos) | (0x2 << GPIO_MODER_MODE3_Pos);
	
	GPIOA->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED2_Pos);
	
	/*Alternate function selection for port 2 and 3 bit*/
	GPIOA->AFR[0] = (0x7 << GPIO_AFRL_AFSEL2_Pos) | (0x7 << GPIO_AFRL_AFSEL3_Pos); /*write 0x7 as stated in the datasheet - https://www.st.com/resource/en/datasheet/stm32f411ce.pdf Table 9*/
	
	USART2->CR1 |= (USART_CR1_UE | (0x1 << USART_CR1_M_Pos));
	/* Write desired baud rate = Fpclk / (8 * (2 - OVER8) * USARTDIV)
	 * OVER8 = 0 by default */
	float baudRate = 9600;
	float usartDivTemp = ((float)SystemCoreClock / 2) / (16 * baudRate);
	uint8_t fraction = 16 * ((float)((uint32_t)(usartDivTemp * 100) % 100) / 100);
	uint32_t mantissa = (uint32_t)usartDivTemp;
	USART2->BRR = ((mantissa << 4) | fraction);
	/*Enable transmit and reception*/
	USART2->CR1 |= USART_CR1_TE;
	USART2->CR1 |= USART_CR1_RE;
}

/* 
 * ==========================================
 * Write data to the USART transmit register then wait for it to complete
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
	while(!((USART2->SR & USART_SR_RXNE)));
	uint8_t result = USART2->DR & 0x3F;
	return result;
}
