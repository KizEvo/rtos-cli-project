#include <stdint.h>
#include "stm32f4xx.h"                  // Device header
#include "uart.h"

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

volatile uint8_t receiveBuff[RECEIVE_BUFF_LEN] = {};
volatile uint8_t receiveBuffIdx = 0;
extern TaskHandle_t xTaskUARTReceiveHandle;
extern SemaphoreHandle_t xBinarySemaphore;
	
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
	float baudRate = 115200;
	float usartDivTemp = ((float)SystemCoreClock / 2) / (16 * baudRate);
	uint8_t fraction = 16 * ((float)((uint32_t)(usartDivTemp * 100) % 100) / 100);
	uint32_t mantissa = (uint32_t)usartDivTemp;
	USART2->BRR = ((mantissa << 4) | fraction);
	/*Enable transmit and interrupt reception*/
	USART2->CR1 |= USART_CR1_TE;
	
	NVIC_SetPriority(USART2_IRQn, configMAX_SYSCALL_INTERRUPT_PRIORITY - 1); /*Set priority higher than configMAX_SYSCALL_INTERRUPT_PRIORITY according to file port.c of freeRTOS at line 822*/
	
	NVIC_EnableIRQ(USART2_IRQn); 	/*Enable NVIC for USART2*/
	
	USART2->CR1 |= (USART_CR1_RXNEIE); /*Interrupt on receiving data*/
	
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
	uint8_t result = USART2->DR;
	return result;
}

/*
 * ==========================================
 * Write string to USART transmit register
 * ==========================================
 */
void UART_WriteString(char *string)
{
	uint16_t i;
	for(i = 0; string[i] <= 127 && string[i] != '\0'; i++)
	{
		UART_WriteByte(string[i]);
	}
}

/*
 * ==========================================
 * Write N length string to USART transmit register
 * ==========================================
 */
void UART_WriteNString(char *string, uint8_t N)
{
	uint16_t i;
	for(i = 0; i < N; i++)
	{
		UART_WriteByte(string[i]);
	}
}

/*
 * ==========================================
 * Write back receive buffer to USART transmit register
 * ==========================================
 */
void UART_WriteReceiveBuffer(void)
{
	uint8_t i;
	for(i = 0; i < receiveBuffIdx; i++)
	{
		UART_WriteByte(receiveBuff[i]);
	}
	receiveBuffIdx = 0; /*Buffer pointer set to the start => Clear buffer*/
}

/*
 * ==========================================
 * USART2 IRQ Handler
 * Deferred interrupt processing freeRTOS
 * ==========================================
 */
void USART2_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE; /*Initialized to pdFALSE - Listing 7.8 Mastering the FreeRTOS Real Time Kernel*/
	
	/*Save data to buffer*/
	receiveBuff[receiveBuffIdx++] = USART2->DR;
	receiveBuffIdx = receiveBuffIdx % RECEIVE_BUFF_LEN;
	USART2->SR &= ~USART_SR_RXNE; /*Clear interrupt flag*/
	
	/*'Give' the semaphore to unblock UARTReceive task, using the interrupt safe API version of give semaphore*/
	if(receiveBuffIdx == 1) xSemaphoreGiveFromISR(xBinarySemaphore, &xHigherPriorityTaskWoken);
	/* Pass the xHigherPriorityTaskWoken value into portYIELD_FROM_ISR().
	If xHigherPriorityTaskWoken was set to pdTRUE inside
	xSemaphoreGiveFromISR() then calling portYIELD_FROM_ISR() will request
	a context switch. If xHigherPriorityTaskWoken is still pdFALSE then
	calling portYIELD_FROM_ISR() will have no effect.*/
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
