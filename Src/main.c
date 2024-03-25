#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f4xx.h"

/*Peripherals headers*/
#include "uart.h"

/*freeRTOS headers*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/*
 * ==========================================
 * Function prototypes
 * ==========================================
 */
void RCC_SysClockConfig(void);
void GPIO_LedsConfig(void);
void vTaskUARTReceive(void *);
void vTaskUARTSend(void *);
void vTaskOthers(void *);
uint8_t compareReceivedDataWithCmd(char *, volatile uint8_t *, uint8_t *, volatile uint8_t *, uint8_t, uint8_t);
uint8_t receivedDataToNumb(volatile uint8_t *buffer, uint8_t idx, uint8_t N, uint16_t *numb);

/*
 * ==========================================
 * Global variables
 * ==========================================
 */
QueueHandle_t queue;
TaskHandle_t xTaskUARTSendHandle = NULL;
TaskHandle_t xTaskOthersHandle = NULL;
SemaphoreHandle_t xBinarySemaphore = NULL;
uint8_t startIdx = 0;

/*
 * ==========================================
 * Main function
 * ==========================================
 */
int main(void)
{
	/*Change clock speed to 100 MHz*/
	RCC_SysClockConfig();
	/*Enable USART2*/
	UART_Config();
	/*Enable 4 LEDs*/
	GPIO_LedsConfig();
	
	xBinarySemaphore = xSemaphoreCreateBinary();
	
	if(xBinarySemaphore != NULL) /*Check the semaphore was created successfully*/
	{
		/*Create Tasks*/
		xTaskCreate(vTaskUARTReceive, "UARTRe", 512, NULL, 3, NULL); /*256 * 4 bytes (configSTACK_DEPTH_TYPE) = 1 KB*/
		xTaskCreate(vTaskUARTSend, "UARTSe", 256, NULL, 1, &xTaskUARTSendHandle);
		xTaskCreate(vTaskOthers, "Others", 512, NULL, 2, &xTaskOthersHandle);
		
		uint8_t queueSize = 20;
		queue = xQueueCreate(queueSize, sizeof(uint8_t));
		
		/*Start the tasks scheduler*/
		vTaskStartScheduler();
	}
	/*Does not reach here*/
	while(1);
}

/*
 * ==========================================
 * vTaskUARTReceive, read incoming data
 * ==========================================
 */
void vTaskUARTReceive(void *params)
{
	char *command[] = {"turn LED on", "turn LED off", "blink LED ", "echo "};
	uint8_t commandLength = 4;
	uint8_t commandCode = 0;
	uint8_t i;
	TickType_t forUARToFinished = pdMS_TO_TICKS(15);
	while(1)
	{
		if((commandCode & 0x80) != 0) /*If it's command 0-2 then do TaskOthers*/
		{
			xQueueSend(queue, (void *)&commandCode, 10);
			commandCode = 0;
			vTaskPrioritySet(xTaskOthersHandle, 3);
		}
		else if((commandCode & 0x40) != 0) /*If it's command 3 then do TaskUARTSend*/
		{
			commandCode = 0;
			vTaskPrioritySet(xTaskUARTSendHandle, 4);
		}
		xSemaphoreTake(xBinarySemaphore, portMAX_DELAY); /*Wait indefinitely*/
		/*Only execute when semaphore is taken, the semaphore is given in UART IRQ Handler*/
		vTaskDelay(forUARToFinished);
		i = 0;
		while(i < commandLength)
		{
			if(compareReceivedDataWithCmd(command[i], receiveBuff, &startIdx, &receiveBuffIdx, i, commandLength))
			{
				switch(i)
				{
					case 0:
						commandCode = 0x81;
						break;
					case 1:
						commandCode = 0x82;
						break;
					case 2:
						commandCode = 0x83;
						break;
					case 3:
						commandCode = 0x41;
						break;
				}
				break;
			}
			i++;
		}
	}
}

/*
 * ==========================================
 * vTaskUARTSend, send serial data 
 * ==========================================
 */
void vTaskUARTSend(void *params)
{
	while(1)
	{
		UART_WriteNString((void *)(&(receiveBuff[startIdx])), receiveBuffIdx);
		startIdx = 0;
		receiveBuffIdx = 0;
		vTaskPrioritySet(xTaskUARTSendHandle, 1);
	}
}

/*
 * ==========================================
 * vTaskOthers, handle other task
 * ==========================================
 */
void vTaskOthers(void *params)
{
	uint8_t statusConvert = 0;
	uint16_t numb = 0;
	uint8_t command = 0;
	BaseType_t status = pdFALSE;
	TickType_t tick;
	while(1)
	{
		status = xQueueReceive(queue, &command, 0);
		if(status == pdTRUE && command < 0x83)
		{
			switch(command)
			{
				case 0x81:
					GPIOD->ODR |= (GPIO_ODR_OD13 | GPIO_ODR_OD12 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
					break;
				case 0x82:
					GPIOD->ODR &= ~(GPIO_ODR_OD13 | GPIO_ODR_OD12 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
					break;
			}
			status = pdFALSE;
			receiveBuffIdx = 0;
			startIdx = 0;
			vTaskPrioritySet(xTaskOthersHandle, 2);
		} 
		else if (status == pdTRUE && command >= 0x83)
		{
			status = pdFALSE;
			startIdx = 0;
			receiveBuffIdx = 0;
			statusConvert = receivedDataToNumb(&receiveBuff[startIdx], startIdx, receiveBuffIdx, &numb);
			tick = pdMS_TO_TICKS(numb);
			while(statusConvert && command >= 0x83)
			{
				xQueueReceive(queue, &command, 0);
				GPIOD->ODR ^= (GPIO_ODR_OD13 | GPIO_ODR_OD12 | GPIO_ODR_OD14 | GPIO_ODR_OD15);
				vTaskDelay(tick);
			}
		}
	}
}

/*
 * ==========================================
 * Config main system clock, we use PLL.
 * ==========================================
 */
void RCC_SysClockConfig(void)
{
	/*PLLSRC = HSI, VCO in = HSI / 2, VCO out = VCO in * 50 = 400MHz, Main SysClock = 400MHz / 4 (PLLP) = 100 MHz (max clock of HCLK)*/
	uint32_t pllcfgrRegister = 0x24003010; /*Reset value, check the datasheet*/

	pllcfgrRegister &= ~RCC_PLLCFGR_PLLM_Msk;
	pllcfgrRegister |= 2 << RCC_PLLCFGR_PLLM_Pos;

	pllcfgrRegister &= ~RCC_PLLCFGR_PLLN_Msk;
	pllcfgrRegister |= (50 << RCC_PLLCFGR_PLLN_Pos);

	pllcfgrRegister &= ~RCC_PLLCFGR_PLLP_Msk;
	pllcfgrRegister |= 1 << RCC_PLLCFGR_PLLP_Pos;

	RCC->PLLCFGR = pllcfgrRegister;
	/*Enable PLL*/
	RCC->CR |= RCC_CR_PLLON;
	/*Config high-speed busses and switch main sys clock to PLL*/	
	uint32_t cfgrRegister = RCC_CFGR_PPRE1_DIV2 | RCC_CFGR_PPRE2_DIV1 | RCC_CFGR_HPRE_DIV1 | RCC_CFGR_SW_PLL;
	/*Set FLASH wait state to 3*/
	FLASH->ACR |= FLASH_ACR_LATENCY_3WS;
	/*Wait till PLL locked*/
	while(!((RCC->CR & RCC_CR_PLLRDY) != 0));
	/*Write to config register*/
	RCC->CFGR = cfgrRegister;
	/*Update SystemCoreClock global variable*/
	SystemCoreClockUpdate();
}

/*
 * ==========================================
 * Enable USER LEDS on the board.
 * ==========================================
 */
void GPIO_LedsConfig(void)
{
	/*Enable LED clocks*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	/*LD2*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE12_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED12_Pos); /*Fast speed*/
	
	/*LD3*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE13_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED13_Pos); /*Fast speed*/
	
	/*LD4*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE14_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED14_Pos); /*Fast speed*/
	
	/*LD5*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE15_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED15_Pos); /*Fast speed*/
}

/*
 * ==========================================
 * Compare input command with received data from UART buffer to see if there is a match.
 * If yes then return 1 else 0
 * ==========================================
 */
uint8_t compareReceivedDataWithCmd(char *command, volatile uint8_t *receivedData, 
																			uint8_t *idx, volatile uint8_t *receiveIdx,
																			uint8_t currCmdIdx, uint8_t totalCmd)
{
	uint8_t i = 0;
	uint8_t tmp = *idx;
	while(command[i] != '\0')
	{
		if(command[i] != receivedData[tmp])
		{
			if(currCmdIdx + 1 >= totalCmd)
			{
				*idx = 0;
				*receiveIdx = 0;
				return 0;
			}
			return 0;
		}
		i++;
		tmp = tmp + 1;
	}
	*idx = tmp;
	return 1;
}

/*
 * ==========================================
 * Convert receivedData buffer of length N to unsigned 16 bit integer
 * Return 1 if successful else 0
 * ==========================================
 */
uint8_t receivedDataToNumb(volatile uint8_t *buffer, uint8_t idx, uint8_t N, uint16_t *numb)
{
	uint8_t status = 1;
	*numb = 0;
	while(idx < N)
	{
		if(buffer[idx] < '0' || buffer[idx] > '9')
		{
			status = 0;
			break;
		}
		*numb = *numb + (buffer[idx] - '0');
		*numb = (*numb) * 10;
		idx++;
	}
	*numb = *numb / 10;
	return status;
}
