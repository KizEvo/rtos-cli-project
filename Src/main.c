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

void delay(uint32_t);
void RCC_SysClockConfig(void);
void GPIO_LedsConfig(void);
void vTaskUART(void *);
void vTaskMain(void *);

/*
 * ==========================================
 * Main function
 * ==========================================
 */
int main(void)
{
	/*Change clock speed to 100 MHz*/
	RCC_SysClockConfig();
	/*Enable 4 LEDs*/
	GPIO_LedsConfig();
	/*Create Tasks*/
	xTaskCreate(vTaskUART, "UART", 256, NULL, 1, NULL); /*256 * 4 bytes (configSTACK_DEPTH_TYPE) = 1 KB*/
	xTaskCreate(vTaskMain, "Main", 256, NULL, 1, NULL);
	/*Start the tasks scheduler*/
	vTaskStartScheduler();
	/*Does not reach here*/
	while(1);
}

/*
 * ==========================================
 * vTaskUART, read incoming data and send to Queue
 * ==========================================
 */
void vTaskUART(void *params)
{
	uint8_t data;
	char buffer[20];
	uint8_t bufferSize = sizeof(buffer);
	uint8_t idx = 0;
	while(1)
	{
		data = UART_ReadByte();
		if(!data && idx > 0)
		{
		}
		if(idx < bufferSize - 1)
		{
			buffer[idx] = (char)data;
			buffer[idx++] = '\0';
		}
		else
		{
			idx = 0;
		}
	}
}

void vTaskMain(void *params)
{
	TickType_t tick = pdMS_TO_TICKS(500);
	while(1)
	{
		GPIOD->ODR ^= GPIO_ODR_OD13 | GPIO_ODR_OD12 | GPIO_ODR_OD14 | GPIO_ODR_OD15;
		vTaskDelay(tick);
	}
}

void RCC_SysClockConfig(void)
{
	/*PLLSRC = HSI, VCO in = HSI / 2, VCO out = VCO in * 50 = 400MHz, Main SysClock = 400MHz / 4 (PLLP) = 100 MHz (max clock of HCLK)*/
	uint32_t pllcfgrRegister = 0x24003010;

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

void GPIO_LedsConfig(void)
{
	/*Enable LED clocks*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
	/*LD2*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE12_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED12_Pos); /*Fast speed*/
	GPIOD->BSRR |= GPIO_BSRR_BS12;
	
	/*LD3*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE13_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED13_Pos); /*Fast speed*/
	
	/*LD4*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE14_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED14_Pos); /*Fast speed*/
	
	/*LD5*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE15_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED15_Pos); /*Fast speed*/
	GPIOD->BSRR |= GPIO_BSRR_BS15;
}

void delay(uint32_t time)
{
	volatile uint32_t i = 0, j = 0;
	while(i < time)
	{
		i++;
		while(j < time)
		{
			j++;
		}
	}
}
