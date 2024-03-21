#include <stdint.h>
#include "stm32f4xx.h"

/*freeRTOS headers*/
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

void delay(uint32_t);

int main(void)
{
	/*Enable LED clocks*/
	RCC->AHB1ENR |= RCC_AHB1ENR_GPIODEN;
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
	
	/*LD2*/
	GPIOD->MODER |= (0x1 << GPIO_MODER_MODE12_Pos); /*Output*/
	GPIOD->OSPEEDR |= (0x2 << GPIO_OSPEEDR_OSPEED12_Pos); /*Fast speed*/
	GPIOD->BSRR |= GPIO_BSRR_BS12;
	
	while(1)
	{
			GPIOD->ODR ^= GPIO_ODR_OD13 | GPIO_ODR_OD12 | GPIO_ODR_OD14 | GPIO_ODR_OD15;
			delay(100000);
	}
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
