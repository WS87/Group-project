#include "stm32f407xx.h"
#include "Board_LED.h"
#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include "M:\Year 2\Design construction and test\Lab 3\PB_LCD_Drivers.c"
#include "M:\Year 2\Design construction and test\Lab 3\PB_LCD_Drivers.h"

uint32_t VDD = 0;


//theoretically sets pin of 5V and inverts equation
uint32_t  LCD_VDD (uint32_t a) {
	uint32_t static b;
	b = (1.5 / a) - 15;
	return(b);
}

//Creating a data register for the ADC
uint32_t dataReg;

void ADC_IRQ (void)
{
		dataReg = ADC1->DR;
}

uint32_t ADC_config () {
	RCC ->APB1ENR |= RCC_APB1ENR_TIM6EN; //enables clock 
	 RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; //enables GPIO clock
	GPIOA->MODER = (GPIOA->MODER & ~GPIO_MODER_MODER1_Msk) | 
	(0x01 << GPIO_MODER_MODER1_Pos);
	
}

//Make Voltage into float 
uint32_t ADCValue;


int main (void) {
	while(1) {
		SystemCoreClockUpdate();
		//ADC using the first eqatuib rearranged
    ADCValue = (ADCValue/1.5)-15;
		
	}	
}