#include "stm32f407xx.h"
#include "Board_LED.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "M:\Year 2\Design construction and test\Lab 3\PB_LCD_Drivers.h"

uint32_t volatile ticks = 0;

void SysTick_Handler(void)
{
	ticks++;
}

void waitForATime(int howLongToWait)
{
	uint32_t whenStarted = ticks;
	while (ticks - whenStarted < howLongToWait);
}

double ConvertADCValue(uint32_t value)
{
	double ret;
	ret = (value + 1.5);
	return ret;
}

void ADC_Config()
{
	// Enable GPIO-C clock
	RCC->AHB1ENR = (RCC->AHB1ENR & ~RCC_AHB1ENR_GPIOCEN_Msk) | (0x1 << RCC_AHB1ENR_GPIOCEN_Pos);
	// Enable ADC1 clock
	RCC->APB2ENR = (RCC->APB2ENR & ~RCC_APB2ENR_ADC1EN_Msk) | (0x1 << RCC_APB2ENR_ADC1EN_Pos);
	// Sets mode register 14 to analog mode
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER4_Msk) | (0x3 << GPIO_MODER_MODER4_Pos); 
	// Set ADC to discontinuous mode
	ADC1->CR1 = (ADC1->CR1 & ~ADC_CR1_DISCEN_Msk) | (0x1 << ADC_CR1_DISCEN_Pos);
	// Set ADC to perform one conversion before raising EOC flag
	ADC1->SQR1 = (ADC1->SQR1 & ~ADC_SQR1_L_Msk) | (0x0 << ADC_SQR1_L_Pos);
	// Set ADC to read input 14
	ADC1->SQR3 = (ADC1->SQR3 & ~ADC_SQR3_SQ1_Msk) | (0xE << ADC_SQR3_SQ1_Pos);
	// Enable ADC :D
	ADC1->CR2 = (ADC1->CR2 & ~ADC_CR2_ADON_Msk) | (0x1 << ADC_CR2_ADON_Msk);
}

int main(void)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock / 2);

	// Initialise LCD
	PB_LCD_Init();
	PB_LCD_Clear();

	// Initialise ADC
	ADC_Config();
	
	// Temporary variables to store ADC output
	uint32_t ADCconv;
	uint16_t y;
	double ADCfinal;
	
	// Main loop
	while (1)
	{
		int finished = 0;
		
		// Begin a conversion with the ADC
		ADC1->CR2 = (ADC1->CR2 & ~ ADC_CR2_SWSTART_Msk) | (0x1 << ADC_CR2_SWSTART_Pos);
		
		while (finished == 0)
		{
			y = ADC1->SR;
			if ((y & ADC_SR_EOC_Msk) == ADC_SR_EOC_Msk)
			{
				// START WAIT
				int i = 0;
				while (i < 1000) i++;
				i = 0;
				// END WAIT
				
				ADCconv = ADC1->DR;
				ADCfinal = ConvertADCValue(ADCconv);
				char ADCconvString[8];
				
				waitForATime(1);
				
				PB_LCD_Clear();
				
				strcpy(ADCconvString, "00000000");
				
				snprintf(ADCconvString, 10, "%.3f", ADCfinal);
				
				PB_LCD_WriteString(ADCconvString, 10);
				
				finished = 1;
			}
		}
	}
}
