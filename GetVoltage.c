#include "stm32f407xx.h"
#include "Board_LED.h"
#include <stdio.h>
#include <stdlib.h>
#include "M:\Year 2\Design construction and test\Lab 3\PB_LCD_Drivers.c"
#include "M:\Year 2\Design construction and test\Lab 3\PB_LCD_Drivers.h"

void VoltmeterConfig()
{
	// RCC ->APB1ENR |= RCC_APB1ENR_TIM6EN; //enables clock up only 
	// RCC->AHB1ENR |= RCC_AHB1ENR_GPIOEEN; //enables GPIO clock
	//setting GPIO to read the analouge signal
	// RCC ->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
	
	// Sets mode register 14 to analog mode
	GPIOC->MODER = (GPIOC->MODER & ~GPIO_MODER_MODER14) | (0x01 << GPIO_MODER_MODER14_Pos); 
	
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN; // Enable ADC preipheral clock
	ADC1->CR2 |= ADC_CR2_ADON; // Enable ADC
	ADC1->CR1 |= ADC_CR1_DISCEN; // Set ADC to discontinuous mode
	ADC1->CR2 |= ADC_CR2_EOCS; // Enable end-of-conversion flag
}

volatile uint32_t ADCconv;
char ADCconvString[10];

int main(void)
{
	SystemCoreClockUpdate();
	SysTick_Config(SystemCoreClock/2);
	
	// Initialise LCD
	PB_LCD_Init();
	PB_LCD_Clear();
	
	// Initialise ADC
	VoltmeterConfig();
	
	// Main loop
	while(1)
	{
		// Begin a conversion with the ADC
		ADC1->CR2 |= ADC_CR2_SWSTART;
		
		// Wait until the conversion is complete
		while (!(ADC1->CR2 & ADC_SR_EOC));
		
		// Store the voltage in a local variable
		ADCconv = ADC1->DR;
		
		// Store ADCconv value as a string in ADCconvString
		sprintf(ADCconvString, "%u", ADCconv);
		PB_LCD_WriteString(ADCconvString, 10);
	}
}
