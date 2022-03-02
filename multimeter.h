#include "stm32f407xx.h"

void SysTick_Handler(void);
void waitForATime(uint32_t howLongToWait);

int convertADCValue(uint16_t value);
double convertADCValue2(uint32_t value);

void AdcConfig(void);
