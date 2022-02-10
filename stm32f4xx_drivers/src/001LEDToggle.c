#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"

#define LOWLED

void delay(void) {

	for(uint32_t i = 0; i < 500000; i++);

}


int main(void) {

	/* STM32F446RE Nucleo has LD2 connected to PA5 */
	GPIO_Handle_t GpioLED;
	GpioLED.pGPIOx = GPIOA;
	GpioLED.Config.PinNumber = GPIO_PIN5;
	GpioLED.Config.Mode = GPIO_MODE_OUT;
	GpioLED.Config.Speed = GPIO_OSPEED_HIGH;

#ifndef LOWLED

	GpioLED.Config.OpType = GPIO_OTYPE_PP;
	GpioLED.Config.PuPdControl = GPIO_NO_PUPD;

#else // use weak pull up in open drain for low current through LED

	GpioLED.Config.OpType = GPIO_OTYPE_OD;
	GpioLED.Config.PuPdControl = GPIO_PIN_PU;

#endif // PPTEST


	GPIO_PeriphClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLED);




	while(1) {

		GPIO_ToggleOutputPin(GpioLED.pGPIOx, GpioLED.Config.PinNumber);
		delay();
	}

	return 0;
}
