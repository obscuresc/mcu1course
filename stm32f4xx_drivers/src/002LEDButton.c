#include <stm32f446xx.h>
#include <stm32f446xx_gpio.h>
#include <sys/_stdint.h>


void delay(void) {

	for(uint32_t i = 0; i < 500000; i++);

}

int main(void) {

	/* PC13 for user button (active low) */
	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.Config.PinNumber = GPIO_PIN13;
	GpioButton.Config.Mode = GPIO_MODE_IN;
	GpioButton.Config.OpType = GPIO_OTYPE_PP;
	GpioButton.Config.PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriphClkControl(GPIOC, ENABLE);
	GPIO_Init(&GpioButton);


	/* STM32F446RE Nucleo has LD2 connected to PA5 */
	GPIO_Handle_t GpioLED;
	GpioLED.pGPIOx = GPIOA;
	GpioLED.Config.PinNumber = GPIO_PIN5;
	GpioLED.Config.Mode = GPIO_MODE_OUT;
	GpioLED.Config.Speed = GPIO_OSPEED_HIGH;
	GpioLED.Config.OpType = GPIO_OTYPE_PP;
	GpioLED.Config.PuPdControl = GPIO_NO_PUPD;


	GPIO_PeriphClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioLED);


	while(1) {

		if(!GPIO_ReadInputPin(GpioButton.pGPIOx, GpioButton.Config.PinNumber)) {

			GPIO_ToggleOutputPin(GpioLED.pGPIOx, GpioLED.Config.PinNumber);
			delay();

		}

	}

	return 0;
}
