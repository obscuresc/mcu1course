#include <stm32f446xx.h>
#include <stm32f446xx_gpio.h>
#include <sys/_stdint.h>

// TODO use memset to default init handlers
GPIO_Handle_t GpioButton = (GPIO_Handle_t) {0};
GPIO_Handle_t GpioLED = (GPIO_Handle_t) {0};

void EXTI1_IRQHandler(void);


void delay(void) {

	for(uint32_t i = 0; i < 5000000/2; i++);

}


int main(void) {

	/* PA1 Button*/
//	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOA;
	GpioButton.Config.PinNumber = GPIO_PIN1;
	GpioButton.Config.Mode = GPIO_MODE_IT_FT;
	GpioButton.Config.OpType = GPIO_OTYPE_PP;
	GpioButton.Config.PuPdControl = GPIO_NO_PUPD;

	GPIO_PeriphClkControl(GPIOA, ENABLE);
	GPIO_Init(&GpioButton);

	GPIO_IRQNumConfig(IRQ_NO_EXTI1, ENABLE);

	const uint8_t IRQEXTIPriority15 = 15;
	GPIO_IRQPriority(IRQ_NO_EXTI1, IRQEXTIPriority15);


	/* PA0 LED */
	/* active high */
//	GPIO_Handle_t GpioLED;
	GpioLED.pGPIOx = GPIOA;
	GpioLED.Config.PinNumber = GPIO_PIN0;
	GpioLED.Config.Mode = GPIO_MODE_OUT;
	GpioLED.Config.Speed = GPIO_OSPEED_HIGH;
	GpioLED.Config.OpType = GPIO_OTYPE_PP;
	GpioLED.Config.PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioLED);


	while(1) {

//		if(GPIO_ReadInputPin(GpioButton.pGPIOx, GpioButton.Config.PinNumber)) {
//
//			GPIO_ToggleOutputPin(GpioLED.pGPIOx, GpioLED.Config.PinNumber);
//			delay();
//
//		}

	}

	return 0;
}


void EXTI1_IRQHandler(void) {

	GPIO_IRQHandling(GpioButton.Config.PinNumber);
	GPIO_ToggleOutputPin(GpioLED.pGPIOx, GpioLED.Config.PinNumber);
	delay();

}
