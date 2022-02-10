#include "string.h"

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"


void delay(void) {

	for(uint32_t i = 0; i < 50000/2; i++);

}

void GPIO_ButtonInits(void) {

	GPIO_Handle_t GpioButton;
	GpioButton.pGPIOx = GPIOC;
	GpioButton.Config.PinNumber = GPIO_PIN13;
	GpioButton.Config.Mode = GPIO_MODE_IN;
	GpioButton.Config.OpType = GPIO_OTYPE_PP;
	GpioButton.Config.PuPdControl = GPIO_NO_PUPD;

	GPIO_Init(&GpioButton);

}

void SPI1_GPIOInits(void) {

	GPIO_Handle_t SPIPins;
	SPIPins.pGPIOx = GPIOA;
	SPIPins.Config.Mode = GPIO_MODE_ALTFN;
	SPIPins.Config.AFMode = 5;
	SPIPins.Config.OpType = GPIO_OTYPE_PP;
	SPIPins.Config.PuPdControl = GPIO_NO_PUPD;
	SPIPins.Config.Speed = GPIO_OSPEED_HIGH;

	// SCLK
	SPIPins.Config.PinNumber = GPIO_PIN5;
	GPIO_Init(&SPIPins);

	// MOSI
	SPIPins.Config.PinNumber = GPIO_PIN7;
	GPIO_Init(&SPIPins);

	// MISO
	// SPIPins.Config.PinNumber = GPIO_PIN6;
	// GPIO_INIT(&SPIPins);

	// NSS
	// SPIPins.pGPIOx = GPIOB;
	// SPIPins.Config.PinNumber = GPIO_PIN6;
	// GPIO_INIT(&SPIPins);

}


void SPI1_Inits(void) {

	SPI_Handle_t SPI1Handle;
	SPI1Handle.pSPIx = SPI2;
	SPI1Handle.SPIConfig.BusConfig = SPI_BUS_CONF_FD;
	SPI1Handle.SPIConfig.DeviceMode = SPI_MODE_MASTER;
	SPI1Handle.SPIConfig.SCLKSpeed = SPI_SCLK_SPEED_DIV2;
	SPI1Handle.SPIConfig.DFF = SPI_DFF_8BIT;
	SPI1Handle.SPIConfig.CPHA = SPI_CPHA_LOW;
	SPI1Handle.SPIConfig.CPOL = SPI_CPOL_LOW;
	SPI1Handle.SPIConfig.SSM = SPI_SSM_EN;

	SPI_Init(&SPI1Handle);

}



int main(void) {

	char* userData = "Hello world";

	GPIO_ButtonInits();
	SPI1_GPIOInits();
	SPI1_Inits();

	/* automatically pull NSS low */
	SPI_SSOEConfig(SPI1, ENABLE);
	SPI_SSIConfig(SPI1, ENABLE);

	while(1) {

		/* upon button press, debounce, and send message via SPI */
		while(!GPIO_ReadInputPin(GPIOA, GPIO_PIN0));
		delay();
		SPI_PeriphControl(SPI1, ENABLE);

		uint8_t dataLen = strlen(userData);
		SPI_Tx(SPI1, &dataLen, sizeof(dataLen));
		SPI_Tx(SPI1, (uint8_t*)userData, strlen(userData));

		/* determine TX finished and close SPI */
		while(!SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));
		SPI_PeriphControl(SPI1, DISABLE);

	}

	return 0;

}
