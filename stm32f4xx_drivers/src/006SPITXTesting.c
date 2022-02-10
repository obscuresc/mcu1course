#include "string.h"

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"


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

	SPI1_GPIOInits();
	SPI1_Inits();
	SPI_SSIConfig(SPI1, ENABLE);

	// enable SPI

	SPI_PeriphControl(SPI1, ENABLE);

	SPI_Tx(SPI1, (uint8_t*)userData, strlen(userData));

	// SPI1 available via CN5
	// PB6 CS (AF5)
	// PA7 MOSI (AF5)
	// PA6 SPI1_MISO (AF5)
	// no idea about CS

	while(1);

	return 0;

}
