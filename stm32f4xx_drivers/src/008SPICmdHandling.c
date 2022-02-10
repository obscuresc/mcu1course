#include "string.h"

#include "stm32f446xx.h"
#include "stm32f446xx_gpio.h"
#include "stm32f446xx_spi.h"

#define CMD_LED_CTRL		0x50
#define CMD_SNSR_READ		0x51
#define CMD_LED_READ		0x52
#define CMD_PRINT			0x53
#define CMD_ID_READ			0x54

#define LED_ON				1
#define LED_OFF				0

#define ANALOG_PIN0			0
#define ANALOG_PIN1			1
#define ANALOG_PIN2			2
#define ANALOG_PIN3			3
#define ANALOG_PIN4			4

#define LED_PIN				9


void delay(void) {

	for(uint32_t i = 0; i < 50000/2; i++);

}


uint8_t SPI_VerifyResponse(uint8_t response) {

	return response == 0xF5;
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
	SPIPins.Config.PinNumber = GPIO_PIN6;
	GPIO_Init(&SPIPins);

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

		// 1. command led control
		uint8_t dummyData = 0xFF;
		uint8_t commandCode = CMD_LED_CTRL;
		SPI_Tx(SPI1, &commandCode, sizeof(commandCode));
		SPI_Rx(SPI1, &dummyData, sizeof(dummyData));

		// 2. receive ACK
		uint8_t ackByte = 0;
		SPI_Tx(SPI1, &dummyData, sizeof(dummyData));
		SPI_Rx(SPI1, &ackByte, sizeof(ackByte));

		uint8_t args[2];
		if(SPI_VerifyResponse(ackByte)) {

			// send arguments
			args[0] = LED_PIN;
			args[1] = LED_ON;
			SPI_Tx(SPI1, args, sizeof(args));
			SPI_Rx(SPI1, &dummyData, sizeof(dummyData));
		}

		// 2. command sensor read
		while(!GPIO_ReadInputPin(GPIOA, GPIO_PIN0));
		delay();

		commandCode = CMD_SNSR_READ;
		SPI_Tx(SPI1, &commandCode, sizeof(commandCode));
		SPI_Rx(SPI1, &dummyData, sizeof(dummyData));
		SPI_Tx(SPI1, &dummyData, sizeof(dummyData));
		SPI_Rx(SPI1, &ackByte, sizeof(ackByte));
		if(SPI_VerifyResponse(ackByte)) {

			// send arguments
			args[0] = ANALOG_PIN0;
			SPI_Tx(SPI1, &args[1], sizeof(args[1]));

			// wait a bit so that slave has time to prepare response
			delay();
			SPI_Rx(SPI1, &dummyData, sizeof(dummyData));

			uint8_t analogRead;
			SPI_Tx(SPI1, &analogRead, sizeof(analogRead));

		}





		/* determine TX finished and close SPI */
		while(!SPI_GetFlagStatus(SPI1, SPI_BSY_FLAG));
		SPI_PeriphControl(SPI1, DISABLE);

	}

	return 0;

}
