/*
 * stm32f446xx.h
 *
 *  Created on: Sep 13, 2021
 *      Author: jack
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include <stdint.h>
#include <stddef.h>

/* processor specific details */
#define NVIC_PRIORITY_BASEADDR ((volatile uint32_t*) 0xE000E400U)

#define NVIC_ISER0			((volatile uint32_t*) 0xE000E100U)
#define NVIC_ISER1			((volatile uint32_t*) 0xE000E104U)
#define NVIC_ISER2			((volatile uint32_t*) 0xE000E108U)
#define NVIC_ISER3			((volatile uint32_t*) 0xE000E11CU)
#define NVIC_ISER4			((volatile uint32_t*) 0xE000E120U)
#define NVIC_ISER5			((volatile uint32_t*) 0xE000E124U)
#define NVIC_ISER6			((volatile uint32_t*) 0xE000E128U)
#define NVIC_ISER7			((volatile uint32_t*) 0xE000E12CU)

#define NVIC_ICER0			((volatile uint32_t*) 0xE000E200U)
#define NVIC_ICER1			((volatile uint32_t*) 0xE000E204U)
#define NVIC_ICER2			((volatile uint32_t*) 0xE000E208U)
#define NVIC_ICER3			((volatile uint32_t*) 0xE000E21CU)
#define NVIC_ICER4			((volatile uint32_t*) 0xE000E220U)
#define NVIC_ICER5			((volatile uint32_t*) 0xE000E224U)
#define NVIC_ICER6			((volatile uint32_t*) 0xE000E228U)
#define NVIC_ICER7			((volatile uint32_t*) 0xE000E22CU)

#define NUM_IR_PRIORITY_BITS		4


/* base addresses of flash and sram memories
 *
 */

#define FLASH_BASEADDR 		0x08000000U
#define ROM_BASEADDR		0x1FFF0000U
#define SRAM1_BASEADDR 		0x20000000U
#define SRAM2_BASEADDR		0x2001C000U
#define SRAM 				SRAM1_BASEADDR


/* base addresses of bus domains
 *
 */

#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0x60000000U

/* base addresses of peripherals
 *
 */

/* AHB1 peripherals */
#define GPIOA_BASEADDR			(AHB1PERIPH_BASEADDR)
#define GPIOB_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR			(AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR			(AHB1PERIPH_BASEADDR + 0x1C00U)
#define CRC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3000U)
#define RCC_BASEADDR			(AHB1PERIPH_BASEADDR + 0x3800U)
#define FLASH_INTFC_BASEADDR	(AHB1PERIPH_BASEADDR + 0x3C00U)
#define BKPSRAM_BASEADDR		(AHB1PERIPH_BASEADDR + 0x4000U)
#define DMA1_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6000U)
#define DMA2_BASEADDR			(AHB1PERIPH_BASEADDR + 0x6400U)
#define USB_OTG_HS_BASEADDR		(AHB1PERIPH_BASEADDR + 0x20000U)

/* AHB2 peripherals */
#define USB_OTG_FS_BASEADDR		(AHB2PERIPH_BASEADDR)
#define DCMI_BASEADDR			(AHB2PERIPH_BASEADDR + 0x50000U)

/* AHB3 peripherals */
#define FMC_CONTROL_BASEADDR	(AHB3PERIPH_BASEADDR)
#define QUADSPI_BASEADDR		(AHB3PERIPH_BASEADDR + 0x1000U)

/* APB1 peripherals */
#define TIM2_BASEADDR			(APB1PERIPH_BASEADDR)
#define TIM3_BASEADDR			(APB1PERIPH_BASEADDR + 0x0400U)
#define TIM4_BASEADDR			(APB1PERIPH_BASEADDR + 0x0800U)
#define TIM5_BASEADDR			(APB1PERIPH_BASEADDR + 0x0C00U)
#define TIM6_BASEADDR			(APB1PERIPH_BASEADDR + 0x1000U)
#define TIM7_BASEADDR			(APB1PERIPH_BASEADDR + 0x1400U)
#define TIM12_BASEADDR			(APB1PERIPH_BASEADDR + 0x1800U)
#define TIM13_BASEADDR			(APB1PERIPH_BASEADDR + 0x1C00U)
#define TIM14_BASEADDR			(APB1PERIPH_BASEADDR + 0x2000U)
#define RTC_BKP_BASEADDR		(APB1PERIPH_BASEADDR + 0x2800U)
#define WWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x2C00U)
#define IWDG_BASEADDR			(APB1PERIPH_BASEADDR + 0x3000U)
#define SPI2_I2S2_BASEADDR		(APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_I2S3_BASEADDR		(APB1PERIPH_BASEADDR + 0x3C00U)
#define SPDIF_RX__BASEADDR		(APB1PERIPH_BASEADDR + 0x4000U)
#define USART2_BASEADDR			(APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR			(APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR			(APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR			(APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR			(APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR			(APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR			(APB1PERIPH_BASEADDR + 0x5C00U)
#define CAN1_BASEADDR			(APB1PERIPH_BASEADDR + 0x6400U)
#define CAN2_BASEADDR			(APB1PERIPH_BASEADDR + 0x6800U)
#define HDMI_CEC_BASEADDR		(APB1PERIPH_BASEADDR + 0x6C00U)
#define PWR_BASEADDR			(APB1PERIPH_BASEADDR + 0x7000U)
#define DAC_BASEADDR			(APB1PERIPH_BASEADDR + 0x7400U)

/* APB2 peripherals */
#define TIM1_BASEADDR			(APB2PERIPH_BASEADDR)
#define TIM8_BASEADDR			(APB2PERIPH_BASEADDR + 0x0400U)
#define USART1_BASEADDR			(APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR			(APB2PERIPH_BASEADDR + 0x1400U)
#define ADC1_ADC2_ADC3_BASEADDR	(APB2PERIPH_BASEADDR + 0x2000U)
#define SDMMC_BASEADDR			(APB2PERIPH_BASEADDR + 0x2C00U)
#define SPI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR			(APB2PERIPH_BASEADDR + 0x3400U)
#define SYSCFG_BASEADDR			(APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR			(APB2PERIPH_BASEADDR + 0x3C00U)
#define TIM9_BASEADDR			(APB2PERIPH_BASEADDR + 0x4000U)
#define TIM10_BASEADDR			(APB2PERIPH_BASEADDR + 0x4400U)
#define TIM11_BASEADDR			(APB2PERIPH_BASEADDR + 0x4800U)
#define SAI1_BASEADDR			(APB2PERIPH_BASEADDR + 0x5800U)
#define SAI2_BASEADDR			(APB2PERIPH_BASEADDR + 0x5C00U)


typedef struct {

	volatile uint32_t CR;
	volatile uint32_t PLLCFGR;
	volatile uint32_t CFGR;
	volatile uint32_t CIR;
	volatile uint32_t AHB1RSTR;
	volatile uint32_t AHB2RSTR;
	volatile uint32_t AHB3RSTR;
	uint32_t RESERVED1;
	volatile uint32_t APB1RSTR;
	volatile uint32_t APB2RSTR;
	uint32_t RESERVED2[2];
	volatile uint32_t AHB1ENR;
	volatile uint32_t AHB2ENR;
	volatile uint32_t AHB3ENR;
	uint32_t RESERVED3;
	volatile uint32_t APB1ENR;
	volatile uint32_t APB2ENR;
	uint32_t RESERVED4[2];
	volatile uint32_t AHB1LPENR;
	volatile uint32_t AHB2LPENR;
	volatile uint32_t AHB3LPENR;
	volatile uint32_t RESERVED5;
	volatile uint32_t APB1LPENR;
	volatile uint32_t APB2LPENR;
	uint32_t RESERVED6[2];
	volatile uint32_t BDCR;
	volatile uint32_t CSR;
	uint32_t RESERVED7[2];
	volatile uint32_t SSCGR;
	volatile uint32_t PLLI2SCFGR;
	volatile uint32_t PLLSAICFGR;
	volatile uint32_t DCKCFGR;
	volatile uint32_t CKGATENR;
	volatile uint32_t DCKCFGR2;

} RCC_RegDef_t;

typedef struct {

	volatile uint32_t MEMRMP;
	volatile uint32_t PMC;
	volatile uint32_t EXTICR[4];
	uint32_t RESERVED1[2];
	volatile uint32_t CMPCR;
	uint32_t RESERVED2[2];
	volatile uint32_t CFGR;

} SYSCFG_RegDef_t;

typedef struct {

	volatile uint32_t MODER;
	volatile uint32_t OTYPER;
	volatile uint32_t OSPEEDR;
	volatile uint32_t PUPDR;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRRL;
	volatile uint32_t LCKR;
	volatile uint32_t AFR[2];

} GPIO_RegDef_t;


typedef struct {

	volatile uint32_t IMR;
	volatile uint32_t EMR;
	volatile uint32_t RTSR;
	volatile uint32_t FTSR;
	volatile uint32_t SWIER;
	volatile uint32_t PR;

} EXTI_RegDef_t;


typedef struct {

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t SR;
	volatile uint32_t DR;
	volatile uint32_t CRCPR;
	volatile uint32_t RXCRCR;
	volatile uint32_t TXCRCR;
	volatile uint32_t I2CFGR;
	volatile uint32_t I2SPR;

} SPI_RegDef_t;


typedef struct {

	volatile uint32_t CR1;
	volatile uint32_t CR2;
	volatile uint32_t OAR1;
	volatile uint32_t OAR2;
	volatile uint32_t DR;
	volatile uint32_t SR1;
	volatile uint32_t SR2;
	volatile uint32_t CCR;
	volatile uint32_t TRISE;
	volatile uint32_t FLTR;

} I2C_RegDef_t;




#define GPIOA 			((GPIO_RegDef_t*) GPIOA_BASEADDR)
#define GPIOB 			((GPIO_RegDef_t*) GPIOB_BASEADDR)
#define GPIOC 			((GPIO_RegDef_t*) GPIOC_BASEADDR)
#define GPIOD 			((GPIO_RegDef_t*) GPIOD_BASEADDR)
#define GPIOE 			((GPIO_RegDef_t*) GPIOE_BASEADDR)
#define GPIOF 			((GPIO_RegDef_t*) GPIOF_BASEADDR)
#define GPIOG 			((GPIO_RegDef_t*) GPIOG_BASEADDR)
#define GPIOH 			((GPIO_RegDef_t*) GPIOH_BASEADDR)

#define RCC				((RCC_RegDef_t*) RCC_BASEADDR)
#define EXTI			((EXTI_RegDef_t*) EXTI_BASEADDR)
#define SYSCFG			((SYSCFG_RegDef_t*) SYSCFG_BASEADDR)
#define SPI1			((SPI_RegDef_t*) SPI1_BASEADDR)
#define SPI2			((SPI_RegDef_t*) SPI2_I2S2_BASEADDR)
#define SPI3			((SPI_RegDef_t*) SPI3_I2S3_BASEADDR)
#define SPI4			((SPI_RegDef_t*) SPI4_BASEADDR)

#define I2C1			((I2C_RegDef_t*) I2C1_BASEADDR)
#define I2C2			((I2C_RegDef_t*) I2C2_BASEADDR)
#define I2C3			((I2C_RegDef_t*) I2C3_BASEADDR)

#define GPIOA_CLK_EN()			RCC->AHB1ENR |= (1 << 0)
#define GPIOB_CLK_EN()			RCC->AHB1ENR |= (1 << 1)
#define GPIOC_CLK_EN()			RCC->AHB1ENR |= (1 << 2)
#define GPIOD_CLK_EN()			RCC->AHB1ENR |= (1 << 3)
#define GPIOE_CLK_EN()			RCC->AHB1ENR |= (1 << 4)
#define GPIOF_CLK_EN()			RCC->AHB1ENR |= (1 << 5)
#define GPIOG_CLK_EN()			RCC->AHB1ENR |= (1 << 6)
#define GPIOH_CLK_EN()			RCC->AHB1ENR |= (1 << 7)
#define CRC_CLK_EN()			RCC->AHB1ENR |= (1 << 12)
#define BKPSRAM_CLK_EN()		RCC->AHB1ENR |= (1 << 18)
#define DMA1_CLK_EN()			RCC->AHB1ENR |= (1 << 21)
#define DMA2_CLK_EN()			RCC->AHB1ENR |= (1 << 22)
#define OTGHS_CLK_EN()			RCC->AHB1ENR |= (1 << 29)
#define OTGHSULPI_CLK_EN()		RCC->AHB1ENR |= (1 << 30)

#define DCMI_CLK_EN()			RCC->AHB2ENR |= (1 << 0)
#define OTFS_CLK_EN()			RCC->AHB2ENR |= (1 << 7)

#define FMC_CLK_EN()			RCC->AHB3ENR |= (1 << 0)
#define QSPI_CLK_EN()			RCC->AHB3ENR |= (1 << 1)

#define TIM2_CLK_EN()			RCC->APB1ENR |= (1 << 0)
#define TIM3_CLK_EN()			RCC->APB1ENR |= (1 << 1)
#define TIM4_CLK_EN()			RCC->APB1ENR |= (1 << 2)
#define TIM5_CLK_EN()			RCC->APB1ENR |= (1 << 3)
#define TIM6_CLK_EN()			RCC->APB1ENR |= (1 << 4)
#define TIM7_CLK_EN()			RCC->APB1ENR |= (1 << 5)
#define TIM12_CLK_EN()			RCC->APB1ENR |= (1 << 6)
#define TIM13_CLK_EN()			RCC->APB1ENR |= (1 << 7)
#define TIM14_CLK_EN()			RCC->APB1ENR |= (1 << 8)
#define WWDG_CLK_EN()			RCC->APB1ENR |= (1 << 11)
#define SPI2_CLK_EN()			RCC->APB1ENR |= (1 << 14)
#define SPI3_CLK_EN()			RCC->APB1ENR |= (1 << 15)
#define SPDIFRX_CLK_EN()		RCC->APB1ENR |= (1 << 16)
#define USART2_CLK_EN()			RCC->APB1ENR |= (1 << 17)
#define USART3_CLK_EN()			RCC->APB1ENR |= (1 << 18)
#define UART4_CLK_EN()			RCC->APB1ENR |= (1 << 19)
#define UART5_CLK_EN()			RCC->APB1ENR |= (1 << 20)
#define I2C1_CLK_EN()			RCC->APB1ENR |= (1 << 21)
#define I2C2_CLK_EN()			RCC->APB1ENR |= (1 << 22)
#define I2C3_CLK_EN()			RCC->APB1ENR |= (1 << 23)
#define FMPI2C1_CLK_EN()		RCC->APB1ENR |= (1 << 24)
#define CAN1_CLK_EN()			RCC->APB1ENR |= (1 << 25)
#define CAN2_CLK_EN()			RCC->APB1ENR |= (1 << 26)
#define CEC_CLK_EN()			RCC->APB1ENR |= (1 << 27)
#define PWR_CLK_EN()			RCC->APB1ENR |= (1 << 28)
#define DAC_CLK_EN()			RCC->APB1ENR |= (1 << 29)

#define TIM1_CLK_EN()			RCC->APB2ENR |= (1 << 0)
#define TIM8_CLK_EN()			RCC->APB2ENR |= (1 << 1)
#define USART1_CLK_EN()			RCC->APB2ENR |= (1 << 4)
#define USART6_CLK_EN()			RCC->APB2ENR |= (1 << 5)
#define ADC1_CLK_EN()			RCC->APB2ENR |= (1 << 8)
#define ADC2_CLK_EN()			RCC->APB2ENR |= (1 << 9)
#define ADC3_CLK_EN()			RCC->APB2ENR |= (1 << 10)
#define SDIO_CLK_EN()			RCC->APB2ENR |= (1 << 11)
#define SPI1_CLK_EN()			RCC->APB2ENR |= (1 << 12)
#define SPI4_CLK_EN()			RCC->APB2ENR |= (1 << 13)
#define SYSCFG_CLK_EN()			RCC->APB2ENR |= (1 << 14)
#define TIM9_CLK_EN()			RCC->APB2ENR |= (1 << 16)
#define TIM10_CLK_EN()			RCC->APB2ENR |= (1 << 17)
#define TIM11_CLK_EN()			RCC->APB2ENR |= (1 << 18)
#define SAI1_CLK_EN()			RCC->APB2ENR |= (1 << 22)
#define SAI2_CLK_EN()			RCC->APB2ENR |= (1 << 23)
#define GPIOA_CLK_DI()			RCC->AHB1ENR |= (1 << 0)
// #define GPIOA_CLK_DI()			RCC->AHB1ENR |= (1 << 0)
#define GPIOB_CLK_DI()			RCC->AHB1ENR &= ~(1 << 1)
#define GPIOC_CLK_DI()			RCC->AHB1ENR &= ~(1 << 2)
#define GPIOD_CLK_DI()			RCC->AHB1ENR &= ~(1 << 3)
#define GPIOE_CLK_DI()			RCC->AHB1ENR &= ~(1 << 4)
#define GPIOF_CLK_DI()			RCC->AHB1ENR &= ~(1 << 5)
#define GPIOG_CLK_DI()			RCC->AHB1ENR &= ~(1 << 6)
#define GPIOH_CLK_DI()			RCC->AHB1ENR &= ~(1 << 7)
#define CRC_CLK_DI()			RCC->AHB1ENR &= ~(1 << 12)
#define BKPSRAM_CLK_DI()		RCC->AHB1ENR &= ~(1 << 18)
#define DMA1_CLK_DI()			RCC->AHB1ENR &= ~(1 << 21)
#define DMA2_CLK_DI()			RCC->AHB1ENR &= ~(1 << 22)
#define OTGHS_CLK_DI()			RCC->AHB1ENR &= ~(1 << 29)
#define OTGHSULPI_CLK_DI()		RCC->AHB1ENR &= ~(1 << 30)

#define DCMI_CLK_DI()			RCC->AHB2ENR &= ~(1 << 0)
#define OTFS_CLK_DI()			RCC->AHB2ENR &= ~(1 << 7)

#define FMC_CLK_DI()			RCC->AHB3ENR &= ~(1 << 0)
#define QSPI_CLK_DI()			RCC->AHB3ENR &= ~(1 << 1)

#define TIM2_CLK_DI()			RCC->APB1ENR &= ~(1 << 0)
#define TIM3_CLK_DI()			RCC->APB1ENR &= ~(1 << 1)
#define TIM4_CLK_DI()			RCC->APB1ENR &= ~(1 << 2)
#define TIM5_CLK_DI()			RCC->APB1ENR &= ~(1 << 3)
#define TIM6_CLK_DI()			RCC->APB1ENR &= ~(1 << 4)
#define TIM7_CLK_DI()			RCC->APB1ENR &= ~(1 << 5)
#define TIM12_CLK_DI()			RCC->APB1ENR &= ~(1 << 6)
#define TIM13_CLK_DI()			RCC->APB1ENR &= ~(1 << 7)
#define TIM14_CLK_DI()			RCC->APB1ENR &= ~(1 << 8)
#define WWDG_CLK_DI()			RCC->APB1ENR &= ~(1 << 11)
#define SPI2_CLK_DI()			RCC->APB1ENR &= ~(1 << 14)
#define SPI3_CLK_DI()			RCC->APB1ENR &= ~(1 << 15)
#define SPDIFRX_CLK_DI()		RCC->APB1ENR &= ~(1 << 16)
#define USART2_CLK_DI()			RCC->APB1ENR &= ~(1 << 17)
#define USART3_CLK_DI()			RCC->APB1ENR &= ~(1 << 18)
#define UART4_CLK_DI()			RCC->APB1ENR &= ~(1 << 19)
#define UART5_CLK_DI()			RCC->APB1ENR &= ~(1 << 20)
#define I2C1_CLK_DI()			RCC->APB1ENR &= ~(1 << 21)
#define I2C2_CLK_DI()			RCC->APB1ENR &= ~(1 << 22)
#define I2C3_CLK_DI()			RCC->APB1ENR &= ~(1 << 23)
#define FMPI2C1_CLK_DI()		RCC->APB1ENR &= ~(1 << 24)
#define CAN1_CLK_DI()			RCC->APB1ENR &= ~(1 << 25)
#define CAN2_CLK_DI()			RCC->APB1ENR &= ~(1 << 26)
#define CEC_CLK_DI()			RCC->APB1ENR &= ~(1 << 27)
#define PWR_CLK_DI()			RCC->APB1ENR &= ~(1 << 28)
#define DAC_CLK_DI()			RCC->APB1ENR &= ~(1 << 29)

#define TIM1_CLK_DI()			RCC->APB2ENR &= ~(1 << 0)
#define TIM8_CLK_DI()			RCC->APB2ENR &= ~(1 << 1)
#define USART1_CLK_DI()			RCC->APB2ENR &= ~(1 << 4)
#define USART6_CLK_DI()			RCC->APB2ENR &= ~(1 << 5)
#define ADC1_CLK_DI()			RCC->APB2ENR &= ~(1 << 8)
#define ADC2_CLK_DI()			RCC->APB2ENR &= ~(1 << 9)
#define ADC3_CLK_DI()			RCC->APB2ENR &= ~(1 << 10)
#define SDIO_CLK_DI()			RCC->APB2ENR &= ~(1 << 11)
#define SPI1_CLK_DI()			RCC->APB2ENR &= ~(1 << 12)
#define SPI4_CLK_DI()			RCC->APB2ENR &= ~(1 << 13)
#define SYSCFG_CLK_DI()			RCC->APB2ENR &= ~(1 << 14)
#define TIM9_CLK_DI()			RCC->APB2ENR &= ~(1 << 16)
#define TIM10_CLK_DI()			RCC->APB2ENR &= ~(1 << 17)
#define TIM11_CLK_DI()			RCC->APB2ENR &= ~(1 << 18)
#define SAI1_CLK_DI()			RCC->APB2ENR &= ~(1 << 22)
#define SAI2_CLK_DI()			RCC->APB2ENR &= ~(1 << 23)

#define GPIOA_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0);} while(0)
#define GPIOB_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1);} while(0)
#define GPIOC_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2);} while(0)
#define GPIOD_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3);} while(0)
#define GPIOE_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4);} while(0)
#define GPIOF_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5);} while(0)
#define GPIOG_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6);} while(0)
#define GPIOH_REG_RESET()		do{RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7);} while(0)

#define SPI1_REG_RESET()		do{RCC->APB2RSTR |= (); RCC->APB2RSTR &= ~(1 << );} while(0)
#define SPI2_REG_RESET()		do{RCC->APB2RSTR |= (); RCC->APB2RSTR &= ~(1 << );} while(0)
#define SPI3_REG_RESET()		do{RCC->APB2RSTR |= (); RCC->APB2RSTR &= ~(1 << );} while(0)
#define SPI4_REG_RESET()		do{RCC->APB2RSTR |= (); RCC->APB2RSTR &= ~(1 << );} while(0)


#define IRQ_NO_WWDG					0
#define IRQ_NO_PVD					1
#define IRQ_NO_TAMP_STAMP			2
#define IRQ_NO_RTC_WKUP				3
#define IRQ_NO_FLASH				4
#define IRQ_NO_RCC					5
#define IRQ_NO_EXTI0				6
#define IRQ_NO_EXTI1				7
#define IRQ_NO_EXTI2				8
#define IRQ_NO_EXTI3				9
#define IRQ_NO_EXTI4				10
#define IRQ_NO_DMA1_STR0			11
#define IRQ_NO_DMA1_STR1			12
#define IRQ_NO_DMA1_STR2			13
#define IRQ_NO_DMA1_STR3			14
#define IRQ_NO_DMA1_STR4			15
#define IRQ_NO_DMA1_STR5			16
#define IRQ_NO_DMA1_STR6			17
#define IRQ_NO_ADC					18
#define IRQ_NO_CAN1_TX				19
#define IRQ_NO_CAN1_RX0				20
#define IRQ_NO_CAN1_RX1				21
#define IRQ_NO_CAN1_SCE				22
#define IRQ_NO_EXTI9_5				23
#define IRQ_NO_TIM1_BRK_TIM9		24
#define IRQ_NO_TIM1UPTIM10			25
#define IRQ_NO_TIM1_TRG_COM_TIM11	26
#define IRQ_NO_TIM1CC				27
#define IRQ_NO_TIM2					28
#define IRQ_NO_TIM3					29
#define IRQ_NO_TIM4					30
#define IRQ_NO_I2C1_EV				31
#define IRQ_NO_I2C1_ER				32
#define IRQ_NO_I2C2_EV				33
#define IRQ_NO_I2C2_ER				34
#define IRQ_NO_SPI1					35
#define IRQ_NO_SPI2					36
#define IRQ_NO_USART1				37
#define IRQ_NO_USART2				38
#define IRQ_NO_USART3				39
#define IRQ_NO_EXTI15_10			40
#define IRQ_NO_RTC_ALARM			41
#define IRQ_NO_OTG_FS_WKUP			42
#define IRQ_NO_TIM8_BRK_TIM12		43
#define IRQ_NO_TIM8_UP_TIM13		44
#define IRQ_NO_TIM8_TRG_COM_TIM14	45
#define IRQ_NO_TIM8CC				46
#define IRQ_NO_DMA1_STR7			47
#define IRQ_NO_FMC					48
#define IRQ_NO_SDIO					49
#define IRQ_NO_TIM5					50
#define IRQ_NO_SPI3					51
#define IRQ_NO_UART4				52
#define IRQ_NO_UART5				53
#define IRQ_NO_TIM6_DAC				54
#define IRQ_NO_TIM7					55
#define IRQ_NO_DMA2_STR0			56
#define IRQ_NO_DMA2_STR1			57
#define IRQ_NO_DMA2_STR2			58
#define IRQ_NO_DMA2_STR3			59
#define IRQ_NO_DMA2_STR4			60
#define IRQ_NO_RESERVED1			61
#define IRQ_NO_RESERVED2			62
#define IRQ_NO_CAN2_TX				63
#define IRQ_NO_CAN2_RX0				64
#define IRQ_NO_CAN2_RX1				65
#define IRQ_NO_CAN2_SCE				66
#define IRQ_NO_OTG_FS				67
#define IRQ_NO_DMA2_STR5			68
#define IRQ_NO_DMA2_STR6			69
#define IRQ_NO_DMA2_STR7			70
#define IRQ_NO_USART6				71
#define IRQ_NO_I2C3_EV				72
#define IRQ_NO_I2C3_ER				73
#define IRQ_NO_OTG_HS_EP1_OUT		74
#define IRQ_NO_OTG_HS_EP1_IN		75
#define IRQ_NO_OTG_HS_WKUP			76
#define IRQ_NO_OTG_HS				77
#define IRQ_NO_DCM					78
#define IRQ_NO_RESERVED3			79
#define IRQ_NO_RESERVED4			80
#define IRQ_NO_FPU					81
#define IRQ_NO_RESERVED5			82
#define IRQ_NO_RESERVED6			83
#define IRQ_NO_SPI4					84
#define IRQ_NO_RESERVED7			85
#define IRQ_NO_RESERVED8			86
#define IRQ_NO_SAI1					87
#define IRQ_NO_RESERVED9			88
#define IRQ_NO_RESERVED10			89
#define IRQ_NO_RESERVED11			90
#define IRQ_NO_SAI2					91
#define IRQ_NO_QUADSPI				92
#define IRQ_NO_HDMI_CEC				93
#define IRQ_NO_SPDIF_RX				94
#define IRQ_NO_FMPI2C1				95
#define IRQ_NO_FMPI2C1_ERROR		96

#define ENABLE 		1
#define DISABLE 	0
#define SET			ENABLE
#define RESET		DISABLE
#define FLAG_RESET	RESET
#define FLAG_SET	SET

#define SPI_CR1_CPHA_BITSHIFT		0
#define SPI_CR1_CPOL_BITSHIFT		1
#define SPI_CR1_MSTR_BITSHIFT		2
#define SPI_CR1_BR_BITSHIFT			3
#define SPI_CR1_ENABLE_BITSHIFT		6
#define SPI_CR1_LSBFIRST_BITSHIFT	7
#define SPI_CR1_SSI_BITSHIFT		8
#define SPI_CR1_SSM_BITSHIFT		9
#define SPI_CR1_RX_ONLY_BITSHIFT	10
#define SPI_CR1_DFF_BITSHIFT		11
#define SPI_CR1_CRCNEXT_BITSHIFT	12
#define SPI_CR1_CRCEN_BITSHIFT		13
#define SPI_CR1_BIDIOE_BISHIFT		14
#define SPI_CR1_BIDIMODE_BITSHIFT	15

#define SPI_CR2_RXDMAEN_BITSHIFT	0
#define SPI_CR2_TXDMAEN_BITSHIFT	1
#define SPI_CR2_SSOE_BITSHIFT		2
#define SPI_CR2_FRF_BITSHIFT		4
#define SPI_CR2_ERRIE_BITSHIFT		5
#define SPI_CR2_RXNEIE_BITSHIFT		6
#define SPI_CR2_TXEIE_BITSHIFT		7

#define SPI_SR_RXNE_BITSHIFT		0
#define SPI_SR_TXE_BITSHIFT			1
#define SPI_SR_CHSIDE_BITSHIFT		2
#define SPI_SR_UDR_BITSHIFT			3
#define SPI_SR_CRCERR_BITSHIFT		4
#define SPI_SR_MODEF_BITSHIFT		5
#define SPI_SR_OVR_BITSHIFT			6
#define SPI_SR_BSY_BITSHIFT			7
#define SPI_SR_FRE_BITSHIFT			8

#define I2C_CR1_PE_BITSHIFT				0
#define I2C_CR1_SMBUS_BITSHIFT			1

#define I2C_CR1_SMBTYPE_BITSHIFT		3
#define I2C_CR1_ENARP_BITSHIFT			4
#define I2C_CR1_ENPEC_BITSHIFT			5
#define I2C_CR1_ENGC_BITSHIFT			6
#define I2C_CR1_NOSTRETCH_BITSHIFT		7
#define I2C_CR1_START_BITSHIFT			8
#define I2C_CR1_STOP_BITSHIFT			9
#define I2C_CR1_ACK_BITSHIFT			10
#define I2C_CR1_POS_BITSHIFT			11
#define I2C_CR1_PEC_BITSHIFT			12
#define I2C_CR1_ALERT_BITSHIFT			13

#define I2C_CR1_SWRST_BITSHIFT			15

#define I2C_CR2_FREQ_BITSHIFT			0
#define I2C_CR2_ITERREN_BITSHIFT		8
#define I2C_CR2_ITEVENTEN_BITSHIFT		9
#define I2C_CR2_ITBUFEN_BITSHIFT		10
#define I2C_CR2_DMAEN_BITSHIFT			11
#define I2C_CR2_LAST_BITSHIFT			12

#define I2C_SR1_SB_BITSHIFT					0
#define I2C_SR1_ADDR_BITSHIFT				1
#define I2C_SR1_BTF_BITSHIFT				2
#define I2C_SR1_ADDIO_BITSHIFT				3
#define I2C_SR1_STOPF_BITSHIFT				4

#define I2C_SR1_RxNE_BITSHIFT				6
#define I2C_SR1_TxE_BITSHIFT				7
#define I2C_SR1_BERR_BITSHIFT				8
#define I2C_SR1_ARLO_BITSHIFT				9
#define I2C_SR1_AF_BITSHIFT					10
#define I2C_SR1_OVR_BITSHIFT				11
#define I2C_SR1_PECERR_BITSHIFT				12

#define I2C_SR1_TIMEOUT_BITSHIFT			14
#define I2C_SR1_SMBALERT_BITSHIFT			15

#define I2C_SR2_MSL_BITSHIFT				0
#define I2C_SR2_BUSY_BITSHIFT				1
#define I2C_SR2_TRA_BITSHIFT				2

#define I2C_SR2_GENCALL_BITSHIFT			4
#define I2C_SR2_SMBDEFAULT_BITSHIFT			5
#define I2C_SR2_SMBHOST_BITSHIFT			6
#define I2C_SR2_DUALF_BITSHIFT				7
#define I2C_SR2_PEC_BITSHIFT				8

#define I2C_CCR_BITSHIFT					0
#define I2C_CCR_DUTY_BITSHIFT				14
#define I2C_CCR_F_S_BITSHIFT				15




#define GPIO_BASEADDR_TO_CODE(x)		((x == GPIOA) ? 0 : \
										(x == GPIOB) ? 1 : \
										(x == GPIOC) ? 2 : \
										(x == GPIOD) ? 3 : \
										(x == GPIOE) ? 4 : \
										(x == GPIOF) ? 5 : \
										(x == GPIOG) ? 6 : \
										(x == GPIOH) ? 7 : 0)



#endif /* INC_STM32F446XX_H_ */
