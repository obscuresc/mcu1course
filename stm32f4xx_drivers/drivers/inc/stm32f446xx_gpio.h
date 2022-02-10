/*
 * stm32f446xx_gpio.h
 *
 *  Created on: 15 Sep. 2021
 *      Author: jack
 */

#ifndef INC_STM32F446XX_GPIO_H_
#define INC_STM32F446XX_GPIO_H_

#include <stdint.h>

#include "stm32f446xx.h"

typedef struct {

	uint8_t PinNumber;
	uint8_t Mode;
	uint8_t PuPdControl;
	uint8_t OpType;
	uint8_t AFMode;
	uint8_t Speed;


} GPIO_PinConfig_t;

typedef struct {

	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t Config;

} GPIO_Handle_t;

#define GPIO_MODE_IN		0
#define GPIO_MODE_OUT		1
#define GPIO_MODE_ALTFN		2
#define GPIO_MODE_ANALOG	3
#define GPIO_MODE_IT_FT		4
#define GPIO_MODE_IT_RT		5
#define GPIO_MODE_IT_RFT	6

#define GPIO_OTYPE_PP		0
#define GPIO_OTYPE_OD		1

#define GPIO_OSPEED_LOW		0
#define GPIO_OSPEED_MED		1
#define GPIO_OSPEED_HIGH	2
#define GPIO_OSPEED_VHIGH	3

#define GPIO_NO_PUPD		0
#define GPIO_PIN_PU			1
#define GPIO_PIN_PD			2

#define GPIO_PIN0			0
#define GPIO_PIN1			1
#define GPIO_PIN2			2
#define GPIO_PIN3			3
#define GPIO_PIN4			4
#define GPIO_PIN5			5
#define GPIO_PIN6			6
#define GPIO_PIN7			7
#define GPIO_PIN8			8
#define GPIO_PIN9			9
#define GPIO_PIN10			10
#define GPIO_PIN11			11
#define GPIO_PIN12			12
#define GPIO_PIN13			13
#define GPIO_PIN14			14
#define GPIO_PIN15			15


/*
 * APIs
 */

void GPIO_PeriphClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx);

void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value);
void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

void GPIO_IRQNumConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void GPIO_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);




#endif /* INC_STM32F446XX_GPIO_H_ */
