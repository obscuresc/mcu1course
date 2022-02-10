/*
 * stm32f446xx_i2c.h
 *
 *  Created on: 8 Feb. 2022
 *      Author: jack
 */

#ifndef INC_STM32F446XX_I2C_H_
#define INC_STM32F446XX_I2C_H_

#include "stm32f446xx.h"

typedef struct {

	uint32_t	SCLSpeed;
	uint8_t		DeviceAddr;
	uint8_t		ACKControl;
	uint16_t	FMDutyControl;

} I2C_Config_t;


typedef struct {

	I2C_RegDef_t *pSPIx;
	I2C_Config_t I2CConfig;


} I2C_Handle_t;


#define I2C_SCL_SPEED_STNDRD	100000
#define I2C_SCL_SPEED_FAST2k	200000
#define I2C_SCL_SPEED_FAST4k	400000

#define I2C_ACK_DI				0
#define I2C_ACK_EN				1

#define I2C_FM_DUTY_2			0
#define I2C_FM_DUTY_16_9		1


void I2C_PeriphClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_PeriphControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi);
void I2C_Init(I2C_Handle_t *pI2CHandle);
void I2C_DeInit(I2C_RegDef_t *pI2Cx);

uint8_t I2C_TxIT(I2C_Handle_t *pI2CxHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t I2C_RxIT(I2C_Handle_t *pI2CxHandle, uint8_t *pRXBuffer, uint32_t len);

static void I2C_TxIT_Handler(I2C_Handle_t *pI2CxHandle);
static void I2C_RxIT_Handler(I2C_Handle_t *pI2CxHandle);
static void I2C_ErrIT_Handler(I2C_Handle_t *pI2CxHandle);

void I2C_ApplicationEventCallback(I2C_Handle_t *pI2CxHandle, uint8_t appEvent);

void I2C_IRQNumConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void I2C_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);

uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint16_t flag);

#endif /* INC_STM32F446XX_I2C_H_ */
