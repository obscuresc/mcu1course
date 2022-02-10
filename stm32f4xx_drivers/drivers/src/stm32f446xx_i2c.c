/*
 * stm32f4xx_i2c.c
 *
 *  Created on: 8 Feb. 2022
 *      Author: jack
 */

#include "stm32f446xx_i2c.h"
#include "stm32f446xx.h"


void I2C_Init(I2C_Handle_t *pI2CHandle) {

	// 1. configure the mode
	pI2CHandle->pSPIx->CR1

	// 2. configure the speed of SCL
	pI2CHandle->pSPIx->CR2 |= (pI2CHandle->I2CConfig->SCLSpeed << I2C_CR2_FREQ_BITSHIFT);

	// 3. configure the device address

	// 4. enable ACKing

	// 5. configure rise time for I2C pins





}


void I2C_DeInit(I2C_RegDef_t *pI2Cx) {

	// TODO view RCC deinit

	pI2Cx->CR1 = (uint32_t) RESET;
	pI2Cx->CR2 = (uint32_t) RESET;

}


void I2C_PeriphControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {

	if(EnOrDi) {

		pI2Cx->CR1 |= (1 << I2C_CR1_PE_BITSHIFT);

	}
	else {

		pI2Cx->CR1 &= ~(1 << I2C_CR1_PE_BITSHIFT);

	}
}


void I2C_PeriphClkControl(I2C_RegDef_t *pI2Cx, uint8_t EnOrDi) {

	if(pI2Cx == I2C1) {

		if(EnOrDi) I2C1_CLK_EN();
		else I2C1_CLK_DI();

	}
	else if(pI2Cx == I2C2) {

		if(EnOrDi) I2C2_CLK_EN();
		else I2C2_CLK_DI();

	}
	else if(pI2Cx == I2C3) {

		if(EnOrDi) I2C3_CLK_EN();
		else I2C3_CLK_DI();

	}

}


void I2C_IRQNumConfig(uint8_t IRQNumber, uint8_t EnOrDi) {

	/* set or clear the associated IRQ number */
	if(EnOrDi) {

		if(IRQNumber < 32) {

			*NVIC_ISER0 |= (1 << IRQNumber);

		}

		else if (IRQNumber < 64) {

			*NVIC_ISER1 |= (1 << (IRQNumber % 32));

		}

		else if (IRQNumber < 96) {

			*NVIC_ISER2 |= (1 << (IRQNumber % 32));

		}

	}

	else {

		if(IRQNumber < 32) {

			*NVIC_ICER0 |= (1 << IRQNumber);

		}

		else if (IRQNumber < 64) {

			*NVIC_ICER1 |= (1 << (IRQNumber % 32));

		}

		else if (IRQNumber < 96) {

			*NVIC_ICER2 |= (1 << (IRQNumber % 32));

		}

	}
}


void I2C_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority) {

	/* find the interrupt priority register */
	const uint8_t fieldLength = 8;
	const uint8_t numIRQInRegister = 4;
	uint8_t registerIndex = IRQNumber / numIRQInRegister;
	uint8_t shiftInRegister = IRQNumber % numIRQInRegister;
	uint8_t resultantShift = (fieldLength * shiftInRegister) + (fieldLength - NUM_IR_PRIORITY_BITS);

	/* clear priority first */
	// *(NVIC_PRIORITY_BASEADDR + 4 * registerIndex) &= ~(0xFF << (fieldLength * shiftInRegister));
	*(NVIC_PRIORITY_BASEADDR + 4 * registerIndex) |= (IRQPriority << resultantShift);

}


uint8_t I2C_GetFlagStatus(I2C_RegDef_t *pI2Cx, uint16_t flag) {

	return (uint8_t) (pI2Cx->SR & flagName);

}
