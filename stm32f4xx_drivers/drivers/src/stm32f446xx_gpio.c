/*
 * stm32f446xx_gpio.c
 *
 *  Created on: 15 Sep. 2021
 *      Author: jack
 */

#include <stm32f446xx_gpio.h>
#include <sys/_stdint.h>


void GPIO_PeriphClkControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi) {

	if(pGPIOx == GPIOA) {

		if(EnOrDi) GPIOA_CLK_EN();
		else GPIOA_CLK_DI();

	}
	else if(pGPIOx == GPIOB) {

		if(EnOrDi) GPIOB_CLK_EN();
		else GPIOB_CLK_DI();

	}
	else if(pGPIOx == GPIOC) {

		if(EnOrDi) GPIOC_CLK_EN();
		else GPIOC_CLK_DI();

	}
	else if(pGPIOx == GPIOD) {

		if(EnOrDi) GPIOD_CLK_EN();
		else GPIOD_CLK_DI();
	}
	else if(pGPIOx == GPIOE) {

		if(EnOrDi) GPIOE_CLK_EN();
		else GPIOE_CLK_DI();

	}
	else if(pGPIOx == GPIOF) {

		if(EnOrDi) GPIOF_CLK_EN();
		else GPIOF_CLK_DI();

	}
	else if(pGPIOx == GPIOG) {

		if(EnOrDi) GPIOG_CLK_EN();
		else GPIOG_CLK_DI();

	}
	else if(pGPIOx == GPIOH) {

		if(EnOrDi) GPIOH_CLK_EN();
		else GPIOH_CLK_DI();

	}

}


void GPIO_Init(GPIO_Handle_t *pGPIOHandle) {

	/* 0 enable the peripheral clock */
	GPIO_PeriphClkControl(pGPIOHandle->pGPIOx, ENABLE);

	/* 1 mode */
	uint32_t temp = 0;
	if(pGPIOHandle->Config.Mode <= GPIO_MODE_ANALOG) {

		temp = (pGPIOHandle->Config.Mode << (2 * pGPIOHandle->Config.PinNumber));
		pGPIOHandle->pGPIOx->MODER &= ~(0b11 << pGPIOHandle->Config.PinNumber);
		pGPIOHandle->pGPIOx->MODER |= temp;

	}
	else {

		if(pGPIOHandle->Config.Mode  == GPIO_MODE_IT_FT) {

			/* clear rt register to be explicit */
			EXTI->RTSR &= ~(1 << pGPIOHandle->Config.PinNumber);
			EXTI->FTSR |= (1 << pGPIOHandle->Config.PinNumber);

		}

		else if(pGPIOHandle->Config.Mode == GPIO_MODE_IT_RT) {

			/* clear ft register to be explicit */
			EXTI->FTSR &= ~(1 << pGPIOHandle->Config.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->Config.PinNumber);

		}

		else if(pGPIOHandle->Config.Mode == GPIO_MODE_IT_RFT) {

			EXTI->FTSR |= (1 << pGPIOHandle->Config.PinNumber);
			EXTI->RTSR |= (1 << pGPIOHandle->Config.PinNumber);

		}

		/* configure GPIO port config in SYSCFG_EXTICR */
		uint8_t registerSelect = pGPIOHandle->Config.PinNumber / 4;
		uint8_t bitFieldShift =  pGPIOHandle->Config.PinNumber % 4;
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		const uint8_t portCodeLen = 4;
		SYSCFG->EXTICR[registerSelect] |= (portCode << portCodeLen * bitFieldShift);

		/* enable EXTI interrupt delivery using IMR */
		EXTI->IMR |= (1 << pGPIOHandle->Config.PinNumber);

	}

	/* 2 speed */
	temp = (pGPIOHandle->Config.Speed << (2 * pGPIOHandle->Config.PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR &= (0b11 << pGPIOHandle->Config.PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;

	/* 3 pupd */
	temp = (pGPIOHandle->Config.PuPdControl << (2 * pGPIOHandle->Config.PinNumber));
	pGPIOHandle->pGPIOx->PUPDR &= (0b11 << pGPIOHandle->Config.PinNumber);
	pGPIOHandle->pGPIOx->PUPDR |= temp;

	/* 4 optype */
	temp = (pGPIOHandle->Config.OpType << (pGPIOHandle->Config.PinNumber));
	pGPIOHandle->pGPIOx->OTYPER &= (0b1 << pGPIOHandle->Config.PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;


	/* 5 alt functionality */
	if(pGPIOHandle->Config.Mode == GPIO_MODE_ALTFN) {

		uint8_t AFRRegIndex = pGPIOHandle->Config.PinNumber / 8;
		uint8_t AFRBits = pGPIOHandle->Config.PinNumber % 8;

		/* clear register first, else anything left over will be ORed */
		pGPIOHandle->pGPIOx->AFR[AFRRegIndex] &= ~(0b1111 << AFRBits);
		pGPIOHandle->pGPIOx->AFR[AFRRegIndex] |= (pGPIOHandle->Config.AFMode << (4 * AFRBits));

	}

}


void GPIO_DeInit(GPIO_RegDef_t *pGPIOx) {

	/* reset registers of the peripheral */
	/* refer to associated bus for reset register */
	if(pGPIOx == GPIOA) {

		GPIOA_REG_RESET();

	}
	else if(pGPIOx == GPIOB) {

		GPIOB_REG_RESET();

	}
	else if(pGPIOx == GPIOC) {

		GPIOC_REG_RESET();

	}
	else if(pGPIOx == GPIOD) {

		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE) {

		GPIOE_REG_RESET();

	}
	else if(pGPIOx == GPIOF) {

		GPIOF_REG_RESET();

	}
	else if(pGPIOx == GPIOG) {

		GPIOG_REG_RESET();

	}
	else if(pGPIOx == GPIOH) {

		GPIOH_REG_RESET();

	}

}


uint8_t GPIO_ReadInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	return (uint8_t) ((pGPIOx->IDR >> PinNumber) & (0x00000001));

}


uint16_t GPIO_ReadInputPort(GPIO_RegDef_t *pGPIOx) {

	return (uint16_t) (pGPIOx->IDR & 0x0000FFFF);

}


void GPIO_WriteOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t value) {

	if(value) {

		pGPIOx->ODR |= (1 << PinNumber);

	}
	else {

		pGPIOx->ODR &= ~(1 << PinNumber);

	}


}


void GPIO_WriteOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t value) {

	pGPIOx->ODR = (uint32_t) value;

}


void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber) {

	pGPIOx->ODR ^= (1 << PinNumber);

}


void GPIO_IRQNumConfig(uint8_t IRQNumber, uint8_t EnOrDi) {


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

void GPIO_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority) {

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



void GPIO_IRQHandling(uint8_t PinNumber) {


	/* clear the relevant EXTI PR register */
	if(EXTI->PR & (1 << PinNumber)) {

		/* note that clearing the PR register is active high per documentation */
		EXTI->PR |= (1 << PinNumber);
	}


}
