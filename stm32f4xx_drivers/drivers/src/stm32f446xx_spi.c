/*
 * stm32f466xx_spi.c
 *
 *  Created on: 14 Oct. 2021
 *      Author: jack
 */

#include "stm32f446xx_spi.h"


void SPI_PeriphClkControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if(pSPIx == SPI1) {

		if(EnOrDi) SPI1_CLK_EN();
		else SPI1_CLK_DI();

	}
	else if(pSPIx == SPI2) {

		if(EnOrDi) SPI2_CLK_EN();
		else SPI2_CLK_DI();

	}
	else if(pSPIx == SPI3) {

		if(EnOrDi) SPI3_CLK_EN();
		else SPI3_CLK_DI();

	}
	else if(pSPIx == SPI4) {

		if(EnOrDi) SPI4_CLK_EN();
		else SPI4_CLK_DI();

	}

}


void SPI_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if(EnOrDi) {

		pSPIx->CR1 |= (1 << SPI_CR1_ENABLE_BITSHIFT);
	}
	else {

		pSPIx->CR1 &= ~(1 << SPI_CR1_ENABLE_BITSHIFT);
	}

}

void SPI_Init(SPI_Handle_t *pSPIHandle) {

	SPI_PeriphClkControl(pSPIHandle->pSPIx, ENABLE);

	uint32_t tempReg = 0;

	tempReg |= pSPIHandle->SPIConfig.DeviceMode << SPI_CR1_MSTR_BITSHIFT;


	if(pSPIHandle->SPIConfig.BusConfig == SPI_BUS_CONF_FD) {

		tempReg &= ~(1 << SPI_CR1_BIDIMODE_BITSHIFT);

	}
	else if(pSPIHandle->SPIConfig.BusConfig == SPI_BUS_CONF_HD) {

		tempReg |= (1 << SPI_CR1_BIDIMODE_BITSHIFT);

	}
	else if(pSPIHandle->SPIConfig.BusConfig == SPI_BUS_CONF_SMPLX_RX) {

		tempReg &= ~(1 << SPI_CR1_BIDIMODE_BITSHIFT);
		tempReg |= (1 << SPI_CR1_RX_ONLY_BITSHIFT);

	}

	tempReg |= pSPIHandle->SPIConfig.SCLKSpeed << SPI_CR1_BR_BITSHIFT;
	tempReg |= pSPIHandle->SPIConfig.DFF << SPI_CR1_DFF_BITSHIFT;
	tempReg |= pSPIHandle->SPIConfig.CPOL << SPI_CR1_CPOL_BITSHIFT;
	tempReg |= pSPIHandle->SPIConfig.CPHA << SPI_CR1_CPHA_BITSHIFT;

	pSPIHandle->pSPIx->CR1 = tempReg;


}


void SPI_DeInit(SPI_RegDef_t *pSPIx) {

	pSPIx->CR1 = (uint32_t) RESET;
	pSPIx->CR2 = (uint32_t) RESET;

}

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint16_t flagName) {

	return (uint8_t) (pSPIx->SR & flagName);

}


void SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len) {

	while(len > 0) {

		// wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_TXE_FLAG) == FLAG_SET);

		// test if 16 bit dff
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF_BITSHIFT)) {

			// load data into data register
			pSPIx->DR = *(uint16_t*)pTxBuffer;

			// decrease by len of 2 because 2 bytes
			len--;
			len--;

			(uint16_t*)pTxBuffer++;

		}
		else {

			// 8 bit dff
			pSPIx->DR = *(uint8_t*)pTxBuffer;

			// decrease by len of 1 because single byte
			len--;

			pTxBuffer++;

		}

	}

}


void SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t len) {

	while(len > 0) {

		// wait until RXE is set
		while(SPI_GetFlagStatus(pSPIx, SPI_RXNE_FLAG) == FLAG_SET);

		// test if 16 bit dff
		if(pSPIx->CR1 & (1 << SPI_CR1_DFF_BITSHIFT)) {

			// load data into data register
			*(uint16_t*)pRxBuffer = pSPIx->DR;

			// decrease by len of 2 because 2 bytes
			len--;
			len--;

			(uint16_t*)pRxBuffer++;

		}
		else {

			// 8 bit dff
			*(uint8_t*)pRxBuffer = pSPIx->DR;

			// decrease by len of 1 because single byte
			len--;

			pRxBuffer++;

		}

	}
}


void SPI_IRQNumConfig(uint8_t IRQNumber, uint8_t EnOrDi) {


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


void SPI_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority) {

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


uint8_t SPI_TxIT(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t len) {

	uint8_t state = pSPIxHandle->TxState;
	if(state != SPI_BUSY_IN_TX) {

		// 1. save the tx buffer addr and len information in global variables
		pSPIxHandle->pTxBuffer = pTxBuffer;
		pSPIxHandle->TxLen = len;

		// 2. mark the spi state as busy in transmission
		// so that no other code can take over same SPI peripheral until transmission is over
		pSPIxHandle->TxState = SPI_BUSY_IN_TX;

		// 3. enable TXEIE control to get interrupt whenever TXE flag is set in SR
		pSPIxHandle->pSPIx->CR2 |= (1 << SPI_CR2_TXEIE_BITSHIFT);

		// 4. data transmission will be handled by ISR code
	}

	return state;

}


void SPI_IRQHandling(SPI_Handle_t *pSPIxHandle) {

	uint8_t temp1 = 0;
	uint8_t temp2 = 0;

	temp1 = pSPIxHandle->pSPIx->SR & (1 << SPI_SR_TXE_BITSHIFT);
	temp2 = pSPIxHandle->pSPIx->CR2 & (1 << SPI_CR2_TXEIE_BITSHIFT);

	// if interrupt called because of TX
	if(temp1 && temp2) {

		// handle TXE
		SPI_TxIT_Handler(pSPIxHandle);

	}

	temp1 = pSPIxHandle->pSPIx->SR & (1 << SPI_SR_RXNE_BITSHIFT);
	temp2 = pSPIxHandle->pSPIx->CR2 & (1 << SPI_CR2_RXNEIE_BITSHIFT);

	// if interrupt called for RX
	if(temp1 && temp2) {

		// handle TXE
		SPI_RxIT_Handler(pSPIxHandle);

	}

	// error
	temp1 = pSPIxHandle->pSPIx->SR & (1 << SPI_SR_OVR_BITSHIFT);
	temp2 = pSPIxHandle->pSPIx->CR2 & (1 << SPI_CR2_ERRIE_BITSHIFT);
	if(temp1 && temp2) {

		SPI_ErrIT_Handler(pSPIxHandle);
	}


}


uint8_t SPI_RxIT(SPI_Handle_t *pSPIxHandle, uint8_t *pRxBuffer, uint32_t len) {

	uint8_t state = pSPIxHandle->RxState;
	if(state != SPI_BUSY_IN_TX) {

		// 1. save the rx buffer addr and len information in global variables
		pSPIxHandle->pRxBuffer = pRxBuffer;
		pSPIxHandle->RxLen = len;

		// 2. mark the spi state as busy in transmission
		// so that no other code can take over same SPI peripheral until transmission is over
		pSPIxHandle->RxState = SPI_BUSY_IN_RX;

		// 3. enable RXNEIE control to get interrupt whenever TXE flag is set in SR
		pSPIxHandle->pSPIx->CR2 |= (1 << SPI_CR2_RXNEIE_BITSHIFT);

		// 4. data transmission will be handled by ISR code
	}

	return state;

}


static void SPI_TxIT_Handler(SPI_Handle_t *pSPIxHandle) {

	// test if 16 bit dff
	if(pSPIxHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF_BITSHIFT)) {

		// load data into data register
		pSPIxHandle->pSPIx->DR = *(uint16_t*)pSPIxHandle->pTxBuffer;

		// decrease by len of 2 because 2 bytes
		pSPIxHandle->TxLen--;
		pSPIxHandle->TxLen--;

		(uint16_t*)pSPIxHandle->pTxBuffer++;

	}
	else {

		// 8 bit dff
		pSPIxHandle->pSPIx->DR = *(uint8_t*)pSPIxHandle->pTxBuffer;

		// decrease by len of 1 because single byte
		pSPIxHandle->TxLen--;

		(uint16_t*)pSPIxHandle->pTxBuffer++;

	}

	if(!pSPIxHandle->TxLen) {

		SPI_CloseTx(pSPIxHandle);
		SPI_ApplicationEventCallback(pSPIxHandle, SPI_EVENT_TX_COMPLETE);

	}

}


static void SPI_RxIT_Handler(SPI_Handle_t *pSPIxHandle) {

	// test if 16 bit dff
	if(pSPIxHandle->pSPIx->CR1 & (1 << SPI_CR1_DFF_BITSHIFT)) {

		// load data from data register
		*(uint16_t*)pSPIxHandle->pRxBuffer = pSPIxHandle->pSPIx->DR;

		// decrease by len of 2 because 2 bytes
		pSPIxHandle->RxLen--;
		pSPIxHandle->RxLen--;

		(uint16_t*)pSPIxHandle->pRxBuffer--;

	}
	else {

		// 8 bit dff
		*(uint8_t*)pSPIxHandle->pRxBuffer = pSPIxHandle->pSPIx->DR;

		// decrease by len of 1 because single byte
		pSPIxHandle->RxLen--;

		(uint16_t*)pSPIxHandle->pRxBuffer--;

	}

	if(!pSPIxHandle->RxLen) {

		SPI_CloseRx(pSPIxHandle);
		SPI_ApplicationEventCallback(pSPIxHandle, SPI_EVENT_RX_COMPLETE);

	}

}


static void SPI_ErrIT_Handler(SPI_Handle_t *pSPIxHandle) {

	// clear the OVR flag
	if(pSPIxHandle->TxState != SPI_BUSY_IN_TX) {

		SPI_ClearOVRFlag(pSPIxHandle);
	}

	// inform the application
	SPI_ApplicationEventCallback(pSPIxHandle, SPI_EVENT_OVR_ERR);


}


void SPI_ClearOVRFlag(SPI_Handle_t *pSPIxHandle) {

	(void)pSPIxHandle->pSPIx->DR;
	(void)pSPIxHandle->pSPIx->SR;

}


void SPI_CloseTx(SPI_Handle_t *pSPIxHandle) {

	// clear interrupt, close comms, inform application tx is over
	pSPIxHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_BITSHIFT);
	pSPIxHandle->pTxBuffer = NULL;
	pSPIxHandle->TxLen = 0;
	pSPIxHandle->TxState = SPI_READY;

}


void SPI_CloseRx(SPI_Handle_t *pSPIxHandle) {

	// clear interrupt, close comms
	pSPIxHandle->pSPIx->CR2 &= ~(1 << SPI_CR2_TXEIE_BITSHIFT);
	pSPIxHandle->pRxBuffer = NULL;
	pSPIxHandle->RxLen = 0;
	pSPIxHandle->RxState = SPI_READY;

}

// weak implementation, application may override
__attribute__((weak)) void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIxHandle, uint8_t appEvent);


void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if(EnOrDi) {

		pSPIx->CR1 |= (1 << SPI_CR1_SSI_BITSHIFT);

	}
	else {

		pSPIx->CR1 &= ~(1 << SPI_CR1_SSI_BITSHIFT);

	}
}


void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi) {

	if(EnOrDi) {

		pSPIx->CR2 |= (1 << SPI_CR2_SSOE_BITSHIFT);

	}
	else {

		pSPIx->CR2 &= ~(1 << SPI_CR2_SSOE_BITSHIFT);

	}
}
