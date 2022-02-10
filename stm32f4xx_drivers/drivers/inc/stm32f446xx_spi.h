/*
 * stm32f466x_spi.h
 *
 *  Created on: 14 Oct. 2021
 *      Author: jack
 */

#ifndef INC_STM32F446XX_SPI_H_
#define INC_STM32F446XX_SPI_H_

#include <stdint.h>

#include "stm32f446xx.h"


typedef struct {

	uint8_t DeviceMode;
	uint8_t BusConfig;
	uint8_t SCLKSpeed;
	uint8_t DFF;
	uint8_t CPOL;
	uint8_t CPHA;
	uint8_t SSM;

} SPI_Config_t;


typedef struct {

	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;
	uint8_t *pTxBuffer;
	uint8_t *pRxBuffer;
	uint32_t TxLen;
	uint32_t RxLen;
	uint8_t TxState;
	uint8_t RxState;

} SPI_Handle_t;


#define SPI_MODE_SLAVE				0
#define SPI_MODE_MASTER				1

#define SPI_BUS_CONF_FD				1
#define SPI_BUS_CONF_HD				2
#define SPI_BUS_CONF_SMPLX_TX		3
#define SPI_BUS_CONF_SMPLX_RX		4

#define SPI_SCLK_SPEED_DIV2			0
#define SPI_SCLK_SPEED_DIV4			1
#define SPI_SCLK_SPEED_DIV8			2
#define SPI_SCLK_SPEED_DIV16		3
#define SPI_SCLK_SPEED_DIV32		4
#define SPI_SCLK_SPEED_DIV64		5
#define SPI_SCLK_SPEED_DIV128		6
#define SPI_SCLK_SPEED_DIV256		7

#define SPI_DFF_8BIT				0
#define SPI_DFF_16BIT				1

#define SPI_CPOL_LOW				0
#define SPI_CPOL_HIGH				1

#define SPI_CPHA_LOW				0
#define SPI_CPHA_HIGH				1

#define SPI_SSM_DI					0
#define SPI_SSM_EN					1

#define SPI_RXNE_FLAG				(1 << SPI_SR_RXNE_BITSHIFT)
#define SPI_TXE_FLAG				(1 << SPI_SR_TXE_BITSHIFT)
#define SPI_CHSIDE_FLAG				(1 << SPI_SR_CHSIDE_BITSHIFT)
#define SPI_UDR_FLAG				(1 << SPI_SR_UDR_BITSHIFT)
#define SPI_CRCERR_FLAG				(1 << SPI_SR_CRCERR_BITSHIFT)
#define SPI_MODEF_FLAG				(1 << SPI_SR_MODEF_BITSHIFT)
#define SPI_OVR_FLAG				(1 << SPI_SR_OVR_BITSHIFT)
#define SPI_BSY_FLAG				(1 << SPI_SR_BSY_BITSHIFT)
#define SPI_FRE_FLAG				(1 << SPI_SR_FRE_BITSHIFT)

#define SPI_READY					0
#define SPI_BUSY_IN_RX				1
#define SPI_BUSY_IN_TX				2

#define SPI_EVENT_TX_COMPLETE		1
#define SPI_EVENT_RX_COMPLETE		2
#define SPI_EVENT_OVR_ERR			3
#define SPI_EVENT_CRC_ERR			4


void SPI_PeriphClkControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_PeriphControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

void SPI_Tx(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t len);
void SPI_Rx(SPI_RegDef_t *pSPIx, uint8_t *pRXBuffer, uint32_t len);

uint8_t SPI_TxIT(SPI_Handle_t *pSPIxHandle, uint8_t *pTxBuffer, uint32_t len);
uint8_t SPI_RxIT(SPI_Handle_t *pSPIxHandle, uint8_t *pRXBuffer, uint32_t len);
static void SPI_TxIT_Handler(SPI_Handle_t *pSPIxHandle);
static void SPI_RxIT_Handler(SPI_Handle_t *pSPIxHandle);
static void SPI_ErrIT_Handler(SPI_Handle_t *pSPIxHandle);

void SPI_ClearOVRFlag(SPI_Handle_t *pSPIxHandle);
void SPI_CloseTx(SPI_Handle_t *pSPIxHandle);
void SPI_CloseRx(SPI_Handle_t *pSPIxHandle);

void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIxHandle, uint8_t appEvent);

void SPI_IRQNumConfig(uint8_t IRQNumber, uint8_t EnOrDi);
void SPI_IRQPriority(uint8_t IRQNumber, uint32_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi);

uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx, uint16_t flag);

#endif /* INC_STM32F446XX_SPI_H_ */
