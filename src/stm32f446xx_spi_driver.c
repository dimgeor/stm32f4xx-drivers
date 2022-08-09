/*
 * stm32f446xx_spi_driver.c
 *
 *  Created on: Jul 28, 2021
 *      Author: Dimitris Georgoudis
 */
#include "stm32f446xx_spi_driver.h"

static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);

/*********************************************************************
 *					Peripheral Clock setup
 *********************************************************************/
/*********************************************************************
 * @fn      		  - SPI_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pSPIx == SPI1){
			SPI1_PCLK_EN();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_EN();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_EN();
		}else if (pSPIx == SPI4){
			SPI4_PCLK_EN();
		}
	}else{
		if(pSPIx == SPI1){
			SPI1_PCLK_DI();
		}else if (pSPIx == SPI2){
			SPI2_PCLK_DI();
		}else if (pSPIx == SPI3){
			SPI3_PCLK_DI();
		}else if (pSPIx == SPI4){
			SPI4_PCLK_DI();
		}
	}
}

/*********************************************************************
 *				Initialization and De-initialization
 *********************************************************************/
/*********************************************************************
 * @fn      		  - SPI_Init
 *
 * @brief             - This function initializes the given SPI peripheral
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	//confuguration of SPI_CR1 register
	uint32_t tempreg = 0;

	//1. configure the device mode
	tempreg |= pSPIHandle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR;

	//2. Configure the bus config
	if(pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_FD){
		//bidi mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_HD){
		//bidi mode should be set
		tempreg |= ( 1 << SPI_CR1_BIDIMODE);
	}else if (pSPIHandle->SPIConfig.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY){
		//BIDI mode should be cleared
		tempreg &= ~( 1 << SPI_CR1_BIDIMODE);
		//RXONLY bit must be set
		tempreg |= ( 1 << SPI_CR1_RXONLY);
	}

	// 3. Configure the spi serial clock speed (baud rate)
	tempreg |= pSPIHandle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR;

	//4.  Configure the DFF
	tempreg |= pSPIHandle->SPIConfig.SPI_DFF << SPI_CR1_DFF;

	//5. configure the CPOL
	tempreg |= pSPIHandle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL;

	//6 . configure the CPHA
	tempreg |= pSPIHandle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA;

	tempreg |= pSPIHandle->SPIConfig.SPI_SSM << SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1 = tempreg;

}
/*********************************************************************
 * @fn      		  - SPI_DeInit
 *
 * @brief             - This function de-initializes the given SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral we want to de-initialize
 * @param[in]         -
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx)
{
	if(pSPIx == SPI1){
		SPI1_REG_RESET();
	}else if (pSPIx == SPI2){
		SPI2_REG_RESET();
	}else if (pSPIx == SPI3){
		SPI3_REG_RESET();
	}else if (pSPIx == SPI4){
		SPI4_REG_RESET();
	}
}

/*********************************************************************
 *					Data Send and Receive
 *********************************************************************/
/*********************************************************************
 * @fn      		  - SPI_SendData
 *
 * @brief             - This function sends data in blocking mode
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - address of data we want to send
 * @param[in]         - length of data
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_SendData(SPI_RegDef_t *pSPIx, uint8_t *pTxBuffer, uint32_t Len)
{
	while(Len > 0){
		//1. wait until TXE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)  == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ){
			//16 bit DFF
			//1. load the data in to the DR
			pSPIx->DR =   *((uint16_t*)pTxBuffer);
			Len--;
			Len--;
			(uint16_t*)pTxBuffer++;
		}else{
			//8 bit DFF
			pSPIx->DR = *pTxBuffer;
			Len--;
			pTxBuffer++;
		}
	}

}
/*********************************************************************
 * @fn      		  - SPI_ReceiveData
 *
 * @brief             - This function receives data in blocking mode
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - address of data we want to receive
 * @param[in]         - length of data
 *
 * @return            -  none
 *
 * @Note              -  none
 */
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, uint8_t *pRxBuffer, uint32_t Len)
{
	while(Len > 0){
		//1. wait until RXNE is set
		while(SPI_GetFlagStatus(pSPIx,SPI_RXNE_FLAG)  == FLAG_RESET );

		//2. check the DFF bit in CR1
		if( (pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) ){
			//16 bit DFF
			//1. load the data from the DR to Rxbuffer address
			*((uint16_t*)pRxBuffer) = pSPIx->DR;
			Len--;
			Len--;
			(uint16_t*)pRxBuffer++;
		}else{
			//8 bit DFF
			*pRxBuffer = pSPIx->DR;
			Len--;
			pRxBuffer++;
		}
	}

}

/*********************************************************************
 * @fn      		  - SPI_SendDataIT
 *
 * @brief             - This function is used to send data in interrupt mode
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         - address of data we want to send
 * @param[in]         - length of data
 *
 * @return            -  state of Tx buffer
 *
 * @Note              -  none
 */
uint8_t SPI_SendDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;

	if(state != SPI_BUSY_IN_TX)
	{
		//1 . Save the Tx buffer address and Len information in some global variables
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;

		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_TXEIE );

	}


	return state;
}

/*********************************************************************
 * @fn      		  - SPI_ReceiveDataIT
 *
 * @brief             - This function is used to receive data in interrupt mode
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         - address of data we want to receive
 * @param[in]         - length of data
 *
 * @return            -  state of Rx buffer
 *
 * @Note              -  none
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, uint8_t *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;

	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;

		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->CR2 |= ( 1 << SPI_CR2_RXNEIE );

	}


	return state;
}


/*********************************************************************
 *				IRQ Configuration and ISR handling
 *********************************************************************/
/*********************************************************************
 * @fn      		  - SPI_IRQInterruptConfig
 *
 * @brief             - This function enables or disables the interrupts for SPIx
 *
 * @param[in]         - the IRQNumber of the interrupt
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -

 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(IRQNumber <= 31){
			*NVIC_ISER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ){
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && IRQNumber < 96 ){
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );

		}
	}else{
		if(IRQNumber <= 31){
			*NVIC_ICER0 |= ( 1 << IRQNumber );

		}else if(IRQNumber > 31 && IRQNumber < 64 ){
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );

		}else if(IRQNumber >= 64 && IRQNumber < 96 ){
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );

		}
	}
}

/*********************************************************************
 * @fn      		  - SPI_IRQPriorityConfig
 *
 * @brief             - This function configures the priority of the interrupts for SPIx
 *
 * @param[in]         - the IRQNumber of the interrupt
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -

 * @return            - none
 *
 * @Note              - none
 */

void SPI_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4;
	//lower part of the section is not implemented so we need to shift the value
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  - SPI_IRQHandling
 *
 * @brief             - This function handles the SPI interrupt
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_IRQHandling(SPI_Handle_t *pHandle)
{
	uint8_t temp1 , temp2;
	//first lets check for TXE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_TXE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_TXEIE);

	if( temp1 && temp2){
		//handle TXE
		spi_txe_interrupt_handle(pHandle);
	}

	// check for RXNE
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_RXNE);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_RXNEIE);

	if( temp1 && temp2){
		//handle RXNE
		spi_rxne_interrupt_handle(pHandle);
	}

	// check for ovr flag
	temp1 = pHandle->pSPIx->SR & ( 1 << SPI_SR_OVR);
	temp2 = pHandle->pSPIx->CR2 & ( 1 << SPI_CR2_ERRIE);

	if( temp1 && temp2){
	//handle ovr error
	spi_ovr_err_interrupt_handle(pHandle);
	}
}

/*********************************************************************
 * @fn      		  - spi_txe_interrupt_handle
 *
 * @brief             - Helper function to handle the interrupts of txe type
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	// check the DFF bit in CR1
	if( (pSPIHandle->pSPIx->CR1 & ( 1 << SPI_CR1_DFF) ) )
	{
		//16 bit DFF
		//1. load the data in to the DR
		pSPIHandle->pSPIx->DR =   *((uint16_t*)pSPIHandle->pTxBuffer);
		pSPIHandle->TxLen--;
		pSPIHandle->TxLen--;
		(uint16_t*)pSPIHandle->pTxBuffer++;
	}else
	{
		//8 bit DFF
		pSPIHandle->pSPIx->DR =   *pSPIHandle->pTxBuffer;
		pSPIHandle->TxLen--;
		pSPIHandle->pTxBuffer++;
	}

	if(! pSPIHandle->TxLen)
	{
		//TxLen is zero , so close the spi transmission and inform the application that
		//TX is over.

		//this prevents interrupts from setting up of TXE flag
		SPI_CloseTransmisson(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
	}

}

/*********************************************************************
 * @fn      		  - spi_rxne_interrupt_handle
 *
 * @brief             - Helper function to handle the interrupts of rxne type
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	//do rxing as per the dff
	if(pSPIHandle->pSPIx->CR1 & ( 1 << 11)){
		//16 bit
		*((uint16_t*)pSPIHandle->pRxBuffer) = (uint16_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen -= 2;
		pSPIHandle->pRxBuffer++;
		pSPIHandle->pRxBuffer++;

	}else{
		//8 bit
		*(pSPIHandle->pRxBuffer) = (uint8_t) pSPIHandle->pSPIx->DR;
		pSPIHandle->RxLen--;
		pSPIHandle->pRxBuffer++;
	}

	if(! pSPIHandle->RxLen){
		//reception is complete
		SPI_CloseReception(pSPIHandle);
		SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_RX_CMPLT);
	}

}

/*********************************************************************
 * @fn      		  - spi_ovr_err_interrupt_handle
 *
 * @brief             - Helper function to handle the interrupts when overun error occurs
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle)
{

	uint8_t temp;
	//1. clear the ovr flag
	if(pSPIHandle->TxState != SPI_BUSY_IN_TX){
		temp = pSPIHandle->pSPIx->DR;
		temp = pSPIHandle->pSPIx->SR;
	}
	(void)temp;
	//2. inform the application
	SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_OVR_ERR);

}

__weak void SPI_ApplicationEventCallback(SPI_Handle_t *pSPIHandle,uint8_t AppEv)
{

	//This is a weak implementation . the user application may override this function.
}



/*********************************************************************
 *					Other Peripheral Control Macros
 *********************************************************************/
/*********************************************************************
 * @fn      		  - SPI_PeripheralControl
 *
 * @brief             -	This function enables or disables the given SPI peripheral
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
		pSPIx->CR1 |= (1<< SPI_CR1_SPE);
	}else{
		pSPIx->CR1 &= ~(1<< SPI_CR1_SPE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSIConfig
 *
 * @brief             -	This function configures the SSI to avoid MODF errors
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
			pSPIx->CR1 |= (1<< SPI_CR1_SSI);
	}else{
			pSPIx->CR1 &= ~(1<< SPI_CR1_SSI);
	}
}

/*********************************************************************
 * @fn      		  - SPI_SSOEConfig
 *
 * @brief             -	This function configures the SSOE bit
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - enable or disable
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_SSOEConfig(SPI_RegDef_t *pSPIx, uint8_t EnOrDi)
{
	if(EnOrDi == ENABLE){
			pSPIx->CR2 |= (1<< SPI_CR2_SSOE);
	}else{
			pSPIx->CR2 &= ~(1<< SPI_CR2_SSOE);
	}
}

/*********************************************************************
 * @fn      		  - SPI_GetFlagStatus
 *
 * @brief             - This function is a helper function to read the flags
 * 						in the SPI status register
 *
 * @param[in]         - base address of the SPI peripheral
 * @param[in]         - the bit of he flag we want
 * @param[in]         -
 *
 * @return            -  1 or 0 depending on set/reset
 *
 * @Note              -  none
 */
uint8_t SPI_GetFlagStatus(SPI_RegDef_t *pSPIx , uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}

/*********************************************************************
 * @fn      		  - SPI_CloseTransmisson
 *
 * @brief             - Helper function to close the transmission of data
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_CloseTransmisson(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_TXEIE);
	pSPIHandle->pTxBuffer = NULL;
	pSPIHandle->TxLen = 0;
	pSPIHandle->TxState = SPI_READY;

}

/*********************************************************************
 * @fn      		  - SPI_CloseReception
 *
 * @brief             - Helper function to close the reception of data
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_CloseReception(SPI_Handle_t *pSPIHandle)
{
	pSPIHandle->pSPIx->CR2 &= ~( 1 << SPI_CR2_RXNEIE);
	pSPIHandle->pRxBuffer = NULL;
	pSPIHandle->RxLen = 0;
	pSPIHandle->RxState = SPI_READY;

}

/*********************************************************************
 * @fn      		  - SPI_ClearOVRFlag
 *
 * @brief             - Helper function to clear the overun flag
 *
 * @param[in]         - handle structure for the SPIx peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void SPI_ClearOVRFlag(SPI_RegDef_t *pSPIx)
{
	uint8_t temp;
	temp = pSPIx->DR;
	temp = pSPIx->SR;
	(void)temp;

}
