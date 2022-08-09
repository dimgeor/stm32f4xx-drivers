/*
 * stm32f44xx_gpio_driver.c
 *
 *  Created on: Jul 21, 2021
 *      Author: dimitris
 */

#include "stm32f446xx_gpio_driver.h"

/*********************************************************************
 *						Peripheral Clock setup
 *********************************************************************/
/*********************************************************************
 * @fn      		  - GPIO_PeriClockControl
 *
 * @brief             - This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -
 *
 * @return            -  none
 *
 * @Note              -  none
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnorDi)
{
	if(EnorDi == ENABLE){
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
	}else{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}else if (pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}else if (pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}else if (pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}else if (pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}else if (pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}else if (pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}else if (pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
	}
}

/*********************************************************************
 *				Initialization and De-initialization
 *********************************************************************/
/*********************************************************************
 * @fn      		  - GPIO_Init
 *
 * @brief             - This function initializes the GPIO port
 *
 * @param[in]         - a structure which contains the base address of the GPIO peripheral and the PIN configuration settings
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t temp = 0; //temporary register

	//enable the peripheral clock

	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	//1. configure the mode
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){
		//non interrupt mode
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
		pGPIOHandle->pGPIOx->MODER &= ~( 0x3 << (2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
		pGPIOHandle->pGPIOx->MODER |= temp; //setting
	}else{
		//interrupt mode
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_FT ){
			//1. configure the FTSR
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->RTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode ==GPIO_MODE_IT_RT ){
			//1 . configure the RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR &= ~( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}else if (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT ){
			//1. configure both FTSR and RTSR
			EXTI->RTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			//Clear the corresponding RTSR bit
			EXTI->FTSR |= ( 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		}

		//2. configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4 ;
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t portcode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp1] = portcode << ( temp2 * 4);

		//3 . enable the exti interrupt delivery using IMR
		EXTI->IMR |= 1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber;
	}

	//2. configure the speed
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->OSPEEDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->OSPEEDR |= temp; //setting

	//3. configure the pupd settings
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber) );
	pGPIOHandle->pGPIOx->PUPDR &= ~( 0x3 << ( 2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //clearing
	pGPIOHandle->pGPIOx->PUPDR |= temp; //setting

	//4. configure the optype
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber );
	pGPIOHandle->pGPIOx->OTYPER &= ~( 0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //clearing
	pGPIOHandle->pGPIOx->OTYPER |= temp; //setting

	//5. configure the alt functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN){
		//configure the alt function registers
		uint8_t temp1, temp2;

		temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber  % 8;
		pGPIOHandle->pGPIOx->AFR[temp1] &= ~(0xF << ( 4 * temp2 ) ); //clearing
		pGPIOHandle->pGPIOx->AFR[temp1] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << ( 4 * temp2 ) ); //setting
	}
}


/*********************************************************************
 * @fn      		  - GPIO_DeInit
 *
 * @brief             - This function de-initializes the GPIO port
 *
 * @param[in]         - the base address of the GPIO peripheral we want to de-initialize
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx)
{
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}else if (pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}else if (pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}else if (pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}else if (pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}else if (pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}else if (pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}else if (pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
}

/*********************************************************************
 *						Data Read and Write
 *********************************************************************/
/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPin
 *
 * @brief             - This function reads the value from a PIN that is in input mode
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - the number of the PIN that we want to read
 * @param[in]         -
 *
 * @return            - the value of the input PIN
 *
 * @Note              - none
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	uint8_t value;
	value = (uint8_t)((pGPIOx->IDR >> PinNumber) & 0x00000001);
	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_ReadFromInputPort
 *
 * @brief             - This function reads the value of the whole PORT (PINs 0-15) that is in input mode at once
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - the value of the INPUT port
 *
 * @Note              - none
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	uint16_t value;
	value = (uint16_t)pGPIOx->IDR;
	return value;
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPin
 *
 * @brief             - This function writes a value to a PIN that is in output mode
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - the number of the PIN that we want to write to
 * @param[in]         - the value that we want to write to the PIN
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value)
{
	if(Value == GPIO_PIN_SET){
		//write 1 to the output data register at the bit field corresponding to the pin number
		pGPIOx->ODR |= ( 1 << PinNumber);
	}else{
		//write 0
		pGPIOx->ODR &= ~( 1 << PinNumber);
	}
}


/*********************************************************************
 * @fn      		  - GPIO_WriteToOutputPort
 *
 * @brief             - This function writes a value to the whole PORT (PINs 0-15) that is in output mode at once
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - the value that we want to write to the PORT
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t Value)
{
	pGPIOx->ODR = Value;
}


/*********************************************************************
 * @fn      		  - GPIO_ToggleOutputPin
 *
 * @brief             - This function toggles the value of a particular PIN
 *
 * @param[in]         - base address of the GPIO peripheral
 * @param[in]         - the number of the PIN that its value we want to toggle
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber)
{
	pGPIOx->ODR ^= ( 1 << PinNumber);
}

/*********************************************************************
 *				IRQ Configuration and ISR handling
 *********************************************************************/
/*********************************************************************
 * @fn      		  - GPIO_IRQInterruptConfig
 *
 * @brief             - This function enables or disables the interrupts for GPIOx
 *
 * @param[in]         - the IRQNumber of the interrupt
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -

 * @return            - none
 *
 * @Note              - none
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnorDi)
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
 * @fn      		  - GPIO_IRQPriorityConfig
 *
 * @brief             - This function configures the priority of the interrupts for GPIOx
 *
 * @param[in]         - the IRQNumber of the interrupt
 * @param[in]         - ENABLE or DISABLE macros
 * @param[in]         -

 * @return            - none
 *
 * @Note              - none
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber, uint8_t IRQPriority)
{
	//find out the ipr register
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section  = IRQNumber % 4;
	//lower part of the section is not implemented so we need to shift the value
	uint8_t shift_amount = ( 8 * iprx_section) + ( 8 - NO_PR_BITS_IMPLEMENTED) ;

	*(  NVIC_PR_BASEADDR + iprx ) |=  ( IRQPriority << shift_amount );
}

/*********************************************************************
 * @fn      		  - GPIO_IRQHandling
 *
 * @brief             - This function clears the EXTI pending register to
 * 						reset the request for interrupts
 *
 * @param[in]         - the number of the PIN that the interrupt occurs on
 * @param[in]         -
 * @param[in]         -
 *
 * @return            - none
 *
 * @Note              - none
 */

void GPIO_IRQHandling(uint8_t PinNumber)
{
	//clear the exti pending register corresponding to the pin number
	if(EXTI->PR & ( 1 << PinNumber))
	{
		EXTI->PR |= ( 1 << PinNumber);
	}
}

