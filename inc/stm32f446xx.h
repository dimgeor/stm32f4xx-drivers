/*
 * stm32f446xx.h
 *
 *  Created on: Jul 20, 2021
 *      Author: dimitris
 */

#ifndef INC_STM32F446XX_H_
#define INC_STM32F446XX_H_

#include<stdint.h>
#include<stddef.h>

#define __vo 	volatile
#define __weak 	__attribute__((weak))

/*********************************************************************
 *					 Processor specific details
 *********************************************************************/
/*********************************************************************
 *					NVIC ISERx register addresses
 *********************************************************************/
#define NVIC_ISER0          ((__vo uint32_t*)0xE000E100U)
#define NVIC_ISER1          ((__vo uint32_t*)0xE000E104U)
#define NVIC_ISER2          ((__vo uint32_t*)0xE000E108U)
#define NVIC_ISER3          ((__vo uint32_t*)0xE000E10CU)

/*********************************************************************
 *					NVIC ICERx register addresses
 *********************************************************************/
#define NVIC_ICER0 			((__vo uint32_t*)0XE000E180U)
#define NVIC_ICER1			((__vo uint32_t*)0XE000E184U)
#define NVIC_ICER2  		((__vo uint32_t*)0XE000E188U)
#define NVIC_ICER3			((__vo uint32_t*)0XE000E18CU)

/*********************************************************************
 *					NVIC Priority register addresses
 *********************************************************************/
#define NVIC_PR_BASEADDR 	((__vo uint32_t*)0xE000E400U)

/*********************************************************************
 *	Number of priority bits implemented in NVIC Priority Register
 *********************************************************************/
#define NO_PR_BITS_IMPLEMENTED  4




/*********************************************************************
 *					STM32F446xx specific details
 *********************************************************************/
/*********************************************************************
 *					Interrupt table numbers
 *********************************************************************/
#define IRQ_NO_EXTI0            6   /* EXTI Line0 Interrupt */
#define IRQ_NO_EXTI1            7   /* EXTI Line1 Interrupt */
#define IRQ_NO_EXTI2            8   /* EXTI Line2 Interrupt */
#define IRQ_NO_EXTI3            9   /* EXTI Line3 Interrupt */
#define IRQ_NO_EXTI4            10  /* EXTI Line4 Interrupt */
#define IRQ_NO_EXTI9_5          23  /* EXTI Lines9_5   Interrupt */
#define IRQ_NO_EXTI15_10        40  /* EXTI Lines15_10 Interrupt */
#define IRQ_NO_SPI1        		35  /* SPI1 Interrupt */
#define IRQ_NO_SPI2        		36  /* SPI2 Interrupt */
#define IRQ_NO_SPI3        		51  /* SPI3 Interrupt */
#define IRQ_NO_SPI4        		84  /* SPI4 Interrupt */
#define IRQ_NO_I2C1_EV          31  /* I2C1 Event Interrupt */
#define IRQ_NO_I2C1_ER          32  /* I2C1 Error Interrupt */
#define IRQ_NO_I2C2_EV          33  /* I2C2 Event Interrupt */
#define IRQ_NO_I2C2_ER          34  /* I2C2 Error Interrupt */
#define IRQ_NO_I2C3_EV          72  /* I2C3 event interrupt */
#define IRQ_NO_I2C3_ER          73  /* I2C3 error interrupt */
#define IRQ_NO_USART1           37  /* USART1 global Interrupt */
#define IRQ_NO_USART2           38  /* USART2 global Interrupt */
#define IRQ_NO_USART3           39  /* USART3 global Interrupt */
#define IRQ_NO_UART4            52  /* UART4 global Interrupt */
#define IRQ_NO_UART5            53  /* UART5 global Interrupt */
#define	IRQ_NO_USART6           71  /* USART6 global interrupt */


/*********************************************************************
 *			Base addresses of Flash, SRAM and ROM memories
 *********************************************************************/
#define FLASH_BASEADDR 			0x08000000U
#define SRAM1_BASEADDR 			0x20000000U
#define SRAM2_BASEADDR			0x2001C000U
#define ROM						0x1FFF0000U
#define SRAM 					SRAM1_BASEADDR

/*********************************************************************
 *			APBx and AHBx Bus Peripheral base addresses
 *********************************************************************/
#define PERIPH_BASEADDR			0x40000000U
#define APB1PERIPH_BASEADDR		PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR		0x40010000U
#define AHB1PERIPH_BASEADDR		0x40020000U
#define AHB2PERIPH_BASEADDR		0x50000000U
#define AHB3PERIPH_BASEADDR		0x60000000U


/*********************************************************************
 *			Base addresses of peripherals on APB1 bus
 *********************************************************************/
#define TIM2_BASEADDR             (APB1PERIPH_BASEADDR + 0x0000U)
#define TIM3_BASEADDR             (APB1PERIPH_BASEADDR + 0x0400U)
#define TIM4_BASEADDR             (APB1PERIPH_BASEADDR + 0x0800U)
#define TIM5_BASEADDR             (APB1PERIPH_BASEADDR + 0x0C00U)
#define TIM6_BASEADDR             (APB1PERIPH_BASEADDR + 0x1000U)
#define TIM7_BASEADDR             (APB1PERIPH_BASEADDR + 0x1400U)
#define TIM12_BASEADDR            (APB1PERIPH_BASEADDR + 0x1800U)
#define TIM13_BASEADDR            (APB1PERIPH_BASEADDR + 0x1C00U)
#define TIM14_BASEADDR            (APB1PERIPH_BASEADDR + 0x2000U)
#define RTC_BASEADDR              (APB1PERIPH_BASEADDR + 0x2800U)
#define WWDG_BASEADDR             (APB1PERIPH_BASEADDR + 0x2C00U)
#define IWDG_BASEADDR             (APB1PERIPH_BASEADDR + 0x3000U)
#define SPI2_BASEADDR             (APB1PERIPH_BASEADDR + 0x3800U)
#define SPI3_BASEADDR             (APB1PERIPH_BASEADDR + 0x3C00U)
#define SPDIFRX_BASEADDR          (APB1PERIPH_BASEADDR + 0x4000U)
#define USART2_BASEADDR           (APB1PERIPH_BASEADDR + 0x4400U)
#define USART3_BASEADDR           (APB1PERIPH_BASEADDR + 0x4800U)
#define UART4_BASEADDR            (APB1PERIPH_BASEADDR + 0x4C00U)
#define UART5_BASEADDR            (APB1PERIPH_BASEADDR + 0x5000U)
#define I2C1_BASEADDR             (APB1PERIPH_BASEADDR + 0x5400U)
#define I2C2_BASEADDR             (APB1PERIPH_BASEADDR + 0x5800U)
#define I2C3_BASEADDR             (APB1PERIPH_BASEADDR + 0x5C00U)
#define FMPI2C1_BASEADDR          (APB1PERIPH_BASEADDR + 0x6000U)
#define CAN1_BASEADDR             (APB1PERIPH_BASEADDR + 0x6400U)
#define CAN2_BASEADDR             (APB1PERIPH_BASEADDR + 0x6800U)
#define CEC_BASEADDR              (APB1PERIPH_BASEADDR + 0x6C00U)
#define PWR_BASEADDR              (APB1PERIPH_BASEADDR + 0x7000U)
#define DAC_BASEADDR              (APB1PERIPH_BASEADDR + 0x7400U)

/*********************************************************************
 *			Base addresses of peripherals on APB2 bus
 *********************************************************************/
#define TIM1_BASEADDR             (APB2PERIPH_BASEADDR + 0x0000U)
#define TIM8_BASEADDR             (APB2PERIPH_BASEADDR + 0x0400U)
#define USART1_BASEADDR           (APB2PERIPH_BASEADDR + 0x1000U)
#define USART6_BASEADDR           (APB2PERIPH_BASEADDR + 0x1400U)
#define ADC1_BASEADDR             (APB2PERIPH_BASEADDR + 0x2000U)
#define ADC2_BASEADDR             (APB2PERIPH_BASEADDR + 0x2100U)
#define ADC3_BASEADDR             (APB2PERIPH_BASEADDR + 0x2200U)
#define SDIO_BASEADDR             (APB2PERIPH_BASEADDR + 0x2C00U)
#define SPI1_BASEADDR             (APB2PERIPH_BASEADDR + 0x3000U)
#define SPI4_BASEADDR             (APB2PERIPH_BASEADDR + 0x3400U)
#define SYSCFG_BASEADDR           (APB2PERIPH_BASEADDR + 0x3800U)
#define EXTI_BASEADDR             (APB2PERIPH_BASEADDR + 0x3C00U)
#define TIM9_BASEADDR             (APB2PERIPH_BASEADDR + 0x4000U)
#define TIM10_BASEADDR            (APB2PERIPH_BASEADDR + 0x4400U)
#define TIM11_BASEADDR            (APB2PERIPH_BASEADDR + 0x4800U)

/*********************************************************************
 *			Base addresses of peripherals on AHB1 bus
 *********************************************************************/
#define GPIOA_BASEADDR			  (AHB1PERIPH_BASEADDR + 0x0000U)
#define GPIOB_BASEADDR			  (AHB1PERIPH_BASEADDR + 0x0400U)
#define GPIOC_BASEADDR		  	  (AHB1PERIPH_BASEADDR + 0x0800U)
#define GPIOD_BASEADDR		  	  (AHB1PERIPH_BASEADDR + 0x0C00U)
#define GPIOE_BASEADDR		  	  (AHB1PERIPH_BASEADDR + 0x1000U)
#define GPIOF_BASEADDR		  	  (AHB1PERIPH_BASEADDR + 0x1400U)
#define GPIOG_BASEADDR		  	  (AHB1PERIPH_BASEADDR + 0x1800U)
#define GPIOH_BASEADDR		  	  (AHB1PERIPH_BASEADDR + 0x1C00U)
#define CRC_BASEADDR          	  (AHB1PERIPH_BASEADDR + 0x3000U)
#define RCC_BASEADDR          	  (AHB1PERIPH_BASEADDR + 0x3800U)

/*********************************************************************
 *			Peripheral register definition structures
 *********************************************************************/

/*********************************************************************
 *					GPIO register definition
 *********************************************************************/
typedef struct
{
	__vo uint32_t MODER;		/* GPIOx port mode register,               												address offset: 0x00*/
	__vo uint32_t OTYPER;	 	/* GPIOx port output type register, 													address offset: 0x04*/
	__vo uint32_t OSPEEDR;		/* GPIOx port output speed register,   													address offset: 0x08*/
	__vo uint32_t PUPDR;		/* GPIOx port pull-up/pull-down register,  												address offset: 0x0C*/
	__vo uint32_t IDR;			/* GPIOx port input data register,  													address offset: 0x10*/
	__vo uint32_t ODR;			/* GPIOx port output data register,  													address offset: 0x14*/
	__vo uint32_t BSSR;			/* GPIOx port bit set/reset register, 													address offset: 0x18*/
	__vo uint32_t LCKR;			/* GPIOx port configuration lock register,  											address offset: 0x1C*/
	__vo uint32_t AFR[2];		/* GPIOx port alternate function register (AFR[0]=low register, AFR[1]=high register),  address offset: 0x20*/

} GPIO_RegDef_t;

/*********************************************************************
 *					RCC register definition
 *********************************************************************/
typedef struct
{
	  __vo uint32_t CR;            /* RCC clock control register,                                  address offset: 0x00 */
	  __vo uint32_t PLLCFGR;       /* RCC PLL configuration register,                              address offset: 0x04 */
	  __vo uint32_t CFGR;          /* RCC clock configuration register,                            address offset: 0x08 */
	  __vo uint32_t CIR;           /* RCC clock interrupt register,                                address offset: 0x0C */
	  __vo uint32_t AHB1RSTR;      /* RCC AHB1 peripheral reset register,                          address offset: 0x10 */
	  __vo uint32_t AHB2RSTR;      /* RCC AHB2 peripheral reset register,                          address offset: 0x14 */
	  __vo uint32_t AHB3RSTR;      /* RCC AHB3 peripheral reset register,                          address offset: 0x18 */
	  uint32_t      RESERVED0;     /* Reserved, 0x1C                                                                    */
	  __vo uint32_t APB1RSTR;      /* RCC APB1 peripheral reset register,                          address offset: 0x20 */
	  __vo uint32_t APB2RSTR;      /* RCC APB2 peripheral reset register,                          address offset: 0x24 */
	  uint32_t      RESERVED1[2];  /* Reserved, 0x28-0x2C                                                               */
	  __vo uint32_t AHB1ENR;       /* RCC AHB1 peripheral clock register,                          address offset: 0x30 */
	  __vo uint32_t AHB2ENR;       /* RCC AHB2 peripheral clock register,                          address offset: 0x34 */
	  __vo uint32_t AHB3ENR;       /* RCC AHB3 peripheral clock register,                          address offset: 0x38 */
	  uint32_t      RESERVED2;     /* Reserved, 0x3C                                                                    */
	  __vo uint32_t APB1ENR;       /* RCC APB1 peripheral clock enable register,                   address offset: 0x40 */
	  __vo uint32_t APB2ENR;       /* RCC APB2 peripheral clock enable register,                   address offset: 0x44 */
	  uint32_t      RESERVED3[2];  /* Reserved, 0x48-0x4C                                                               */
	  __vo uint32_t AHB1LPENR;     /* RCC AHB1 peripheral clock enable in low power mode register, address offset: 0x50 */
	  __vo uint32_t AHB2LPENR;     /* RCC AHB2 peripheral clock enable in low power mode register, address offset: 0x54 */
	  __vo uint32_t AHB3LPENR;     /* RCC AHB3 peripheral clock enable in low power mode register, address offset: 0x58 */
	  uint32_t      RESERVED4;     /* Reserved, 0x5C                                                                    */
	  __vo uint32_t APB1LPENR;     /* RCC APB1 peripheral clock enable in low power mode register, address offset: 0x60 */
	  __vo uint32_t APB2LPENR;     /* RCC APB2 peripheral clock enable in low power mode register, address offset: 0x64 */
	  uint32_t      RESERVED5[2];  /* Reserved, 0x68-0x6C                                                               */
	  __vo uint32_t BDCR;          /* RCC Backup domain control register,                          address offset: 0x70 */
	  __vo uint32_t CSR;           /* RCC clock control & status register,                         address offset: 0x74 */
	  uint32_t      RESERVED6[2];  /* Reserved, 0x78-0x7C                                                               */
	  __vo uint32_t SSCGR;         /* RCC spread spectrum clock generation register,               address offset: 0x80 */
	  __vo uint32_t PLLI2SCFGR;    /* RCC PLLI2S configuration register,                           address offset: 0x84 */
	  __vo uint32_t PLLSAICFGR;    /* RCC PLLSAI configuration register,                           address offset: 0x88 */
	  __vo uint32_t DCKCFGR;       /* RCC Dedicated Clocks configuration register,                 address offset: 0x8C */
	  __vo uint32_t CKGATENR;      /* RCC Clocks Gated ENable Register,                            address offset: 0x90 */
	  __vo uint32_t DCKCFGR2;      /* RCC Dedicated Clocks configuration register 2,               address offset: 0x94 */

} RCC_RegDef_t;

/*********************************************************************
 *					EXTI register definition
 *********************************************************************/
typedef struct
{
  __vo uint32_t IMR;    /* EXTI Interrupt mask register,            address offset: 0x00 */
  __vo uint32_t EMR;    /* EXTI Event mask register,                address offset: 0x04 */
  __vo uint32_t RTSR;   /* EXTI Rising trigger selection register,  address offset: 0x08 */
  __vo uint32_t FTSR;   /* EXTI Falling trigger selection register, address offset: 0x0C */
  __vo uint32_t SWIER;  /* EXTI Software interrupt event register,  address offset: 0x10 */
  __vo uint32_t PR;     /* EXTI Pending register,                   address offset: 0x14 */

} EXTI_RegDef_t;

/*********************************************************************
 *					SYSCFG register definition
 *********************************************************************/
typedef struct
{
  __vo uint32_t MEMRMP;       /* SYSCFG memory remap register,                      address offset: 0x00      */
  __vo uint32_t PMC;          /* SYSCFG peripheral mode configuration register,     address offset: 0x04      */
  __vo uint32_t EXTICR[4];    /* SYSCFG external interrupt configuration registers, address offset: 0x08-0x14 */
  uint32_t      RESERVED[2];  /* Reserved, 0x18-0x1C                                                          */
  __vo uint32_t CMPCR;        /* SYSCFG Compensation cell control register,         address offset: 0x20      */
  uint32_t      RESERVED1[2]; /* Reserved, 0x24-0x28                                                          */
  __vo uint32_t CFGR;         /* SYSCFG Configuration register,                     address offset: 0x2C      */

} SYSCFG_RegDef_t;

/*********************************************************************
 *					SPI register definition
 *********************************************************************/
typedef struct
{
  __vo uint32_t CR1;        /* SPI control register 1 (not used in I2S mode),      address offset: 0x00 */
  __vo uint32_t CR2;        /* SPI control register 2,                             address offset: 0x04 */
  __vo uint32_t SR;         /* SPI status register,                                address offset: 0x08 */
  __vo uint32_t DR;         /* SPI data register,                                  address offset: 0x0C */
  __vo uint32_t CRCPR;      /* SPI CRC polynomial register (not used in I2S mode), address offset: 0x10 */
  __vo uint32_t RXCRCR;     /* SPI RX CRC register (not used in I2S mode),         address offset: 0x14 */
  __vo uint32_t TXCRCR;     /* SPI TX CRC register (not used in I2S mode),         address offset: 0x18 */
  __vo uint32_t I2SCFGR;    /* SPI_I2S configuration register,                     address offset: 0x1C */
  __vo uint32_t I2SPR;      /* SPI_I2S prescaler register,                         address offset: 0x20 */
} SPI_RegDef_t;

/*********************************************************************
 *					I2C register definition
 *********************************************************************/
typedef struct
{
  __vo uint32_t CR1;        /* I2C Control register 1,     address offset: 0x00 */
  __vo uint32_t CR2;        /* I2C Control register 2,     address offset: 0x04 */
  __vo uint32_t OAR1;       /* I2C Own address register 1, address offset: 0x08 */
  __vo uint32_t OAR2;       /* I2C Own address register 2, address offset: 0x0C */
  __vo uint32_t DR;         /* I2C Data register,          address offset: 0x10 */
  __vo uint32_t SR1;        /* I2C Status register 1,      address offset: 0x14 */
  __vo uint32_t SR2;        /* I2C Status register 2,      address offset: 0x18 */
  __vo uint32_t CCR;        /* I2C Clock control register, address offset: 0x1C */
  __vo uint32_t TRISE;      /* I2C TRISE register,         address offset: 0x20 */
  __vo uint32_t FLTR;       /* I2C FLTR register,          address offset: 0x24 */
} I2C_RegDef_t;

/*********************************************************************
 *					USART register definition
 *********************************************************************/
typedef struct
{
  __vo uint32_t SR;         /* USART Status register,                   address offset: 0x00 */
  __vo uint32_t DR;         /* USART Data register,                     address offset: 0x04 */
  __vo uint32_t BRR;        /* USART Baud rate register,                address offset: 0x08 */
  __vo uint32_t CR1;        /* USART Control register 1,                address offset: 0x0C */
  __vo uint32_t CR2;        /* USART Control register 2,                address offset: 0x10 */
  __vo uint32_t CR3;        /* USART Control register 3,                address offset: 0x14 */
  __vo uint32_t GTPR;       /* USART Guard time and prescaler register, address offset: 0x18 */
} USART_RegDef_t;

/*********************************************************************
 *					Peripheral definitions
 * 		i.e. peripheral base addresses typecasted to xxx_RegDef_t
 *********************************************************************/
#define GPIOA 		((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB 		((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC 		((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD 		((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE 		((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF 		((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG 		((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH 		((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define RCC 		((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI        ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG      ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

#define SPI1		((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI2		((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3		((SPI_RegDef_t*)SPI3_BASEADDR)
#define SPI4		((SPI_RegDef_t*)SPI4_BASEADDR)

#define I2C1 		((I2C_RegDef_t*)I2C1_BASEADDR)
#define I2C2 		((I2C_RegDef_t*)I2C2_BASEADDR)
#define I2C3 		((I2C_RegDef_t*)I2C3_BASEADDR)

#define USART1		((USART_RegDef_t*)USART1_BASEADDR)
#define USART2		((USART_RegDef_t*)USART2_BASEADDR)
#define USART3		((USART_RegDef_t*)USART3_BASEADDR)
#define UART4		((USART_RegDef_t*)UART4_BASEADDR)
#define UART5		((USART_RegDef_t*)UART5_BASEADDR)
#define USART6		((USART_RegDef_t*)USART6_BASEADDR)
/*********************************************************************
 *						Clock enable macros
 *********************************************************************/

/*********************************************************************
 *				Clock enable macros for GPIOx peripherals
 *********************************************************************/
#define GPIOA_PCLK_EN()			(RCC->AHB1ENR |= (1 << 0))
#define GPIOB_PCLK_EN()			(RCC->AHB1ENR |= (1 << 1))
#define GPIOC_PCLK_EN()			(RCC->AHB1ENR |= (1 << 2))
#define GPIOD_PCLK_EN()			(RCC->AHB1ENR |= (1 << 3))
#define GPIOE_PCLK_EN()			(RCC->AHB1ENR |= (1 << 4))
#define GPIOF_PCLK_EN()			(RCC->AHB1ENR |= (1 << 5))
#define GPIOG_PCLK_EN()			(RCC->AHB1ENR |= (1 << 6))
#define GPIOH_PCLK_EN()			(RCC->AHB1ENR |= (1 << 7))

/*********************************************************************
 *				Clock enable macros for I2Cx peripherals
 *********************************************************************/
#define I2C1_PCLK_EN() 			(RCC->APB1ENR |= (1 << 21))
#define I2C2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 22))
#define I2C3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 23))

/*********************************************************************
 *				Clock enable macros for SPIx peripherals
 *********************************************************************/
#define SPI1_PCLK_EN() 			(RCC->APB2ENR |= (1 << 12))
#define SPI2_PCLK_EN() 			(RCC->APB1ENR |= (1 << 14))
#define SPI3_PCLK_EN() 			(RCC->APB1ENR |= (1 << 15))
#define SPI4_PCLK_EN() 			(RCC->APB2ENR |= (1 << 13))

/*********************************************************************
 *				Clock enable macros for USARTx peripherals
 *********************************************************************/
#define USART1_PCLK_EN() 		(RCC->APB2ENR |= (1 << 4))
#define USART2_PCLK_EN() 		(RCC->APB1ENR |= (1 << 17))
#define USART3_PCLK_EN() 		(RCC->APB1ENR |= (1 << 18))
#define UART4_PCLK_EN() 		(RCC->APB1ENR |= (1 << 19))
#define UART5_PCLK_EN() 		(RCC->APB1ENR |= (1 << 20))
#define USART6_PCLK_EN() 		(RCC->APB2ENR |= (1 << 5))

/*********************************************************************
 *				Clock enable macros for SYSCFG peripheral
 *********************************************************************/
#define SYSCFG_PCLK_EN() 		(RCC->APB2ENR |= (1 << 14))


/*********************************************************************
 *						Clock disable macros
 *********************************************************************/

/*********************************************************************
 *				Clock disable macros for GPIOx peripherals
 *********************************************************************/
#define GPIOA_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 0))
#define GPIOB_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 1))
#define GPIOC_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 2))
#define GPIOD_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 3))
#define GPIOE_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 4))
#define GPIOF_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 5))
#define GPIOG_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 6))
#define GPIOH_PCLK_DI()			(RCC->AHB1ENR &= ~(1 << 7))

/*********************************************************************
 *				Clock disable macros for I2Cx peripherals
 *********************************************************************/
#define I2C1_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 21))
#define I2C2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 22))
#define I2C3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 23))

/*********************************************************************
 *				Clock disable macros for SPIx peripherals
 *********************************************************************/
#define SPI1_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 12))
#define SPI2_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 14))
#define SPI3_PCLK_DI() 			(RCC->APB1ENR &= ~(1 << 15))
#define SPI4_PCLK_DI() 			(RCC->APB2ENR &= ~(1 << 13))

/*********************************************************************
 *				Clock disable macros for USARTx peripherals
 *********************************************************************/
#define USART1_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 4))
#define USART2_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 17))
#define USART3_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 18))
#define UART4_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 19))
#define UART5_PCLK_DI() 		(RCC->APB1ENR &= ~(1 << 20))
#define USART6_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 5))

/*********************************************************************
 *				Clock disable macros for SYSCFGx peripheral
 *********************************************************************/
#define SYSCFG_PCLK_DI() 		(RCC->APB2ENR &= ~(1 << 14))


/*********************************************************************
 *					Macros to reset GPIOx peripheral
 *********************************************************************/
#define GPIOA_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 0); RCC->AHB1RSTR &= ~(1 << 0); }while(0)
#define GPIOB_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 1); RCC->AHB1RSTR &= ~(1 << 1); }while(0)
#define GPIOC_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 2); RCC->AHB1RSTR &= ~(1 << 2); }while(0)
#define GPIOD_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 3); RCC->AHB1RSTR &= ~(1 << 3); }while(0)
#define GPIOE_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 4); RCC->AHB1RSTR &= ~(1 << 4); }while(0)
#define GPIOF_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 5); RCC->AHB1RSTR &= ~(1 << 5); }while(0)
#define GPIOG_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 6); RCC->AHB1RSTR &= ~(1 << 6); }while(0)
#define GPIOH_REG_RESET()		do{ RCC->AHB1RSTR |= (1 << 7); RCC->AHB1RSTR &= ~(1 << 7); }while(0)


/*********************************************************************
 *	 Macro to return a code (0 to 7) for a given GPIO base address
 *********************************************************************/
#define GPIO_BASEADDR_TO_CODE(X)  ( (X == GPIOA) ? 0 :\
									(X == GPIOB) ? 1 :\
									(X == GPIOC) ? 2 :\
									(X == GPIOD) ? 3 :\
									(X == GPIOE) ? 4 :\
									(X == GPIOF) ? 5 :\
									(X == GPIOG) ? 6 :\
									(X == GPIOH) ? 7 :0 )


/*********************************************************************
 *					Macros to reset SPIx peripheral
 *********************************************************************/
#define SPI1_REG_RESET()		do{ RCC->APB2RSTR |= (1 << 12); RCC->APB2RSTR &= ~(1 << 12); }while(0)
#define SPI2_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 14); RCC->APB1RSTR &= ~(1 << 14); }while(0)
#define SPI3_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 15); RCC->APB1RSTR &= ~(1 << 15); }while(0)
#define SPI4_REG_RESET()		do{ RCC->APB2RSTR |= (1 << 13); RCC->APB2RSTR &= ~(1 << 13); }while(0)

/*********************************************************************
 *					Macros to reset I2Cx peripheral
 *********************************************************************/
#define I2C1_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 21); RCC->APB1RSTR &= ~(1 << 21); }while(0)
#define I2C2_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 22); RCC->APB1RSTR &= ~(1 << 22); }while(0)
#define I2C3_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 23); RCC->APB1RSTR &= ~(1 << 23); }while(0)

/*********************************************************************
 *					Macros to reset USARTx peripheral
 *********************************************************************/
#define USART1_REG_RESET()		do{ RCC->APB2RSTR |= (1 << 4); RCC->APB2RSTR &= ~(1 << 4); }while(0)
#define USART2_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 17); RCC->APB1RSTR &= ~(1 << 17); }while(0)
#define USART3_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 18); RCC->APB1RSTR &= ~(1 << 18); }while(0)
#define UART4_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 19); RCC->APB1RSTR &= ~(1 << 19); }while(0)
#define UART5_REG_RESET()		do{ RCC->APB1RSTR |= (1 << 20); RCC->APB1RSTR &= ~(1 << 20); }while(0)
#define USART6_REG_RESET()		do{ RCC->APB2RSTR |= (1 << 5); RCC->APB2RSTR &= ~(1 << 5); }while(0)
/*********************************************************************
 *						Generic macros
 *********************************************************************/
#define ENABLE 				1
#define DISABLE 			0
#define SET 				ENABLE
#define RESET 				DISABLE
#define GPIO_PIN_SET 		SET
#define GPIO_PIN_RESET 		RESET
#define FLAG_RESET			RESET
#define FLAG_SET			SET


/*********************************************************************
 *			Bit position definitions of SPI peripheral
 *********************************************************************/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7

/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/*********************************************************************
 *			Bit position definitions of I2C peripheral
 *********************************************************************/
/*
 * Bit position definitions I2C_CR1
 */
#define I2C_CR1_PE						0
#define I2C_CR1_NOSTRETCH  				7
#define I2C_CR1_START 					8
#define I2C_CR1_STOP  				 	9
#define I2C_CR1_ACK 				 	10
#define I2C_CR1_SWRST  				 	15

/*
 * Bit position definitions I2C_CR2
 */
#define I2C_CR2_FREQ				 	0
#define I2C_CR2_ITERREN				 	8
#define I2C_CR2_ITEVTEN				 	9
#define I2C_CR2_ITBUFEN 			    10

/*
 * Bit position definitions I2C_OAR1
 */
#define I2C_OAR1_ADD0    				0
#define I2C_OAR1_ADD71 				 	1
#define I2C_OAR1_ADD98  			 	8
#define I2C_OAR1_ADDMODE   			 	15

/*
 * Bit position definitions I2C_SR1
 */

#define I2C_SR1_SB 					 	0
#define I2C_SR1_ADDR 				 	1
#define I2C_SR1_BTF 					2
#define I2C_SR1_ADD10 					3
#define I2C_SR1_STOPF 					4
#define I2C_SR1_RXNE 					6
#define I2C_SR1_TXE 					7
#define I2C_SR1_BERR 					8
#define I2C_SR1_ARLO 					9
#define I2C_SR1_AF 					 	10
#define I2C_SR1_OVR 					11
#define I2C_SR1_TIMEOUT 				14

/*
 * Bit position definitions I2C_SR2
 */
#define I2C_SR2_MSL						0
#define I2C_SR2_BUSY 					1
#define I2C_SR2_TRA 					2
#define I2C_SR2_GENCALL 				4
#define I2C_SR2_DUALF 					7

/*
 * Bit position definitions I2C_CCR
 */
#define I2C_CCR_CCR 					 0
#define I2C_CCR_DUTY 					14
#define I2C_CCR_FS  				 	15

/**************************************************************
 *		Bit position definitions of USART peripheral
 **************************************************************/
/*
 * Bit position definitions USART_CR1
 */
#define USART_CR1_SBK					0
#define USART_CR1_RWU 					1
#define USART_CR1_RE  					2
#define USART_CR1_TE 					3
#define USART_CR1_IDLEIE 				4
#define USART_CR1_RXNEIE  				5
#define USART_CR1_TCIE					6
#define USART_CR1_TXEIE					7
#define USART_CR1_PEIE 					8
#define USART_CR1_PS 					9
#define USART_CR1_PCE 					10
#define USART_CR1_WAKE  				11
#define USART_CR1_M 					12
#define USART_CR1_UE 					13
#define USART_CR1_OVER8  				15



/*
 * Bit position definitions USART_CR2
 */
#define USART_CR2_ADD   				0
#define USART_CR2_LBDL   				5
#define USART_CR2_LBDIE  				6
#define USART_CR2_LBCL   				8
#define USART_CR2_CPHA   				9
#define USART_CR2_CPOL   				10
#define USART_CR2_STOP   				12
#define USART_CR2_LINEN   				14


/*
 * Bit position definitions USART_CR3
 */
#define USART_CR3_EIE   				0
#define USART_CR3_IREN   				1
#define USART_CR3_IRLP  				2
#define USART_CR3_HDSEL   				3
#define USART_CR3_NACK   				4
#define USART_CR3_SCEN   				5
#define USART_CR3_DMAR  				6
#define USART_CR3_DMAT   				7
#define USART_CR3_RTSE   				8
#define USART_CR3_CTSE   				9
#define USART_CR3_CTSIE   				10
#define USART_CR3_ONEBIT   				11

/*
 * Bit position definitions USART_SR
 */

#define USART_SR_PE        				0
#define USART_SR_FE        				1
#define USART_SR_NE        				2
#define USART_SR_ORE       				3
#define USART_SR_IDLE       			4
#define USART_SR_RXNE        			5
#define USART_SR_TC        				6
#define USART_SR_TXE        			7
#define USART_SR_LBD        			8
#define USART_SR_CTS        			9

#include "stm32f446xx_gpio_driver.h"
#include "stm32f446xx_spi_driver.h"
#include "stm32f446xx_i2c_driver.h"
#include "stm32f446xx_usart_driver.h"

#endif /* INC_STM32F446XX_H_ */
