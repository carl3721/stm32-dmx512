/*
 * startup_stm32f4xx.c
 * 
 * STM32 DMX512
 * Copyright (C) 2012 Erik Van Hamme, all rights reserved
 * 
 * This library is free software; you can redistribute it and/or
 * modify it under the terms of the GNU Lesser General Public
 * License as published by the Free Software Foundation; either
 * version 3 of the License, or (at your option) any later version.
 * 
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * Lesser General Public License for more details.
 * 
 * You should have received a copy of the GNU Lesser General Public
 * License along with this library; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA  02110-1301  USA
 */

/* 
 * This is the application startup file. It is basically a C translation
 * of the assembler based startup file that ships with the ST examples for the
 * STM32
 */

/*----------Macro definition--------------------------------------------------*/
#define WEAK __attribute__ ((weak))

/*----------Symbols defined in linker script----------------------------------*/
extern unsigned long _sidata;    /*!< Start address for the initialization
                                      values of the .data section.            */
extern unsigned long _sdata;     /*!< Start address for the .data section     */
extern unsigned long _edata;     /*!< End address for the .data section       */
extern unsigned long _sbss;      /*!< Start address for the .bss section      */
extern unsigned long _ebss;      /*!< End address for the .bss section        */
extern unsigned long _stack_top; /*!< Top address for stack                   */


/*----------Function prototypes-----------------------------------------------    */
extern int  main(void);              /*!< The entry point for the application.    */
extern void SystemInit(void);        /*!< Setup the microcontroller system(CMSIS) */
extern void __libc_init_array(void); /*!< Calls CTORS of static objects           */
void Default_Reset_Handler(void);    /*!< Default reset handler                   */
static void Default_Handler(void);   /*!< Default exception handler               */


/**
 * @brief  This is the code that gets called when the processor first
 *         starts execution following a reset event. Only the absolutely
 *         necessary set is performed, after which the application
 *         supplied main() routine is called.
 * @param  None
 * @retval None
 */
void Default_Reset_Handler(void)
{
	/* Initialize data and bss */
	unsigned long *pulSrc, *pulDest;

	/* Copy the data segment initializers from flash to SRAM */
	pulSrc = &_sidata;

	for(pulDest = &_sdata; pulDest < &_edata; ) {
		*(pulDest++) = *(pulSrc++);
	}

	/* Zero fill the bss segment. */
	for(pulDest = &_sbss; pulDest < &_ebss; ) {
		*(pulDest++) = 0;
	}

	/* Initialise global class instances (call constructors). */
	__libc_init_array();

	/* Setup the microcontroller system. */
	SystemInit();

	/* Call the application's entry point.*/
	main();
}


/**
 * @brief  This is the code that gets called when the processor receives an
 *         unexpected interrupt.  This simply enters an infinite loop,
 *         preserving the system state for examination by a debugger.
 * @param  None
 * @retval None
 */
static void Default_Handler(void)
{
	/* Go into an infinite loop. */
	while (1)
	{
	}
}

/*----------Declaration of the default fault handlers-------------------------*/
/* System exception vector handler */
void WEAK Reset_Handler(void);
void WEAK NMI_Handler(void);
void WEAK HardFault_Handler(void);
void WEAK MemManage_Handler(void);
void WEAK BusFault_Handler(void);
void WEAK UsageFault_Handler(void);
void WEAK SVC_Handler(void);
void WEAK DebugMon_Handler(void);
void WEAK PendSV_Handler(void);
void WEAK SysTick_Handler(void);

/* External exception vector handler */
void WEAK WWDG_IRQHandler(void);                   /* Window WatchDog              */                                        
void WEAK PVD_IRQHandler(void);                    /* PVD through EXTI Line detection */                        
void WEAK TAMP_STAMP_IRQHandler(void);             /* Tamper and TimeStamps through the EXTI line */            
void WEAK RTC_WKUP_IRQHandler(void);               /* RTC Wakeup through the EXTI line */                      
void WEAK FLASH_IRQHandler(void);                  /* FLASH                        */                                          
void WEAK RCC_IRQHandler(void);                    /* RCC                          */                                            
void WEAK EXTI0_IRQHandler(void);                  /* EXTI Line0                   */                        
void WEAK EXTI1_IRQHandler(void);                  /* EXTI Line1                   */                          
void WEAK EXTI2_IRQHandler(void);                  /* EXTI Line2                   */                          
void WEAK EXTI3_IRQHandler(void);                  /* EXTI Line3                   */                          
void WEAK EXTI4_IRQHandler(void);                  /* EXTI Line4                   */                          
void WEAK DMA1_Stream0_IRQHandler(void);           /* DMA1 Stream 0                */                  
void WEAK DMA1_Stream1_IRQHandler(void);           /* DMA1 Stream 1                */                   
void WEAK DMA1_Stream2_IRQHandler(void);           /* DMA1 Stream 2                */                   
void WEAK DMA1_Stream3_IRQHandler(void);           /* DMA1 Stream 3                */                   
void WEAK DMA1_Stream4_IRQHandler(void);           /* DMA1 Stream 4                */                   
void WEAK DMA1_Stream5_IRQHandler(void);           /* DMA1 Stream 5                */                   
void WEAK DMA1_Stream6_IRQHandler(void);           /* DMA1 Stream 6                */                   
void WEAK ADC_IRQHandler(void);                    /* ADC1, ADC2 and ADC3s         */                   
void WEAK CAN1_TX_IRQHandler(void);                /* CAN1 TX                      */                         
void WEAK CAN1_RX0_IRQHandler(void);               /* CAN1 RX0                     */                          
void WEAK CAN1_RX1_IRQHandler(void);               /* CAN1 RX1                     */                          
void WEAK CAN1_SCE_IRQHandler(void);               /* CAN1 SCE                     */                          
void WEAK EXTI9_5_IRQHandler(void);                /* External Line[9:5]s          */                          
void WEAK TIM1_BRK_TIM9_IRQHandler(void);          /* TIM1 Break and TIM9          */         
void WEAK TIM1_UP_TIM10_IRQHandler(void);          /* TIM1 Update and TIM10        */         
void WEAK TIM1_TRG_COM_TIM11_IRQHandler(void);     /* TIM1 Trigger and Commutation and TIM11 */
void WEAK TIM1_CC_IRQHandler(void);                /* TIM1 Capture Compare         */                          
void WEAK TIM2_IRQHandler(void);                   /* TIM2                         */                   
void WEAK TIM3_IRQHandler(void);                   /* TIM3                         */                   
void WEAK TIM4_IRQHandler(void);                   /* TIM4                         */                   
void WEAK I2C1_EV_IRQHandler(void);                /* I2C1 Event                   */                          
void WEAK I2C1_ER_IRQHandler(void);                /* I2C1 Error                   */                          
void WEAK I2C2_EV_IRQHandler(void);                /* I2C2 Event                   */                          
void WEAK I2C2_ER_IRQHandler(void);                /* I2C2 Error                   */                            
void WEAK SPI1_IRQHandler(void);                   /* SPI1                         */                   
void WEAK SPI2_IRQHandler(void);                   /* SPI2                         */                   
void WEAK USART1_IRQHandler(void);                 /* USART1                       */                   
void WEAK USART2_IRQHandler(void);                 /* USART2                       */                   
void WEAK USART3_IRQHandler(void);                 /* USART3                       */                   
void WEAK EXTI15_10_IRQHandler(void);              /* External Line[15:10]s        */                          
void WEAK RTC_Alarm_IRQHandler(void);              /* RTC Alarm (A and B) through EXTI Line */                 
void WEAK OTG_FS_WKUP_IRQHandler(void);            /* USB OTG FS Wakeup through EXTI line */                       
void WEAK TIM8_BRK_TIM12_IRQHandler(void);         /* TIM8 Break and TIM12         */         
void WEAK TIM8_UP_TIM13_IRQHandler(void);          /* TIM8 Update and TIM13        */         
void WEAK TIM8_TRG_COM_TIM14_IRQHandler(void);     /* TIM8 Trigger and Commutation and TIM14 */
void WEAK TIM8_CC_IRQHandler(void);                /* TIM8 Capture Compare         */                          
void WEAK DMA1_Stream7_IRQHandler(void);           /* DMA1 Stream7                 */                          
void WEAK FSMC_IRQHandler(void);                   /* FSMC                         */                   
void WEAK SDIO_IRQHandler(void);                   /* SDIO                         */                   
void WEAK TIM5_IRQHandler(void);                   /* TIM5                         */                   
void WEAK SPI3_IRQHandler(void);                   /* SPI3                         */                   
void WEAK UART4_IRQHandler(void);                  /* UART4                        */                   
void WEAK UART5_IRQHandler(void);                  /* UART5                        */                   
void WEAK TIM6_DAC_IRQHandler(void);               /* TIM6 and DAC1&2 underrun errors */                   
void WEAK TIM7_IRQHandler(void);                   /* TIM7                         */
void WEAK DMA2_Stream0_IRQHandler(void);           /* DMA2 Stream 0                */                   
void WEAK DMA2_Stream1_IRQHandler(void);           /* DMA2 Stream 1                */                   
void WEAK DMA2_Stream2_IRQHandler(void);           /* DMA2 Stream 2                */                   
void WEAK DMA2_Stream3_IRQHandler(void);           /* DMA2 Stream 3                */                   
void WEAK DMA2_Stream4_IRQHandler(void);           /* DMA2 Stream 4                */                   
void WEAK ETH_IRQHandler(void);                    /* Ethernet                     */                   
void WEAK ETH_WKUP_IRQHandler(void);               /* Ethernet Wakeup through EXTI line */                     
void WEAK CAN2_TX_IRQHandler(void);                /* CAN2 TX                      */                          
void WEAK CAN2_RX0_IRQHandler(void);               /* CAN2 RX0                     */                          
void WEAK CAN2_RX1_IRQHandler(void);               /* CAN2 RX1                     */                          
void WEAK CAN2_SCE_IRQHandler(void);               /* CAN2 SCE                     */                          
void WEAK OTG_FS_IRQHandler(void);                 /* USB OTG FS                   */                   
void WEAK DMA2_Stream5_IRQHandler(void);           /* DMA2 Stream 5                */                   
void WEAK DMA2_Stream6_IRQHandler(void);           /* DMA2 Stream 6                */                   
void WEAK DMA2_Stream7_IRQHandler(void);           /* DMA2 Stream 7                */                   
void WEAK USART6_IRQHandler(void);                 /* USART6                       */                    
void WEAK I2C3_EV_IRQHandler(void);                /* I2C3 event                   */                          
void WEAK I2C3_ER_IRQHandler(void);                /* I2C3 error                   */                          
void WEAK OTG_HS_EP1_OUT_IRQHandler(void);         /* USB OTG HS End Point 1 Out   */                   
void WEAK OTG_HS_EP1_IN_IRQHandler(void);          /* USB OTG HS End Point 1 In    */                   
void WEAK OTG_HS_WKUP_IRQHandler(void);            /* USB OTG HS Wakeup through EXTI */                         
void WEAK OTG_HS_IRQHandler(void);                 /* USB OTG HS                   */                   
void WEAK DCMI_IRQHandler(void);                   /* DCMI                         */                   
void WEAK CRYP_IRQHandler(void);                   /* CRYP crypto                  */                   
void WEAK HASH_RNG_IRQHandler(void);               /* Hash and Rng                 */
void WEAK FPU_IRQHandler(void);                    /* FPU                          */

/**
 *@brief The minimal vector table for a Cortex M3.  Note that the proper constructs
 *       must be placed on this to ensure that it ends up at physical address
 *       0x00000000.
 */
__attribute__ ((section(".isr_vector")))
void (* const g_pfnVectors[])(void) =
{
	/*----------Core Exceptions------------------------------------------------ */
	(void *)&_stack_top,       /*!< The initial stack pointer                   */
	Reset_Handler,             /*!< Reset Handler                               */
	NMI_Handler,               /*!< NMI Handler                                 */
	HardFault_Handler,         /*!< Hard Fault Handler                          */
	MemManage_Handler,         /*!< MPU Fault Handler                           */
	BusFault_Handler,          /*!< Bus Fault Handler                           */
	UsageFault_Handler,        /*!< Usage Fault Handler                         */
	0, 0, 0, 0,                /*!< Reserved                                    */
	SVC_Handler,               /*!< SVCall Handler                              */
	DebugMon_Handler,          /*!< Debug Monitor Handler                       */
	0,                         /*!< Reserved                                    */
	PendSV_Handler,            /*!< PendSV Handler                              */
	SysTick_Handler,           /*!< SysTick Handler                             */

	/*----------External Exceptions---------------------------------------------*/
	WWDG_IRQHandler,                   /*!< Window WatchDog              */                                        
	PVD_IRQHandler,                    /*!< PVD through EXTI Line detection */                        
	TAMP_STAMP_IRQHandler,             /*!< Tamper and TimeStamps through the EXTI line */            
	RTC_WKUP_IRQHandler,               /*!< RTC Wakeup through the EXTI line */                      
	FLASH_IRQHandler,                  /*!< FLASH                        */                                          
	RCC_IRQHandler,                    /*!< RCC                          */                                            
	EXTI0_IRQHandler,                  /*!< EXTI Line0                   */                        
	EXTI1_IRQHandler,                  /*!< EXTI Line1                   */                          
	EXTI2_IRQHandler,                  /*!< EXTI Line2                   */                          
	EXTI3_IRQHandler,                  /*!< EXTI Line3                   */                          
	EXTI4_IRQHandler,                  /*!< EXTI Line4                   */                          
	DMA1_Stream0_IRQHandler,           /*!< DMA1 Stream 0                */                  
	DMA1_Stream1_IRQHandler,           /*!< DMA1 Stream 1                */                   
	DMA1_Stream2_IRQHandler,           /*!< DMA1 Stream 2                */                   
	DMA1_Stream3_IRQHandler,           /*!< DMA1 Stream 3                */                   
	DMA1_Stream4_IRQHandler,           /*!< DMA1 Stream 4                */                   
	DMA1_Stream5_IRQHandler,           /*!< DMA1 Stream 5                */                   
	DMA1_Stream6_IRQHandler,           /*!< DMA1 Stream 6                */                   
	ADC_IRQHandler,                    /*!< ADC1, ADC2 and ADC3s         */                   
	CAN1_TX_IRQHandler,                /*!< CAN1 TX                      */                         
	CAN1_RX0_IRQHandler,               /*!< CAN1 RX0                     */                          
	CAN1_RX1_IRQHandler,               /*!< CAN1 RX1                     */                          
	CAN1_SCE_IRQHandler,               /*!< CAN1 SCE                     */                          
	EXTI9_5_IRQHandler,                /*!< External Line[9:5]s          */                          
	TIM1_BRK_TIM9_IRQHandler,          /*!< TIM1 Break and TIM9          */         
	TIM1_UP_TIM10_IRQHandler,          /*!< TIM1 Update and TIM10        */         
	TIM1_TRG_COM_TIM11_IRQHandler,     /*!< TIM1 Trigger and Commutation and TIM11 */
	TIM1_CC_IRQHandler,                /*!< TIM1 Capture Compare         */                          
	TIM2_IRQHandler,                   /*!< TIM2                         */                   
	TIM3_IRQHandler,                   /*!< TIM3                         */                   
	TIM4_IRQHandler,                   /*!< TIM4                         */                   
	I2C1_EV_IRQHandler,                /*!< I2C1 Event                   */                          
	I2C1_ER_IRQHandler,                /*!< I2C1 Error                   */                          
	I2C2_EV_IRQHandler,                /*!< I2C2 Event                   */                          
	I2C2_ER_IRQHandler,                /*!< I2C2 Error                   */                            
	SPI1_IRQHandler,                   /*!< SPI1                         */                   
	SPI2_IRQHandler,                   /*!< SPI2                         */                   
	USART1_IRQHandler,                 /*!< USART1                       */                   
	USART2_IRQHandler,                 /*!< USART2                       */                   
	USART3_IRQHandler,                 /*!< USART3                       */                   
	EXTI15_10_IRQHandler,              /*!< External Line[15:10]s        */                          
	RTC_Alarm_IRQHandler,              /*!< RTC Alarm (A and B) through EXTI Line */                 
	OTG_FS_WKUP_IRQHandler,            /*!< USB OTG FS Wakeup through EXTI line */                       
	TIM8_BRK_TIM12_IRQHandler,         /*!< TIM8 Break and TIM12         */         
	TIM8_UP_TIM13_IRQHandler,          /*!< TIM8 Update and TIM13        */         
	TIM8_TRG_COM_TIM14_IRQHandler,     /*!< TIM8 Trigger and Commutation and TIM14 */
	TIM8_CC_IRQHandler,                /*!< TIM8 Capture Compare         */                          
	DMA1_Stream7_IRQHandler,           /*!< DMA1 Stream7                 */                          
	FSMC_IRQHandler,                   /*!< FSMC                         */                   
	SDIO_IRQHandler,                   /*!< SDIO                         */                   
	TIM5_IRQHandler,                   /*!< TIM5                         */                   
	SPI3_IRQHandler,                   /*!< SPI3                         */                   
	UART4_IRQHandler,                  /*!< UART4                        */                   
	UART5_IRQHandler,                  /*!< UART5                        */                   
	TIM6_DAC_IRQHandler,               /*!< TIM6 and DAC1&2 underrun errors */                   
	TIM7_IRQHandler,                   /*!< TIM7                         */
	DMA2_Stream0_IRQHandler,           /*!< DMA2 Stream 0                */                   
	DMA2_Stream1_IRQHandler,           /*!< DMA2 Stream 1                */                   
	DMA2_Stream2_IRQHandler,           /*!< DMA2 Stream 2                */                   
	DMA2_Stream3_IRQHandler,           /*!< DMA2 Stream 3                */                   
	DMA2_Stream4_IRQHandler,           /*!< DMA2 Stream 4                */                   
	ETH_IRQHandler,                    /*!< Ethernet                     */                   
	ETH_WKUP_IRQHandler,               /*!< Ethernet Wakeup through EXTI line */                     
	CAN2_TX_IRQHandler,                /*!< CAN2 TX                      */                          
	CAN2_RX0_IRQHandler,               /*!< CAN2 RX0                     */                          
	CAN2_RX1_IRQHandler,               /*!< CAN2 RX1                     */                          
	CAN2_SCE_IRQHandler,               /*!< CAN2 SCE                     */                          
	OTG_FS_IRQHandler,                 /*!< USB OTG FS                   */                   
	DMA2_Stream5_IRQHandler,           /*!< DMA2 Stream 5                */                   
	DMA2_Stream6_IRQHandler,           /*!< DMA2 Stream 6                */                   
	DMA2_Stream7_IRQHandler,           /*!< DMA2 Stream 7                */                   
	USART6_IRQHandler,                 /*!< USART6                       */                    
	I2C3_EV_IRQHandler,                /*!< I2C3 event                   */                          
	I2C3_ER_IRQHandler,                /*!< I2C3 error                   */                          
	OTG_HS_EP1_OUT_IRQHandler,         /*!< USB OTG HS End Point 1 Out   */                   
	OTG_HS_EP1_IN_IRQHandler,          /*!< USB OTG HS End Point 1 In    */                   
	OTG_HS_WKUP_IRQHandler,            /*!< USB OTG HS Wakeup through EXTI */                         
	OTG_HS_IRQHandler,                 /*!< USB OTG HS                   */                   
	DCMI_IRQHandler,                   /*!< DCMI                         */                   
	CRYP_IRQHandler,                   /*!< CRYP crypto                  */                   
	HASH_RNG_IRQHandler,               /*!< Hash and Rng                 */
	FPU_IRQHandler                     /*!< FPU                          */
};

/**
 *@brief Provide weak aliases for each Exception handler to the Default_Handler.
 *       As they are weak aliases, any function with the same name will override
 *       this definition.
 */
#pragma weak Reset_Handler = Default_Reset_Handler
#pragma weak NMI_Handler = Default_Handler
#pragma weak HardFault_Handler = Default_Handler
#pragma weak MemManage_Handler = Default_Handler
#pragma weak BusFault_Handler = Default_Handler
#pragma weak UsageFault_Handler = Default_Handler
#pragma weak SVC_Handler = Default_Handler
#pragma weak DebugMon_Handler = Default_Handler
#pragma weak PendSV_Handler = Default_Handler
#pragma weak SysTick_Handler = Default_Handler
#pragma weak WWDG_IRQHandler = Default_Handler                                                           
#pragma weak PVD_IRQHandler = Default_Handler                                            
#pragma weak TAMP_STAMP_IRQHandler = Default_Handler                         
#pragma weak RTC_WKUP_IRQHandler = Default_Handler                                     
#pragma weak FLASH_IRQHandler = Default_Handler                                                            
#pragma weak RCC_IRQHandler = Default_Handler                                                                
#pragma weak EXTI0_IRQHandler = Default_Handler                                          
#pragma weak EXTI1_IRQHandler = Default_Handler                                            
#pragma weak EXTI2_IRQHandler = Default_Handler                                            
#pragma weak EXTI3_IRQHandler = Default_Handler                                            
#pragma weak EXTI4_IRQHandler = Default_Handler                                            
#pragma weak DMA1_Stream0_IRQHandler = Default_Handler                             
#pragma weak DMA1_Stream1_IRQHandler = Default_Handler                              
#pragma weak DMA1_Stream2_IRQHandler = Default_Handler                              
#pragma weak DMA1_Stream3_IRQHandler = Default_Handler                              
#pragma weak DMA1_Stream4_IRQHandler = Default_Handler                              
#pragma weak DMA1_Stream5_IRQHandler = Default_Handler                              
#pragma weak DMA1_Stream6_IRQHandler = Default_Handler                              
#pragma weak ADC_IRQHandler = Default_Handler                                       
#pragma weak CAN1_TX_IRQHandler = Default_Handler                                         
#pragma weak CAN1_RX0_IRQHandler = Default_Handler                                         
#pragma weak CAN1_RX1_IRQHandler = Default_Handler                                         
#pragma weak CAN1_SCE_IRQHandler = Default_Handler                                         
#pragma weak EXTI9_5_IRQHandler = Default_Handler                                          
#pragma weak TIM1_BRK_TIM9_IRQHandler = Default_Handler                   
#pragma weak TIM1_UP_TIM10_IRQHandler = Default_Handler                   
#pragma weak TIM1_TRG_COM_TIM11_IRQHandler = Default_Handler     
#pragma weak TIM1_CC_IRQHandler = Default_Handler                                          
#pragma weak TIM2_IRQHandler = Default_Handler                                      
#pragma weak TIM3_IRQHandler = Default_Handler                                      
#pragma weak TIM4_IRQHandler = Default_Handler                                      
#pragma weak I2C1_EV_IRQHandler = Default_Handler                                          
#pragma weak I2C1_ER_IRQHandler = Default_Handler                                          
#pragma weak I2C2_EV_IRQHandler = Default_Handler                                          
#pragma weak I2C2_ER_IRQHandler = Default_Handler                                            
#pragma weak SPI1_IRQHandler = Default_Handler                                      
#pragma weak SPI2_IRQHandler = Default_Handler                                      
#pragma weak USART1_IRQHandler = Default_Handler                                    
#pragma weak USART2_IRQHandler = Default_Handler                                    
#pragma weak USART3_IRQHandler = Default_Handler                                    
#pragma weak EXTI15_10_IRQHandler = Default_Handler                                        
#pragma weak RTC_Alarm_IRQHandler = Default_Handler                               
#pragma weak OTG_FS_WKUP_IRQHandler = Default_Handler                                   
#pragma weak TIM8_BRK_TIM12_IRQHandler = Default_Handler                  
#pragma weak TIM8_UP_TIM13_IRQHandler = Default_Handler                   
#pragma weak TIM8_TRG_COM_TIM14_IRQHandler = Default_Handler     
#pragma weak TIM8_CC_IRQHandler = Default_Handler                                          
#pragma weak DMA1_Stream7_IRQHandler = Default_Handler                                     
#pragma weak FSMC_IRQHandler = Default_Handler                                      
#pragma weak SDIO_IRQHandler = Default_Handler                                      
#pragma weak TIM5_IRQHandler = Default_Handler                                      
#pragma weak SPI3_IRQHandler = Default_Handler                                      
#pragma weak UART4_IRQHandler = Default_Handler                                     
#pragma weak UART5_IRQHandler = Default_Handler                                     
#pragma weak TIM6_DAC_IRQHandler = Default_Handler                                  
#pragma weak TIM7_IRQHandler = Default_Handler                   
#pragma weak DMA2_Stream0_IRQHandler = Default_Handler                              
#pragma weak DMA2_Stream1_IRQHandler = Default_Handler                              
#pragma weak DMA2_Stream2_IRQHandler = Default_Handler                              
#pragma weak DMA2_Stream3_IRQHandler = Default_Handler                              
#pragma weak DMA2_Stream4_IRQHandler = Default_Handler                              
#pragma weak ETH_IRQHandler = Default_Handler                                       
#pragma weak ETH_WKUP_IRQHandler = Default_Handler                                    
#pragma weak CAN2_TX_IRQHandler = Default_Handler                                          
#pragma weak CAN2_RX0_IRQHandler = Default_Handler                                         
#pragma weak CAN2_RX1_IRQHandler = Default_Handler                                         
#pragma weak CAN2_SCE_IRQHandler = Default_Handler                                         
#pragma weak OTG_FS_IRQHandler = Default_Handler                                    
#pragma weak DMA2_Stream5_IRQHandler = Default_Handler                              
#pragma weak DMA2_Stream6_IRQHandler = Default_Handler                              
#pragma weak DMA2_Stream7_IRQHandler = Default_Handler                              
#pragma weak USART6_IRQHandler = Default_Handler                                     
#pragma weak I2C3_EV_IRQHandler = Default_Handler                                          
#pragma weak I2C3_ER_IRQHandler = Default_Handler                                          
#pragma weak OTG_HS_EP1_OUT_IRQHandler = Default_Handler                            
#pragma weak OTG_HS_EP1_IN_IRQHandler = Default_Handler                             
#pragma weak OTG_HS_WKUP_IRQHandler = Default_Handler                                     
#pragma weak OTG_HS_IRQHandler = Default_Handler                                    
#pragma weak DCMI_IRQHandler = Default_Handler                                      
#pragma weak CRYP_IRQHandler = Default_Handler                                      
#pragma weak HASH_RNG_IRQHandler = Default_Handler               
#pragma weak FPU_IRQHandler = Default_Handler                    
