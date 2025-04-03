#include <stdint.h>

/* Symbols from the linker script */
extern uint32_t _data_loadaddr;
extern uint32_t _sdata;
extern uint32_t _edata;
extern uint32_t _sbss;
extern uint32_t _ebss;
extern uint32_t _sstack;
extern uint32_t _estack;

/* Function prototypes */
int main(void);
void Default_Handler(void);
void Reset_Handler(void);
void NMI_Handler(void) __attribute__((weak, alias("Default_Handler")));
void HardFault_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SVC_Handler(void) __attribute__((weak, alias("Default_Handler")));
void PendSV_Handler(void) __attribute__((weak, alias("Default_Handler")));
void SysTick_Handler(void) __attribute__((weak, alias("Default_Handler")));
void WWDG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void PVD_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RTC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void FLASH_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void RCC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI0_1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI2_3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void EXTI4_15_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void UCPD1_UCPD2_USB_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel2_3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void DMA1_Channel4_7_DMAMUX_DMA2_Channel1_5_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC_COMP_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_4_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM6_DAC_LPTIM1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM7_LPTIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM14_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM15_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM16_FDCAN_IT0_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM17_FDCAN_IT1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_I2C3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_SPI3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_LPUART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART3_6_LPUART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CEC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void AES_RNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* Vector table */
#define VECTOR_TABLE_SIZE (48U)
__attribute__((used, section(".vector_table")))
const uintptr_t vector_table[VECTOR_TABLE_SIZE] = {
    (uintptr_t)&_estack,                                        /* Initial stack pointer */
    (uintptr_t)Reset_Handler,                                   /* Reset handler */
    (uintptr_t)NMI_Handler,                                     /* NMI handler */
    (uintptr_t)HardFault_Handler,                               /* Hard fault handler */
    (uintptr_t)0, 0, 0, 0, 0, 0, 0,                             /* Reserved */
    (uintptr_t)SVC_Handler,                                     /* SVCall handler */
    (uintptr_t)0, 0,                                            /* Reserved */
    (uintptr_t)PendSV_Handler,                                  /* PendSV handler */
    (uintptr_t)SysTick_Handler,                                 /* SysTick handler */
    (uintptr_t)WWDG_IRQHandler,                                 /* Window Watchdog */
    (uintptr_t)PVD_IRQHandler,                                  /* PVD */
    (uintptr_t)RTC_IRQHandler,                                  /* RTC */
    (uintptr_t)FLASH_IRQHandler,                                /* Flash */
    (uintptr_t)RCC_IRQHandler,                                  /* RCC */
    (uintptr_t)EXTI0_1_IRQHandler,                              /* EXTI Line 0 and 1 */
    (uintptr_t)EXTI2_3_IRQHandler,                              /* EXTI Line 2 and 3 */
    (uintptr_t)EXTI4_15_IRQHandler,                             /* EXTI Line 4 to 15 */
    (uintptr_t)UCPD1_UCPD2_USB_IRQHandler,                      /* UCPD1 / UCPD2 / USB */
    (uintptr_t)DMA1_Channel1_IRQHandler,                        /* DMA1 Channel 1 */
    (uintptr_t)DMA1_Channel2_3_IRQHandler,                      /* DMA1 Channel 2 and 3 */
    (uintptr_t)DMA1_Channel4_7_DMAMUX_DMA2_Channel1_5_IRQHandler,/* DMA1 Channel 4-7, DMAMUX */
    (uintptr_t)ADC_COMP_IRQHandler,                             /* ADC1 */
    (uintptr_t)TIM1_BRK_UP_TRG_COM_IRQHandler,                  /* TIM1 Break, Update, Trigger, Commutation */
    (uintptr_t)TIM1_CC_IRQHandler,                              /* TIM1 Capture Compare */
    (uintptr_t)TIM2_IRQHandler,                                 /* TIM2 */
    (uintptr_t)TIM3_4_IRQHandler,                               /* TIM3 */
    (uintptr_t)TIM6_DAC_LPTIM1_IRQHandler,                      /* TIM6 */
    (uintptr_t)TIM7_LPTIM2_IRQHandler,                          /* TIM7 */
    (uintptr_t)TIM14_IRQHandler,                                /* TIM14 */
    (uintptr_t)TIM15_IRQHandler,                                /* TIM15 */
    (uintptr_t)TIM16_FDCAN_IT0_IRQHandler,                      /* TIM16 */
    (uintptr_t)TIM17_FDCAN_IT1_IRQHandler,                      /* TIM17 */
    (uintptr_t)I2C1_IRQHandler,                                 /* I2C1 */
    (uintptr_t)I2C2_I2C3_IRQHandler,                            /* I2C2 */
    (uintptr_t)SPI1_IRQHandler,                                 /* SPI1 */
    (uintptr_t)SPI2_SPI3_IRQHandler,                            /* SPI2 */
    (uintptr_t)USART1_IRQHandler,                               /* USART1 */
    (uintptr_t)USART2_LPUART2_IRQHandler,                       /* USART2 */
    (uintptr_t)USART3_6_LPUART1_IRQHandler,                     /* LPUART1 */
    (uintptr_t)CEC_IRQHandler,                                  /* LPTIM1 */
    (uintptr_t)AES_RNG_IRQHandler,                              /* LPTIM2 */
};

/* Reset handler */
void Reset_Handler(void) {

    /* Copy .data section from Flash to RAM */
    uint32_t *src = &_data_loadaddr;
    uint32_t *dst = &_sdata;
    while (dst < &_edata) {
        *dst++ = *src++;
    }

    /* Zero initialize the .bss section */
    dst = &_sbss;
    while (dst < &_ebss) {
        *dst++ = 0;
    }

    /* Call main */
    main();

    /* Infinite loop if main returns */
    while (1);
}

/* Default handler */
void Default_Handler(void) {
    while (1);
}
