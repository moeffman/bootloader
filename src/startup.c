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
void DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void ADC1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_BRK_UP_TRG_COM_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM1_CC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM3_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM6_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM7_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM14_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM15_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM16_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void TIM17_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void I2C2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void SPI2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART2_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void USART3_6_LPUART1_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void CEC_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));
void AES_RNG_IRQHandler(void) __attribute__((weak, alias("Default_Handler")));

/* Vector table */
#define VECTOR_TABLE_SIZE (48U)
__attribute__((used, section(".vector_table")))
static void (*vector_table[VECTOR_TABLE_SIZE])(void) = {
    (void (*)(void))(&_estack),          /* Initial stack pointer */
    Reset_Handler,                      /* Reset handler */
    NMI_Handler,                        /* NMI handler */
    HardFault_Handler,                  /* Hard fault handler */
    0, 0, 0, 0, 0, 0, 0,                /* Reserved */
    SVC_Handler,                        /* SVCall handler */
    0, 0,                               /* Reserved */
    PendSV_Handler,                     /* PendSV handler */
    SysTick_Handler,                    /* SysTick handler */
    WWDG_IRQHandler,                    /* Window Watchdog */
    PVD_IRQHandler,                     /* PVD */
    RTC_IRQHandler,                     /* RTC */
    FLASH_IRQHandler,                   /* Flash */
    RCC_IRQHandler,                     /* RCC */
    EXTI0_1_IRQHandler,                 /* EXTI Line 0 and 1 */
    EXTI2_3_IRQHandler,                 /* EXTI Line 2 and 3 */
    EXTI4_15_IRQHandler,                /* EXTI Line 4 to 15 */
    UCPD1_UCPD2_USB_IRQHandler,         /* UCPD1 / UCPD2 / USB */
    DMA1_Channel1_IRQHandler,           /* DMA1 Channel 1 */
    DMA1_Channel2_3_IRQHandler,         /* DMA1 Channel 2 and 3 */
    DMA1_Ch4_7_DMAMUX1_OVR_IRQHandler,  /* DMA1 Channel 4-7, DMAMUX */
    ADC1_IRQHandler,                    /* ADC1 */
    TIM1_BRK_UP_TRG_COM_IRQHandler,     /* TIM1 Break, Update, Trigger, Commutation */
    TIM1_CC_IRQHandler,                 /* TIM1 Capture Compare */
    TIM2_IRQHandler,                    /* TIM2 */
    TIM3_IRQHandler,                    /* TIM3 */
    TIM6_IRQHandler,                    /* TIM6 */
    TIM7_IRQHandler,                    /* TIM7 */
    TIM14_IRQHandler,                   /* TIM14 */
    TIM15_IRQHandler,                   /* TIM15 */
    TIM16_IRQHandler,                   /* TIM16 */
    TIM17_IRQHandler,                   /* TIM17 */
    I2C1_IRQHandler,                    /* I2C1 */
    I2C2_IRQHandler,                    /* I2C2 */
    SPI1_IRQHandler,                    /* SPI1 */
    SPI2_IRQHandler,                    /* SPI2 */
    USART1_IRQHandler,                  /* USART1 */
    USART2_IRQHandler,                  /* USART2 */
    USART3_6_LPUART1_IRQHandler,        /* LPUART1 */
    CEC_IRQHandler,                     /* LPTIM1 */
    AES_RNG_IRQHandler,                 /* LPTIM2 */
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
