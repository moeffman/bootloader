/*
 * bootloader.h - USART bootloader for STM32G0
 * MIT License — Copyright (c) 2025 Oskar Arnudd
 * See LICENSE file for details.
 */

#ifndef APP_H
#define APP_H

#include <stdint.h>

#define FLASH_BASE      (0x40022000)
#define FLASH_KEYR      (*(volatile uint32_t*)(FLASH_BASE + 0x08))
#define FLASH_SR        (*(volatile uint32_t*)(FLASH_BASE + 0x10))
#define FLASH_CR        (*(volatile uint32_t*)(FLASH_BASE + 0x14))

#define RCC_BASE        (0x40021000)
#define RCC_IOPRSTR     (*(volatile uint32_t*)(RCC_BASE + 0x24))
#define RCC_AHBRSTR     (*(volatile uint32_t*)(RCC_BASE + 0x28))
#define RCC_APBRSTR1    (*(volatile uint32_t*)(RCC_BASE + 0x2C))
#define RCC_APBRSTR2    (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define RCC_IOPENR      (*(volatile uint32_t*)(RCC_BASE + 0x34))
#define RCC_AHBENR      (*(volatile uint32_t*)(RCC_BASE + 0x38))
#define RCC_APBENR1     (*(volatile uint32_t*)(RCC_BASE + 0x3C))
#define RCC_APBENR2     (*(volatile uint32_t*)(RCC_BASE + 0x40))

#define GPIOA_BASE      (0x50000000)
#define GPIOA_MODER     (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_OSPEEDR   (*(volatile uint32_t*)(GPIOA_BASE + 0x08))
#define GPIOA_AFRL      (*(volatile uint32_t*)(GPIOA_BASE + 0x20))

#define SYSCFG_BASE     (0x40010000)
#define SYSCFG_CFGR1    (*(volatile uint32_t*)(SYSCFG_BASE + 0x00))

#define NVIC_BASE       (0xE000E100)
#define NVIC_ISER       (*(volatile uint32_t*)(NVIC_BASE + 0x00))
#define NVIC_ICER       (*(volatile uint32_t*)(NVIC_BASE + 0x80))

#define CRC_BASE        (0x40023000)
#define CRC_DR          (*(volatile uint32_t*)(CRC_BASE + 0x00))
#define CRC_CR          (*(volatile uint32_t*)(CRC_BASE + 0x08))

#define TIM14_BASE      (0x40002000)
#define TIM14_CR1       (*(volatile uint16_t*)(TIM14_BASE + 0x00))
#define TIM14_DIER      (*(volatile uint16_t*)(TIM14_BASE + 0x0C))
#define TIM14_SR        (*(volatile uint16_t*)(TIM14_BASE + 0x10))
#define TIM14_EGR       (*(volatile uint16_t*)(TIM14_BASE + 0x14))
#define TIM14_PSC       (*(volatile uint16_t*)(TIM14_BASE + 0x28))
#define TIM14_ARR       (*(volatile uint16_t*)(TIM14_BASE + 0x2C))

#define USART2_BASE     (0x40004400)
#define USART2_CR1      (*(volatile uint32_t*)(USART2_BASE + 0x00))
#define USART2_BRR      (*(volatile uint32_t*)(USART2_BASE + 0x0C))
#define USART2_RQR      (*(volatile uint32_t*)(USART2_BASE + 0x18))
#define USART2_ISR      (*(volatile uint32_t*)(USART2_BASE + 0x1C))
#define USART2_RDR      (*(volatile uint32_t*)(USART2_BASE + 0x24))
#define USART2_TDR      (*(volatile uint32_t*)(USART2_BASE + 0x28))

#define BIT0    (1 << 0)
#define BIT1    (1 << 1)
#define BIT2    (1 << 2)
#define BIT3    (1 << 3)
#define BIT4    (1 << 4)
#define BIT5    (1 << 5)
#define BIT6    (1 << 6)
#define BIT7    (1 << 7)
#define BIT8    (1 << 8)
#define BIT9    (1 << 9)
#define BIT10   (1 << 10)
#define BIT11   (1 << 11)
#define BIT12   (1 << 12)
#define BIT13   (1 << 13)
#define BIT14   (1 << 14)
#define BIT15   (1 << 15)
#define BIT16   (1 << 16)
#define BIT17   (1 << 17)
#define BIT18   (1 << 18)
#define BIT19   (1 << 19)
#define BIT20   (1 << 20)
#define BIT21   (1 << 21)
#define BIT22   (1 << 22)
#define BIT23   (1 << 23)
#define BIT24   (1 << 24)
#define BIT25   (1 << 25)
#define BIT26   (1 << 26)
#define BIT27   (1 << 27)
#define BIT28   (1 << 28)
#define BIT29   (1 << 29)
#define BIT30   (1 << 30)
#define BIT31   (1 << 31)

#define APP_MEMORY_START 0x08004000
#define APP_MEMORY_END 0x08020000
#define VECTOR_TABLE_SIZE 0xC0
#define CHUNK_SIZE 256
#define FIRMWARE_MAX_SIZE 20000
#define BOOTLOADER_WAIT_TIME 3

typedef enum{
    BLSTATE_IDLE,
    BLSTATE_UPDATING,
    BLSTATE_FLASHRW,
    BLSTATE_JUMPTOAPP,
} bootloader_state_t;

typedef enum{
    USTATE_HANDSHAKE,
    USTATE_SIZE,
    USTATE_FILE,
    USTATE_CRC,
} update_state_t;

int main(void);

#endif
