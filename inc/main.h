/*
 * bootloader.h - USART bootloader for STM32G0
 * MIT License — Copyright (c) 2025 Oskar Arnudd
 * See LICENSE file for details.
 */

#ifndef BOOTLOADER_H
#define BOOTLOADER_H

#include <stdint.h>
#include <stdbool.h>

#define FLASH_BASE                  (0x40022000)
#define FLASH_KEYR                  (*(volatile uint32_t*)(FLASH_BASE + 0x08))
#define FLASH_SR                    (*(volatile uint32_t*)(FLASH_BASE + 0x10))
#define FLASH_CR                    (*(volatile uint32_t*)(FLASH_BASE + 0x14))
#define FLASH_SR_PROGERR            (1 << 3)
#define FLASH_SR_WRPERR             (1 << 4)
#define FLASH_SR_PGAERR             (1 << 5)
#define FLASH_SR_SIZERR             (1 << 6)
#define FLASH_SR_PGSERR             (1 << 7)
#define FLASH_SR_BSY1               (1 << 16)
#define FLASH_CR_PG                 (1 << 0)
#define FLASH_CR_PER                (1 << 1)
#define FLASH_CR_PNB_POS            (3)
#define FLASH_CR_PNB(addr)          (((addr) & 0x3FF) << FLASH_CR_PNB_POS)
#define FLASH_CR_PNB_MSK            FLASH_CR_PNB(0x3FF)
#define FLASH_CR_STRT               (1 << 16)
#define FLASH_CR_LOCK               (1 << 31)

#define RCC_BASE                    (0x40021000)
#define RCC_IOPRSTR                 (*(volatile uint32_t*)(RCC_BASE + 0x24))
#define RCC_AHBRSTR                 (*(volatile uint32_t*)(RCC_BASE + 0x28))
#define RCC_APBRSTR1                (*(volatile uint32_t*)(RCC_BASE + 0x2C))
#define RCC_APBRSTR2                (*(volatile uint32_t*)(RCC_BASE + 0x30))
#define RCC_IOPENR                  (*(volatile uint32_t*)(RCC_BASE + 0x34))
#define RCC_AHBENR                  (*(volatile uint32_t*)(RCC_BASE + 0x38))
#define RCC_APBENR1                 (*(volatile uint32_t*)(RCC_BASE + 0x3C))
#define RCC_APBENR2                 (*(volatile uint32_t*)(RCC_BASE + 0x40))

#define RCC_GPIOA                   (1 << 0)
#define RCC_FLASH                   (1 << 8)
#define RCC_CRC                     (1 << 12)
#define RCC_SYSCFG                  (1 << 0)
#define RCC_TIM14                   (1 << 15)
#define RCC_USART2                  (1 << 17)

#define GPIOA_BASE                  (0x50000000)
#define GPIOA_MODER                 (*(volatile uint32_t*)(GPIOA_BASE + 0x00))
#define GPIOA_OTYPER                (*(volatile uint32_t*)(GPIOA_BASE + 0x04))
#define GPIOA_OSPEEDR               (*(volatile uint32_t*)(GPIOA_BASE + 0x08))
#define GPIOA_ODR                   (*(volatile uint32_t*)(GPIOA_BASE + 0x14))
#define GPIOA_BSSR                  (*(volatile uint32_t*)(GPIOA_BASE + 0x18))
#define GPIOA_AFRL                  (*(volatile uint32_t*)(GPIOA_BASE + 0x20))

#define GPIO_MODER_OUTPUT           (0x1)
#define GPIO_MODER_AF               (0x2)
#define GPIO_MODER_POS(pin)         (pin * 2)
#define GPIO_MODER_MSK(pin)         (0x3 << GPIO_MODER_POS(pin))
#define GPIO_MODER(pin, mode)       (mode & 0x3) << GPIO_MODER_POS(pin);
#define GPIO_AF1                    (0x1)
#define GPIO_AFRL_POS(pin)          (pin * 4)
#define GPIO_AFRL_MSK(pin)          (0xF << GPIO_AFRL_POS(pin))
#define GPIO_AFRL(pin, mode)        (mode & 0xF) << GPIO_AFRL_POS(pin);

#define SYSCFG_BASE                 (0x40010000)
#define SYSCFG_CFGR1                (*(volatile uint32_t*)(SYSCFG_BASE + 0x00))
#define SYSCFG_CFGR1_MEM_MODE_POS   (0)
#define SYSCFG_CFGR1_MEM_MODE(n)    (((n) & 0x3) << SYSCFG_CFGR1_MEM_MODE_POS)
#define SYSCFG_CFGR1_MEM_MODE_MSK   SYSCFG_CFGR1_MEM_MODE(0x3)
#define SYSCFG_MEM_MODE_SRAM        (0x3)

#define NVIC_BASE                   (0xE000E100)
#define NVIC_ISER                   (*(volatile uint32_t*)(NVIC_BASE + 0x00))
#define NVIC_ICER                   (*(volatile uint32_t*)(NVIC_BASE + 0x80))

#define NVIC_TIM14                  (1 << 19)
#define NVIC_USART2                 (1 << 28)

#define CRC_BASE                    (0x40023000)
#define CRC_DR                      (*(volatile uint32_t*)(CRC_BASE + 0x00))
#define CRC_CR                      (*(volatile uint32_t*)(CRC_BASE + 0x08))
#define CRC_INIT                    (*(volatile uint32_t*)(CRC_BASE + 0x10))
#define CRC_CR_RESET                (1 << 0)
#define CRC_CR_REV_IN_POS           (5)
#define CRC_CR_REV_IN(n)            (((n) & 0x3) << CRC_CR_REV_IN_POS)
#define CRC_CR_REV_IN_MSK           CRC_CR_REV_IN(0x3)
#define CRC_CR_REV_OUT              (1 << 7)

#define TIM14_BASE                  (0x40002000)
#define TIM14_CR1                   (*(volatile uint32_t*)(TIM14_BASE + 0x00))
#define TIM14_DIER                  (*(volatile uint32_t*)(TIM14_BASE + 0x0C))
#define TIM14_SR                    (*(volatile uint32_t*)(TIM14_BASE + 0x10))
#define TIM14_EGR                   (*(volatile uint32_t*)(TIM14_BASE + 0x14))
#define TIM14_CNT                   (*(volatile uint32_t*)(TIM14_BASE + 0x24))
#define TIM14_PSC                   (*(volatile uint32_t*)(TIM14_BASE + 0x28))
#define TIM14_ARR                   (*(volatile uint32_t*)(TIM14_BASE + 0x2C))
#define TIM_CR1_CEN                 (1 << 0)
#define TIM_DIER_UIE                (1 << 0)
#define TIM_SR_UIF                  (1 << 0)
#define TIM_EGR_UG                  (1 << 0)

#define USART2_BASE                 (0x40004400)
#define USART2_CR1                  (*(volatile uint32_t*)(USART2_BASE + 0x00))
#define USART2_BRR                  (*(volatile uint32_t*)(USART2_BASE + 0x0C))
#define USART2_RQR                  (*(volatile uint32_t*)(USART2_BASE + 0x18))
#define USART2_ISR                  (*(volatile uint32_t*)(USART2_BASE + 0x1C))
#define USART2_RDR                  (*(volatile uint32_t*)(USART2_BASE + 0x24))
#define USART2_TDR                  (*(volatile uint32_t*)(USART2_BASE + 0x28))

#define USART_CR1_UE                (1 << 0)
#define USART_CR1_RE                (1 << 2)
#define USART_CR1_TE                (1 << 3)
#define USART_CR1_RXFNEIE           (1 << 5)
#define USART_ISR_RXFNE             (1 << 6)
#define USART_ISR_TC                (1 << 6)
#define USART_ISR_TXFNF             (1 << 7)
#define USART_ISR_TEACK             (1 << 21)

#define APP_MEMORY_START    0x08004000
#define APP_MEMORY_END      0x08020000
#define RAM_START           0x20000000
#define RAM_END             0x20009000
#define FLASH_PAGE_SIZE     0x800
#define VECTOR_TABLE_SIZE   0xC0

#define BR_115200 0x8B
#define BR_230400 0x45
#define BR_460800 0x23

// CONFIG
#define FIRMWARE_MAX_SIZE 70000
#define CHUNK_SIZE 256
#define PACKET_HEADER_SIZE 8
#define PACKET_DATA_SIZE 256
#define PACKET_TOTAL_SIZE (PACKET_HEADER_SIZE + PACKET_DATA_SIZE)
#define BOOTLOADER_WAIT_TIME_500MS   50
#define RETRANSMISSION_INTERVAL_10MS 9
#define TIMER_PRESCALAR_1000HZ       0x3E7F

// SEQUENCE MAPPINGS
#define SEQ_UPDATE          1
#define SEQ_HANDSHAKE       2
#define SEQ_HANDSHAKE_ACK   3
#define SEQ_SIZE            4
#define SEQ_SIZE_ACK        5
#define SEQ_DATA_START      100
#define SEQ_DATA_END        65533
#define SEQ_CRC             65534
#define SEQ_FINISHED        65535

typedef struct{
    uint16_t seq;       // Sequence number
    uint16_t len;       // Number of bytes to read from data
    uint32_t crc;       // Checksum
    uint8_t data[PACKET_DATA_SIZE];
} Packet_t;

typedef union{
    Packet_t packet;
    uint8_t bytes[PACKET_TOTAL_SIZE];
} BTP_t;

int main(void);

#endif
