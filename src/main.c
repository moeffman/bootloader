/*
 * bootloader.c - USART bootloader for STM32G0
 * MIT License — Copyright (c) 2025 Oskar Arnudd
 * See LICENSE file for details.
 */

#include "main.h"
#include <stdint.h>
#include <stdbool.h>

update_state_t update_state = USTATE_HANDSHAKE;
bootloader_state_t bootloader_state = BLSTATE_IDLE;
static uint8_t firmware_update_start_timer = BOOTLOADER_WAIT_TIME;
static uint8_t firmware_data[FIRMWARE_MAX_SIZE];
static uint32_t firmware_data_index = 0;
static uint32_t firmware_size = 0;
static uint8_t firmware_size_index = 0;
static uint32_t CRC = 0;
static uint8_t CRC_index = 0;

static uint32_t calculate_checksum();
static void flash_erase(void);
static void flash_page_erase(uint32_t address);
static void flash_region_erase(uint32_t start, uint32_t end);
static void flash_unlock(void);
static void flash_lock(void);
static void flash_write_64(uint32_t address, uint8_t* buffer, uint32_t buffer_length);
static void firmware_update();
static void jump_to_application(void);

static void init_peripherals(void);
static void deinit_peripherals(void);

static void usart_send_string(char* string);
static void usart_send_byte(uint8_t byte);
static void usart_send_number(uint32_t number);

int main(void)
{
    init_peripherals();

    while (1) {
        if(bootloader_state == BLSTATE_JUMPTOAPP){
            jump_to_application();
        }

        if(bootloader_state == BLSTATE_FLASHRW){
            firmware_update();

            bootloader_state++;
        }

        if(bootloader_state == BLSTATE_UPDATING){
            firmware_update_start_timer = BOOTLOADER_WAIT_TIME;
        }
    }
}

static void init_peripherals(void)
{
    // GPIO init
    // RCC enable GPIOA
    RCC_IOPENR |= BIT0;

    // GPIOA_MODER AF for pin 2 and 3
    GPIOA_MODER &= ~BIT4;
    GPIOA_MODER |= BIT5;
    GPIOA_MODER &= ~BIT6;
    GPIOA_MODER |= BIT7;

    // GPIO_SPEEDR high for pin 2 and 3
    GPIOA_OSPEEDR &= ~BIT4;
    GPIOA_OSPEEDR |= BIT5;
    GPIOA_OSPEEDR &= ~BIT6;
    GPIOA_OSPEEDR |= BIT7;

    // GPIO_AFRL AF1 for pin 2 and 3
    GPIOA_AFRL &= ~(0x3 << 9);
    GPIOA_AFRL |= BIT8;
    GPIOA_AFRL &= ~(0x3 << 13);
    GPIOA_AFRL |= BIT12;

    // USART2 init
    // RCC enable USART2
    RCC_APBENR1 |= BIT17;

    // USART Disable
    USART2_CR1 &= ~BIT0;

    // Turning TE on
    USART2_CR1 |= BIT3;

    // Turning RXFNEIE on
    USART2_CR1 |= BIT5;

    // USART receiver enable
    USART2_CR1 |= BIT2;
    // BRR USARTDIV = FREQ/BAUDRATE
    // 16Mhz / 115200 = 139 = 0x8B
    USART2_BRR = 0x8B;

    // USART Enable
    USART2_CR1 |= BIT0;

    // Enable USART2 in NVIC
    NVIC_ISER |= (BIT28);

    // TIM14 init
    // RCC enable TIM14
    RCC_APBENR2 |= BIT15;

    // Update interrupt enabled
    TIM14_DIER |= BIT0;

    // Re-initialize counter
    TIM14_EGR |= BIT0;

    // Prescaler 15999Hz => Timer clock : (16MHz / (15999Hz + 1)) = 1000Hz
    TIM14_PSC = 0x3E7F;

    // Auto-reload value set to 999, making it fire every second
    TIM14_ARR = 0x3E7;

    // Counter enabled
    TIM14_CR1 |= BIT0;

    // Enable TIM14 in NVIC
    NVIC_ISER |= (BIT19);
}

static uint32_t calculate_checksum()
{
    // RCC CRC enable
    RCC_AHBENR |= BIT12;

    // Reset CRC
    CRC_CR |= BIT0;
    // REV_OUT 1 - Bit-reversed output format
    CRC_CR |= BIT7;
    // REV_IN 11 - Bit reversal done by word
    CRC_CR |= (0x3 << 5);

    for(int i = 0; i < firmware_size; i+=4){
        uint32_t word = 0;

        word |= firmware_data[i];
        word |= (i + 1 < firmware_size) ? firmware_data[i + 1] << 8 : 0xFF << 8;
        word |= (i + 2 < firmware_size) ? firmware_data[i + 2] << 16 : 0xFF << 16;
        word |= (i + 3 < firmware_size) ? firmware_data[i + 3] << 24 : 0xFF << 24;

        // CRC data
        CRC_DR = word;
    }
    return CRC_DR ^ 0xFFFFFFFF;
}

static void flash_unlock()
{
    // Unlock sequence
    FLASH_KEYR = 0x45670123;
    FLASH_KEYR = 0xCDEF89AB;
}

static void flash_lock()
{
    // Setting CR_LOCK
    FLASH_CR |= BIT31;
}

static void flash_page_erase(uint32_t address)
{
    // Ensuring SR_BSY1 flag is 0
    while(FLASH_SR & BIT16);

    // Setting CR_PER
    FLASH_CR |= BIT1;
    // Clearing CR_PNB
    FLASH_CR &= ~(0x3FF << 3);
    // Setting CR_PNB
    FLASH_CR |= (((address - 0x08000000) / 0x800) << 3);

    // Setting CR_STRT
    FLASH_CR |= BIT16;

    // Ensuring SR_BSY1 flag is 0
    while(FLASH_SR & BIT16);
    // Resetting CR_PER
    FLASH_CR &= ~BIT1;
}

static void flash_region_erase(uint32_t start, uint32_t end)
{
    for(uint32_t i = start; i < end; i += 0x800){
        flash_page_erase(i);
    }
}

static void flash_erase()
{
    flash_region_erase(APP_MEMORY_START, APP_MEMORY_END);
}

static void flash_write_64(uint32_t address, uint8_t* buffer, uint32_t buffer_length)
{
    uint32_t full_chunks = buffer_length / 8;
    uint32_t remaining = buffer_length % 8;

    // Ensuring SR_BSY1 flag is 0
    while(FLASH_SR & BIT16);

    // Clearing error flags
    FLASH_CR |= BIT3 | BIT4 | BIT5 | BIT6 | BIT7;

    // Setting CR_PG
    FLASH_CR |= BIT0;

    // Writing data
    for(uint32_t i = 0; i < full_chunks; i++, address+=8){
        uint32_t word1 = buffer[i * 8] |
                        (buffer[i * 8 + 1] << 8)  |
                        (buffer[i * 8 + 2] << 16) |
                        (buffer[i * 8 + 3] << 24);

        uint32_t word2 = buffer[i * 8 + 4] |
                        (buffer[i * 8 + 5] << 8)  |
                        (buffer[i * 8 + 6] << 16) |
                        (buffer[i * 8 + 7] << 24);

        *(volatile uint32_t*)(address) =  word1;
        *(volatile uint32_t*)(address+4) =  word2;

        // Ensuring SR_BSY1 flag is 0
        while(FLASH_SR & BIT16);
    }

    // Writing any remaining non-full chunk
    if(remaining > 0){
        uint8_t temp[8] = {0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF,0xFF};

        for(uint32_t i = 0; i < remaining; i++){
            temp[i] = buffer[full_chunks * 8 + i];
        }

        uint32_t word1 = temp[0] | temp[1] << 8 | temp[2] << 16 | temp[3] << 24;
        uint32_t word2 = temp[4] | temp[5] << 8 | temp[6] << 16 | temp[7] << 24;

        *(volatile uint32_t*)(address) =  word1;
        *(volatile uint32_t*)(address+4) =  word2;

        // Ensuring SR_BSY1 flag is 0
        while(FLASH_SR & BIT16);
    }

    // Resetting CR_PG
    FLASH_CR &= ~BIT0;
}

static void usart_send_string(char* string)
{
    for(uint32_t i = 0; string[i]; i++){
	usart_send_byte(string[i]);
    }
}

static void usart_send_byte(uint8_t byte)
{
    // Wait for USART2 TDR
    while(!(USART2_ISR & BIT7));

    // Respecting TE minimum period
    while(!(USART2_ISR & BIT21));

    // Writing data to TDR
    USART2_TDR = byte;

    // Waiting for Transmission Complete Flag
    while(!(USART2_ISR & BIT6));
}

static void usart_send_number(uint32_t number)
{
    char buffer[11];  // Max 10 digits + null terminator
    int index = 10;  // Start at last position
    buffer[index] = '\0';  // Null-terminate string

    if (number == 0) {
        usart_send_byte('0');
        return;
    }

    while (number > 0 && index > 0) {
        index--;
        buffer[index] = (number % 10) + '0';
        number /= 10;
    }

    usart_send_string(&buffer[index]);  // Send only valid part
}

static void firmware_update()
{
        usart_send_string("\r\n");
        usart_send_string("Checksum verified!\r\n");

        // Erase old firmware
        flash_unlock();
        usart_send_string("Rewriting flash..\r\n");
        flash_erase();

        // Write new firmware
        flash_write_64(APP_MEMORY_START, firmware_data, firmware_size);
        usart_send_string("Flash written!\r\n");
        flash_lock();
        usart_send_string("Firmware update completed!\r\n");

        usart_send_string("Starting application..\r\n");
}

static void jump_to_application(void)
{
    // Disable IRQs
    __asm volatile ("cpsid i");

    uint32_t* app_vector = (uint32_t*)(APP_MEMORY_START);
    uint32_t* sram_vector = (uint32_t*)(0x20000000);

    // RCC enable SYSCFG
    RCC_APBENR2 |= BIT0;

    // SYSCFG_CFGR1 MEM_MODE
    SYSCFG_CFGR1 &= ~0x3;
    SYSCFG_CFGR1 |= 0x3;

    for(uint32_t i = 0; i < VECTOR_TABLE_SIZE / 4; i++){
        sram_vector[i] = app_vector[i];
    }

    for (volatile int i = 0; i < 10000; i++);

    uint32_t app_stack = app_vector[0];
    uint32_t app_reset = app_vector[1];

    if (app_reset < APP_MEMORY_START || app_reset > APP_MEMORY_END){
        usart_send_string("Invalid reset location. Aborting\r\n");
        return;
    }

    deinit_peripherals();

    // Set MSP to applications stack
    __asm volatile ("msr msp, %0" :: "r" (app_stack));

    // Jump to application reset handler
    void (*app_reset_handler)(void) = (void (*)(void)) app_reset;
    app_reset_handler();
}

static void deinit_peripherals(void)
{
    // GPIOA_MODER
    GPIOA_MODER = 0xEBFFFFFF;
    // GPIOA_OSPEEDR
    GPIOA_OSPEEDR = 0x0C000000;
    // GPIOA_AFRL
    GPIOA_AFRL = 0;

    // USART2_CR1
    USART2_CR1 = 0;
    // USART2_BRR
    USART2_BRR = 0;

    // CRC_CR
    CRC_CR = 0;

    // TIM_14 CR1
    TIM14_CR1 = 0;
    TIM14_DIER = 0;
    TIM14_EGR = 0;
    TIM14_PSC = 0;
    TIM14_ARR = 0xFFFF;

    // RCC_IOPRSTR
    // GPIOA
    RCC_IOPRSTR |= BIT0;
    RCC_IOPRSTR &= ~BIT0;
    RCC_IOPENR &= ~BIT0;

    // RCC_AHBRSTR
    // CRC
    RCC_AHBRSTR|= BIT12;
    RCC_AHBRSTR &= ~BIT12;
    RCC_AHBENR &= ~BIT12;
    // FLASH
    RCC_AHBRSTR|= BIT8;
    RCC_AHBRSTR &= ~BIT8;
    RCC_AHBENR &= ~BIT8;

    // RCC_APBRSTR1
    // USART2
    RCC_APBRSTR1 |= BIT17;
    RCC_APBRSTR1 &= ~BIT17;
    RCC_APBENR1 &= ~BIT17;
    // RCC_APBRSTR2
    // SYSCFG
    RCC_APBRSTR2 |= BIT0;
    RCC_APBRSTR2 &= ~BIT0;
    RCC_APBENR2 &= ~BIT0;
    // TIM14
    RCC_APBRSTR2 |= BIT15;
    RCC_APBRSTR2 &= ~BIT15;
    RCC_APBENR2 &= ~BIT15;

    // NVIC
    // TIM_14
    NVIC_ICER |= BIT19;
    // USART2
    NVIC_ICER |= BIT28;
}

void USART2_IRQHandler(void)
{
    if(USART2_ISR & BIT5){

        uint8_t data = USART2_RDR;

        if(bootloader_state == BLSTATE_IDLE){
            if(data == 'U'){
                bootloader_state = BLSTATE_UPDATING;
            }
        }

        if(bootloader_state == BLSTATE_UPDATING){
            switch (update_state) {
                case USTATE_HANDSHAKE:
                    if(data == 'U'){
                        usart_send_string("ACK");
                    }else if(data == 'A'){
                        update_state++;
                        usart_send_string("ACK");
                    }
                break;

                case USTATE_SIZE:
                    firmware_size |= data << (24 - (8 * firmware_size_index++));

                    if(firmware_size_index > 3){
                        update_state++;

                        usart_send_number(firmware_size);
                    }
                break;

                case USTATE_FILE:
                    firmware_data[firmware_data_index++] = data;

                    if(firmware_data_index >= firmware_size){
                        update_state++;
                    }
                break;

                case USTATE_CRC:
                    CRC |= data << (24 - (8 * CRC_index++));

                    if(CRC_index > 3){
                        if(CRC == calculate_checksum()){
                            bootloader_state = BLSTATE_FLASHRW;

                            usart_send_number(CRC);
                        }
                    }
                break;
            }
        }
    }
}

void TIM14_IRQHandler(void)
{
    // Clearing update interrupt flag
    TIM14_SR &= ~BIT0;

    if(bootloader_state == BLSTATE_IDLE){
        if(--firmware_update_start_timer <= 0){
            bootloader_state = BLSTATE_JUMPTOAPP;
        }
    }
}
