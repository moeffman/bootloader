/*
 * bootloader.c - USART bootloader for STM32G0
 * MIT License — Copyright (c) 2025 Oskar Arnudd
 * See LICENSE file for details.
 */

#include "main.h"

static volatile uint8_t application_start_timer = BOOTLOADER_WAIT_TIME;
static volatile uint8_t data_buffer[264];
static volatile uint16_t data_buffer_index = 0;
static volatile bool retransmit_requested = false;
static volatile bool update_message_requested = false;
static volatile bool updating = false;

static void init_peripherals(void);
static void deinit_peripherals(void);

static void flash_unlock(void);
static void flash_lock(void);
static void flash_page_erase(uint32_t address);
static void flash_region_erase(uint32_t start, uint32_t end);
static void flash_erase(void);
static void flash_write_64(uint32_t address, const uint8_t* buffer, uint32_t buffer_length);

static void firmware_update(uint32_t firmware_size, const uint8_t* firmware_data);
static uint32_t calculate_checksum(uint32_t firmware_size, const uint8_t* firmware_data);
static void jump_to_application(void);

static inline bool packet_received(uint16_t len, bool length_extracted);
static inline void packet_get_length(Packet_t* packet, bool* length_extracted);
static void packet_encode(Packet_t* packet, uint16_t seq, uint16_t len, const uint8_t* data);
static bool packet_decode(Packet_t* packet);
static void process_handshake(BTP_t* incoming, BTP_t* outgoing);
static void process_size(BTP_t* incoming, BTP_t* outgoing, uint32_t* const firmware_size);
static void process_firmware_chunk(BTP_t* incoming, BTP_t* outgoing, uint8_t* const firmware_data);
static void process_crc(BTP_t* incoming, BTP_t* outgoing, uint32_t* const received_crc);
static inline void update_saved_btp(BTP_t* saved_btp, BTP_t* outgoing);
static inline void clear_incoming_btp(BTP_t* incoming);

static void usart_send_byte(uint8_t byte);
static void usart_send_btp(uint16_t data_len, const uint8_t* data);
static void usart_send_btp_string(const char* string);
static void usart_send_btp_number(uint32_t number);
static void usart_send_btp_update_request(void);
static void usart_send_btp_ack(Packet_t* packet, uint8_t* bytes, uint16_t seq);

static uint32_t string_length(const char* string);
static bool strings_match(const char* str1, const char* str2);

int main(void)
{
    BTP_t outgoing_btp = {0};
    Packet_t* outgoing_packet = &outgoing_btp.packet;
    uint8_t* outgoing_bytes = outgoing_btp.bytes;
    BTP_t incoming_btp = {0};
    Packet_t* incoming_packet = &incoming_btp.packet;
    BTP_t saved_btp = {0};

    static uint8_t firmware_data[FIRMWARE_MAX_SIZE];
    uint32_t firmware_size = 0;
    uint32_t received_crc = 0;
    bool length_extracted = false;

    init_peripherals();

    while (1) {
        if(application_start_timer <= 0){
            jump_to_application();
        }

        // Parsing packet header when first 8 bytes are received
        packet_get_length(incoming_packet, &length_extracted);

        // Checking if we received a full packet (data >= len)
        if(packet_received(incoming_packet->len, length_extracted)){
            length_extracted = false;
            application_start_timer = BOOTLOADER_WAIT_TIME;

            // Checking if packet is valid
            if(!packet_decode(incoming_packet)){
                usart_send_btp_string("\r\nCRC mismatch on message with sequence number ");
                usart_send_btp_number(incoming_packet->seq);
                usart_send_btp_string("\r\nRetrying..\r\n");
                continue;
            }
            // Delay re-transmissions
            TIM14_CNT = 0;

            switch(incoming_packet->seq){
                case SEQ_HANDSHAKE:
                    updating = true;
                    process_handshake(&incoming_btp, &outgoing_btp);
                    break;
                case SEQ_SIZE:
                    process_size(&incoming_btp, &outgoing_btp, &firmware_size);

                    if(firmware_size > FIRMWARE_MAX_SIZE){
                        usart_send_btp_string("Error: Firmware too large, exiting\r\n");
                        usart_send_btp_ack(outgoing_packet, outgoing_bytes, SEQ_FINISHED);
                        updating = false;
                    }
                    break;
                case SEQ_CRC:
                    process_crc(&incoming_btp, &outgoing_btp, &received_crc);

                    if(received_crc == calculate_checksum(firmware_size, firmware_data)){
                        firmware_update(firmware_size, firmware_data);
                    }else{
                        usart_send_btp_string("Error: Firmware CRC did not match, exiting\r\n");
                        usart_send_btp_ack(outgoing_packet, outgoing_bytes, SEQ_FINISHED);
                        updating = false;
                    }
                    break;
            }

            if(incoming_packet->seq >= SEQ_DATA_START && incoming_packet->seq <= SEQ_DATA_END){
                process_firmware_chunk(&incoming_btp, &outgoing_btp, firmware_data);
            }
            update_saved_btp(&saved_btp, &outgoing_btp);
            clear_incoming_btp(&incoming_btp);
        }
        if(retransmit_requested){
            usart_send_btp(saved_btp.packet.len, saved_btp.bytes);
            retransmit_requested = false;
        }
        if(update_message_requested){
            usart_send_btp_update_request();
            update_message_requested = false;
        }
    }
}

static void init_peripherals(void)
{
    // Enable IRQs
    __asm volatile ("cpsie i");

    //
    // RCC
    //
    RCC_IOPENR |= RCC_GPIOA;
    RCC_AHBENR |= RCC_FLASH;
    RCC_AHBENR |= RCC_CRC;
    RCC_APBENR1 |= RCC_USART2;
    RCC_APBENR2 |= RCC_SYSCFG;
    RCC_APBENR2 |= RCC_TIM14;

    //
    // GPIOA
    //
    GPIOA_MODER &= ~BIT4;
    GPIOA_MODER |= BIT5;
    GPIOA_MODER &= ~BIT6;
    GPIOA_MODER |= BIT7;

    GPIOA_OSPEEDR &= ~BIT4;
    GPIOA_OSPEEDR |= BIT5;
    GPIOA_OSPEEDR &= ~BIT6;
    GPIOA_OSPEEDR |= BIT7;

    GPIOA_AFRL &= ~(0x3 << 9);
    GPIOA_AFRL |= BIT8;
    GPIOA_AFRL &= ~(0x3 << 13);
    GPIOA_AFRL |= BIT12;

    // PA_12 Output
    GPIOA_MODER |= BIT24;
    GPIOA_MODER &= ~BIT25;

    GPIOA_OSPEEDR |= BIT24;
    GPIOA_OSPEEDR |= BIT25;

    GPIOA_OTYPER &= ~BIT12;


    //
    // USART2
    //
    // USART Disable
    USART2_CR1 &= ~USART_CR1_UE;
    USART2_CR1 |= USART_CR1_RE;
    USART2_CR1 |= USART_CR1_TE;
    USART2_CR1 |= USART_CR1_RXFNEIE;

    // BRR USARTDIV = FREQ/BAUDRATE
    // 16Mhz / 115200 = 139 = 0x8B
    USART2_BRR = BR_460800;

    // USART Enable
    USART2_CR1 |= USART_CR1_UE;

    //
    // TIM14
    //
    TIM14_CR1 &= ~BIT0;
    TIM14_PSC = 0x3E7F; // 15999 => 1000Hz
    TIM14_ARR = 0x5; // 5, making it fire every ~6ms
    TIM14_EGR |= BIT0;
    TIM14_SR &= ~BIT0;
    TIM14_CNT = 0;
    TIM14_DIER |= BIT0;
    TIM14_CR1 |= BIT0;

    //
    // CRC
    //
    CRC_CR |= BIT7;         // Bit-reversed
    CRC_CR |= (0x3 << 5);   // Bit reversal done by word

    //
    // NVIC
    //
    NVIC_ISER = NVIC_USART2;
    NVIC_ISER = NVIC_TIM14;
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
    RCC_IOPRSTR |= RCC_GPIOA;
    RCC_IOPRSTR &= ~RCC_GPIOA;
    RCC_IOPENR &= ~RCC_GPIOA;

    // RCC_AHBRSTR
    // CRC
    RCC_AHBRSTR |= RCC_CRC;
    RCC_AHBRSTR &= ~RCC_CRC;
    RCC_AHBENR &= ~RCC_CRC;
    // FLASH
    RCC_AHBRSTR |= RCC_FLASH;
    RCC_AHBRSTR &= ~RCC_FLASH;
    RCC_AHBENR &= ~RCC_FLASH;

    // RCC_APBRSTR1
    // USART2
    RCC_APBRSTR1 |= RCC_USART2;
    RCC_APBRSTR1 &= ~RCC_USART2;
    RCC_APBENR1 &= ~RCC_USART2;
    // RCC_APBRSTR2
    // SYSCFG
    RCC_APBRSTR2 |= RCC_SYSCFG;
    RCC_APBRSTR2 &= ~RCC_SYSCFG;
    RCC_APBENR2 &= ~RCC_SYSCFG;
    // TIM14
    RCC_APBRSTR2 |= RCC_TIM14;
    RCC_APBRSTR2 &= ~RCC_TIM14;
    RCC_APBENR2 &= ~RCC_TIM14;

    // NVIC
    // TIM_14
    NVIC_ICER = BIT19;
    // USART2
    NVIC_ICER = BIT28;
}

static void flash_unlock()
{
    if(FLASH_CR & BIT31){
        // Unlock sequence
        FLASH_KEYR = 0x45670123;
        FLASH_KEYR = 0xCDEF89AB;
        while(FLASH_CR & BIT31);
    }
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

static void flash_write_64(uint32_t address, const uint8_t* buffer, uint32_t buffer_length)
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

static void firmware_update(uint32_t firmware_size, const uint8_t* firmware_data)
{
        // Erase old firmware
        flash_unlock();
        flash_erase();

        // Write new firmware
        flash_write_64(APP_MEMORY_START, firmware_data, firmware_size);
        usart_send_btp_string("Flash written!\r\n");
        flash_lock();
        usart_send_btp_string("Firmware update completed, starting application..\r\n");

        jump_to_application();
}

static uint32_t calculate_checksum(uint32_t firmware_size, const uint8_t* firmware_data)
{
    CRC_CR |= BIT0; // Reset CRC

    for(uint32_t i = 0; i < firmware_size; i+=4){
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

__attribute__((noreturn))
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

    uint32_t app_stack = app_vector[0];
    uint32_t app_reset = app_vector[1];

    if (app_reset < APP_MEMORY_START || app_reset > APP_MEMORY_END){
        usart_send_btp_string("Invalid reset location. Aborting\r\n");
        usart_send_btp_string("app_reset = ");
        usart_send_btp_number(app_reset);
        usart_send_btp_string("\r\n");
        while(1);
    }
    if (app_stack < RAM_START || app_stack > RAM_END){
        usart_send_btp_string("Invalid stack location. Aborting\r\n");
        usart_send_btp_string("app_stack = ");
        usart_send_btp_number(app_stack);
        usart_send_btp_string("\r\n");
        while(1);
    }

    // Sending a last ACK to tell the python script to stop listening
    BTP_t btp;
    char* msg = "ACK";
    packet_encode(&btp.packet, 65535, string_length(msg), (uint8_t*)msg);
    usart_send_btp(btp.packet.len, btp.bytes);

    deinit_peripherals();

    // Perform MSP setup and jump in a single assembly block
    __asm volatile (
        "msr msp, %0\n\t"    // Set MSP to app_stack
        "bx %1\n\t"          // Jump to app_reset (using bx to ensure Thumb mode)
        :                    // No outputs
        : "r" (app_stack), "r" (app_reset) // Inputs
        :                    // No clobbers needed
    );

    while(1); // Unreachable
}

static inline bool packet_received(uint16_t len, bool length_extracted)
{
    return length_extracted && (data_buffer_index >= len + PACKET_HEADER_SIZE || data_buffer_index >= PACKET_TOTAL_SIZE);
}

static inline void packet_get_length(Packet_t* packet, bool* length_extracted)
{
    if(data_buffer_index >= 8 && !*length_extracted){
        packet->len = data_buffer[2] | data_buffer[3] << 8;
        *length_extracted = true;
    }
}

static bool packet_decode(Packet_t* packet)
{
    // Extracting header
    packet->seq = data_buffer[0] | (data_buffer[1] << 8);
    packet->len = data_buffer[2] | (data_buffer[3] << 8);
    packet->crc = data_buffer[4] |
                (data_buffer[5] << 8) |
                (data_buffer[6] << 16) |
                (data_buffer[7] << 24);

    if(packet->len > PACKET_DATA_SIZE){
        return false;
    }

    for(uint16_t i = 0; i < packet->len; i++){
        packet->data[i] = data_buffer[8 + i];
    }

    data_buffer_index = 0;

    CRC_CR |= BIT0; // Reset CRC
    CRC_DR = packet->seq | packet->len << 16;

    for(uint32_t i = 0; i < packet->len; i += 4){
        uint32_t word = 0;

        word |= packet->data[i];
        word |= (i + 1 < packet->len) ? packet->data[i + 1] << 8 : 0xFF << 8;
        word |= (i + 2 < packet->len) ? packet->data[i + 2] << 16 : 0xFF << 16;
        word |= (i + 3 < packet->len) ? packet->data[i + 3] << 24 : 0xFF << 24;

        // CRC data
        CRC_DR = word;
    }

    return (CRC_DR ^ 0xFFFFFFFF) == packet->crc;
}

static void packet_encode(Packet_t* packet, uint16_t seq, uint16_t len, const uint8_t* data)
{
    packet->seq = seq;
    packet->len = len;


    for(uint8_t i = 0; i < packet->len; i++){
        packet->data[i] = data[i];
    }

    CRC_CR |= BIT0; // Reset CRC
    CRC_DR = packet->seq | packet->len << 16;

    for(uint32_t i = 0; i < packet->len; i+=4){
        uint32_t word = 0;

        word |= packet->data[i];
        word |= (i + 1 < packet->len) ? packet->data[i + 1] << 8 : 0xFF << 8;
        word |= (i + 2 < packet->len) ? packet->data[i + 2] << 16 : 0xFF << 16;
        word |= (i + 3 < packet->len) ? packet->data[i + 3] << 24 : 0xFF << 24;

        // CRC data
        CRC_DR = word;
    }

    packet->crc = CRC_DR ^ 0xFFFFFFFF;
}

static void process_handshake(BTP_t* incoming, BTP_t* outgoing)
{
    if(strings_match((char*)incoming->packet.data, "ACK")){
        usart_send_btp_ack(&outgoing->packet, outgoing->bytes, SEQ_HANDSHAKE_ACK);
    }
}

static void process_size(BTP_t* incoming, BTP_t* outgoing, uint32_t* const firmware_size)
{
    *firmware_size = 0;
    for(uint8_t i = 0; i < incoming->packet.len; i++){
        *firmware_size |= incoming->packet.data[i] << (8 * i);
    }
    usart_send_btp_ack(&outgoing->packet, outgoing->bytes, SEQ_SIZE_ACK);
}

static void process_firmware_chunk(BTP_t* incoming, BTP_t* outgoing, uint8_t* const firmware_data)
{
    uint32_t chunk_start = (incoming->packet.seq - SEQ_DATA_START) * CHUNK_SIZE;
    for(uint16_t i = 0; i < incoming->packet.len; i++){
        firmware_data[chunk_start + i] = incoming->packet.data[i];
    }
    usart_send_btp_ack(&outgoing->packet, outgoing->bytes, incoming->packet.seq);
}

static void process_crc(BTP_t* incoming, BTP_t* outgoing, uint32_t* const received_crc)
{
    *received_crc = 0;
    for(uint16_t i = 0; i < incoming->packet.len; i++){
        *received_crc |= incoming->packet.data[i] << (8 * i);
    }
    usart_send_btp_ack(&outgoing->packet, outgoing->bytes, SEQ_CRC_ACK);
}

static inline void update_saved_btp(BTP_t* saved_btp, BTP_t* outgoing)
{
    for(uint8_t i = 0; i < outgoing->packet.len + PACKET_HEADER_SIZE; i++){
        saved_btp->bytes[i] = outgoing->bytes[i];
    }
}

static inline void clear_incoming_btp(BTP_t* incoming)
{
    for(uint16_t i = 0; i < PACKET_TOTAL_SIZE; i++){
        incoming->bytes[i] = 0;
    }
}

static void usart_send_byte(uint8_t byte)
{
    // Wait for USART2 TDR
    while(!(USART2_ISR & USART_ISR_TXFNF));

    // Respecting TE minimum period
    while(!(USART2_ISR & USART_ISR_TEACK));

    // Writing data to TDR
    USART2_TDR = byte;

    // Waiting for Transmission Complete Flag
    while(!(USART2_ISR & USART_ISR_TC));
}

static void usart_send_btp(uint16_t data_len, const uint8_t* data)
{
    for(uint8_t i = 0; i < data_len + PACKET_HEADER_SIZE; i++){
	usart_send_byte(data[i]);
    }
}

static void usart_send_btp_string(const char* string)
{
    uint16_t len = string_length(string);

    // seq = 0
    usart_send_byte(0);
    usart_send_byte(0);

    // len
    usart_send_byte(len & 0xFF);
    usart_send_byte((len << 8) & 0xFF);

    // crc = 0
    usart_send_byte(0);
    usart_send_byte(0);
    usart_send_byte(0);
    usart_send_byte(0);

    // string
    for(uint8_t i = 0; i < len; i++){
	usart_send_byte(string[i]);
    }
}

static void usart_send_btp_number(uint32_t number)
{
    char buffer[11];  // Max 10 digits + null terminator
    int index = 10;
    buffer[index] = '\0';

    if (number == 0) {
        usart_send_byte('0');
        return;
    }

    while (number > 0 && index > 0) {
        index--;
        buffer[index] = (number % 10) + '0';
        number /= 10;
    }

    usart_send_btp_string(&buffer[index]);
}

static void usart_send_btp_update_request(void)
{
    BTP_t btp;
    packet_encode(&btp.packet, SEQ_UPDATE, string_length("update"), (uint8_t*)"update");
    usart_send_btp(btp.packet.len, btp.bytes);
}

static void usart_send_btp_ack(Packet_t* packet, uint8_t* bytes, uint16_t seq)
{
    packet_encode(packet, seq, string_length("ACK"), (uint8_t*)"ACK");
    usart_send_btp(packet->len, bytes);
}

static uint32_t string_length(const char* string)
{
    uint32_t i = 0;
    while(string[i]){
	i++;
    }
    return i;
}

static bool strings_match(const char* str1, const char* str2)
{
    uint32_t i = 0;
    for(; str1[i]; i++){
	if(str1[i] != str2[i]){
	    return false;
	}
    }
    if(str2[i]){
	return false;
    }
    return true;
}

void USART2_LPUART2_IRQHandler(void)
{
    if(USART2_ISR & USART_ISR_RXFNE){
        TIM14_CNT = 0;
        uint8_t data = USART2_RDR;

        if(data_buffer_index < sizeof(data_buffer)){
            data_buffer[data_buffer_index++] = data;
        }
    }
}

void TIM14_IRQHandler(void)
{
    // Clearing update interrupt flag
    TIM14_SR &= ~BIT0;

    if(updating){
        retransmit_requested = true;
    }else{
        application_start_timer--;
        update_message_requested = true;
    }

    data_buffer_index = 0;
}
