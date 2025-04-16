/*
 * bootloader.c - USART bootloader for STM32G0
 * MIT License — Copyright (c) 2025 Oskar Arnudd
 * See LICENSE file for details.
 */

#include "main.h"

static volatile uint8_t application_start_timer = BOOTLOADER_WAIT_TIME_500MS;
static volatile uint8_t data_buffer[264];
static volatile uint16_t data_buffer_index = 0;
static volatile bool retransmit_requested = false;
static volatile bool update_message_requested = false;
static bool updating = false;

static void init_peripherals(void);
static void deinit_peripherals(void);

static void flash_unlock(void);
static void flash_lock(void);
static void flash_page_erase(const uint32_t address);
static void flash_region_erase(const uint32_t start, const uint32_t firmware_size);
static void flash_write_64(uint32_t address, const uint8_t* const buffer, const uint32_t buffer_length);

static void jump_to_application(void);
static inline bool packet_received(const uint16_t len, const bool length_extracted);
static inline void packet_get_length(Packet_t* const packet, bool* const length_extracted);
static void packet_encode(Packet_t* const packet, const uint16_t seq, const uint16_t len, const uint8_t* const data);
static bool packet_decode(Packet_t* const packet);
static void process_handshake(const BTP_t* const incoming, BTP_t* const outgoing);
static void process_size(const BTP_t* const incoming, BTP_t* const outgoing, uint32_t* const firmware_size);
static void process_firmware_chunk(const BTP_t* const incoming, BTP_t* const outgoing);
static bool crc_verified(const BTP_t* const incoming, BTP_t* const outgoing, const uint32_t firmware_size);
static inline void update_saved_btp(BTP_t* const saved_btp, const BTP_t* const outgoing);
static inline void clear_incoming_btp(BTP_t* const incoming);

static void usart_send_byte(const uint8_t byte);
static void usart_send_btp(const uint16_t data_len, const uint8_t* const data);
static void usart_send_btp_string(const char* const string);
static void usart_send_btp_number(uint32_t number);
static void usart_send_btp_update_request(void);
static void usart_send_btp_ack(Packet_t* const packet, const uint8_t* const bytes, const uint16_t seq);

static uint32_t string_length(const char* string);
static bool strings_match(const char* str1, const char* str2);

int main(void)
{
    BTP_t outgoing_btp = {0};
    BTP_t incoming_btp = {0};
    Packet_t* incoming_packet = &incoming_btp.packet;
    BTP_t saved_btp = {0};

    uint32_t firmware_size = 0;
    bool length_extracted = false;

    init_peripherals();

    while (1) {
        if(application_start_timer <= 0){
            jump_to_application();
        }

        // Parsing packet header when at least 8 bytes have been received
        packet_get_length(incoming_packet, &length_extracted);

        // Checking if we received a packet (data >= len)
        if(packet_received(incoming_packet->len, length_extracted)){
            length_extracted = false;
            application_start_timer = BOOTLOADER_WAIT_TIME_500MS;

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
                    break;
                case SEQ_CRC:
                    flash_lock();
                    if(crc_verified(&incoming_btp, &outgoing_btp, firmware_size)){
                        usart_send_btp_string("Firmware update completed, starting application..\r\n");
                        jump_to_application();
                    }else{
                        updating = false;
                        usart_send_btp_string("CRC did not match, exiting..\r\n");
                        usart_send_btp_ack(&outgoing_btp.packet, outgoing_btp.bytes, SEQ_FINISHED);
                    }
                    break;
            }
            if(incoming_packet->seq >= SEQ_DATA_START && incoming_packet->seq <= SEQ_DATA_END){
                process_firmware_chunk(&incoming_btp, &outgoing_btp);
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
    // Setting PA2, PA3 to USART2 TX/RX (AF1)
    GPIOA_MODER &= ~GPIO_MODER_MSK(2);
    GPIOA_MODER |= GPIO_MODER(2, GPIO_MODER_AF);
    GPIOA_MODER &= ~GPIO_MODER_MSK(3);
    GPIOA_MODER |= GPIO_MODER(3, GPIO_MODER_AF);
    GPIOA_AFRL &= ~GPIO_AFRL_MSK(2);
    GPIOA_AFRL |= GPIO_AFRL(2, GPIO_AF1);
    GPIOA_AFRL &= ~GPIO_AFRL_MSK(3);
    GPIOA_AFRL |= GPIO_AFRL(3, GPIO_AF1);

    //
    // USART2
    //
    USART2_CR1 &= ~USART_CR1_UE;
    USART2_CR1 |= USART_CR1_RE;
    USART2_CR1 |= USART_CR1_TE;
    USART2_CR1 |= USART_CR1_RXNEIE;
    USART2_BRR = BR_460800;
    USART2_CR1 |= USART_CR1_UE;

    //
    // TIM14
    //
    TIM14_CR1 &= ~TIM_CR1_CEN;
    TIM14_PSC = TIMER_PRESCALAR_1000HZ;
    TIM14_ARR = RETRANSMISSION_INTERVAL_10MS;
    TIM14_CNT = 0;
    TIM14_DIER |= TIM_DIER_UIE; // Generate interrupt on overflow
    TIM14_CR1 |= TIM_CR1_CEN;

    //
    // CRC
    //
    CRC_CR |= CRC_CR_REV_OUT;     // Bit-reversed
    CRC_CR &= ~CRC_CR_REV_IN_MSK;
    CRC_CR |= CRC_CR_REV_IN(0x3); // Bit reversal done by word

    //
    // NVIC
    //
    NVIC_ISER = NVIC_USART2;
    NVIC_ISER = NVIC_TIM14;
}

static void deinit_peripherals(void)
{
    GPIOA_MODER = 0xEBFFFFFF; // 0xEBFFFFFF is just the MODER reset value for GPIOA, nothing fancy going on here
    GPIOA_OSPEEDR = 0x0C000000; // Same as above
    GPIOA_AFRL = 0;

    USART2_CR1 = 0;
    USART2_BRR = 0;

    CRC_CR = 0;

    TIM14_CR1 = 0;
    TIM14_DIER = 0;
    TIM14_EGR = 0;
    TIM14_PSC = 0;
    TIM14_ARR = 0xFFFF;

    RCC_IOPRSTR |= RCC_GPIOA;
    RCC_IOPRSTR &= ~RCC_GPIOA;
    RCC_IOPENR &= ~RCC_GPIOA;

    RCC_AHBRSTR |= RCC_CRC;
    RCC_AHBRSTR &= ~RCC_CRC;
    RCC_AHBENR &= ~RCC_CRC;

    RCC_AHBRSTR |= RCC_FLASH;
    RCC_AHBRSTR &= ~RCC_FLASH;
    RCC_AHBENR &= ~RCC_FLASH;

    RCC_APBRSTR1 |= RCC_USART2;
    RCC_APBRSTR1 &= ~RCC_USART2;
    RCC_APBENR1 &= ~RCC_USART2;

    RCC_APBRSTR2 |= RCC_SYSCFG;
    RCC_APBRSTR2 &= ~RCC_SYSCFG;
    RCC_APBENR2 &= ~RCC_SYSCFG;

    RCC_APBRSTR2 |= RCC_TIM14;
    RCC_APBRSTR2 &= ~RCC_TIM14;
    RCC_APBENR2 &= ~RCC_TIM14;

    NVIC_ICER = NVIC_TIM14;
    NVIC_ICER = NVIC_USART2;
}

static void flash_unlock(void)
{
    if(FLASH_CR & FLASH_CR_LOCK){
        // Unlock sequence
        FLASH_KEYR = 0x45670123;
        FLASH_KEYR = 0xCDEF89AB;
        while(FLASH_CR & FLASH_CR_LOCK);
    }
}

static void flash_lock(void)
{
    FLASH_CR |= FLASH_CR_LOCK;
}

static void flash_page_erase(const uint32_t address)
{
    while(FLASH_SR & FLASH_SR_BSY1);

    FLASH_CR |= FLASH_CR_PER;
    FLASH_CR &= ~FLASH_CR_PNB_MSK;
    FLASH_CR |= FLASH_CR_PNB((address - 0x08000000) / FLASH_PAGE_SIZE);
    FLASH_CR |= FLASH_CR_STRT;

    while(FLASH_SR & FLASH_SR_BSY1);

    if (FLASH_SR & (FLASH_SR_PROGERR |
                    FLASH_SR_WRPERR |
                    FLASH_SR_PGAERR |
                    FLASH_SR_SIZERR |
                    FLASH_SR_PGSERR
                    )){
        usart_send_btp_string("Erase error flags at address ");
        usart_send_btp_number(address);
        usart_send_btp_string("\r\n");
    }

    FLASH_CR &= ~FLASH_CR_PER;
    if (*(volatile uint32_t*)(address) != 0xFFFFFFFF) {
        usart_send_btp_string("Erase verification failed at address ");
        usart_send_btp_number(address);
        usart_send_btp_string("\r\n");
    }
}

static void flash_region_erase(const uint32_t start, const uint32_t firmware_size)
{
    uint32_t end = APP_MEMORY_START + ((firmware_size + FLASH_PAGE_SIZE - 1) & (~FLASH_PAGE_SIZE - 1));
    end += FLASH_PAGE_SIZE;
    for(uint32_t i = start, j = 0; i < end; i += FLASH_PAGE_SIZE, j++){
        flash_page_erase(i);
    }
}

static void flash_write_64(uint32_t address, const uint8_t* const buffer, const uint32_t buffer_length)
{
    if (buffer_length == 0){
        return;
    }

    uint32_t full_chunks = buffer_length / 8;
    uint32_t remaining = buffer_length % 8;

    while(FLASH_SR & FLASH_SR_BSY1);

    // Checking error flags
    if (FLASH_SR & (FLASH_SR_PROGERR |
                    FLASH_SR_WRPERR |
                    FLASH_SR_PGAERR |
                    FLASH_SR_SIZERR |
                    FLASH_SR_PGSERR
                    )){
        usart_send_btp_string("Write error flags at address ");
        usart_send_btp_number(address);
        usart_send_btp_string("\r\n");
    }

    FLASH_CR |= FLASH_CR_PG;

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

        while(FLASH_SR & FLASH_SR_BSY1);
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

        while(FLASH_SR & FLASH_SR_BSY1);
    }
    FLASH_CR &= ~FLASH_CR_PG;
}

static void jump_to_application(void)
{
    // Disable IRQs
    __asm volatile ("cpsid i");

    uint32_t* app_vector = (uint32_t*)(APP_MEMORY_START);
    uint32_t* sram_vector = (uint32_t*)(0x20000000);

    /*
     * No VTOR, so we use SYSCFG MEM_MODE to copy the vector table and
     * stack address to RAM and then map RAM to 0x00000000
     */
    RCC_APBENR2 |= RCC_SYSCFG;
    SYSCFG_CFGR1 &= ~SYSCFG_CFGR1_MEM_MODE_MSK;
    SYSCFG_CFGR1 |= SYSCFG_CFGR1_MEM_MODE(SYSCFG_MEM_MODE_SRAM);

    for(uint32_t i = 0; i < VECTOR_TABLE_SIZE / 4; i++){
        sram_vector[i] = app_vector[i];
    }

    uint32_t app_stack = app_vector[0];
    uint32_t app_reset = app_vector[1];

    if (app_reset < APP_MEMORY_START || app_reset > APP_MEMORY_END){
        usart_send_btp_string("Invalid reset location. Aborting\r\n");
        usart_send_btp_string("Reset location:");
        usart_send_btp_number(app_reset);
        usart_send_btp_string("\r\n");
        while(1);
    }
    if (app_stack < RAM_START || app_stack > RAM_END){
        usart_send_btp_string("Invalid stack location. Aborting\r\n");
        usart_send_btp_string("Stack location:");
        usart_send_btp_number(app_stack);
        usart_send_btp_string("\r\n");
        while(1);
    }

    // Sending a last ACK to tell the python script to stop listening
    BTP_t btp;
    char* msg = "ACK";
    packet_encode(&btp.packet, SEQ_FINISHED, string_length(msg), (uint8_t*)msg);
    usart_send_btp(btp.packet.len, btp.bytes);

    deinit_peripherals();

    /*
     * Perform MSP setup and jump in a single assembly block.
     * Used to do this using a function pointer, pointing to
     * app_reset, but with optimization level -Os, it seems
     * the optimizer prefered keeping the app_reset on the stack,
     * which lead to some interesting results when moving the stack
     * pointer before being able to use that address
     */
    __asm volatile (
        "msr msp, %0\n\t"
        "bx %1\n\t"
        :                                   // No outputs
        : "r" (app_stack), "r" (app_reset)  // Inputs
        :                                   // No clobbers
    );

    while(1); // Unreachable
}

static inline bool packet_received(const uint16_t len, const bool length_extracted)
{
    return length_extracted && (data_buffer_index >= len + PACKET_HEADER_SIZE || data_buffer_index >= PACKET_TOTAL_SIZE);
}

static inline void packet_get_length(Packet_t* const packet, bool* const length_extracted)
{
    if(data_buffer_index >= 8 && !*length_extracted){
        packet->len = data_buffer[2] | data_buffer[3] << 8;
        *length_extracted = true;
    }
}

static bool packet_decode(Packet_t* const packet)
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

    CRC_INIT = 0xFFFFFFFF;
    CRC_CR |= CRC_CR_RESET;
    CRC_DR = packet->seq | packet->len << 16;

    for(uint32_t i = 0; i < packet->len; i += 4){
        uint32_t word = 0;

        word |= packet->data[i];
        word |= (i + 1 < packet->len) ? packet->data[i + 1] << 8 : 0xFF << 8;
        word |= (i + 2 < packet->len) ? packet->data[i + 2] << 16 : 0xFF << 16;
        word |= (i + 3 < packet->len) ? packet->data[i + 3] << 24 : 0xFF << 24;

        CRC_DR = word;
    }

    data_buffer_index = 0;
    return (CRC_DR ^ 0xFFFFFFFF) == packet->crc;
}

static void packet_encode(Packet_t* const packet, const uint16_t seq, const uint16_t len, const uint8_t* const data)
{
    packet->seq = seq;
    packet->len = len;

    for(uint8_t i = 0; i < packet->len; i++){
        packet->data[i] = data[i];
    }

    CRC_INIT = 0xFFFFFFFF;
    CRC_CR |= CRC_CR_RESET;
    CRC_DR = packet->seq | packet->len << 16;

    for(uint32_t i = 0; i < packet->len; i+=4){
        uint32_t word = 0;

        word |= packet->data[i];
        word |= (i + 1 < packet->len) ? packet->data[i + 1] << 8 : 0xFF << 8;
        word |= (i + 2 < packet->len) ? packet->data[i + 2] << 16 : 0xFF << 16;
        word |= (i + 3 < packet->len) ? packet->data[i + 3] << 24 : 0xFF << 24;

        CRC_DR = word;
    }

    packet->crc = CRC_DR ^ 0xFFFFFFFF;
}

static void process_handshake(const BTP_t* const incoming, BTP_t* const outgoing)
{
    if(strings_match((char*)incoming->packet.data, "ACK")){
        usart_send_btp_ack(&outgoing->packet, outgoing->bytes, SEQ_HANDSHAKE_ACK);
    }
}

static void process_size(const BTP_t* const incoming, BTP_t* const outgoing, uint32_t* const firmware_size)
{
    *firmware_size = 0;
    for(uint8_t i = 0; i < incoming->packet.len; i++){
        *firmware_size |= incoming->packet.data[i] << (8 * i);
    }

    if(*firmware_size > FIRMWARE_MAX_SIZE){
        usart_send_btp_string("Error: Firmware too large, exiting\r\n");
        usart_send_btp_ack(&outgoing->packet, outgoing->bytes, SEQ_FINISHED);
        updating = false;
        return;
    }

    // Disabling timer during flash erase to avoid sending repeat messages
    TIM14_CR1 &= ~TIM_CR1_CEN;
    flash_unlock();
    flash_region_erase(APP_MEMORY_START, *firmware_size);
    TIM14_CR1 |= TIM_CR1_CEN;

    usart_send_btp_ack(&outgoing->packet, outgoing->bytes, SEQ_SIZE_ACK);
}

static void process_firmware_chunk(const BTP_t* const incoming, BTP_t* const outgoing)
{
    uint32_t address = APP_MEMORY_START + (incoming->packet.seq - SEQ_DATA_START) * CHUNK_SIZE;
    flash_write_64(address, incoming->packet.data, incoming->packet.len);

    usart_send_btp_ack(&outgoing->packet, outgoing->bytes, incoming->packet.seq);
}

static bool crc_verified(const BTP_t* const incoming, BTP_t* const outgoing, const uint32_t firmware_size)
{
    uint32_t received_crc = 0;
    for(uint16_t i = 0; i < incoming->packet.len; i++){
        received_crc |= incoming->packet.data[i] << (8 * i);
    }

    CRC_INIT = 0xFFFFFFFF;
    CRC_CR |= CRC_CR_RESET;

    uint8_t* addr = (uint8_t*)APP_MEMORY_START;

    for(uint32_t i = 0; i < firmware_size; i+=4){
        uint32_t word = 0;

        word |= addr[i];
        word |= (i + 1 < firmware_size) ? addr[i + 1] << 8 : 0xFF << 8;
        word |= (i + 2 < firmware_size) ? addr[i + 2] << 16 : 0xFF << 16;
        word |= (i + 3 < firmware_size) ? addr[i + 3] << 24 : 0xFF << 24;

        CRC_DR = word;
    }

    uint32_t crc = ~CRC_DR;

    if(received_crc != crc){
        return false;
    }
    return true;
}

static inline void update_saved_btp(BTP_t* const saved_btp, const BTP_t* const outgoing)
{
    for(uint8_t i = 0; i < outgoing->packet.len + PACKET_HEADER_SIZE; i++){
        saved_btp->bytes[i] = outgoing->bytes[i];
    }
}

static inline void clear_incoming_btp(BTP_t* const incoming)
{
    for(uint16_t i = 0; i < PACKET_TOTAL_SIZE; i++){
        incoming->bytes[i] = 0;
    }
}

static void usart_send_byte(const uint8_t byte)
{
    while(!(USART2_ISR & USART_ISR_TXE));

    USART2_TDR = byte;

    while(!(USART2_ISR & USART_ISR_TC));
}

static void usart_send_btp(const uint16_t data_len, const uint8_t* const data)
{
    for(uint8_t i = 0; i < data_len + PACKET_HEADER_SIZE; i++){
	usart_send_byte(data[i]);
    }
}

static void usart_send_btp_string(const char* const string)
{
    uint16_t len = string_length(string);

    // seq = 0
    usart_send_byte(0);
    usart_send_byte(0);

    // len
    usart_send_byte(len & 0xFF);
    usart_send_byte((len >> 8) & 0xFF);

    // crc = 0
    usart_send_byte(0);
    usart_send_byte(0);
    usart_send_byte(0);
    usart_send_byte(0);

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

static void usart_send_btp_ack(Packet_t* const packet, const uint8_t* const bytes, const uint16_t seq)
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
    if(USART2_ISR & USART_ISR_RXNE){
        TIM14_CNT = 0;
        uint8_t data = USART2_RDR;

        if(data_buffer_index < sizeof(data_buffer)){
            data_buffer[data_buffer_index++] = data;
        }
    }
}

void TIM14_IRQHandler(void)
{
    TIM14_SR &= ~TIM_SR_UIF;

    if(updating){
        retransmit_requested = true;
    }else{
        application_start_timer--;
        update_message_requested = true;
    }

    data_buffer_index = 0;
}
