# STM32G0 Bootloader

A lightweight USART-based bootloader for STM32G0 microcontrollers.

## Features

- UART (USART) firmware update
- CRC32 validation using STM32 hardware CRC
- Flash programming and jump to application
- Minimal startup code and linker script
- Works with Python upload script

## Hardware

- Tested on STM32G071RB Nucleo board
- Should work on other STM32G0 series with minimal changes

## Getting Started

### Build

```bash
make
