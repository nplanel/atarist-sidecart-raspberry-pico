/**
 * File: constants.h
 * Author: Diego Parrilla Santamaría
 * Date: July 2023
 * Copyright: 2023 - GOODDATA LABS SL
 * Description: Constants used throughout the project.
 */

#pragma once

#include <stdint.h>
#include "pico/stdlib.h"

// GPIO constants for SELECT button.
extern const uint32_t SELECT_GPIO;

// GPIO constants for the read address from the bus
extern const uint READ_ADDR_GPIO_BASE;
extern const uint READ_ADDR_PIN_COUNT;
extern const uint READ_SIGNAL_GPIO_BASE;
extern const uint READ_SIGNAL_PIN_COUNT;

// Write data to the bus
extern const uint WRITE_DATA_GPIO_BASE;
extern const uint WRITE_DATA_PIN_COUNT;
extern const uint WRITE_SIGNAL_GPIO_BASE;
extern const uint WRITE_SIGNAL_PIN_COUNT;

// Frequency constants.
extern const float SAMPLE_DIV_FREQ;
extern const uint32_t RP2040_CLOCK_FREQ_KHZ;

extern const uint8_t ROM_BANKS;
extern const uint32_t FLASH_ROM_LOAD_OFFSET;
extern const uint32_t FLASH_ROM4_LOAD_OFFSET;
extern const uint32_t FLASH_ROM3_LOAD_OFFSET;
extern const uint32_t ROM_IN_RAM_ADDRESS;
extern const uint32_t ROMS_START_ADDRESS;
extern const uint32_t ROM4_START_ADDRESS;
extern const uint32_t ROM3_START_ADDRESS;
extern const uint32_t ROM_SIZE_BYTES;
extern const uint32_t ROM_SIZE_WORDS;
extern const uint32_t ROM_SIZE_LONGWORDS;
extern const uint32_t CONFIG_FLASH_OFFSET;
extern const uint32_t CONFIG_FLASH_SIZE;
extern const uint32_t CONFIG_VERSION;
extern const uint32_t CONFIG_MAGIC;

// Morse code
#define DOT_DURATION_MS 250
#define DASH_DURATION_MS 750
#define SYMBOL_GAP_MS 250
#define CHARACTER_GAP_MS 500

typedef struct
{
    char character;
    const char *morse;
} MorseCode;

extern MorseCode morseAlphabet[]; // This is a declaration, not a definition.
