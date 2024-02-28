/**
 * File: rtcemul.h
 * Author: Diego Parrilla Santamaría
 * Date: July 2023
 * Copyright: 2023 - GOODDATA LABS SL
 * Description: Header file for the RTC emulator C program.
 */

#ifndef NETUSBEE_H
#define NETUSBEE_H

#include "debug.h"
#include "constants.h"

#include <inttypes.h>
#include <stdbool.h>
#include <stdio.h>

#include "pico/stdlib.h"
#include "hardware/pio.h"
#include "hardware/dma.h"
#include <hardware/watchdog.h>
#include "hardware/structs/bus_ctrl.h"
#include "pico/cyw43_arch.h"
#include "hardware/rtc.h"

#include "time.h"

#include "lwip/dns.h"
#include "lwip/netif.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"

#include "../../build/romemul.pio.h"

#include "tprotocol.h"
#include "commands.h"
#include "config.h"
#include "network.h"
#include "filesys.h"

extern int lookup_data_rom_dma_channel;

// Interrupt handler callback for DMA completion
void __not_in_flash_func(netusbee_dma_irq_handler_req_callback)(void);
void __not_in_flash_func(netusbee_dma_irq_handler_lookup_callback)(void);

// Function Prototypes
int init_netusbee(bool safe_config_reboot);


/////
struct pkt {
    uint16_t len;
    uint8_t data[1514];
};

#define PKTBUFSZ 16
extern int txpktcnt;
extern struct pkt *txpktbuf;


#endif // RTCEMUL_H
