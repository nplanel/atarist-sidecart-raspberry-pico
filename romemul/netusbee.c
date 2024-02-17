
#include "include/netusbee.h"

#define rtl8129_send_data(addr, data) do { uint16_t *m = (uint16_t*)addr; *m = data << 8; } while(0)

void __not_in_flash_func(reg_0x01_init)(uint8_t reg, bool r_w, uint8_t rdata, uint32_t addr) {
    if (!r_w) {
        rtl8129_send_data(addr, 0x00);
    }
}

#define RAWADDR_LEN 16
static uint32_t rawaddr[RAWADDR_LEN];
static int rawaddr_idx;

void dump_rawaddr() {
    int i;
    fprintf(stderr, "============\n");
    for(i=0;i<rawaddr_idx;i++) {
        fprintf(stderr, "0x%08x\n", rawaddr[i]);
    }
}


// Interrupt handler callback for DMA completion
void __not_in_flash_func(netusbee_dma_irq_handler_lookup_callback)(void)
{
    // Read the address to process
    uint32_t addr = (uint32_t)dma_hw->ch[lookup_data_rom_dma_channel].al3_read_addr_trig;

    // Avoid priting anything inside an IRQ handled function
    // DPRINTF("DMA LOOKUP: $%x\n", addr);
    uint8_t reg = (addr >> 8) & 0x9f;
    uint8_t wdata = addr & 0xff;
    bool rw = (addr >= ROM3_START_ADDRESS); // R_/W write register

    rawaddr[rawaddr_idx%RAWADDR_LEN] = addr;
    rawaddr_idx++;
/*
    switch(reg) {
        case 0x01:
            reg_0x01_init(reg, rw, wdata, addr);
            break;
        default:
        DPRINTF("REG not supported : $%x\n", addr);
    }
*/

    // Clear the interrupt request for the channel
    dma_hw->ints1 = 1u << lookup_data_rom_dma_channel;
}

int init_netusbee(bool safe_config_reboot)
{

    DPRINTF("Waiting for register access...\n");

    bool error = false;
    bool show_blink = true;
    srand(time(0));
    while (!error)
    {
        tight_loop_contents();

        /* Delay the blink to the main loop */
        if (show_blink)
        {
            blink_morse('N');
            show_blink = false;
        }

        int c = getchar();
        if (c == 'd') {
            dump_rawaddr();
        }
        if (c == 'r') {
            rawaddr_idx = 0;
        }

    }
    return 0;
}