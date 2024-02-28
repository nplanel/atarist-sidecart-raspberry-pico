
#include "include/netusbee.h"
#include "include/network.h"

#include "hardware/structs/systick.h"

#define rtl8129_send_data(a,v) netusbee_send_data(a,v)

static inline void netusbee_send_data(uint32_t addr, uint16_t data) {
     volatile uint16_t *m = (volatile uint16_t *)addr;
     *m = ((uint16_t)data) << 8;
}

static struct rtl8129_state {
    uint8_t CR;
    uint8_t ISR;
    uint8_t CLDA0;
    uint8_t CLDA1;
    uint8_t IMR;
    uint8_t TSR;
    uint8_t TPSR;
    uint8_t TBCR0;
    uint8_t TBCR1;
    uint8_t RSAR0;
    uint8_t DCR;
    uint8_t CRDA0;
    uint8_t CRDA1;
    uint8_t CSNSAV;
    uint8_t CNTR0;
    uint8_t CNTR1;
    uint8_t RSR;
    uint8_t RCR;
    uint8_t TCR;
    uint8_t PSTART;
    uint8_t PSTOP;
    

    // page 1 R/W
    uint8_t PAR0;
    uint8_t PAR1;
    uint8_t PAR2;
    uint8_t PAR3;
    uint8_t PAR4;
    uint8_t PAR5;
    uint8_t CURR;
    uint8_t MAR0;
    uint8_t MAR1;
    uint8_t MAR2;
    uint8_t MAR3;
    uint8_t MAR4;
    uint8_t MAR5;
    uint8_t MAR6;
    uint8_t MAR7;

    // page 3
    uint8_t INTR; // RO
    uint8_t FMWP;

    uint8_t _9346CR;
    uint8_t BPAGE;
    uint8_t CONFIG0;
    uint8_t CONFIG1;
    uint8_t CONFIG2;
    uint8_t CONFIG3;
    uint8_t CONFIG4;


    uint8_t buf[0x6000];
    uint16_t bufidx;
} rtl;

#define REG_CR_PAGE_MASK 0xC0
#define REG_CR_PAGE0 0x00
#define REG_CR_PAGE1 0x40
#define REG_CR_PAGE2 0x80
#define REG_CR_PAGE3 0xC0


static int pktdata;

void __not_in_flash_func(reset_port)() {
    memset(&rtl, 0, sizeof(rtl));
    // TODO set default register state here
    // http://www.ethernut.de/pdf/8019asds.pdf page 16
    rtl.CR = 0x21;
    rtl.ISR = 0x00;
    rtl.CLDA0 = 0x00;

    rtl._9346CR = 0x00;
    rtl.BPAGE = 0x00;
    rtl.CONFIG0 = 0x00;
    rtl.CONFIG1 = 0x80;
    rtl.CONFIG2 = 0x00;
    rtl.CONFIG3 = 0x01;
    rtl.CONFIG4 = 0x00;

    // acknoledge reset
    rtl.ISR |= 0x80;
}

void __not_in_flash_func(reg_0x1f_reset_port)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    reset_port();
    if (!r_w) { // read
        rtl8129_send_data(addr, 0x00);
    }
}

void __not_in_flash_func(reg_0x07)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // ISR
                rtl8129_send_data(addr, rtl.ISR);
                break;
            case REG_CR_PAGE1: // CURR
                rtl8129_send_data(addr, rtl.CURR);
                break;
            default: // no page 2,3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // ISR write
                rtl.ISR = wdata;
                break;
            case REG_CR_PAGE1: // CURR
                rtl.CURR = wdata;
                break;
            case REG_CR_PAGE3: // TEST
                break;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x0a)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // 8019ID0
                rtl8129_send_data(addr, 0x50);
                break;
            case REG_CR_PAGE1: // MAR2
                rtl8129_send_data(addr, rtl.MAR2);
                break;
            default: // no page 2,3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // RBCR0
                break;
            case REG_CR_PAGE1: // MAR2
                rtl.MAR2 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}

void __not_in_flash_func(reg_0x0b)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // 8019ID1
                rtl8129_send_data(addr, 0x70);
                break;
            case REG_CR_PAGE1: // MAR3
                rtl8129_send_data(addr, rtl.MAR3);
                break;
            case REG_CR_PAGE2: // INTR
                rtl8129_send_data(addr, rtl.INTR);
                break;
            default: // no page 2,3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // RBCR1
                break;
            case REG_CR_PAGE1: // MAR2
                rtl.MAR3 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}

void __not_in_flash_func(reg_0x0c)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // RSR
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE1: // MAR4
                rtl8129_send_data(addr, rtl.MAR4);
                break;
            case REG_CR_PAGE2: // RCR
                rtl8129_send_data(addr, rtl.RCR);
                break;
            default: // no page 2,3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // RCR
                rtl.RCR = wdata;
                break;
            case REG_CR_PAGE1: // MAR4
                rtl.MAR4 = wdata;
                break;
            case REG_CR_PAGE3: // FMWP
                rtl.FMWP = wdata;
                break;
            default: // no page 3
                break;
        }
    }
}

void __not_in_flash_func(rtl_eeprom)(uint8_t data) {
    struct ee {
        union {
            struct {
                uint8_t emm1:1;
                uint8_t emm0:1;
                uint8_t _bit5:1;
                uint8_t _bit4:1;
                uint8_t cs:1;
                uint8_t clk:1;
                uint8_t din:1;
                uint8_t dout:1;
            };
            uint8_t data;
        }
    };
    static uint8_t cdata; // current data
    static struct ee oldee;
    struct ee ee;
    ee.data = data;
    if (ee.cs == 0) {
        oldee = ee;
        return;
    }
    if ((oldee.clk == 0) && (ee.clk == 1)) {
        data |= ee.din;
        data <<= 1;
        // TODO count bits and logic to return dout
    }

    oldee = ee;
}

void __not_in_flash_func(reg_0x01)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CLDA0 : Local DMA Address counter
                rtl8129_send_data(addr, rtl.CLDA0);
                break;
            case REG_CR_PAGE1: // PAR0
                rtl8129_send_data(addr, rtl.PAR0);
                break;
            case REG_CR_PAGE2: // PSTART
                rtl8129_send_data(addr, rtl.PSTART);
                break;
            case REG_CR_PAGE3: // 9346CR
                // rtl_eeprom read data bit0
                rtl8129_send_data(addr, 0x01);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // PSTART
                rtl.PSTART = wdata;
                break;
            case REG_CR_PAGE1: // PAR0
                rtl.PAR0 = wdata;
                break;
            case REG_CR_PAGE3: // 9346CR
                rtl_eeprom(wdata);
                break;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x02)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CLDA1 : Local DMA Address counter
                rtl8129_send_data(addr, rtl.CLDA1);
                break;
            case REG_CR_PAGE1: // PAR1
                rtl8129_send_data(addr, rtl.PAR1);
                break;
            case REG_CR_PAGE2: // PSTOP
                rtl8129_send_data(addr, rtl.PSTOP);
                break;
            case REG_CR_PAGE3: // BPAGE
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // PSTOP
                rtl.PSTOP = wdata;
                break;
            case REG_CR_PAGE1: // PAR0
                rtl.PAR0 = wdata;
                break;
            case REG_CR_PAGE3: // BPAGE
                rtl.BPAGE = wdata;
                break;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x03)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // BNRY : Boundary Register
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE1: // PAR2
                rtl8129_send_data(addr, rtl.PAR2);
                break;
            case REG_CR_PAGE2: // no page 2
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE3: // CONFIG0
                rtl8129_send_data(addr, rtl.CONFIG0);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // BNRY
                break;
            case REG_CR_PAGE1: // PAR2
                rtl.PAR2 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}


void __not_in_flash_func(reg_0x05)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // NCR : Number of Collisions Register
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE1: // PAR4
                rtl8129_send_data(addr, rtl.PAR4);
                break;
            case REG_CR_PAGE2: // no page 2
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE3: // CONFIG2
                rtl8129_send_data(addr, rtl.CONFIG2);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // TBCR0
                rtl.TBCR0 = wdata;
                break;
            case REG_CR_PAGE1: // PAR4
                rtl.PAR4 = wdata;
                break;
            case REG_CR_PAGE3: // CONFIG2
                rtl.CONFIG2 = wdata;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x00_CR)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        rtl8129_send_data(addr, rtl.CR);
    } else {
        rtl.CR = wdata;
        
        if (rtl.CR == 0x22) { // rx path
            rtl.ISR |= 1<<6; // DMA complete
        }
        if (rtl.CR == 0x12) { // remote write operation
            // send packet from buffer
            uint16_t pktlen = (rtl.TBCR1 << 8) | rtl.TBCR0;
            pktdata = pktlen;
        }
    }
}

void __not_in_flash_func(reg_0x10_RDMA)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        rtl8129_send_data(addr, rtl.CR);
    } else {
        rtl.buf[rtl.bufidx] = wdata;
        rtl.bufidx++; // end tx buffer : & 0x6000-1
    }
}

void __not_in_flash_func(reg_0x0f)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CNTR2
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE1: // MAR7
                rtl8129_send_data(addr, rtl.MAR7);
                break;
            case REG_CR_PAGE2: // IMR
                rtl8129_send_data(addr, rtl.IMR);
                break;
            case REG_CR_PAGE3: // no page 3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // IMR
                rtl.IMR = wdata;
                break;
            case REG_CR_PAGE1: // MAR7
                rtl.MAR7 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}

void __not_in_flash_func(reg_0x04)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // TSR
                rtl8129_send_data(addr, rtl.TSR);
                break;
            case REG_CR_PAGE1: // PAR3
                rtl8129_send_data(addr, rtl.PAR3);
                break;
            case REG_CR_PAGE2: // TPSR
                rtl8129_send_data(addr, rtl.TPSR);
                break;
            case REG_CR_PAGE3: // no page 3
                rtl8129_send_data(addr, rtl.CONFIG3);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // TPSR
                rtl.TPSR = wdata;
                rtl.bufidx = rtl.TPSR << 8;
                break;
            case REG_CR_PAGE1: // PAR3
                rtl.PAR3 = wdata;
                break;
            case REG_CR_PAGE3:
                rtl.CONFIG3 = wdata;
                break;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x06)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // FIFO
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE1: // PAR5
                rtl8129_send_data(addr, rtl.PAR5);
                break;
            case REG_CR_PAGE2: // no page 2
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE3: // CONFIG3
                rtl8129_send_data(addr, rtl.CONFIG3);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // TBCR1
                rtl.TBCR1 = wdata;
                break;
            case REG_CR_PAGE1: // PAR5
                rtl.PAR5 = wdata;
                break;
            case REG_CR_PAGE3: // CONFIG3
                rtl.CONFIG3 = wdata;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x08)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CRDA0
                rtl8129_send_data(addr, rtl.CRDA0);
                break;
            case REG_CR_PAGE1: // MAR0
                rtl8129_send_data(addr, rtl.MAR0);
                break;
            case REG_CR_PAGE2: // no page 2
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE3: // CSNSAV
                rtl8129_send_data(addr, rtl.CSNSAV);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // RSAR0
                rtl.RSAR0 = wdata;
                break;
            case REG_CR_PAGE1: // MAR0
                rtl.MAR0 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}

void __not_in_flash_func(reg_0x09)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CRDA1
                rtl8129_send_data(addr, rtl.CRDA1);
                break;
            case REG_CR_PAGE1: // MAR1
                rtl8129_send_data(addr, rtl.MAR1);
                break;
            case REG_CR_PAGE2: // no page 2
                rtl8129_send_data(addr, 0x00);
                break;
            case REG_CR_PAGE3: // no page 3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // RSAR0
                rtl.RSAR0 = wdata;
                break;
            case REG_CR_PAGE1: // MAR1
                rtl.MAR1 = wdata;
                break;
            case REG_CR_PAGE3: // HLTCLK
                //rtl.HLTCLK = wdata;
                break;
            default: // no page 2
                break;
        }
    }
}

void __not_in_flash_func(reg_0x0d)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CNTR0
                rtl8129_send_data(addr, rtl.CNTR0);
                break;
            case REG_CR_PAGE1: // MAR5
                rtl8129_send_data(addr, rtl.MAR5);
                break;
            case REG_CR_PAGE2: // TCR
                rtl8129_send_data(addr, rtl.TCR);
                break;
            case REG_CR_PAGE3: // no page 3
                rtl8129_send_data(addr, rtl.CONFIG4);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // TCR
                rtl.TCR = wdata;
                break;
            case REG_CR_PAGE1: // MAR5
                rtl.MAR5 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}

void __not_in_flash_func(reg_0x0e)(uint8_t reg, bool r_w, uint8_t wdata, uint32_t addr) {
    if (!r_w) { // read
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // CNTR1
                rtl8129_send_data(addr, rtl.CNTR1);
                break;
            case REG_CR_PAGE1: // MAR6
                rtl8129_send_data(addr, rtl.MAR6);
                break;
            case REG_CR_PAGE2: // DCR
                rtl8129_send_data(addr, rtl.DCR);
                break;
            case REG_CR_PAGE3: // no page 3
                rtl8129_send_data(addr, 0x00);
                break;
        }
    } else {
        switch(rtl.CR & 0xC0) { // register page
            case REG_CR_PAGE0: // DCR
                rtl.DCR = wdata;
                break;
            case REG_CR_PAGE1: // MAR6
                rtl.MAR6 = wdata;
                break;
            default: // no page 2,3
                break;
        }
    }
}


#define RAWADDR_LEN 1024
struct rawdata {
    uint32_t addr;
    uint8_t data;
    uint8_t CRpage;
};
static struct rawdata rawaddr[RAWADDR_LEN];
static int rawaddr_idx;

#include "hexdump.h"
void dump_rawaddr() {
    int i;
    fprintf(stderr, "============ %d\n", pktdata);
    for(i=0;i<rawaddr_idx;i++) {
        struct rawdata *rd = &rawaddr[i];
        fprintf(stderr, "0x%08x ", rd->addr);
        uint8_t reg = ((rd->addr & 0xffff) >> 9) & 0x9f;
        uint8_t wdata = rd->addr & 0xff;
        bool rw = (rd->addr >= ROM3_START_ADDRESS); // R_/W write register
        if (!rw) { // read
            wdata = rd->data;
        }
        fprintf(stderr, "0x%02x %c %d 0x%02x\n", reg, rw ? 'W' : 'R', rd->CRpage, wdata);
    }
    fprintf(stderr, "============ %d\n", rtl.bufidx);
    hexdump(rtl.buf, rtl.bufidx);
}

#include "ne2000.h"
static NE2000State ne2k;

void dump_ne2k() {
    fprintf(stderr, "cmd 0x%02x\n", ne2k.cmd);
    fprintf(stderr, "isr 0x%02x\n", ne2k.isr);
    fprintf(stderr, "dcfg 0x%02x\n", ne2k.dcfg);
    fprintf(stderr, "tcnt 0x%02x\n", ne2k.tcnt);
    fprintf(stderr, "rcnt 0x%02x\n", ne2k.rcnt);
    fprintf(stderr, "txpkt %d\n", txpktcnt);
}

static int meascnt;
static uint32_t meas[128];
void printmeas() {
    for(int i=0;i<meascnt;i++) {
        printf("%d\n",meas[i]);
    }
}

//addr 50200028

void __not_in_flash_func(netusbee_dma_irq_handler_req_callback)(void)
{
    // Read the address to process
    const uint32_t addr = (const uint32_t)dma_hw->ch[1].al3_read_addr_trig;
    uint16_t value = *((uint16_t *)addr);

    printf("addr %x v %x\n", addr, value);

    dma_hw->ints1 = 1u << 1;//lookup_data_rom_dma_channel;
}

void __isr __not_in_flash_func(netusbee_dma_irq_handler_lookup_callback)(void)
{
    systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00FFFFFF;

    uint32_t stopc = 0, startc = 0;

    // Read the address to process
    const uint32_t addr = (const uint32_t)dma_hw->ch[1].al3_read_addr_trig;

    // Avoid priting anything inside an IRQ handled function
    // DPRINTF("DMA LOOKUP: $%x\n", addr);
    const uint16_t regdata = addr >> 1;
    const uint8_t reg = (regdata >> 8) & 0x1f;
    const uint8_t wdata = regdata & 0xff;
    const bool rw = (addr & 0x10000);//>= ROM3_START_ADDRESS); // R_/W write register

    if (!rw) { // read
    startc = systick_hw->cvr;
        uint16_t r = ne2000_read(&ne2k, reg, 1);
        netusbee_send_data(addr, r);
    stopc = systick_hw->cvr;
    } else {
        ne2000_write(&ne2k, reg, wdata, 1);
    }
    // Clear the interrupt request for the channel
    dma_hw->ints1 = 1u << 1;//lookup_data_rom_dma_channel;

    meas[meascnt] = startc - stopc;
    if ((meascnt < 128) &&(startc != stopc))  {
        meascnt++;
    }
}


// Interrupt handler callback for DMA completion
void __not_in_flash_func(netusbeeold_dma_irq_handler_lookup_callback)(void)
{
    // Read the address to process
    const uint32_t addr = (const uint32_t)dma_hw->ch[lookup_data_rom_dma_channel].al3_read_addr_trig;

    // Avoid priting anything inside an IRQ handled function
    // DPRINTF("DMA LOOKUP: $%x\n", addr);
    uint8_t reg = ((addr & 0xffff) >> 9) & 0x9f;
    uint8_t wdata = addr & 0xff;
    bool rw = (addr >= ROM3_START_ADDRESS); // R_/W write register

    uint8_t cr = rtl.CR >> 6;
    switch(reg) {
        case 0x00:
            reg_0x00_CR(reg, rw, wdata, addr);
            break;
        case 0x01:
            reg_0x01(reg, rw, wdata, addr);
            break;
        case 0x02:
            reg_0x02(reg, rw, wdata, addr);
            break;
        case 0x03:
            reg_0x03(reg, rw, wdata, addr);
            break;
        case 0x04:
            reg_0x04(reg, rw, wdata, addr);
            break;
        case 0x05:
            reg_0x05(reg, rw, wdata, addr);
            break;
        case 0x06:
            reg_0x06(reg, rw, wdata, addr);
            break;
        case 0x07:
            reg_0x07(reg, rw, wdata, addr);
            break;
        case 0x08:
            reg_0x08(reg, rw, wdata, addr);
            break;
        case 0x09:
            reg_0x09(reg, rw, wdata, addr);
            break;
        case 0x0a:
            reg_0x0a(reg, rw, wdata, addr);
            break;
        case 0x0b:
            reg_0x0b(reg, rw, wdata, addr);
            break;
        case 0x0c:
            reg_0x0c(reg, rw, wdata, addr);
            break;
        case 0x0d:
            reg_0x0d(reg, rw, wdata, addr);
            break;
        case 0x0e:
            reg_0x0e(reg, rw, wdata, addr);
            break;
        case 0x0f:
            reg_0x0f(reg, rw, wdata, addr);
            break;
        case 0x10:
            reg_0x10_RDMA(reg, rw, wdata, addr);
            break;
        case 0x18:
        case 0x19:
        case 0x1a:
        case 0x1b:
        case 0x1c:
        case 0x1d:
        case 0x1e:
        case 0x1f:
            reg_0x1f_reset_port(reg, rw, wdata, addr);
            break;
        default:
        DPRINTF("REG not supported : $%x reg 0x%02x page %d\n", addr, reg, cr);
    }

    //if ((reg != 0x03)  && (reg != 0x07))
    {
        struct rawdata *rawd = &rawaddr[rawaddr_idx%RAWADDR_LEN];
        rawd->addr = addr;
        rawd->CRpage = cr;
        if (!rw) { // read
            uint16_t *m = (uint16_t*)addr;
            rawd->data = *m >> 8;
        } else {
            rawd->data = wdata;
        }
        if (rawaddr_idx < RAWADDR_LEN) {
            rawaddr_idx++;
        }
    }


clear_intr:
    // Clear the interrupt request for the channel
    dma_hw->ints1 = 1u << lookup_data_rom_dma_channel;
}


int txpktcnt;
struct pkt *txpktbuf;

void bench() {
    systick_hw->csr = 0x5;
    systick_hw->rvr = 0x00FFFFFF;

    uint32_t new, old, t0, t1;
    old=systick_hw->cvr;

    t0=time_us_32();
    sleep_us(123);
    new=systick_hw->cvr;
    t1=time_us_32();
    printf("\n          old-new=%d\n",old-new);
    printf("            t1-t0=%d\n",t1-t0);
    printf("(old-new)/(t1-t0)=%.1f\n",(old-new)/(t1-t0*1.0));
}

static int rxpktcnt;
static int rxpktcnterr;
static int oldtxpktcnt;

static void printinfo() {
    DPRINTF("========================== %d rx %d/%d\n",oldtxpktcnt,rxpktcnterr,rxpktcnt);
}


int init_netusbee(bool safe_config_reboot)
{
    blink_morse('N');

bench();
    network_init();
    char *pass = NULL;
    network_connect(true, false, &pass);
    network_connect(true, false, &pass);
    //cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);

    uint8_t rp2040mac[6];
    cyw43_wifi_get_mac(&cyw43_state, CYW43_ITF_STA, rp2040mac);
    DPRINTF("mac %02x:%02x:%02x:%02x:%02x:%02x\n", rp2040mac[0], rp2040mac[1], rp2040mac[2], rp2040mac[3], rp2040mac[4], rp2040mac[5]);

    txpktcnt = 0;
    txpktbuf = malloc(sizeof(struct pkt)*PKTBUFSZ);
    memset(txpktbuf, 0, sizeof(struct pkt)*PKTBUFSZ);

    ne2000_reset(&ne2k);
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

        if (uart_is_readable(uart0))
        {
            int c = getchar();
            if (c == 'd') {
                dump_rawaddr();
            }
            if (c == 'n') {
                dump_ne2k();
            }
            if (c == 'r') {
                rawaddr_idx = 0;
            }
            if (c == 'p') {
                uint8_t arppkt[] = {
     0xff,0xff,0xff,0xff,0xff,0xff,0xd8,0xec,0xe5,0xe1,0x8a,0x50,0x08,0x06,0x00,0x01,
     0x08,0x00,0x06,0x04,0x00,0x01,0xd8,0xec,0xe5,0xe1,0x8a,0x50,0xc0,0xa8,0xb2,0x01,
     0xff,0xff,0xff,0xff,0xff,0xff,0xc0,0xa8,0xb2,0x21,0x00,0x00,0x00,0x00,0x00,0x00,
     0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00
                };
                ssize_t r = ne2000_receive(&ne2k, arppkt, sizeof(arppkt));
                fprintf(stderr, "recv %d\n", r);
            }
            if (c == 'e') {
                extern void printee(NE2000State *s);
                printee(&ne2k);
            }
            if (c == 'm') {
                printmeas();
            }
            if (c == 'i') {
                printinfo();
            }
        }

        if (oldtxpktcnt != txpktcnt) {
            struct pkt *p = &txpktbuf[oldtxpktcnt];
            printinfo();
            oldtxpktcnt++;
            static bool first_packet = true;
            if (first_packet) {
                first_packet = false;
                memcpy(ne2000_mac, &p->data[6], 6);
                printf("======= ne2000 mac %02x:%02x:%02x:%02x:%02x:%02x\n", ne2000_mac[0], ne2000_mac[1], ne2000_mac[2], ne2000_mac[3], ne2000_mac[4], ne2000_mac[5]);
            }
            memcpy(&p->data[6], rp2040mac, 6);
            hexdump(p->data, p->len);
            int ret = cyw43_send_ethernet(&cyw43_state, CYW43_ITF_STA, p->len, (void *)p->data, false);
            if (ret) {
                DPRINTF("send_ethernet failed: %d\n", ret);
                continue;
            }            
        }

        {
            cyw43_t *self = &cyw43_state;
            //int status = cyw43_tcpip_link_status(&cyw43_state, CYW43_ITF_STA);
            struct netif *netif = &self->netif[CYW43_ITF_STA];
            err_t netif_ethernet_input(struct pbuf *p, struct netif *inp);
            netif->input = netif_ethernet_input;

            if (cyw43_ll_has_work(&self->cyw43_ll)) {
                cyw43_ll_process_packets(&self->cyw43_ll);
            }
        }

    }
    return 0;
}

err_t netif_ethernet_input(struct pbuf *p, struct netif *inp) {
    static uint8_t pkt[1514];
    memcpy(pkt, p->payload, p->len);
    uint8_t bcast[6] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
    if (memcmp(pkt, bcast, 6) != 0) {
        memcpy(pkt, ne2000_mac, 6);
    }
    hexdump(pkt, p->len);
    rxpktcnt++;
    int r = ne2000_receive(&ne2k, pkt, p->len);
    if (r <= 0) {
        rxpktcnterr++;
        printf("can't recv pkt %d\n", r);
    }

    pbuf_free(p);
    return ERR_OK;
}

/* dhcp offer

0x000000: ff ff ff ff ff ff d8 ec e5 e1 8a 50 08 00 45 00 ...........P..E.
0x000010: 01 48 c9 59 00 00 40 11 3d a2 c0 a8 b2 01 ff ff .H.Y..@.=.......
0x000020: ff ff 00 43 00 44 01 34 cf 3f 02 01 06 00 ad de ...C.D.4.?......
0x000030: 12 23 00 00 80 00 00 00 00 00 c0 a8 b2 3d c0 a8 .#...........=..
0x000040: b2 01 00 00 00 00 cd 28 08 82 04 65 00 00 00 00 .......(...e....
0x000050: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x000060: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x000070: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x000080: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x000090: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x0000a0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x0000b0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x0000c0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x0000d0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x0000e0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x0000f0: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x000100: 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 00 ................
0x000110: 00 00 00 00 00 00 63 82 53 63 35 01 02 36 04 c0 ......c.Sc5..6..
0x000120: a8 b2 01 33 04 00 01 51 80 3a 04 00 00 a8 c0 3b ...3...Q.:.....;
0x000130: 04 00 01 27 50 01 04 ff ff ff 00 1c 04 c0 a8 b2 ...'P...........
0x000140: ff 03 04 c0 a8 b2 01 06 08 01 01 01 01 08 08 04 ................
0x000150: 04 ff 00 00 00 00     

*/
