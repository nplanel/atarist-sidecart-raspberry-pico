/*
 * QEMU NE2000 emulation
 *
 * Copyright (c) 2003-2004 Fabrice Bellard
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*
#include "qemu/osdep.h"
#include "net/eth.h"
#include "qemu/module.h"
#include "exec/memory.h"
#include "hw/irq.h"
#include "migration/vmstate.h"
#include "trace.h"
*/
#include "ne2000.h"

/* debug NE2000 card */
//#define DEBUG_NE2000

#define MAX_ETH_FRAME_SIZE 1514

#define E8390_CMD       0x00    /* The command register (for all pages) */
/* Page 0 register offsets. */
#define EN0_CLDALO      0x01    /* Low byte of current local dma addr  RD */
#define EN0_STARTPG     0x01    /* Starting page of ring bfr WR */
#define EN0_CLDAHI      0x02    /* High byte of current local dma addr  RD */
#define EN0_STOPPG      0x02    /* Ending page +1 of ring bfr WR */
#define EN0_BOUNDARY    0x03    /* Boundary page of ring bfr RD WR */
#define EN0_TSR         0x04    /* Transmit status reg RD */
#define EN0_TPSR        0x04    /* Transmit starting page WR */
#define EN0_NCR         0x05    /* Number of collision reg RD */
#define EN0_TCNTLO      0x05    /* Low  byte of tx byte count WR */
#define EN0_FIFO        0x06    /* FIFO RD */
#define EN0_TCNTHI      0x06    /* High byte of tx byte count WR */
#define EN0_ISR         0x07    /* Interrupt status reg RD WR */
#define EN0_CRDALO      0x08    /* low byte of current remote dma address RD */
#define EN0_RSARLO      0x08    /* Remote start address reg 0 */
#define EN0_CRDAHI      0x09    /* high byte, current remote dma address RD */
#define EN0_RSARHI      0x09    /* Remote start address reg 1 */
#define EN0_RCNTLO      0x0a    /* Remote byte count reg WR */
#define EN0_RTL8029ID0  0x0a    /* Realtek ID byte #1 RD */
#define EN0_RCNTHI      0x0b    /* Remote byte count reg WR */
#define EN0_RTL8029ID1  0x0b    /* Realtek ID byte #2 RD */
#define EN0_RSR         0x0c    /* rx status reg RD */
#define EN0_RXCR        0x0c    /* RX configuration reg WR */
#define EN0_TXCR        0x0d    /* TX configuration reg WR */
#define EN0_COUNTER0    0x0d    /* Rcv alignment error counter RD */
#define EN0_DCFG        0x0e    /* Data configuration reg WR */
#define EN0_COUNTER1    0x0e    /* Rcv CRC error counter RD */
#define EN0_IMR         0x0f    /* Interrupt mask reg WR */
#define EN0_COUNTER2    0x0f    /* Rcv missed frame error counter RD */

#define EN1_PHYS        0x11
#define EN1_CURPAG      0x17
#define EN1_MULT        0x18

#define EN2_STARTPG     0x21    /* Starting page of ring bfr RD */
#define EN2_STOPPG      0x22    /* Ending page +1 of ring bfr RD */

#define EN3_EEPROM      0x31
#define EN3_CONFIG0     0x33
#define EN3_CONFIG1     0x34
#define EN3_CONFIG2     0x35
#define EN3_CONFIG3     0x36

/*  Register accessed at EN_CMD, the 8390 base addr.  */
#define E8390_STOP      0x01    /* Stop and reset the chip */
#define E8390_START     0x02    /* Start the chip, clear reset */
#define E8390_TRANS     0x04    /* Transmit a frame */
#define E8390_RREAD     0x08    /* Remote read */
#define E8390_RWRITE    0x10    /* Remote write  */
#define E8390_NODMA     0x20    /* Remote DMA */
#define E8390_PAGE0     0x00    /* Select page chip registers */
#define E8390_PAGE1     0x40    /* using the two high-order bits */
#define E8390_PAGE2     0x80    /* Page 3 is invalid. */

/* Bits in EN0_ISR - Interrupt status register */
#define ENISR_RX        0x01    /* Receiver, no error */
#define ENISR_TX        0x02    /* Transmitter, no error */
#define ENISR_RX_ERR    0x04    /* Receiver, with error */
#define ENISR_TX_ERR    0x08    /* Transmitter, with error */
#define ENISR_OVER      0x10    /* Receiver overwrote the ring */
#define ENISR_COUNTERS  0x20    /* Counters need emptying */
#define ENISR_RDC       0x40    /* remote dma complete */
#define ENISR_RESET     0x80    /* Reset completed */
#define ENISR_ALL       0x3f    /* Interrupts we will enable */

/* Bits in received packet status byte and EN0_RSR*/
#define ENRSR_RXOK      0x01    /* Received a good packet */
#define ENRSR_CRC       0x02    /* CRC error */
#define ENRSR_FAE       0x04    /* frame alignment error */
#define ENRSR_FO        0x08    /* FIFO overrun */
#define ENRSR_MPA       0x10    /* missed pkt */
#define ENRSR_PHY       0x20    /* physical/multicast address */
#define ENRSR_DIS       0x40    /* receiver disable. set in monitor mode */
#define ENRSR_DEF       0x80    /* deferring */

/* Transmitted packet status, EN0_TSR. */
#define ENTSR_PTX 0x01  /* Packet transmitted without error */
#define ENTSR_ND  0x02  /* The transmit wasn't deferred. */
#define ENTSR_COL 0x04  /* The transmit collided at least once. */
#define ENTSR_ABT 0x08  /* The transmit collided 16 times, and was deferred. */
#define ENTSR_CRS 0x10  /* The carrier sense was lost. */
#define ENTSR_FU  0x20  /* A "FIFO underrun" occurred during transmit. */
#define ENTSR_CDH 0x40  /* The collision detect "heartbeat" signal was lost. */
#define ENTSR_OWC 0x80  /* There was an out-of-window collision. */

uint8_t ne2000_mac[6];

int eecounter = 0;
uint8_t ee[1024];

void printee(NE2000State *s) {
    for(int i=0;i<6;i++) {
        fprintf(stderr, "0x%02x\n", s->mem[i+4]);
    }
}

static void __not_in_flash_func(ne2000_eeprom)(NE2000State *s) {
     uint8_t mem[16] = { 1,2,3,4,
    0x28,
    0xcd,
    0xc1,
    0x04,
    0x59,
    0x41};
    static struct ee oldee;
    struct ee ee;
    ee.data = s->eeprom;
    if (ee.cs == 0) {
        oldee = ee;
        return;
    }
    static uint8_t bitcount;
    static bool valid_cmd;
    static u_int16_t cmd;
    static uint16_t addr;
    static uint16_t v;
    if ((oldee.cs == 0) && (ee.cs == 1)) {
        bitcount = 0;
        valid_cmd = false;
        cmd = 0;
        addr = 0;
        v = 0;
        oldee = ee;
        return;
    }
    if ((oldee.clk == 0) && (ee.clk == 1)) {
        bitcount++;
        if (!valid_cmd) {
            cmd <<= 1;
            cmd |= ee.din ? 1 : 0;
        }
    }
    if (((oldee.clk == 0) && (ee.clk == 1))) {// || ((oldee.clk == 1) && (ee.clk == 0))) {
        if (bitcount == 10) { // 16 bits mode
            valid_cmd = true;
            addr = cmd & 0x3f;
            v = ((uint16_t)mem[(addr<<1)])<<8 | mem[(addr<<1)+1];
            //printf("addr %x cmd %x v %x\n", addr, cmd, v);
        }
        if (valid_cmd && (bitcount >= 11) && (cmd & 0x1c0) == 0x180) { // READ command
            uint32_t bit = v & (1<<(15-(bitcount-11)));
            s->eeprom &= 0xfe;
            s->eeprom |= bit ? 1 : 0;
        }
    }
    oldee = ee;
}

void __not_in_flash_func(ne2000_reset)(NE2000State *s)
{
    int i;

    memset(s, 0, sizeof(NE2000State));
    s->cmd = 0x21; // default reset value
    s->dcfg = 0x00;

    s->isr = ENISR_RESET;
    memcpy(s->mem, ne2000_mac, 6);
    s->mem[14] = 0x57;
    s->mem[15] = 0x57;

    /* duplicate prom data */
    for(i = 15;i >= 0; i--) {
        s->mem[2 * i] = s->mem[i];
        s->mem[2 * i + 1] = s->mem[i];
    }

  //  ne2000_post_load(s, 1);

}

static void __not_in_flash_func(ne2000_update_irq)(NE2000State *s)
{
    int isr;
    isr = (s->isr & s->imr) & 0x7f;
#if defined(DEBUG_NE2000)
    printf("NE2000: Set IRQ to %d (%02x %02x)\n",
           isr ? 1 : 0, s->isr, s->imr);
#endif
    //qemu_set_irq(s->irq, (isr != 0));
}

static int __not_in_flash_func(ne2000_buffer_full)(NE2000State *s)
{
    int avail, index, boundary;

    if (s->stop <= s->start) {
        return 1;
    }

    index = s->curpag << 8;
    boundary = s->boundary << 8;
    if (index < boundary)
        avail = boundary - index;
    else
        avail = (s->stop - s->start) - (index - boundary);
    if (avail < (MAX_ETH_FRAME_SIZE + 4))
        return 1;
    return 0;
}


#define ETH_ALEN 1504
unsigned int __not_in_flash_func(net_crc32)(const uint8_t *buf, size_t size) {
    return 0;
}

ssize_t __not_in_flash_func(ne2000_receive)(NE2000State *s, const uint8_t *buf, size_t size_)
{
    //NE2000State *s = qemu_get_nic_opaque(nc);
    size_t size = size_;
    uint8_t *p;
    unsigned int total_len, next, avail, len, index, mcast_idx;
    static const uint8_t broadcast_macaddr[6] =
        { 0xff, 0xff, 0xff, 0xff, 0xff, 0xff };

#if defined(DEBUG_NE2000)
    printf("NE2000: received len=%zu\n", size);
#endif

    if (s->cmd & E8390_STOP)
        return -1;
    if (ne2000_buffer_full(s))
        return -2;

    /* XXX: check this */
    s->rxcr |= 0x10;
    if (s->rxcr & 0x10) {
        /* promiscuous: receive all */
    } else {
        if (!memcmp(buf,  broadcast_macaddr, 6)) {
            /* broadcast address */
            if (!(s->rxcr & 0x04))
                return size;
        } else if (buf[0] & 0x01) {
            /* multicast */
            if (!(s->rxcr & 0x08))
                return size;
            mcast_idx = net_crc32(buf, ETH_ALEN) >> 26;
            if (!(s->mult[mcast_idx >> 3] & (1 << (mcast_idx & 7))))
                return size;
        } else if (s->mem[0] == buf[0] &&
                   s->mem[2] == buf[1] &&
                   s->mem[4] == buf[2] &&
                   s->mem[6] == buf[3] &&
                   s->mem[8] == buf[4] &&
                   s->mem[10] == buf[5]) {
            /* match */
        } else {
            return size;
        }
    }

    index = s->curpag << 8;
    if (index >= NE2000_PMEM_END) {
        index = s->start;
    }
    /* 4 bytes for header */
    total_len = size + 4;
    /* address for next packet (4 bytes for CRC) */
    next = index + ((total_len + 4 + 255) & ~0xff);
    if (next >= s->stop)
        next -= (s->stop - s->start);
    /* prepare packet header */
    p = s->mem + index;
    s->rsr = ENRSR_RXOK; /* receive status */
    /* XXX: check this */
    if (buf[0] & 0x01)
        s->rsr |= ENRSR_PHY;
    p[0] = s->rsr;
    p[1] = next >> 8;
    p[2] = total_len;
    p[3] = total_len >> 8;
    index += 4;

    /* write packet data */
    while (size > 0) {
        if (index <= s->stop)
            avail = s->stop - index;
        else
            break;
        len = size;
        if (len > avail)
            len = avail;
        memcpy(s->mem + index, buf, len);
        buf += len;
        index += len;
        if (index == s->stop)
            index = s->start;
        size -= len;
    }
    s->curpag = next >> 8;

    /* now we can signal we have received something */
    s->isr |= ENISR_RX;
    ne2000_update_irq(s);

    return size_;
}

static int eehackcnt;
static uint8_t eehack[] = {
0x8c,
0x8e,
0x8e,
0x8c,
0x8c,
0x8c,
0x8c,
0x8c,
0x8e,
//addr 2 cmd 182 v 28cd
0x8c,
0x8c,
0x8d,
0x8c,

0x8d,
0x8c,
0x8c,
0x8c,

0x8d,
0x8d,
0x8c,
0x8c,

0x8d,
0x8d,
0x8c,
0x8d,

0x8c,
0x8e,
0x8e,
0x8c,
0x8c,
0x8c,
0x8c,
0x8c,
0x8e,
//addr 3 cmd 183 v c104
0x8d,
0x8d,
0x8c,
0x8c,

0x8c,
0x8c,
0x8c,
0x8d,

0x8c,
0x8c,
0x8c,
0x8c,

0x8c,
0x8d,
0x8c,
0x8c,

0x8c,
0x8e,
0x8e,
0x8c,
0x8c,
0x8c,
0x8c,
0x8e,
0x8c,
//addr 4 cmd 184 v 5941
0x8c,
0x8d,
0x8c,
0x8d,
0x8d,
0x8c,
0x8c,
0x8d,
0x8c,
0x8d,
0x8c,
0x8c,
0x8c,
0x8c,
0x8c,
0x8d,
};

static inline void __not_in_flash_func(ne2000_ioport_write)(void *opaque, uint32_t addr, uint32_t val)
{
    NE2000State *s = opaque;
    int offset, page, index;

    addr &= 0xf;
    trace_ne2000_ioport_write(addr, val);
    if (addr == E8390_CMD) {
        /* control register */
        s->cmd = val;
        if (!(val & E8390_STOP)) { /* START bit makes no sense on RTL8029... */
            s->isr &= ~ENISR_RESET;
            /* test specific case: zero length transfer */
            if ((val & (E8390_RREAD | E8390_RWRITE)) &&
                s->rcnt == 0) {
                s->isr |= ENISR_RDC;
                ne2000_update_irq(s);
            }
            if (val & E8390_TRANS) {
                index = (s->tpsr << 8);
                /* XXX: next 2 lines are a hack to make netware 3.11 work */
                if (index >= NE2000_PMEM_END)
                    index -= NE2000_PMEM_SIZE;
                /* fail safe: check range on the transmitted length  */
                if (index + s->tcnt <= NE2000_PMEM_END) {
                    // qemu_send_packet(qemu_get_queue(s->nic), s->mem + index, s->tcnt);
                    txpktbuf[txpktcnt].len = s->tcnt;
                    memcpy(txpktbuf[txpktcnt].data, s->mem + index, s->tcnt);
                    if (txpktcnt < PKTBUFSZ) {
                        txpktcnt++;
                    }
                }
                /* signal end of transfer */
                s->tsr = ENTSR_PTX;
                s->isr |= ENISR_TX;
                s->cmd &= ~E8390_TRANS;
                ne2000_update_irq(s);
            }
        }
    } else {
        page = s->cmd >> 6;
        offset = addr | (page << 4);
        switch(offset) {
        case EN0_STARTPG:
            if (val << 8 <= NE2000_PMEM_END) {
                s->start = val << 8;
            }
            break;
        case EN0_STOPPG:
            if (val << 8 <= NE2000_PMEM_END) {
                s->stop = val << 8;
            }
            break;
        case EN0_BOUNDARY:
            if (val << 8 < NE2000_PMEM_END) {
                s->boundary = val;
            }
            break;
        case EN0_IMR:
            s->imr = val;
            ne2000_update_irq(s);
            break;
        case EN0_TPSR:
            s->tpsr = val;
            break;
        case EN0_TCNTLO:
            s->tcnt = (s->tcnt & 0xff00) | val;
            break;
        case EN0_TCNTHI:
            s->tcnt = (s->tcnt & 0x00ff) | (val << 8);
            break;
        case EN0_RSARLO:
            s->rsar = (s->rsar & 0xff00) | val;
            break;
        case EN0_RSARHI:
            s->rsar = (s->rsar & 0x00ff) | (val << 8);
            break;
        case EN0_RCNTLO:
            s->rcnt = (s->rcnt & 0xff00) | val;
            break;
        case EN0_RCNTHI:
            s->rcnt = (s->rcnt & 0x00ff) | (val << 8);
            break;
        case EN0_RXCR:
            s->rxcr = val;
            break;
        case EN0_DCFG:
            s->dcfg = val;
            break;
        case EN0_ISR:
            s->isr &= ~(val & 0x7f);
            ne2000_update_irq(s);
            break;
        case EN1_PHYS ... EN1_PHYS + 5:
            s->phys[offset - EN1_PHYS] = val;
            break;
        case EN1_CURPAG:
            if (val << 8 < NE2000_PMEM_END) {
                s->curpag = val;
            }
            break;
        case EN1_MULT ... EN1_MULT + 7:
            s->mult[offset - EN1_MULT] = val;
            break;
        case EN3_EEPROM:
            s->eeprom = (val & 0xfe) | (s->eeprom & 0x1);
            ne2000_eeprom(s);
            break;
        }
    }
}

static inline uint32_t __not_in_flash_func(ne2000_ioport_read)(void *opaque, uint32_t addr)
{
    NE2000State *s = opaque;
    int offset, page, ret;

    addr &= 0xf;
    if (addr == E8390_CMD) {
        ret = s->cmd;
    } else {
        page = s->cmd >> 6;
        offset = addr | (page << 4);
        switch(offset) {
        case EN0_TSR:
            ret = s->tsr;
            break;
        case EN0_BOUNDARY:
            ret = s->boundary;
            break;
        case EN0_ISR:
            ret = s->isr;
            break;
        case EN0_RSARLO:
            ret = s->rsar & 0x00ff;
            break;
        case EN0_RSARHI:
            ret = s->rsar >> 8;
            break;
        case EN1_PHYS ... EN1_PHYS + 5:
            ret = s->phys[offset - EN1_PHYS];
            break;
        case EN1_CURPAG:
            ret = s->curpag;
            break;
        case EN1_MULT ... EN1_MULT + 7:
            ret = s->mult[offset - EN1_MULT];
            break;
        case EN0_RSR:
            ret = s->rsr;
            break;
        case EN2_STARTPG:
            ret = s->start >> 8;
            break;
        case EN2_STOPPG:
            ret = s->stop >> 8;
            break;
        case EN0_RTL8029ID0:
            ret = 0x50;
            break;
        case EN0_RTL8029ID1:
            ret = 0x70; // 0x70=8019 
            break;
        case EN3_EEPROM:
            ret = s->eeprom;
            //printf("0x%02x,\n",ret);
            //ret = eehack[eehackcnt];
            //eehackcnt = (eehackcnt + 1) % sizeof(eehack);
            break;
        case EN3_CONFIG0:
            ret = 0;          /* 10baseT media */
            break;
        case EN3_CONFIG2:
            ret = 0x40;       /* 10baseT active */
            break;
        case EN3_CONFIG3:
            ret = 0x40;       /* Full duplex */
            break;
        default:
            ret = 0x00;
            break;
        }
    }
    trace_ne2000_ioport_read(addr, ret);
    return ret;
}

static inline void __not_in_flash_func(ne2000_mem_writeb)(NE2000State *s, uint32_t addr,
                                     uint32_t val)
{
    if (addr < 32 ||
        (addr >= NE2000_PMEM_START && addr < NE2000_MEM_SIZE)) {
        s->mem[addr] = val;
    }
}

static inline uint32_t __not_in_flash_func(ne2000_mem_readb)(NE2000State *s, uint32_t addr)
{
    if (addr < 32 ||
        (addr >= NE2000_PMEM_START && addr < NE2000_MEM_SIZE)) {
        return s->mem[addr];
    } else {
        return 0xff;
    }
}

static inline void __not_in_flash_func(ne2000_dma_update)(NE2000State *s, int len)
{
    s->rsar += len;
    /* wrap */
    /* XXX: check what to do if rsar > stop */
    if (s->rsar == s->stop)
        s->rsar = s->start;

    if (s->rcnt <= len) {
        s->rcnt = 0;
        /* signal end of transfer */
        s->isr |= ENISR_RDC;
        ne2000_update_irq(s);
    } else {
        s->rcnt -= len;
    }
}

static void __not_in_flash_func(ne2000_asic_ioport_write)(void *opaque, uint32_t addr, uint32_t val)
{
    NE2000State *s = opaque;

#ifdef DEBUG_NE2000
    printf("NE2000: asic write val=0x%04x\n", val);
#endif
    if (s->rcnt == 0)
        return;
    /* 8 bit access */
    ne2000_mem_writeb(s, s->rsar, val);
    ne2000_dma_update(s, 1);
}

static uint32_t __not_in_flash_func(ne2000_asic_ioport_read)(void *opaque, uint32_t addr)
{
    NE2000State *s = opaque;
    int ret;

    /* 8 bit access */
    ret = ne2000_mem_readb(s, s->rsar);
    ne2000_dma_update(s, 1);
#ifdef DEBUG_NE2000
    printf("NE2000: asic read val=0x%04x\n", ret);
#endif
    return ret;
}

static void __not_in_flash_func(ne2000_reset_ioport_write)(void *opaque, uint32_t addr, uint32_t val)
{
    /* nothing to do (end of reset pulse) */
}

static uint32_t __not_in_flash_func(ne2000_reset_ioport_read)(void *opaque, uint32_t addr)
{
    NE2000State *s = opaque;
    ne2000_reset(s);
    return 0;
}

int __not_in_flash_func(ne2000_post_load)(void* opaque, int version_id)
{
    NE2000State* s = opaque;

    if (version_id < 2) {
        s->rxcr = 0x0c;
    }
    return 0;
}

uint64_t __not_in_flash_func(ne2000_read)(void *opaque, hwaddr addr,
                            unsigned size)
{
    NE2000State *s = opaque;
    uint64_t val;

    if (addr < 0x10) {
        val = ne2000_ioport_read(s, addr);
    } else if (addr == 0x10) {
        val = ne2000_asic_ioport_read(s, addr);
    } else if (addr == 0x1f) {
        val = ne2000_reset_ioport_read(s, addr);
    }
    trace_ne2000_read(addr, val);

    return val;
}

void __not_in_flash_func(ne2000_write)(void *opaque, hwaddr addr,
                         uint64_t data, unsigned size)
{
    NE2000State *s = opaque;

    trace_ne2000_write(addr, data);
    if (addr < 0x10) {
        ne2000_ioport_write(s, addr, data);
    } else if (addr == 0x10) {
        ne2000_asic_ioport_write(s, addr, data);
    } else if (addr == 0x1f) {
        ne2000_reset_ioport_write(s, addr, data);
    }
}

/*
static const MemoryRegionOps ne2000_ops = {
    .read = ne2000_read,
    .write = ne2000_write,
    .endianness = DEVICE_LITTLE_ENDIAN,
};
*/
/***********************************************************/
/* PCI NE2000 definitions */

/*
void ne2000_setup_io(NE2000State *s, DeviceState *dev, unsigned size)
{
    memory_region_init_io(&s->io, OBJECT(dev), &ne2000_ops, s, "ne2000", size);
}
*/