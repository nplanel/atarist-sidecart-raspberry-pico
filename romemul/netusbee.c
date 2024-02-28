
#include "include/netusbee.h"
#include "include/network.h"

#include "hardware/structs/systick.h"

#define rtl8129_send_data(a,v) netusbee_send_data(a,v)

static inline void netusbee_send_data(uint32_t addr, uint16_t data) {
     volatile uint16_t *m = (volatile uint16_t *)addr;
     *m = ((uint16_t)data) << 8;
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
