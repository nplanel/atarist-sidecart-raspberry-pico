#ifndef HW_NE2000_H
#define HW_NE2000_H

//#include "qemu/units.h"
//#include "net/net.h"
#include "include/netusbee.h"

#define KiB 1024

#define NE2000_PMEM_SIZE    (32 * KiB)
#define NE2000_PMEM_START   (16 * KiB)
#define NE2000_PMEM_END     (NE2000_PMEM_SIZE+NE2000_PMEM_START)
#define NE2000_MEM_SIZE     NE2000_PMEM_END


#define trace_ne2000_read(addr, val)
#define trace_ne2000_write(addr, val)
#define trace_ne2000_ioport_read(addr, val)
#define trace_ne2000_ioport_write(addr, val)

#define cpu_to_le16(val) (val)
#define le16_to_cpu(val) (val)
#define stl_le_p(addr, val) do { uint32_t *m =(uint32_t*)addr; *m = val; } while(0)
#define ldl_le_p(addr) ({ uint32_t *m =(uint32_t*)addr; *m; })

    struct ee {
        union {
            struct {
                uint8_t dout:1;
                uint8_t din:1;
                uint8_t clk:1;
                uint8_t cs:1;
                uint8_t _bit4:1;
                uint8_t _bit5:1;
                uint8_t emm0:1;
                uint8_t emm1:1;
            };
            uint8_t data;
        };
    };

typedef struct NE2000State {
    struct ee oldee;
    uint8_t bitcount;
    bool valid_cmd;
    u_int16_t eecmd;
    uint16_t addr;
    uint16_t v;

    //MemoryRegion io;
    uint8_t cmd;
    uint32_t start;
    uint32_t stop;
    uint8_t boundary;
    uint8_t tsr;
    uint8_t tpsr;
    uint16_t tcnt;
    uint16_t rcnt;
    uint32_t rsar;
    uint8_t rsr;
    uint8_t rxcr;
    uint8_t isr;
    uint8_t dcfg;
    uint8_t imr;
    uint8_t phys[6]; /* mac address */
    uint8_t curpag;
    uint8_t mult[8]; /* multicast mask array */
    //qemu_irq irq;
    //NICState *nic;
    //NICConf c;
    uint8_t mem[NE2000_MEM_SIZE];
    uint8_t eeprom;
} NE2000State;

extern uint8_t ne2000_mac[6];

//void ne2000_setup_io(NE2000State *s, DeviceState *dev, unsigned size);
//extern const VMStateDescription vmstate_ne2000;
void ne2000_reset(NE2000State *s);
//ssize_t ne2000_receive(NetClientState *nc, const uint8_t *buf, size_t size_);
ssize_t ne2000_receive(NE2000State *s, const uint8_t *buf, size_t size_);

typedef uint32_t hwaddr;

uint64_t ne2000_read(void *opaque, hwaddr addr, unsigned size);
void ne2000_write(void *opaque, hwaddr addr, uint64_t data, unsigned size);

#endif
