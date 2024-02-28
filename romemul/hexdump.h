#ifndef __HEXDUMP_H_
#define __HEXDUMP_H_
#include <stdio.h>
#include <ctype.h>
 
#ifndef HEXDUMP_COLS
#define HEXDUMP_COLS 16
#endif

#ifndef hexdump_printf
#define hexdump_printf(fmt, ...) do { fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)
#endif

#ifndef hexdump_endline_printf
#define hexdump_endline_printf(fmt, ...) do { fprintf(stderr, fmt, ##__VA_ARGS__); } while (0)
#endif

static void hexdump(void *mem, unsigned int len){
        unsigned int i, j;
	for(i = 0; i < len + ((len % HEXDUMP_COLS) ? (HEXDUMP_COLS - len % HEXDUMP_COLS) : 0); i++)        {
                /* print offset */
                if(i % HEXDUMP_COLS == 0) {
                        hexdump_printf("0x%06x: ", i);
                }
 
                /* print hex data */
                if(i < len)               {
                        hexdump_printf("%02x ", 0xFF & ((char*)mem)[i]);
                }
                else { /* end of block, just aligning for ASCII dump */
                        hexdump_printf("   ");
                }
                
                /* print ASCII dump */
                if(i % HEXDUMP_COLS == (HEXDUMP_COLS - 1))                {
                        for(j = i - (HEXDUMP_COLS - 1); j <= i; j++)                        {
                                if(j >= len) {/* end of block, not really printing */
                                        hexdump_printf(" ");
                                }
                                else if(isprint(((char*)mem)[j])) /* printable char */
                                {
                                        hexdump_printf("%c", 0xFF & ((char*)mem)[j]);        
                                }
                                else { /* other char */
                                        hexdump_printf(".");
                                }
                        }
                        hexdump_endline_printf("\n");
                }
        }
}
#endif
