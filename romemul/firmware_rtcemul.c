#include "include/firmware_rtcemul.h"

const uint16_t rtcemulROM[] = {
    0xABCD, 0xEF42, 0x0000, 0x0000, 0x08FA, 0x001E, 0x0000, 0x0000, 0x6081, 0x577E, 0x0000, 0x0022, 0x454D, 0x554C, 0x0000, 0x7E3F,
    0x3F3C, 0x0025, 0x4E4E, 0x548F, 0x51CF, 0xFFF6, 0x4879, 0x000B, 0xFFFF, 0x4E4D, 0x588F, 0x0800, 0x0001, 0x6700, 0x0004, 0x4E75,
    0x4879, 0x00FA, 0x0372, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x4879, 0x00FA, 0x03AD, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x6100, 0x0042,
    0x4A40, 0x6626, 0x4879, 0x00FA, 0x03C9, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x6100, 0x011E, 0x4A40, 0x6610, 0x4879, 0x00FA, 0x03E7,
    0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x4E75, 0x4879, 0x00FA, 0x03FF, 0x3F3C, 0x0009, 0x4E41, 0x3F3C, 0x0007, 0x4E41, 0x508F, 0x4E75,
    0x3E3C, 0x003C, 0x3F07, 0x303C, 0x0300, 0x323C, 0x0000, 0x6100, 0x019E, 0x3E1F, 0x4A40, 0x6660, 0x0C79, 0xFFFF, 0x00FB, 0x0008,
    0x6604, 0x7000, 0x4E75, 0x3007, 0x0280, 0x0000, 0x00FF, 0x80FC, 0x000A, 0x0280, 0x00FF, 0x00FF, 0x0680, 0x0030, 0x0030, 0x4840,
    0x3F00, 0x4840, 0x3F00, 0x3F3C, 0x0002, 0x4E41, 0x588F, 0x3F3C, 0x0002, 0x4E41, 0x588F, 0x4879, 0x00FA, 0x0428, 0x3F3C, 0x0009,
    0x4E41, 0x5C8F, 0x3C3C, 0x0032, 0x3F3C, 0x0025, 0x4E4E, 0x548F, 0x51CE, 0xFFF6, 0x51CF, 0xFF8E, 0x70FF, 0x4E75, 0x303C, 0x0302,
    0x323C, 0x0004, 0x2638, 0x00B8, 0x6100, 0x0124, 0x4A40, 0x660A, 0x21FC, 0x00FA, 0x013E, 0x00B8, 0x4E75, 0x70FF, 0x4E75, 0x204F,
    0x4A79, 0x0000, 0x059E, 0x6702, 0x5449, 0x0817, 0x0005, 0x6604, 0x4E68, 0x5D88, 0x3028, 0x0006, 0xB07C, 0x0017, 0x670E, 0xB07C,
    0x0016, 0x6718, 0x2079, 0x00FB, 0x000E, 0x4ED0, 0x3F3C, 0x0017, 0x4E4E, 0x548F, 0xD0BC, 0x3C00, 0x0000, 0x4E73, 0x2028, 0x0002,
    0x90BC, 0x3C00, 0x0000, 0x2F00, 0x3F3C, 0x0016, 0x4E4E, 0x5C8F, 0x4E73, 0x303C, 0x0301, 0x323C, 0x0000, 0x6100, 0x00B2, 0x4A40,
    0x6600, 0xFF98, 0x4879, 0x00FB, 0x000A, 0x3F3C, 0x0006, 0x3F3C, 0x0019, 0x4E4E, 0x508F, 0x7000, 0x3F3C, 0x0017, 0x4E4E, 0x548F,
    0xD0BC, 0x3C00, 0x0000, 0x2E00, 0x3F07, 0x3F3C, 0x002D, 0x4E41, 0x588F, 0x4847, 0x3F07, 0x3F3C, 0x002B, 0x4E41, 0x588F, 0x7000,
    0x4E75, 0x48E7, 0xFF00, 0x2C00, 0xE19E, 0x7207, 0x2606, 0x0801, 0x0000, 0x6704, 0xE88B, 0x6002, 0xE19E, 0x0283, 0x0000, 0x000F,
    0x0C83, 0x0000, 0x000A, 0x6D08, 0x0683, 0x0000, 0x0037, 0x6006, 0x0683, 0x0000, 0x0030, 0x3F03, 0x3F3C, 0x0002, 0x4E41, 0x588F,
    0x51C9, 0xFFCA, 0x4CDF, 0x00FF, 0x4E75, 0x2E3C, 0x0000, 0x00C2, 0x4FEF, 0xFF3E, 0x244F, 0x43F9, 0x00FA, 0x028A, 0xE44F, 0x5347,
    0x24D9, 0x51CF, 0xFFFC, 0x4E93, 0x4FEF, 0x00C2, 0x4E75, 0x2E3C, 0x0000, 0x00F6, 0x4FEF, 0xFF0A, 0x244F, 0x264F, 0x43F9, 0x00FA,
    0x027A, 0xE44F, 0x5347, 0x24D9, 0x51CF, 0xFFFC, 0x377C, 0x4E71, 0x00D0, 0x4E93, 0x4FEF, 0x00F6, 0x4E75, 0x2439, 0x00FB, 0x0004,
    0xC4FC, 0x00DD, 0xD43C, 0x0035, 0x5841, 0x207C, 0x00FB, 0x0000, 0x4840, 0x1039, 0x00FB, 0xABCD, 0x4840, 0x2248, 0xD2C0, 0x1011,
    0x2008, 0x8041, 0x2240, 0x1011, 0x4A41, 0x6700, 0x009C, 0x2008, 0x8042, 0x2240, 0x1011, 0xB27C, 0x0002, 0x6700, 0x008C, 0x4842,
    0x2008, 0x8042, 0x2240, 0x1011, 0xB27C, 0x0004, 0x6700, 0x007A, 0x2008, 0x8043, 0x2240, 0x1011, 0xB27C, 0x0006, 0x6700, 0x006A,
    0x4843, 0x2008, 0x8043, 0x2240, 0x1011, 0xB27C, 0x0008, 0x6700, 0x0058, 0x2008, 0x8044, 0x2240, 0x1011, 0xB27C, 0x000A, 0x6700,
    0x0048, 0x4844, 0x2008, 0x8044, 0x2240, 0x1011, 0xB27C, 0x000C, 0x6736, 0x2008, 0x8045, 0x2240, 0x1011, 0xB27C, 0x000E, 0x6728,
    0x4845, 0x2008, 0x8045, 0x2240, 0x1011, 0xB27C, 0x0010, 0x6718, 0x2008, 0x8046, 0x2240, 0x1011, 0xB27C, 0x0012, 0x670A, 0x4846,
    0x2008, 0x8046, 0x2240, 0x1011, 0x4842, 0x4E75, 0x2E3C, 0xFFFF, 0x000F, 0x4847, 0xB4B9, 0x00FB, 0x0000, 0x670E, 0x51CF, 0xFFF6,
    0x4847, 0x51CF, 0xFFEE, 0x70FF, 0x4E75, 0x4240, 0x4E75, 0x4E71, 0x4E75, 0x5369, 0x6465, 0x6361, 0x7254, 0x2052, 0x6561, 0x6C20,
    0x5469, 0x6D65, 0x2043, 0x6C6F, 0x636B, 0x202D, 0x2076, 0x302E, 0x302E, 0x320D, 0x0A2B, 0x0D0A, 0x002B, 0x2D20, 0x5365, 0x7420,
    0x7665, 0x6374, 0x6F72, 0x732E, 0x2E2E, 0x0D0A, 0x002B, 0x2D20, 0x5175, 0x6572, 0x7969, 0x6E67, 0x2061, 0x204E, 0x5450, 0x2073,
    0x6572, 0x7665, 0x722E, 0x2E2E, 0x000D, 0x0A2B, 0x2D20, 0x5365, 0x7474, 0x696E, 0x6720, 0x6461, 0x7465, 0x2061, 0x6E64, 0x2074,
    0x696D, 0x652E, 0x0D0A, 0x002B, 0x2D20, 0x4461, 0x7465, 0x2061, 0x6E64, 0x2074, 0x696D, 0x6520, 0x7365, 0x742E, 0x0D0A, 0x000D,
    0x0A53, 0x6964, 0x6563, 0x6172, 0x7420, 0x6572, 0x726F, 0x7220, 0x636F, 0x6D6D, 0x756E, 0x6963, 0x6174, 0x696F, 0x6E2E, 0x2052,
    0x6573, 0x6574, 0x210D, 0x0A00, 0x0808, 0x0000, 0x1B23, 0x1109, 0x1555, 0x3000
};
uint16_t rtcemulROM_length = sizeof(rtcemulROM) / sizeof(rtcemulROM[0]);

