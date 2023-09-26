#include "include/firmware_floppyemul.h"

const uint16_t floppyemulROM[] = {
    0xABCD, 0xEF42, 0x0000, 0x0000, 0x40FA, 0x001E, 0x00FA, 0x001E, 0x5C8C, 0x573A, 0x0000, 0x0022, 0x454D, 0x554C, 0x0000, 0x7E3F,
    0x3F3C, 0x0025, 0x4E4E, 0x548F, 0x51CF, 0xFFF6, 0x4879, 0x000B, 0xFFFF, 0x4E4D, 0x588F, 0x0800, 0x0001, 0x6700, 0x0004, 0x4E75,
    0x2F3C, 0x00FA, 0x0454, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x2F3C, 0x00FA, 0x046F, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x6100, 0x0082,
    0x4A40, 0x6718, 0x2F3C, 0x00FA, 0x0489, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x3F3C, 0x0007, 0x4E41, 0x548F, 0x4E75, 0x2F3C, 0x00FA,
    0x04B5, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x6100, 0x0074, 0x4A40, 0x6702, 0x4E75, 0x2F3C, 0x00FA, 0x04F3, 0x3F3C, 0x0009, 0x4E41,
    0x5C8F, 0x6100, 0x00BE, 0x4A40, 0x670A, 0x3F3C, 0x0007, 0x4E41, 0x548F, 0x4E75, 0x2F3C, 0x00FA, 0x04D1, 0x3F3C, 0x0009, 0x4E41,
    0x5C8F, 0x6100, 0x007C, 0x2F3C, 0x00FA, 0x0520, 0x3F3C, 0x0009, 0x4E41, 0x5C8F, 0x3F3C, 0x0007, 0x4E41, 0x548F, 0x6000, 0x0092,
    0x343C, 0x0008, 0xE34A, 0x3F02, 0x303C, 0x0204, 0x323C, 0x0000, 0x6100, 0x0332, 0x341F, 0x4A40, 0x6704, 0x51CA, 0xFFEA, 0x4E75,
    0x303C, 0x0200, 0x323C, 0x0010, 0x2638, 0x0472, 0x2838, 0x0476, 0x2A38, 0x047E, 0x2C38, 0x00B8, 0x6100, 0x030A, 0x4E75, 0x21F9,
    0x00FA, 0xF02A, 0x0472, 0x21F9, 0x00FA, 0xF02E, 0x0476, 0x21F9, 0x00FA, 0xF032, 0x047E, 0x21F9, 0x00FA, 0xF026, 0x00B8, 0x4E75,
    0x21FC, 0x00FA, 0x01BE, 0x0472, 0x21FC, 0x00FA, 0x01E4, 0x0476, 0x21FC, 0x00FA, 0x0240, 0x047E, 0x21FC, 0x00FA, 0x0262, 0x00B8,
    0x4E75, 0x303C, 0x0201, 0x323C, 0x0000, 0x6100, 0x02B8, 0x4E75, 0x7001, 0x4278, 0x0446, 0x31FC, 0x0002, 0x04A6, 0x42B8, 0x045E,
    0x3F3C, 0x0001, 0x4267, 0x4267, 0x3F3C, 0x0001, 0x4267, 0x42A7, 0x4879, 0x0000, 0x2000, 0x3F3C, 0x0008, 0x4E4E, 0x4FEF, 0x0014,
    0x43F9, 0x0000, 0x2000, 0x2049, 0x323C, 0x00FF, 0x4282, 0xD459, 0x51C9, 0xFFFC, 0xB47C, 0x1234, 0x6602, 0x4ED0, 0x4E75, 0x302F,
    0x0004, 0xB079, 0x00FA, 0xF024, 0x6712, 0xB03C, 0x0001, 0x6604, 0x426F, 0x0004, 0x2079, 0x00FA, 0xF02A, 0x4ED0, 0x203C, 0x00FA,
    0xF004, 0x4E75, 0x302F, 0x000E, 0xB079, 0x00FA, 0xF024, 0x6712, 0xB03C, 0x0001, 0x6604, 0x426F, 0x000E, 0x207C, 0x00FA, 0xF02E,
    0x4ED0, 0x51C3, 0x206F, 0x0006, 0x2408, 0x4A82, 0x6700, 0x0020, 0x7000, 0x2200, 0x2400, 0x2800, 0x302F, 0x000C, 0x322F, 0x000A,
    0x3439, 0x00FA, 0xF004, 0x382F, 0x0004, 0x6100, 0x00C4, 0x4280, 0x4A03, 0x6602, 0x4E75, 0x4FEF, 0x0014, 0x4CDF, 0x02FE, 0x4E73,
    0x302F, 0x0004, 0xB079, 0x00FA, 0xF024, 0x6712, 0xB03C, 0x0001, 0x6604, 0x426F, 0x0004, 0x207C, 0x00FA, 0xF032, 0x4ED0, 0x7000,
    0x4E75, 0x204F, 0x0817, 0x0005, 0x6604, 0x4E68, 0x5D88, 0x3028, 0x0006, 0xB07C, 0x0008, 0x6706, 0xB07C, 0x0009, 0x660C, 0x0C68,
    0x0001, 0x0010, 0x660C, 0x4268, 0x0010, 0x2079, 0x00FA, 0xF026, 0x4ED0, 0x48E7, 0x7F40, 0x4FEF, 0xFFEC, 0x50C3, 0x5C88, 0x2F68,
    0x0002, 0x0006, 0x3F68, 0x0012, 0x000A, 0x4280, 0x3028, 0x000C, 0x5340, 0x4281, 0x3228, 0x000E, 0x4282, 0x3428, 0x0010, 0x4284,
    0x3839, 0x00FA, 0xF01A, 0xC2C4, 0xD041, 0x3839, 0x00FA, 0xF01C, 0xC4C4, 0xD042, 0x3F40, 0x000C, 0x0C50, 0x0009, 0x6708, 0x426F,
    0x0004, 0x6000, 0xFF20, 0x3F7C, 0x0001, 0x0004, 0x6000, 0xFF16, 0x4A44, 0x6600, 0x0008, 0x2248, 0x6100, 0x0004, 0x4E75, 0x5341,
    0x6100, 0x0008, 0x51C9, 0xFFFA, 0x4E75, 0x48A7, 0x5000, 0x3802, 0x3600, 0x4843, 0x3602, 0x303C, 0x0202, 0x323C, 0x0004, 0x6100,
    0x0104, 0x6610, 0x227C, 0x00FA, 0xF036, 0x5344, 0x10D9, 0x51CC, 0xFFFC, 0x4240, 0x4C9F, 0x000A, 0x4E75, 0x48E7, 0xFFC0, 0x4840,
    0x1039, 0x00FB, 0xABCD, 0x207C, 0x00FB, 0x0000, 0x4240, 0x4840, 0x2248, 0xD2C0, 0x1011, 0x2008, 0x8041, 0x2240, 0x1011, 0x4A41,
    0x6700, 0x00BA, 0x2008, 0x8042, 0x2240, 0x1011, 0xB27C, 0x0002, 0x6700, 0x00AA, 0x4842, 0x2008, 0x8042, 0x2240, 0x1011, 0xB27C,
    0x0004, 0x6700, 0x0098, 0x2008, 0x8043, 0x2240, 0x1011, 0xB27C, 0x0006, 0x6700, 0x0088, 0x4843, 0x2008, 0x8043, 0x2240, 0x1011,
    0xB27C, 0x0008, 0x6700, 0x0076, 0x2008, 0x8044, 0x2240, 0x1011, 0xB27C, 0x000A, 0x6700, 0x0066, 0x4844, 0x2008, 0x8044, 0x2240,
    0x1011, 0xB27C, 0x000C, 0x6754, 0x2008, 0x8045, 0x2240, 0x1011, 0xB27C, 0x000E, 0x6746, 0x4845, 0x2008, 0x8045, 0x2240, 0x1011,
    0xB27C, 0x0010, 0x6736, 0x2008, 0x8046, 0x2240, 0x1011, 0xB27C, 0x0012, 0x6728, 0x4846, 0x2008, 0x8046, 0x2240, 0x1011, 0xB27C,
    0x0014, 0x6718, 0x2008, 0x8047, 0x2240, 0x1011, 0xB27C, 0x0016, 0x670A, 0x4847, 0x2008, 0x8047, 0x2240, 0x1011, 0x4CDF, 0x03FF,
    0x4240, 0x4E75, 0x243C, 0x1284, 0xFBCD, 0xE09A, 0x9478, 0x0466, 0xD439, 0x00FF, 0x8209, 0x5841, 0x6100, 0xFF00, 0x3E3C, 0xFFFF,
    0xB4B9, 0x00FA, 0xF000, 0x6708, 0x51CF, 0xFFF6, 0x70FF, 0x4E75, 0x4240, 0x4E75, 0x5369, 0x6465, 0x6361, 0x7254, 0x2046, 0x6C6F,
    0x7070, 0x7920, 0x456D, 0x756C, 0x6174, 0x6F72, 0x0D0A, 0x002B, 0x2D20, 0x4C6F, 0x6164, 0x696E, 0x6720, 0x7468, 0x6520, 0x696D,
    0x6167, 0x652E, 0x2E2E, 0x0D0A, 0x0045, 0x7272, 0x6F72, 0x2063, 0x6F6D, 0x6D75, 0x6E69, 0x6361, 0x7469, 0x6E67, 0x2077, 0x6974,
    0x6820, 0x5369, 0x6465, 0x6361, 0x7274, 0x2E20, 0x5265, 0x7365, 0x7421, 0x0D0A, 0x002B, 0x2D20, 0x5361, 0x7669, 0x6E67, 0x2074,
    0x6865, 0x206F, 0x6C64, 0x2076, 0x6563, 0x746F, 0x7273, 0x0D0A, 0x002B, 0x2D20, 0x496E, 0x6974, 0x6961, 0x6C69, 0x7A69, 0x6E67,
    0x2074, 0x6865, 0x206E, 0x6577, 0x2076, 0x6563, 0x746F, 0x7273, 0x0D0A, 0x002B, 0x2D20, 0x5365, 0x7474, 0x696E, 0x6720, 0x7570,
    0x2074, 0x6865, 0x2042, 0x5042, 0x206F, 0x6620, 0x7468, 0x6520, 0x656D, 0x756C, 0x6174, 0x6564, 0x2064, 0x6973, 0x6B0D, 0x0A00,
    0x2B2D, 0x2042, 0x6F6F, 0x7469, 0x6E67, 0x2066, 0x726F, 0x6D20, 0x7468, 0x6520, 0x656D, 0x756C, 0x6174, 0x6564, 0x2064, 0x6973,
    0x6B0D, 0x0A00
};
uint16_t floppyemulROM_length = sizeof(floppyemulROM) / sizeof(floppyemulROM[0]);

