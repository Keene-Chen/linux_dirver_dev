#ifndef __FONT_1_H__
#define __FONT_1_H__

const unsigned char hello_world_16x16[][32] = {
    { 0x00, 0x00, 0x80, 0xE0, 0xFC, 0x1C, 0x88, 0xE0, 0x7C, 0x3C, 0xE8,
      0xE0, 0xA0, 0xF0, 0x70, 0x20, 0x00, 0x03, 0x03, 0x7F, 0x7F, 0x31,
      0x1D, 0x2F, 0x27, 0x60, 0x7F, 0x7F, 0x02, 0x1E, 0x1C, 0x18 }, /*"你",0*/
    { 0x00, 0x40, 0x40, 0xC0, 0xF8, 0xF8, 0xE0, 0xE0, 0x50, 0x10, 0x10,
      0xD0, 0xF0, 0x38, 0x98, 0x90, 0x00, 0x40, 0x44, 0x67, 0x3F, 0x1C,
      0x3F, 0x33, 0x21, 0x21, 0x61, 0x7F, 0x7F, 0x01, 0x01, 0x01 }, /*"好",1*/
    { 0x80, 0x80, 0x80, 0xF0, 0xF0, 0x80, 0x80, 0xF8, 0xF8, 0x80, 0x80,
      0xFC, 0xFC, 0xC0, 0xC0, 0xC0, 0x00, 0x00, 0x00, 0x7F, 0x7F, 0x20,
      0x20, 0x2F, 0x2F, 0x24, 0x24, 0x2F, 0x2F, 0x30, 0x30, 0x20 }, /*"世",2*/
    { 0x00, 0x00, 0x00, 0xF8, 0xF8, 0xA8, 0xA8, 0xF8, 0xF8, 0xA8, 0xA8,
      0xA8, 0xF8, 0xF8, 0x08, 0x00, 0x00, 0x08, 0x48, 0x4D, 0x65, 0x3E,
      0x3F, 0x0F, 0x01, 0x01, 0x7F, 0x7E, 0x05, 0x0D, 0x0C, 0x04 }
}; /*"界",3*/

const unsigned char hello_world_32x32[]
                                     [60] = {
                                         { 0x00, 0x00, 0x00, 0x00, 0xC0, 0xF8, 0x3C, 0x0C, 0x04,
                                           0xE0, 0xFC, 0x7C, 0x4C, 0x40, 0x40, 0x40, 0x40, 0xC0,
                                           0xC0, 0x00, 0x00, 0x10, 0x1C, 0x0F, 0xFF, 0xFF, 0x01,
                                           0x04, 0x87, 0xFB, 0x78, 0x10, 0xFF, 0xFF, 0x02, 0x20,
                                           0x61, 0xC1, 0x80, 0x00, 0x80, 0x80, 0x80, 0x80, 0xFF,
                                           0xFF, 0x88, 0x8E, 0x87, 0x81, 0xA0, 0xE0, 0xFF, 0xFF,
                                           0x80, 0x80, 0x80, 0x83, 0x87, 0x86 }, /*"你",0*/
                                         { 0x00, 0x80, 0x80, 0x80, 0xF8, 0xFC, 0x8C, 0x80, 0x80,
                                           0x80, 0x10, 0x10, 0x10, 0x10, 0x90, 0xF0, 0x78, 0x18,
                                           0x10, 0x00, 0x00, 0x00, 0xC0, 0xFC, 0x3F, 0x03, 0xE0,
                                           0xFF, 0x1F, 0x10, 0x10, 0x10, 0x10, 0xFF, 0xFF, 0x10,
                                           0x10, 0x18, 0x18, 0x10, 0x80, 0xC0, 0xE0, 0xB1, 0x99,
                                           0x8F, 0x87, 0x8F, 0x9C, 0x9C, 0xA0, 0xA0, 0xE0, 0xFF,
                                           0xFF, 0x80, 0x80, 0x80, 0x80, 0x80 }, /*"好",1*/
                                         { 0x00, 0x00, 0x00, 0x00, 0xF0, 0xF0, 0x10, 0x00, 0xFC,
                                           0xFC, 0xFC, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x04, 0x80,
                                           0x80, 0x00, 0x00, 0x02, 0x02, 0x02, 0xFF, 0xFF, 0x02,
                                           0x02, 0xFF, 0xFF, 0xFF, 0x02, 0x02, 0x02, 0xFF, 0xFF,
                                           0x03, 0x01, 0x01, 0x00, 0x80, 0x80, 0x80, 0xA0, 0xFF,
                                           0xFF, 0xA0, 0xA0, 0xA7, 0xA7, 0xA7, 0xA1, 0xA1, 0xA1,
                                           0xA7, 0xA7, 0xA0, 0xB0, 0xB0, 0xA0 }, /*"世",2*/
                                         { 0x00, 0x00, 0x00, 0x00, 0xFC, 0xFC, 0x48, 0x48, 0x48,
                                           0x48, 0xF8, 0xF8, 0x48, 0x48, 0x48, 0xFC, 0xFC, 0x00,
                                           0x00, 0x00, 0x00, 0x00, 0x00, 0x80, 0x87, 0xC7, 0x62,
                                           0xF2, 0xFE, 0xCE, 0x07, 0x07, 0xCE, 0xDA, 0x72, 0x67,
                                           0xC7, 0xC0, 0xC0, 0x40, 0x80, 0x80, 0x81, 0xC1, 0xE0,
                                           0xB0, 0xB8, 0x9F, 0x87, 0x80, 0x80, 0x80, 0xFF, 0xFF,
                                           0xC0, 0x80, 0x80, 0x80, 0x80, 0x80 }, /*"界",3*/

                                     };

#endif // __FONT_1_H__