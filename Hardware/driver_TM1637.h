#ifndef __DRIVER_YM1637_H__
#define __DRIVER_YM1637_H__
#include "ti_msp_dl_config.h"
// 0~9段码
//extern const uint8_t TM1637_SEG_TABLE[] =
//{
//    0x3F, //0
//    0x06, //1
//    0x5B, //2
//    0x4F, //3
//    0x66, //4
//    0x6D, //5
//    0x7D, //6
//    0x07, //7
//    0x7F, //8
//    0x6F  //9
//};
void TM1637_Init(void);
void TM1637_Display(uint16_t num);
#endif