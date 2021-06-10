
#ifndef DS1302_H
#define DS1302_H

//#include "stm32f10x.h"
#include "stm32f4xx_hal.h"
#include "main.h"

void InitClock(void);
void Read1302Ram(u8 *p, u8 add, u8 cnt);
void Write1302Ram(u8 *p, u8 add, u8 cnt);
void BurstRead1302Clock(u8 *p);
void BurstWrite1302Clock(u8 *p);
void BurstRead1302Ram(u8 *p);
void BurstWrite1302Ram(u8 *p);
//void DS1302_GetTime(u8 *time);
void Read1302Time(u8 *time);
void TestDS1302(void);
void Write1302ByteTime(u8 add, u8 data); //Byte

#endif
