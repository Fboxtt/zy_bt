//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: flash_operate.c
//	模块功能	: BootLoader IAP操作源文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#include "flash_operate.h"

#define EraseFlash()	  //擦除操作
#define WriteFlash()	   //写入操作
#define ReadFlash()		   //读出操作

#define BIT0	0x01
#define BIT1	0x02
#define BIT2	0x04
#define BIT3	0x08
#define BIT4	0x10
#define BIT5	0x20
#define BIT6	0x40
#define BIT7	0x80


