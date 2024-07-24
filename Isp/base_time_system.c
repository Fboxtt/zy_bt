//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: base_time_system.c
//	模块功能	: 定时器配置C文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#include "base_time_system.h"

#define	APROM_AREA	            0x55//APROM区
#define DATA_AREA               0xAA//DATA区
#define LDROM_AREA	            0X96//LDROM区

#define TIME_CHECK_BOOT_10MS  50//50*10=500 复位后500ms用于接受进入BOOT的指令
uint8_t Time_10ms_count = 0;
void BaseTimeSystemInit(uint8_t enable_disable)
{
    TMA0_Init();
    if(enable_disable)
    {
        TMA0_Start();
    }
    else
    {
        TMA0_Stop();
    }
        
}
void BaseTimeSystemScan()
{
    Time_10ms_count++;
    if(Time_10ms_count>=TIME_CHECK_BOOT_10MS)
        ResetFlag=1;
}

