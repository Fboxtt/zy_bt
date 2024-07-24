//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: base_time_system.c
//	ģ�鹦��	: ��ʱ������C�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************
#include "base_time_system.h"

#define	APROM_AREA	            0x55//APROM��
#define DATA_AREA               0xAA//DATA��
#define LDROM_AREA	            0X96//LDROM��

#define TIME_CHECK_BOOT_10MS  50//50*10=500 ��λ��500ms���ڽ��ܽ���BOOT��ָ��
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

