//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: base_time_system.h
//	ģ�鹦��	: ��ʱ������C�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************
#ifndef _BASE_TIME_SYSTEM_H_
#define _BASE_TIME_SYSTEM_H_
#include "BAT32G137.h"
#include "flash_operate.h"
#include "boot_core.h"
#include "cg_tma.h"

void BaseTimeSystemInit(uint8_t enable_disable);
void BaseTimeSystemScan(void);

#endif
