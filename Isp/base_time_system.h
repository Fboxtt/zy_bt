//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: base_time_system.h
//	模块功能	: 定时器配置C文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
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
