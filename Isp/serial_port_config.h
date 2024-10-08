//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: seroal_port_config.h
//	模块功能	: uart通讯头文件
//  更正日期	: 2021/12/1
// 	版本		: V0.1
//************************************************************
#ifndef _SERIAL_PORT_CONFIG_H_
#define _SERIAL_PORT_CONFIG_H_
#include "BAT32G137.h"
#include "userdefine.h"
#include "cg_sci.h"
#include "cg_macrodriver.h"
//UART设置接口
#define UartBaud				25600		    	 //初始默认波特率
#define	Fsoc					48000000	    	//主频选择
#define CommunicationIOInit()	GPIO_SET_MUX_MODE(P23CFG,GPIO_MUX_TXD0);GPIO_SET_MUX_MODE(P24CFG,GPIO_MUX_RXD0)//通讯IO设置
//**********************UART通讯接口**********************************
extern uint8_t UartReceFlag;		  			//UART0接收完一帧标志位
extern uint8_t UartSendFlag;		  			//UART0发送完一Byte标志位
extern uint32_t CmmuReadNumber;
//*********************************************************************
void UartInit(uint32_t baud);
void UartSendOneByte(uint8_t input_data);

#endif
