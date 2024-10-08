//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: seroal_port_config.h
//	ģ�鹦��	: uartͨѶͷ�ļ�
//  ��������	: 2021/12/1
// 	�汾		: V0.1
//************************************************************
#ifndef _SERIAL_PORT_CONFIG_H_
#define _SERIAL_PORT_CONFIG_H_
#include "BAT32G137.h"
#include "userdefine.h"
#include "cg_sci.h"
#include "cg_macrodriver.h"
//UART���ýӿ�
#define UartBaud				25600		    	 //��ʼĬ�ϲ�����
#define	Fsoc					48000000	    	//��Ƶѡ��
#define CommunicationIOInit()	GPIO_SET_MUX_MODE(P23CFG,GPIO_MUX_TXD0);GPIO_SET_MUX_MODE(P24CFG,GPIO_MUX_RXD0)//ͨѶIO����
//**********************UARTͨѶ�ӿ�**********************************
extern uint8_t UartReceFlag;		  			//UART0������һ֡��־λ
extern uint8_t UartSendFlag;		  			//UART0������һByte��־λ
extern uint32_t CmmuReadNumber;
//*********************************************************************
void UartInit(uint32_t baud);
void UartSendOneByte(uint8_t input_data);

#endif
