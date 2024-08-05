//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: serial_port_config.c
//	模块功能	: 串口设置C文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#include "communication_protocol.h"

uint8_t CmmuReadNumber;	//通讯当前读取数据为一帧中的第几个数
uint8_t UartReceFlag;				//UART0接收完一帧标志位
uint8_t UartSendFlag;				//UART0发送完一Byte标志位

#define BIT7 0x80
#define BIT6 0x40
#define BIT5 0x20
#define BIT4 0x10
#define BIT3 0x08
#define BIT2 0x04
#define BIT1 0x02
#define BIT0 0x01

void UartInit(uint32_t baud)
{
    SCI0_Init();
    /* UART0 Start, Setting baud rate */
    UART0_BaudRate(Fsoc, baud);
    UART0_Start();
}

void UartSendOneByte(uint8_t input_data)
{
    SCI0->TXD0 = input_data;
    while(!UartSendFlag);
    UartSendFlag = 0;
//    while (SCI0->SSR00 & (_0040_SCI_UNDER_EXECUTE | _0020_SCI_VALID_STORED))
//    {
//        ;
//    }
}
#define SLAVE_ADDRESS 0x01//设备地址
void UartReceData()//接收数据帧
{
	if(!UartReceFlag)
	{		
        CommuData[CmmuReadNumber] = SCI0->RXD0;		//将接收数据载入缓存
		if(CommuData[0] == SLAVE_ADDRESS)
		{
			CmmuReadNumber++;
		}
		if(CmmuReadNumber >= 3) {
			if(CmmuReadNumber>=(3 + CommuData[1] * 0x100 + CommuData[2] + 1)) //数据数量超过256的话需要修改CmmuReadNumber类型
			{
				/* 开启看门狗和清狗 */
				UartReceFlag = 1;	  //表示接收到一帧数据
			}
		}

	}
}

//void UART_Work() 
//{	
//	if(READ_TI)
//	{
//		CLEAR_TI;			 //中断进来要把TI清0
//		UartSendFlag=1; 	 //BootLoader发送标志
//	}
//	if(READ_RI)		
//	{
//		CLEAR_RI;			//中断进来要把RI清0
//		UartReceData();
//	}  
//}

