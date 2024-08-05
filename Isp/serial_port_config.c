//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: serial_port_config.c
//	ģ�鹦��	: ��������C�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************
#include "communication_protocol.h"

uint8_t CmmuReadNumber;	//ͨѶ��ǰ��ȡ����Ϊһ֡�еĵڼ�����
uint8_t UartReceFlag;				//UART0������һ֡��־λ
uint8_t UartSendFlag;				//UART0������һByte��־λ

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
#define SLAVE_ADDRESS 0x01//�豸��ַ
void UartReceData()//��������֡
{
	if(!UartReceFlag)
	{		
        CommuData[CmmuReadNumber] = SCI0->RXD0;		//�������������뻺��
		if(CommuData[0] == SLAVE_ADDRESS)
		{
			CmmuReadNumber++;
		}
		if(CmmuReadNumber >= 3) {
			if(CmmuReadNumber>=(3 + CommuData[1] * 0x100 + CommuData[2] + 1)) //������������256�Ļ���Ҫ�޸�CmmuReadNumber����
			{
				/* �������Ź����幷 */
				UartReceFlag = 1;	  //��ʾ���յ�һ֡����
			}
		}

	}
}

//void UART_Work() 
//{	
//	if(READ_TI)
//	{
//		CLEAR_TI;			 //�жϽ���Ҫ��TI��0
//		UartSendFlag=1; 	 //BootLoader���ͱ�־
//	}
//	if(READ_RI)		
//	{
//		CLEAR_RI;			//�жϽ���Ҫ��RI��0
//		UartReceData();
//	}  
//}

