//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: communication_protocol.c
//	ģ�鹦��	: ͨѶЭ��C�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************
#include "boot_core.h"
#include "communication_protocol.h"

uint8_t CommunicationCheckNumber;			//У��λ
commu_length_t CmmuLength;						//�������ݳ���
uint8_t CMDBuff;								//����洢����
commu_data_t CommuData[CommunicationLength1];	//ͨѶ���ջ���
commu_data_t CmdSendData[CommunicationLength1];	//���ͻ���
commu_length_t CmmuSendLength;		            //�������ݳ���
uint8_t CRCchecksum[4];

#define SEND_ADDRESS 0x01
#define SEND_BMS_TYPE 0x01
#define SEND_SHAKE_1 0x55
#define SEND_SHAKE_2 0xAA
extern volatile uint8_t ACK;
//����һ֡����
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data)
{
	uint8_t i;
	uint8_t check_sum = 0;
	UartSendOneByte(SEND_ADDRESS);	//����֡ͷ
	
	UartSendOneByte((Data_len + 5) >> 8);		 				 		//���������򳤶ȸ�8λ
	UartSendOneByte(Data_len + 5);		 			 	//���������򳤶ȵ�8λ
	UartSendOneByte(SEND_BMS_TYPE);					//���͵���������
	UartSendOneByte(Command);					 	//���Ϳ�����
	UartSendOneByte(SEND_SHAKE_1);					//������1
	UartSendOneByte(SEND_SHAKE_2);					//������1
	UartSendOneByte(ACK);							//����Ӧ����
	check_sum = ((Data_len + 5) >> 8) + (Data_len + 5) + SEND_BMS_TYPE + Command + SEND_SHAKE_1 + SEND_SHAKE_2 + ACK;
	for(i=0;i<Data_len;i++)	  					 	//����������
	{
		UartSendOneByte(*(Data+i));
		check_sum+=	*(Data+i);
	}	
	UartSendOneByte(check_sum);						//����У��λ��8λ
	// UartSendOneByte(CommunicationCommandEnd);		//����֡β   
}
#define TYPE_TO_SHAKE_LENTH 4
uint8_t AnalysisData()//��������֡������
{
	uint8_t cmd = NO_CMD;
    uint8_t data_len;
	uint8_t i;
	uint8_t check_sum = 0;
	data_len = CommuData[1] * 0x100 + CommuData[2];

	cmd = CommuData[4];
	//���㵥�����͵�����λ��У���
	for(i=1; i<data_len + 3; i++)
	{
	   check_sum+=CommuData[i];
	}
	if((cmd&0x80) != 0) {
		ACK = ERR_CMD_ID;
	}
	//У��ɹ�,��ȡ������
	if(check_sum!=(CommuData[3 + data_len]))
	{
        cmd = ERROR_CHECK_FAIL;
	}
	CmmuLength = data_len - TYPE_TO_SHAKE_LENTH;//ȡ����

    return cmd;
}

void ClearCommu()
{
    CommuData[0] = 0; //�������������ͷ��׼���´δ������ݵ���
    CmmuReadNumber = 0; //���¼�����׼���´δ������ݵ���
    UartReceFlag = 0; //���������ɱ�־
}




