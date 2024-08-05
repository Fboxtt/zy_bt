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

//����һ֡����
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data)
{
	uint8_t i;
	uint8_t check_sum = 0;
	UartSendOneByte(CommunicationCommandHeader);	//����֡ͷ
	UartSendOneByte(Command);					 	//���Ϳ�����
	UartSendOneByte(0);		 				 		//���������򳤶ȸ�8λ
	UartSendOneByte(Data_len);		 			 	//���������򳤶ȵ�8λ
	check_sum = CommunicationCommandHeader+Command+Data_len;
	for(i=0;i<Data_len;i++)	  					 	//����������
	{
		UartSendOneByte(*(Data+i));
		check_sum+=	*(Data+i);
	}	
	UartSendOneByte(check_sum);						//����У��λ��8λ
	UartSendOneByte(CommunicationCommandEnd);		//����֡β   
}
#define TYPE_TO_SHAKE_LENTH 4
uint8_t AnalysisData()//��������֡������
{
	uint8_t cmd = NO_CMD;
    uint8_t data_len;
	uint8_t i;
	uint8_t check_sum = 0;
	data_len = CommuData[1] * 0x100 + CommuData[2];

	cmd = CommuData[3];
	//���㵥�����͵�����λ��У���
	for(i=3; i<data_len; i++)
	{
	   check_sum+=CommuData[i];
	}
	if((CommuData[3]&0x80) != 0) {
		cmd = TYPE_FAIL;
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
    CommuData[0] = 0;
    CmmuReadNumber = 0;
    UartReceFlag = 0;
}




