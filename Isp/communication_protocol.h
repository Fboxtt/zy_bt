//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: serial_port_config.h
//	ģ�鹦��	: ��������H�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************
#ifndef _COMMUNICATION_PROTOCOL_H_
#define _COMMUNICATION_PROTOCOL_H_

#include "serial_port_config.h"//����ͨѶ�ײ������ļ�
/*************************ͨѶЭ����غ궨��*******************************/
//֡��ʽ��֡ͷ+������+�����򳤶�(2Byte)+������+У��λ(1Byte)+֡β
/**************************************************************************/
#define CommunicationCommandHeader   0X68		//����֡ͷ
#define CommunicationCommandEnd		 0x16		//����֡β
#define CommunicationLength1        (64+7)

/* ����оƬ������ƺ��ʵ��������� */
#define commu_bool_t uint8_t                //bool����������
#define commu_data_t uint8_t           //���ݶ�Ӧ����������
#define commu_addr_t uint16_t           //��ַ��Ӧ����������
#define commu_length_t uint16_t         //���ݳ��ȶ�Ӧ����������
#define commu_cmd_t  uint8_t                //�������������

extern commu_length_t CmmuLength;		                //�������ݳ���
extern commu_cmd_t CMDBuff;		                        //����洢����
extern commu_data_t CommuData[CommunicationLength1];	//ͨѶ���ջ���
extern commu_data_t CmdSendData[CommunicationLength1];  //���ͻ���
extern commu_length_t CmmuSendLength;		            //�������ݳ���
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data);    
uint8_t AnalysisData(void);
void ClearCommu(void);
void UartReceData(void);
#endif

