//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: communication_protocol.c
//	模块功能	: 通讯协议C文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#include "boot_core.h"
#include "communication_protocol.h"

uint8_t CommunicationCheckNumber;			//校验位
commu_length_t CmmuLength;						//接收数据长度
uint8_t CMDBuff;								//命令存储缓存
commu_data_t CommuData[CommunicationLength1];	//通讯接收缓存
commu_data_t CmdSendData[CommunicationLength1];	//发送缓存
commu_length_t CmmuSendLength;		            //发送数据长度
uint8_t CRCchecksum[4];

#define SEND_ADDRESS 0x01
#define SEND_BMS_TYPE 0x01
#define SEND_SHAKE_1 0x55
#define SEND_SHAKE_2 0xAA
extern volatile uint8_t ACK;
//发送一帧命令
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data)
{
	uint8_t i;
	uint8_t check_sum = 0;
	UartSendOneByte(SEND_ADDRESS);	//发送帧头
	
	UartSendOneByte((Data_len + 5) >> 8);		 				 		//发送数据域长度高8位
	UartSendOneByte(Data_len + 5);		 			 	//发送数据域长度低8位
	UartSendOneByte(SEND_BMS_TYPE);					//发送单板类型码
	UartSendOneByte(Command);					 	//发送控制码
	UartSendOneByte(SEND_SHAKE_1);					//握手字1
	UartSendOneByte(SEND_SHAKE_2);					//握手字1
	UartSendOneByte(ACK);							//发送应答码
	check_sum = ((Data_len + 5) >> 8) + (Data_len + 5) + SEND_BMS_TYPE + Command + SEND_SHAKE_1 + SEND_SHAKE_2 + ACK;
	for(i=0;i<Data_len;i++)	  					 	//发送数据域
	{
		UartSendOneByte(*(Data+i));
		check_sum+=	*(Data+i);
	}	
	UartSendOneByte(check_sum);						//发送校验位低8位
	// UartSendOneByte(CommunicationCommandEnd);		//发送帧尾   
}
#define TYPE_TO_SHAKE_LENTH 4
uint8_t AnalysisData()//分析接收帧的数据
{
	uint8_t cmd = NO_CMD;
    uint8_t data_len;
	uint8_t i;
	uint8_t check_sum = 0;
	data_len = CommuData[1] * 0x100 + CommuData[2];

	cmd = CommuData[4];
	//计算单板类型到数据位的校验和
	for(i=1; i<data_len + 3; i++)
	{
	   check_sum+=CommuData[i];
	}
	if((cmd&0x80) != 0) {
		ACK = ERR_CMD_ID;
	}
	//校验成功,提取控制码
	if(check_sum!=(CommuData[3 + data_len]))
	{
        cmd = ERROR_CHECK_FAIL;
	}
	CmmuLength = data_len - TYPE_TO_SHAKE_LENTH;//取长度

    return cmd;
}

void ClearCommu()
{
    CommuData[0] = 0; //清除缓冲区数据头，准备下次串口数据到来
    CmmuReadNumber = 0; //重新计数，准备下次串口数据到来
    UartReceFlag = 0; //清除传输完成标志
}




