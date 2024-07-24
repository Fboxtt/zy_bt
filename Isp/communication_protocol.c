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

//发送一帧命令
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data)
{
	uint8_t i;
	uint8_t check_sum = 0;
	UartSendOneByte(CommunicationCommandHeader);	//发送帧头
	UartSendOneByte(Command);					 	//发送控制码
	UartSendOneByte(0);		 				 		//发送数据域长度高8位
	UartSendOneByte(Data_len);		 			 	//发送数据域长度低8位
	check_sum = CommunicationCommandHeader+Command+Data_len;
	for(i=0;i<Data_len;i++)	  					 	//发送数据域
	{
		UartSendOneByte(*(Data+i));
		check_sum+=	*(Data+i);
	}	
	UartSendOneByte(check_sum);						//发送校验位低8位
	UartSendOneByte(CommunicationCommandEnd);		//发送帧尾   
}

uint8_t AnalysisData()//分析接收帧的数据
{
	uint8_t cmd = NO_CMD;
    uint8_t data_len;
	uint8_t i;
	uint8_t check_sum = 0;
	data_len = CommuData[3];
	for(i=0;i<(data_len+4);i++)
	{
	   check_sum+=CommuData[i];
	}
	//校验成功,提取控制码
	if((check_sum==(CommuData[data_len+4]))&&(CommuData[3]<=64))
	{
		cmd = CommuData[1];		
		CmmuLength=CommuData[3];//取长度		
	}
	else
	{
        cmd = DEAL_FAIL;
	}
    return cmd;
}

void ClearCommu()
{
    CommuData[0] = 0;
    CmmuReadNumber = 0;
    UartReceFlag = 0;
}




