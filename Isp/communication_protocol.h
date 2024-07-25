//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: serial_port_config.h
//	模块功能	: 串口设置H文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#ifndef _COMMUNICATION_PROTOCOL_H_
#define _COMMUNICATION_PROTOCOL_H_

#include "serial_port_config.h"//串口通讯底层驱动文件
/*************************通讯协议相关宏定义*******************************/
//帧格式：帧头+控制码+数据域长度(2Byte)+数据域+校验位(1Byte)+帧尾
/**************************************************************************/
#define CommunicationCommandHeader   0X68		//命令帧头
#define CommunicationCommandEnd		 0x16		//命令帧尾
#define CommunicationLength1        (64+7)

/* 依据芯片特性设计合适的数据类型 */
#define commu_bool_t uint8_t                //bool型数据类型
#define commu_data_t uint8_t           //数据对应的数据类型
#define commu_addr_t uint16_t           //地址对应的数据类型
#define commu_length_t uint16_t         //数据长度对应的数据类型
#define commu_cmd_t  uint8_t                //命令的数据类型

extern commu_length_t CmmuLength;		                //接收数据长度
extern commu_cmd_t CMDBuff;		                        //命令存储缓存
extern commu_data_t CommuData[CommunicationLength1];	//通讯接收缓存
extern commu_data_t CmdSendData[CommunicationLength1];  //发送缓存
extern commu_length_t CmmuSendLength;		            //接收数据长度
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data);    
uint8_t AnalysisData(void);
void ClearCommu(void);
void UartReceData(void);
#endif

