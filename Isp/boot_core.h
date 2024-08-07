//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: boot_core.h
//	模块功能	: BootLoader核心
//  更正日期	: 2022/1/1
// 	版本		: V1.0
//************************************************************
#ifndef _BOOT_CORE_H_
#define _BOOT_CORE_H_
/* 请将需要的头文件导入 */
#include "BAT32G137.h"
#include "communication_protocol.h"//通讯协议头文件
#include "flash_operate.h"//Flash操作头文件
#include "serial_port_config.h"//串口通讯底层驱动文件
#include "base_time_system.h"
#include "tea.h"

/* 依据芯片特性设计合适的数据类型 */
#define boot_bool_t 	uint8_t         	//bool型数据类型
#define boot_data_t 	uint8_t    	        //数据对应的数据类型
#define boot_addr_t 	uint32_t    	    //地址对应的数据类型
#define boot_length_t 	uint16_t  		    //数据长度对应的数据类型
#define boot_cmd_t  	uint8_t         	//命令的数据类型
#define boot_flag_t 	uint8_t    	        //外部标志类型 

#define READ_FLASH_ENABLE						//使能后允许执行读FLASH操作
//#define ENCRYPT_ENABLE						    //使能通讯加密，使能后会对接收的更新数据进行解密操作	
//#define ENCRYPT_UID_ENABLE					    //使能UID加密功能，使能后会在跳转至APP前进行一次UID解密判断，若不一致就拒绝跳转到APP工作
//#define FLASH_BUFF_ENABLE						//Flash缓存功能开关，使能后会在FLASH区域开辟一个代码缓存区用于存储传输到来的新代码数据

#define IC_EDITION              "BAT32G13701"
#define IC_EDITION_LENTH              11

#define Edition                 "BAT32G13701"	//BOOT代码的版本号
#define EditionLength           11				//版本号长度
//私有协议新增内容
#define TYPE_FAIL 0xDE//主机命令类型错误
#define CHECK_FAIL 0xDF //校验错误

// //主站发送来的控制码类型
// #define ENTER_BOOTMODE 			0x01		//进入更新模式，即握手信号
// #define ENTER_APPMODE			0x0f		//跳转执行用户程序
// #define WRITE_FLASH				0x22		//更新程序命令
// #define SET_ADDRESS			    0x30		//设置MCU开始更新的地址
// #define	SET_BAUD				0x25		//设置波特率
// #define	READ_IC_INF				0x03		//读取芯片型号
// #define	READ_BOOT_CODE_INF		0x04		//读取Boot代码版本号
// #define EARSE_ALL				0x06		//擦除所有APROM
// #define READ_FLASH              0x23        //读FLASH指定地址
//主站发送来的控制码类型 私有协议修改内容
#define	READ_BOOT_CODE_INF		0x10		//读取Boot代码版本号
#define	READ_IC_INF				0x51		//读取芯片型号
#define HEX_INFO                0x52        //接收HEX文件信息
#define ENTER_BOOTMODE 			0x53		//进入更新模式，即握手信号
// #define ENTER_APPMODE			0x0f		//跳转执行用户程序

// #define SET_ADDRESS			    0x30		//设置MCU开始更新的地址
// #define	SET_BAUD				0x25		//设置波特率
#define EARSE_ALL				0x54		//擦除所有APROM
#define WRITE_FLASH				0x55		//更新程序命令
#define READ_FLASH              0x56        //读FLASH指定地址


#define ERR_NO                  0x00        // 无异常
#define ERR_CMD_LEN             0x02        // 从机接收到的包长度和命令长度不对
#define ERR_CMD_ID              0x04        // 没有命令
#define ERR_CHECK               0x06        // 主机某个包校验和错误
#define ERR_OPERATE             0x07        // 未能完成主机要求的操作
#define ERR_PACKET_NUMBER       0x21        // 主机包的序号跳错误
#define ERR_MEM_NOT_ENOUGH      0x22        // 主机hex文件过大无法写入

//从站回应控制码类型
#define DEAL_SUCCESS 			0X9F		//回应操作成功
#define DEAL_FAIL				0xDF		//回应操作失败
#define	RETURN_IC_INF			0xA3		//回应芯片型号
#define	RETURN_BOOT_CODE_INF	0XA4		//回应BOOT程序版本号
#define RETURN_FLASH            0xA5        //回应读出的FLASH信息
//错误类型
#define	ERROR_CHECK_FAIL		0x01		//表示通讯校验失败
#define	ERROR_BURN_FAIL			0x02		//表示烧写校验错误
#define	ERROR_CMD_FAIL			0x04		//表示命令错误
//空闲
#define NO_CMD					0x00		//表示无命令

#define  RETURN_FLASH_APROM     0x00		//选择APROM
#define  RETURN_FLASH_DATA      0x01		//选择DATA Flash
#define  RETURN_FLASH_LDROM     0x02		//选择APROM
#define  RETURN_FLASH_XDATA     0x03		//选择XDATA
#define  RETURN_FLASH_SFR       0x04		//选择SFR
#define  RETURN_FLASH_RAM	    0x05		//选择RAM
#define  RETURN_FLASH_UID       0x06		//选择UID

#define HandShakes				3			 //握手次数设置

#define BOOT_BOOL_TRUE     1
#define BOOT_BOOL_FALSE    0
#define BOOT_ENABLE        1
#define BOOT_DISABLE       0

/*     此处为通讯相关接口，需要在通讯协议文件中定义此部分内容      */
#define CommunicationLength1    (64+7)
extern boot_length_t CmmuLength;		             //接收数据长度
extern boot_cmd_t CMDBuff;		                     //命令存储缓存
extern boot_data_t CommuData[CommunicationLength1];	 //通讯接收缓存
extern boot_data_t CmdSendData[CommunicationLength1];//发送缓存
extern boot_length_t CmmuSendLength;		         //接收数据长度
extern uint32_t NewBaud;							 //新波特率存储													
extern uint8_t CurrState;							 //存储当前芯片的状态,0:BOOT模式  1:APP运行态     2:代码缓存就绪态
extern boot_bool_t ResetFlag;
extern void BootCheckReset(void);		//检测是否有复位信号
extern uint8_t CheckUID(void);
void BootInit(void);
boot_cmd_t BootCmdRun(boot_cmd_t cmd);

#endif
