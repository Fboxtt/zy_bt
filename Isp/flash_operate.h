//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: flash_operate.h
//	模块功能	: BootLoader IAP操作头文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#ifndef _FLASH_OPERATE_H_
#define _FLASH_OPERATE_H_

#include "BAT32G137.h"
#include "boot_core.h"

#define APP_ADDR                0X2000				//APP的起始位置
#define APP_SIZE                (0x10000 - 0X2000)	//APP代码最大长度

//#define VECTOR_OFFSET           0x1c
#define APP_VECTOR_ADDR         APP_ADDR

#define LDROM_ADDR				0X0000				//LDROM的起始位置
#define LDROM_SIZE				0x2000				//LDROM的大小

#define DATA_ADDR				0x500200			//程序状态标志DATA Flash的起始位置
#define DATA_SIZE				0x500				//程序状态标志DATA的大小

#define ONE_PAGE_SIZE           512                 //一页的长度

#define IAP_CHECK_ADRESS 		0x1C00     		//更新成功数字码存储的起始地址
#define IAP_CHECK_LENGTH		4		  			//更新成功数字码长度,最大14Byte
#define IAP_CHECK_AREA			APROM_AREA			//标志所处区域
#define	IAP_CHECK_NUMBER		0XAA,0X55,0X55,0XAA //表示APP代码区程序正常的数字码，最大14Byte

#define TOTAL_CHECKSUM_ADRESS     0x1C04     		//上位机发送校验和存储地址
#define TOTAL_CHECKSUM_LENGTH        2

#define APP_BUFF_ADDR           0X10000		        //APP缓存区的起始位置
#define APP_BUFF_SIZE           (0x10000-0X2000)	//APP缓存区最大长度
#define	BUFF_CHECK_NUMBER		0X55,0XAA,0XAA,0X55 //表示APP缓存区装载完备的数字码，最大14Byte

#define UID_ENC_ADRESS			0x1FE00		        //UID密文存储地址
#define UID_ENC_SIZE			16					//UID密文长度
#define UID_SIZE				(128/8)				//UID有效长度
#define UID_ENC_AREA_AREA		APROM_AREA			//UID密文所在的存储区域

#define APP_TO_BOOT             0x55
#define BOOT_TO_APP             0xAA
#define	APROM_AREA	            0x55				//APROM区
#define DATA_AREA               0xAA				//DATA区
#define LDROM_AREA	            0X96				//LDROM区
#define APROM_BUFF_AREA			0x69				//APP缓存区
#define UID_ENC_AREA			0x22				//UID密文存储区

extern uint8_t IAP_IapLength;	        //用于IAP操作数据长度缓存

extern uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area);//写多字节IAP操作
extern void IAP_ReadMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint16_t len,uint8_t area); //读多字节IAP操作
extern uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area);  //读单字节IAP操作
extern void IAP_Reset(void);			 		                    //复位启动								
extern void IAP_Erase_ALL(uint8_t area);						    //将目标区域全擦
extern void IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area);   //擦除一个块（512B）
extern void IAP_FlagWrite(uint8_t flag);
extern uint8_t IAP_CheckAPP(void);
extern void IAP_ReadEncUID(uint8_t* buff);
extern void IAP_Remap(void);//将缓存区的代码装载如运行区
extern uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area);//写单字节IAP操作
#endif
