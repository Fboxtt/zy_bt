//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: flash_operate.h
//	ģ�鹦��	: BootLoader IAP����ͷ�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************
#ifndef _FLASH_OPERATE_H_
#define _FLASH_OPERATE_H_

#include "BAT32G137.h"
#include "boot_core.h"

#define APP_ADDR                0X2000				//APP����ʼλ��
#define APP_SIZE                (0x10000 - 0X2000)	//APP������󳤶�

//#define VECTOR_OFFSET           0x1c
#define APP_VECTOR_ADDR         APP_ADDR

#define LDROM_ADDR				0X0000				//LDROM����ʼλ��
#define LDROM_SIZE				0x2000				//LDROM�Ĵ�С

#define DATA_ADDR				0x500200			//����״̬��־DATA Flash����ʼλ��
#define DATA_SIZE				0x500				//����״̬��־DATA�Ĵ�С

#define ONE_PAGE_SIZE           512                 //һҳ�ĳ���

#define IAP_CHECK_ADRESS 		0x1C00     		//���³ɹ�������洢����ʼ��ַ
#define IAP_CHECK_LENGTH		4		  			//���³ɹ������볤��,���14Byte
#define IAP_CHECK_AREA			APROM_AREA			//��־��������
#define	IAP_CHECK_NUMBER		0XAA,0X55,0X55,0XAA //��ʾAPP���������������������룬���14Byte

#define TOTAL_CHECKSUM_ADRESS     0x1C04     		//��λ������У��ʹ洢��ַ
#define TOTAL_CHECKSUM_LENGTH        2

#define APP_BUFF_ADDR           0X10000		        //APP����������ʼλ��
#define APP_BUFF_SIZE           (0x10000-0X2000)	//APP��������󳤶�
#define	BUFF_CHECK_NUMBER		0X55,0XAA,0XAA,0X55 //��ʾAPP������װ���걸�������룬���14Byte

#define UID_ENC_ADRESS			0x1FE00		        //UID���Ĵ洢��ַ
#define UID_ENC_SIZE			16					//UID���ĳ���
#define UID_SIZE				(128/8)				//UID��Ч����
#define UID_ENC_AREA_AREA		APROM_AREA			//UID�������ڵĴ洢����

#define APP_TO_BOOT             0x55
#define BOOT_TO_APP             0xAA
#define	APROM_AREA	            0x55				//APROM��
#define DATA_AREA               0xAA				//DATA��
#define LDROM_AREA	            0X96				//LDROM��
#define APROM_BUFF_AREA			0x69				//APP������
#define UID_ENC_AREA			0x22				//UID���Ĵ洢��

extern uint8_t IAP_IapLength;	        //����IAP�������ݳ��Ȼ���

extern uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area);//д���ֽ�IAP����
extern void IAP_ReadMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint16_t len,uint8_t area); //�����ֽ�IAP����
extern uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area);  //�����ֽ�IAP����
extern void IAP_Reset(void);			 		                    //��λ����								
extern void IAP_Erase_ALL(uint8_t area);						    //��Ŀ������ȫ��
extern void IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area);   //����һ���飨512B��
extern void IAP_FlagWrite(uint8_t flag);
extern uint8_t IAP_CheckAPP(void);
extern void IAP_ReadEncUID(uint8_t* buff);
extern void IAP_Remap(void);//���������Ĵ���װ����������
extern uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area);//д���ֽ�IAP����
#endif
