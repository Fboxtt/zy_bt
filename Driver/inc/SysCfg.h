/******************************************************************************
 * Copyright(C)	2022	
 * All rights reserved.

 * 文件名称:	Syscfg.h
 * 描    述:	系统参数定义
 * 当前版本:	Ver 1.0
 * 创 建 人:	zml
 * 创建日期:	2022-07-05
 * 备    注:	便于其他文件统一引用.
 *
 *-----------------------------------------------------------------------------
 * 修 改 人:
 * 修改日期:
 * 修改内容: 

******************************************************************************/

#ifndef _SYS_CFG_H
#define	_SYS_CFG_H

#include "includes.h"

/******************************************************************************
**  时钟主频
******************************************************************************/
#define FCCLK 		(12000000)//(44236800) 12000000
#define HSECLK		(12000000)//(11059200)

/******************************************************************************
**  数据转换
******************************************************************************/
#define LOBYTE(w)				((UCHAR)(w))
#define HIBYTE(w)				((UCHAR)(((USHORT)(w) >> 8) & 0xFF))
#define MAKEWORD(Lo, Hi)		((USHORT)(((UCHAR)(Lo)) | ((USHORT)((UCHAR)(Hi))) << 8))
#define LOWORD(l)				((USHORT)(l))
#define HIWORD(l)				((USHORT)(((ULONG)(l)>>16)&0xFFFF))
#define MAKELONG(wLo, wHi)		((ULONG)(((USHORT)(wLo))|((ULONG)((USHORT)(wHi)))<<16))

#define HEX2DEC_BYTE(w)				(((w&0xF0)>>4)*10+(w&0x0F))
#define DEC2HEX_BYTE(w)				(((w/10)<<4) | (w%10))

/******************************************************************************
** 告警/保护/故障/其他状态 
******************************************************************************/
//告警
#define	ALARM_PACK_OV			 (0x00000001)			
#define ALARM_BATT_OV			 (0x00000002)   		
#define ALARM_CELL_OV			 (0x00000004)			
#define ALARM_BATT_UV			 (0x00000010)			
#define ALARM_CELL_UV			 (0x00000020)			
#define	ALARM_CHG_OC			 (0x00000040)			
#define ALARM_DIS_OC			 (0x00000080)			
#define	ALARM_CHG_OT			 (0x00000100)			
#define	ALARM_DIS_OT			 (0x00000200)			
#define	ALARM_CHG_UT			 (0x00000400)			
#define	ALARM_DIS_UT			 (0x00000800)			
#define	ALARM_SOC_L				 (0x00001000)			
#define ALARM_END_LIFE		 (0x00010000)	
#define ALARM_MOS_HT       (0x00020000)

	

//保护
#define	PROTECT_PACK_OV			(0x00000001)			
#define PROTECT_BATT_OV			(0x00000002)    		
#define PROTECT_CELL_OV			(0x00000004)			
#define PROTECT_BATT_UV			(0x00000010)			
#define PROTECT_CELL_UV			(0x00000020)			
#define	PROTECT_CHG_OC			(0x00000040)			
#define PROTECT_DIS_OC			(0x00000080)			
#define	PROTECT_CHG_OT			(0x00000100)			
#define	PROTECT_DIS_OT			(0x00000200)			
#define	PROTECT_CHG_UT			(0x00000400)			
#define	PROTECT_DIS_UT			(0x00000800)			
#define PROTECT_SHORT				(0x00004000)			
#define	PROTECT_REV					(0x00008000)			
#define PROTECT_CHG_UTOC		(0x00010000)			
#define PROTECT_CHG_UTOV		(0x00040000)			
#define PROTECT_CHG_SHORT		(0x00080000)			
#define PROTECT_PSP					(0x00100000)	
#define PROTECT_BQ_SCD			(0x00200000)	//BQ769X0检测短路
#define PROTECT_BQ_OCD			(0x00400000)	//BQ769X0检测放电过流
#define PROTECT_MOS_HT      (0x00800000)  
#define	PROTECT_CHG_UT_LIMIT	(0x01000000)	//充电低温极限保护值


//失效
#define	FAULT_V_SENSOR			(0x00000001)			
#define FAULT_T_SENSOR			(0x00000002)			
#define FAULT_CHG						(0x00000004)			
#define FAULT_DIS						(0x00000008)			
#define	FAULT_CELL_BAD			(0x00000010)	
#define FAULT_SWITCH_OFF		(0x00000080)	
#define	FAULT_LIFE_END			(0x00000100)

#define FAULT_BQ_UV					(0x00010000)	//BQ769X0二次保护欠压 	//
#define FAULT_BQ_OV					(0x00020000)	//BQ769X0二次保护过压
#define FAULT_BQ_DEVIECE		(0x00040000)	//BQ769X0芯片失效
#define FAULT_BQ_OVERWR			(0x00080000)	//BQ769X0芯片失效
#define FAULT_FUSE_OFF      (0x00100000)     //保险丝熔断异常

#define  PROTECTSTATUS_MASK 	(PROTECT_CELL_UV\
                                    |PROTECT_CHG_OC\
                                    |PROTECT_DIS_OC\
                                    |PROTECT_CHG_OT\
                                    |PROTECT_DIS_OT\
                                    |PROTECT_CHG_UT\
																		|PROTECT_CHG_UT_LIMIT\
                                    |PROTECT_DIS_UT\
                                    |PROTECT_SHORT\
                                    |PROTECT_REV\
                                    |PROTECT_CHG_UTOC )	

//|PROTECT_CHG_SHORT;|PROTECT_BQ_OCD;PROTECT_BATT_UV\

#define  FAULT_CALIBRATED_VC_MASK  (FAULT_V_SENSOR\
                                        |FAULT_T_SENSOR\
                                        |FAULT_CHG\
                                        |FAULT_DIS\
                                        |FAULT_CELL_BAD\
                                        |FAULT_SWITCH_OFF\
                                        |FAULT_BQ_UV\
                                        |FAULT_BQ_OV\
                                        |FAULT_BQ_DEVIECE\
                                        |FAULT_BQ_OVERWR)	
//其他信息
#define	INFO_HEAETER_CONFIG			(0x00000001)	
#define INFO_HEATER_ON					(0x00000002)			
#define INFO_CHG_FULL_T					(0x00000004)	
#define INFO_BATT_FULL					(0x00000008)
#define INFO_CHG_LIMITED_ON			(0x00000010)
#define	INFO_DIS_LIMITED_ON	    (0x00000020)
#define INFO_CHG_MOS_OFF				(0x00000040)		   	
#define INFO_DIS_MOS_OFF				(0x00000080)
#define	INFO_LOWVOL_LIMIT       (0x00000100)
#define	INFO_LOWTEMP_FORCECHG		(0x00000200)
#define	INFO_MULT_BATT					(0x00000400)
#define	INFO_CAN_MASTER					(0x00000800)
#define	INFO_CAN_SLAVE					(0x00001000)
#define	INFO_CALENDAR_REACH			(0x00002000)
#define	INFO_TEST_KB  					(0x00004000)
#define INFO_CALIBRATED_V				(0x00008000)			
#define INFO_CALIBRATED_C				(0x00010000)
#define INFO_LOST_CANBOX				(0x00020000)
#define	INFO_NO_INV_TIMEOUTE  	(0x00040000)			
#define FUSEEN_OPEN             (0x00080000)
#define	INFO_CELL_CTO						(0x00100000)


//充放电状态
#define STATUS_IDLE				(0x00)				
#define STATUS_CHG				(0x01)				
#define STATUS_DIS				(0x02)				
#define STATUS_FULL				(0x04)				

//MOS开关状态
#define 	MOS_ON		(0x01)
#define 	MOS_OFF		(0x02)
#define 	MOS_INIT	(0X00)
                             
//MOS开关控制
#define MOS_SWITCH_ON	(1)
#define MOS_SWITCH_OFF	(0)

/******************************************************************************
** EEPROM地址---更换为FLASH数据区
******************************************************************************/	
//固化参数块
#define	EEPROM_ADDR_BMS_CRC						(0x00500200)
#define	EEPROM_ADDR_BMS								(EEPROM_ADDR_BMS_CRC+2)					//140+2字节*--192字节

#define	EEPROM_ADDR_KB_CRC						(0x005002C0)				
#define	EEPROM_ADDR_KB								(EEPROM_ADDR_KB_CRC+2)					//76+2字节--96字节

#define EEPROM_ADDR_DELAYTIME_CRC 		(0x00500320)				
#define EEPROM_ADDR_DELAYTIME    			(EEPROM_ADDR_DELAYTIME_CRC+2)		//8+2字节-16字节

#define EEPROM_DESIGN_CAP_CRC					(0x00500330)       
#define EEPROM_DESIGN_CAP 	 					(EEPROM_DESIGN_CAP_CRC+2)       //8+2字节--16字节

#define EEADDR_MOSHTPROTECT_DATE_CRC	 (0x00500340)                 
#define EEADDR_MOSHTPROTECT_DATE			(EEADDR_MOSHTPROTECT_DATE_CRC+2) //8+2字节--16字节

#define	EEPROM_STARTRUN_DATA_CRC			(0x00500350)											
#define	EEPROM_STARTRUN_DATA					(EEPROM_STARTRUN_DATA_CRC+2)			//8+2字节----16字节

#define EEPROM_ADDR_SERIAL_NUM_CRC		(0x00500360)			
#define EEPROM_ADDR_SERIAL_NUM				(EEPROM_ADDR_SERIAL_NUM_CRC+2)		//20+2字节--48字节

#define EEPROM_ADDR_UNIQUE_NUM_CRC    (0x00500390)
#define EEPROM_ADDR_UNIQUE_NUM        (EEPROM_ADDR_UNIQUE_NUM_CRC+2)

//运行更新数据块
#define	EEPROM_ADDR_LIFE_CRC					(0x00500400)				
#define	EEPROM_ADDR_LIFE							(EEPROM_ADDR_LIFE_CRC+2)				//16+2字节--32字节

#define	EEPROM_ADDR_HISTORY_CRC				(0x00500420)			
#define	EEPROM_ADDR_HISTORY						(EEPROM_ADDR_HISTORY_CRC+2)				//64+2字节--80字节

#define EEADDR_POWEROFF_DATE_CRC			(0x00500470)
#define EEADDR_POWEROFF_DATE					(EEADDR_POWEROFF_DATE_CRC+2)			//8+2字节--16字节

#define	EEADDR_LIFEEND_TIME_CRC				(0x00500480)										 //16+2字节-32字节
#define	EEADDR_LIFEEND_TIME						(EEADDR_LIFEEND_TIME_CRC+2)				

/*-----------------------------------------------------------*/
#define EEADDR_RTCTIME_CRC		        0x0340													//MDate  RTC开机赋值 -8byte
#define EEADDR_RTCTIME			        	(EEADDR_RTCTIME_CRC+2)

#define EEADDR_FUSESTATE_DATE_CRC      0x0350
#define EEADDR_FUSESTATE_DATE         (EEADDR_FUSESTATE_DATE_CRC+2)


#define EEADDR_WORKTIMES_DATE_CRC			 0x0330														//高仙需要上报 -8byte
#define EEADDR_WORKTIMES_DATE					(EEADDR_WORKTIMES_DATE_CRC+2)


//定义跳转时需要保存的信息
#define KEEPON_DATAS_ADDR               0x20002FD0    //预留48字节数据



//告警记录索引地址
#define  	ADDR_HISALM_INDEX     			(0x1000)	 			//??????
#define  	ADDR_DAYRCD_INDEX	    		(0x1008)				//?????
#define  	ADDR_MINRCD_INDEX			   	(0X1010)				//??????
#define  	ADDR_CULSTER_HISALM_INDEX     	(0x1018)	 			//???????
#define  	ADDR_CULSTER_MINRCD_INDEX		(0X1020)				//???????
//告警记录地址
#define  	ADDR_HISALM_RECORD     		(0x1100)	 			//长度为500*24BYTE = 0x2EE0,范围（0x1100-0x3fe0
#define  	ADDR_DAY_RECORD				(0x3ff0)				//长度为3*365*12BYTES = 0x3354,范围（0x3ff0,0x7344
#define  	ADDR_MIN_RECORD			   	(0X7350)				//长度为14*24*4*12bytes = 0x3f00,范围（0x7350，0xB250
#define		ADDR_CULSTER_MIN_RECORD		(0Xb260)				//长度为7*24*4*12,范围(0xb260,0xd1e0)
//告警长度定义
#define		HISALM_MAX_COUNT  			(500)					//300条
#define		DAYRECORD_MAX_COUNT  		(365*3)				//3年
#define		MINRECORD_MAX_COUNT  		(14*24*4) 			//14天
#define		CLU_MINRECORD_MAX_COUNT  	(7*24*4) 			//14天

/*告警等级*/		
#define MASK_BMS_WARN		0
#define MASK_BMS_PROTECT	1
#define MASK_BMS_FAULT		2
#define MASK_BMS_EVENT		3

//BMS告警等级分类
#define	xALARM_PACK_OV			 0			
#define xALARM_BATT_OV			 1    		
#define xALARM_CELL_OV			 2			
#define xALARM_BATT_UV			 4			
#define xALARM_CELL_UV			 5			
#define	xALARM_CHG_OC			 6			
#define xALARM_DIS_OC			 7			
#define	xALARM_CHG_OT			 8			
#define	xALARM_DIS_OT			 9			
#define	xALARM_CHG_UT			 10		
#define	xALARM_DIS_UT			 11		
#define	xALARM_SOC_L			 12		
#define xALARM_END_LIFE			 16		
				

//保护
#define	xPROTECT_PACK_OV		0		
#define xPROTECT_BATT_OV		1 		
#define xPROTECT_CELL_OV		2		
#define xPROTECT_BATT_UV		4		
#define xPROTECT_CELL_UV		5		
#define	xPROTECT_CHG_OC			6		
#define xPROTECT_DIS_OC			7		
#define	xPROTECT_CHG_OT			8		
#define	xPROTECT_DIS_OT			9		
#define	xPROTECT_CHG_UT			10
#define	xPROTECT_DIS_UT			11
#define xPROTECT_SHORT			14
#define	xPROTECT_REV			15
#define xPROTECT_CHG_UTOC		16
#define xPROTECT_CHG_UTOV		18
#define xPROTECT_CHG_SHORT		19
#define xPROTECT_PSP			20
#define xPROTECT_BQ_SCD			21
#define xPROTECT_BQ_OCD			22

//失效
#define	xFAULT_V_SENSOR			0	
#define xFAULT_T_SENSOR			1	
#define xFAULT_CHG				2	
#define xFAULT_DIS				3	
#define	xFAULT_CELL_BAD			4
		
#define xFAULT_BQ_UV			16
#define xFAULT_BQ_OV			17
#define xFAULT_BQ_DEVIECE		18
#define xFAULT_BQ_OVERWR		19

//MASK_BMS_EVENT
#define	xEVENT_POWER_ON			0			 			 
#define xEVENT_POWER_OFF		1						 
#define xEVENT_MANNULA_DOWN		2						 
#define xEVENT_ABNORMAL_QUIT	3						 
#define	xEVENT_SW_UPDATE		4 		 			 					 					 
#define xEVENT_SOC_LOW		 	5				 
#define xEVENT_SOC_DROP			6 						 
#define xEVENT_CELL_DIFF		7 
#define xEVENT_PARALLEL_FAULT	8						 
#define xEVENT_PARALLEL_COMLOST	9
#define xEVENT_CLIENT_COMLOST	10





#endif


