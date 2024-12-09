/******************************************************************************
 * Copyright(C)	2020
 * All rights reserved.

 * 文件名称:	Typedefs.h
 * 描    述:	类型定义
 * 当前版本:	Ver 1.0
 * 创 建 人:	zml
 * 创建日期:	2022-07-05
 * 备    注: 	包括通用类型和本项目专用类型.
 *
 *-----------------------------------------------------------------------------
 * 修 改 人:
 * 修改日期:
 * 修改内容: 

******************************************************************************/

#ifndef _TYPEDEFS_H
#define	_TYPEDEFS_H

/******************************************************************************
**  通用类型定义 
******************************************************************************/
typedef		char				CHAR;			//c
typedef 	short				SHORT;			//s
typedef		int					INT_8;			//i
typedef		long				LONG;			//l

typedef		unsigned char		UCHAR;			//uc
typedef		unsigned short		USHORT;			//us
typedef		unsigned int		UINT;			//ui
typedef		unsigned long		ULONG;			//ul
   
typedef		UCHAR				BOOL;			//b


typedef     unsigned char       BYTE    ;
typedef     unsigned char       UINT8   ;
    
typedef     char                UBYTE  ;
typedef     char                INT8   ;
    
typedef     unsigned short      WORD;
typedef     unsigned short      UINT16;
    
typedef     signed   short      UWORD  ;
typedef     signed   short      INT16  ;
    
typedef     unsigned int        DWORD  ;
typedef     unsigned int        UINT32  ;
    
typedef     signed   int        UDWORD ;
typedef     signed   int        INT32 ;
    
typedef     void *              PVOID  ;
typedef     unsigned char  *    PBYTE ;
    
// typedef     float               FP32;    /*  单精度浮点数（32位长度）    */
// typedef     double              FP64; 

// #ifndef		TRUE
// 	#define		TRUE			1
// #endif

// #ifndef		FALSE
// 	#define		FALSE			0
// #endif

// #ifndef		NULL
// 	#define		NULL			((void *)0)
// #endif
	


// /******************************************************************************
// **  本项目专用类型
// ******************************************************************************/
// //RTC时间结构体
// typedef struct
// {
//   WORD   byYear;          /*年         */
//   BYTE   byMonth;         /*月         */
//   BYTE   byWeek;          /*星期        */
	
//   BYTE   byDay;           /*日期        */
//   BYTE   byHour;          /*小时        */
//   BYTE   byMin;           /*分钟        */
//   BYTE   bySecond;        /*秒          */
// }MDate,*pMDate;

// //年月日
// typedef struct
// {
// 	BYTE bitYear:4;
// 	BYTE bitMonth:4;
// 	BYTE byDay;	
// }MDDateYMD;	  	

// //年月日时分
// typedef struct
// {
// 	BYTE bitYear:4;
// 	BYTE bitMonth:4;
// 	BYTE byDay;	
// 	BYTE byHour;
// 	BYTE byMin;
// }MDDateYMDHM;   	
	
// //SBS(Smart Battery System)
// typedef struct tagSBS
// {
// 	ULONG		ulPackV;						//Pack
// 	ULONG		ulBattV;						//Batt 
// 	USHORT 		usCellV[CELL_COUNT];			//Cell 
// 	LONG 		lCurrent;						//current
// 	SHORT		sTemp[TEMP_COUNT];				//temp
// 	USHORT		usRemainAH;
// 	USHORT		usFccAH;
// 	USHORT		usBiaAH;
// 	ULONG		ulOtherInfo;				//其他信息
// 	ULONG		ulAlarmStatus;			//告警状态
// 	ULONG		ulProtectStatus;		//保护状态
// 	ULONG		ulFaultStatus;			//失效状态
// 	ULONG		ulBalanceStatus;		//均衡状态
// 	USHORT		usBattStatus;				//充放电状态
// 	USHORT		usSOC_Percent;			//SOC	
// 	ULONG		ulSOH_Percent;			//SOH					 
// 	ULONG		ulDisTimes;					//放电次数
// 	ULONG		ulTotalDisAH;				//放电AH数  
// }TSBS;

// //保护参数(Battery Management System)
// typedef struct tagBMS
// { 
// 	ULONG	ulCHG_SwitchV;					
// 	ULONG	ulSwitch_PB_DiffV;		
	
// 	ULONG	ulCHG_Bls_StartV;				
// 	ULONG	ulCHG_Bls_StopV;				

// 	ULONG	ulPack_OVA_Threshold;			
// 	ULONG	ulPack_OVA_Resume;				
// 	ULONG	ulPack_OVP_Threshold;			
// 	ULONG	ulPack_OVP_Resume;				

// 	ULONG	ulBatt_OVA_Threshold;			
// 	ULONG	ulBatt_OVA_Resume;				
// 	ULONG	ulBatt_OVP_Threshold;			
// 	ULONG	ulBatt_OVP_Resume;				

// 	USHORT	usCell_OVA_Threshold;			
// 	USHORT	usCell_OVA_Resume;				
// 	USHORT	usCell_OVP_Threshold;			
// 	USHORT	usCell_OVP_Resume;				
	
// 	ULONG	ulBatt_UVA_Threshold;			
// 	ULONG	ulBatt_UVA_Resume;				
// 	ULONG	ulBatt_UVP_Threshold;			
// 	ULONG	ulBatt_UVP_Resume;				
	
// 	USHORT	usCell_UVA_Threshold;			
// 	USHORT	usCell_UVA_Resume;				
// 	USHORT	usCell_UVP_Threshold;			
// 	USHORT	usCell_UVP_Resume;				
	
// 	LONG	lCHG_OCA_Threshold;				
// 	LONG	lCHG_OCA_Resume;				
// 	LONG	lCHG_OCP_Threshold;				

// 	LONG	lDIS_OCA_Threshold;				
// 	LONG	lDIS_OCA_Resume;				
// 	LONG	lDIS_OCP_Threshold;				


// 	SHORT 	sCHG_OTA_Threshold;				
// 	SHORT 	sCHG_OTA_Resume;				
// 	SHORT 	sCHG_OTP_Threshold;				
// 	SHORT 	sCHG_OTP_Resume;				

// 	SHORT 	sDIS_OTA_Threshold;				
// 	SHORT 	sDIS_OTA_Resume;				
// 	SHORT 	sDIS_OTP_Threshold;				
// 	SHORT 	sDIS_OTP_Resume;				

// 	SHORT	sCHG_UTA_Threshold;				
// 	SHORT	sCHG_UTA_Resume;				
// 	SHORT	sCHG_UTP_Threshold;				
// 	SHORT	sCHG_UTP_Resume;				

// 	SHORT	sDIS_UTA_Threshold;				
// 	SHORT	sDIS_UTA_Resume;				
// 	SHORT	sDIS_UTP_Threshold;				
// 	SHORT	sDIS_UTP_Resume;				
	
// 	SHORT	sHEATER_START_T;				
// 	SHORT	sHEATER_STOP_T;					
// }TBMS;	

// //KB值
// typedef struct tagKB        
// {
//  USHORT usPackVK;        
//  USHORT usBattVK;           
//  USHORT usCellVK[CELL_COUNT];            
//  USHORT usChgCurrK;      	
//  SHORT 	sChgCurrB;      	
//  USHORT usDisCurrK;      	
//  SHORT 	sDisCurrB;         
//  USHORT usChgCurrSK;      	
//  SHORT 	sChgCurrSB;      	
//  USHORT usDisCurrSK;      	
//  SHORT 	sDisCurrSB;        
//  USHORT usChgCurrSSK;      	
//  SHORT 	sChgCurrSSB;      	
//  USHORT usDisCurrSSK;      	
//  SHORT 	sDisCurrSSB; 	 	
//  USHORT usTempK[TEMP_DEF];                                             //??????K
// }TKB;


// typedef struct
// {
//  ULONG  ulUpdateType;             //跳转更新类型，can与RS485
//  ULONG  ulOtherInfo;
    
// }TDATAKEEP;


//版本信息VERSION
typedef struct tagVersion
{
	USHORT	usMajorVer;				    	 
	USHORT	usMinorVer;				    	 
	USHORT	usRevision;						 
	USHORT	usCompileYear;					 
	UCHAR		ucCompileMonth;			    	 
	UCHAR		ucCompileDay;					  
	CHAR    cHWversion[30];					 
	CHAR    cFuncVersion[40];				 
}TVER;

// //ADC参数
// typedef struct tagADC
// {
//     USHORT	usPackVADC;         	 
// 		USHORT	usBattVADC;        	 
//     USHORT	usCellVADC[CELL_COUNT];		 
//     USHORT	usChgCurrADC;				 
//     USHORT	usDisCurrADC;				 
//     USHORT	usTempADC[TEMP_DEF];		 
// }TADC;                                   

// //寿命信息
// typedef	struct tagLife
// {
// 	USHORT	usSOC_Percent;	             			 
// 	ULONG   ulSOH_Percent;				 
// 	ULONG	ulRemainPointmAs;			 
// 	ULONG	lSingleDis_Ah;			 	 
// }TLIFE;

// typedef struct tagTimes
// {
// 	USHORT usReset_Times;
// 	USHORT usSohupdate_Times;
// 	USHORT usReserve[2];
// }TTIMES;

// //容量信息
// typedef struct
// {
// 	ULONG	ulModuleDesignCap;
// 	ULONG 	ulModuleFactoryCap;
// }TCAP;

// typedef struct 
// {
// 	USHORT   sAlarm;
//   USHORT   sAlarmRe;
//   USHORT	 sProtect;   
// 	USHORT	 sProtectRe;
// }TMOSHTDATA;

// //保险丝信息,01为默认，02位使能
// typedef struct
// {
// 	USHORT	usFuseEn;
// 	USHORT 	usFusestate;
// }TFUSESTATE;

// //限流值参数
// typedef struct tagPara
// {
// 	USHORT ChgDelayCount_1C;
// 	USHORT ChgDelayCount_2C; 
// 	USHORT DisDelayCount_1C; 
// 	USHORT DisDelayCount_2C;	
// }TDelayTimePara;

// typedef struct 
// {
// 	SHORT	sValua;
// 	UCHAR	ucMask;
// 	BOOL 	g_bTurn;
// }TTEXTREME;

// typedef struct 
// {
// 	USHORT	usValua;
// 	UCHAR	ucMask;
// 	BOOL 	g_bTurn;
// }TVEXTREME;

// /*??????*/
// typedef struct
// {
 
// 	MDate   stInstallDate;			//8字节
// 	ULONG		ulSingleRunTime;		
// 	ULONG 	ulTotalRunTime;		 
// 	ULONG		ulTotalDisAh;			
// 	ULONG		ulTotalDisCycle;		
	
// 	ULONG		ulCondi_Low5Degree;		
// 	ULONG		ulCondi_5to15Degree;	
// 	ULONG		ulCondi_15to35Degree;
// 	ULONG		ulCondi_35to50Degree;
// 	ULONG		ulCondi_Above50Degree;
// 	ULONG		ulCell_Low5Degree;		
// 	ULONG		ulCell_5to15Degree;		
// 	ULONG		ulCell_15to35Degree;	
// 	ULONG		ulCell_35to50Degree;	
// 	ULONG		ulCell_Above50Degree;
// }HisModuleRunRcd;



// typedef struct 
// {
// 	BYTE		ucLifeEndStart;		  //是否触发寿命终止并开始闪灯
// 	BYTE		ucCurrLightState;  	//当前是否需要闪灯
// 	BYTE		ucLifeEndOver ;			//产品寿命终止失效
// 	BYTE		ucCurrLightTimes;		//当前闪灯次数值
	
// 	ULONG		ulCurrLightTime;		//当前闪灯累计的时间值
// 	ULONG		ulNextLightTime;		//距离下次闪灯运行时间
// 	//ULONG		Rev;
// 	BYTE		ucSohUpdate ;				//可更新soh
// 	BYTE		ucSohFacter;				//soh的校准
// 	USHORT	usRev;
// }TLIFEENDTIME;

// /******************************************************************************
// **  CAN通信专用类型
// ******************************************************************************/
// //配置信息
// typedef struct
// {
// 	ULONG 	ulSerialNum;	   		//序列号值
// 	BYTE  	byAddr;					//地址
//   BOOL  	bMasterFlg;				//主机标志
// }TBCU_CFG;

// //CAN总线控制
// typedef struct
// {
// 	ULONG 	ulMasterSerialNum;	   	//序列号值	
// 	BYTE 		byParallelCnt;					//并联数量
// 	BYTE		byCurrentCANbusAddr;	//当前总线占用值
// 	BOOL 		bCANBusRight;
// }TBCU_BUS;
  
// //电芯最大最小电压温度
// typedef struct
// {
// 	USHORT	usMaxCellVolt;
// 	USHORT	usMinCellVolt;
// 	SHORT		sMaxCellTemp;
// 	SHORT		sMinCellTemp;
// }TBICMAXMIN;

// //簇状态

// typedef struct		 
// {	
// 	ULONG		ulBCUComStatus[1];	 //没两位表示通讯状态：00为初始化；0x01为已连接；0x02为通讯断开  MAX_BCU_NUM*2/32
// 	ULONG 	ulChgMosSatus[1];	//MAX_BCU_NUM*2/32
// 	ULONG 	ulDisMosSatus[1];	//MAX_BCU_NUM*2/32
// }TCLUSTER_MAXMIN;

// typedef struct 
// {
// 	BYTE		byBrand;
// 	BYTE		byCurrRate;
// 	WORD 		wBattModuleCap;
// 	DWORD		dwClusterCap;
	
// 	WORD 		byTotalBatteryNum;
// 	WORD 		wComFailCount;
// 	WORD		wBmsFailCount;
// 	WORD		wCellFailCount;
// 	WORD 		wChgMosOffCount;
// 	WORD 		wDisMosOffCnout;

// 	SHORT		sSysMaxCellTemp;
// 	SHORT		sSysMinCellTemp;
// 	USHORT	usSysMaxCellVolt;
// 	USHORT	usSysMinCellVolt;
// 	ULONG		ulSysMaxBattVolt;
// 	ULONG		ulSysMinBattVolt;
// 	LONG		lSysMaxCurrent;
// 	LONG		lSysMinCurrent;		
//   USHORT	usSysMaxSOC;
// 	USHORT	usSysMinSOC;
//   USHORT	usSysMaxSOH;
// 	USHORT	usSysMinSOH;
	
// 	USHORT	usClusterChgVoltLimit;
// 	USHORT	usClusterDisVoltLimit;	
	
// 	DWORD		dwClusterChgCurLimit;
// 	DWORD		dwClusterDisCurLimit;
	
// 	SHORT		sClusterTemp;
// 	USHORT	usClusterVoltInV; 		 
// 	LONG		lClusterCurrInA;
// 	USHORT	usClusterSoc;			 
// 	USHORT	usClusterSoh;
	
// 	ULONG		ulClusterWarn;	   		 
// 	ULONG   ulClusterAlarm;
// 	ULONG		ulClusterCycles;
	
// 	WORD 		wReserve[10];	
// }TClusterSBS;


// /***********************************************
// **历史告警结构体
// ***********************************************/
// //存储索引记录
// typedef struct
// {
//    WORD CurPos;
//    WORD Total;
//    WORD Crc;
// }TIndexRecord;

// //发送索引
// typedef struct
// {
//    WORD  wCount;
//    WORD  wIndex;
//    WORD  wArrayCount;
// }TSendIndex;

// //簇分钟记录
// typedef struct
// {
// 	MDDateYMDHM stDateMin;			//MIN??
// 	BYTE 	byClusterSoc;			//1% 
// 	BYTE 	bySysMaxSOC;	  		//1% 
// 	BYTE 	bySysMinSOC;			//1% 
// 	INT8	cClusterTemp;	   		//1℃
// 	WORD	wClusterPackVolt;		//0.1v
// 	SHORT	sClusterCurrAvg;		//0.1A
// }HisClusterMinRcd;

// //模组分钟记录
// typedef struct
// {
// 	MDDateYMDHM stDateMin;	//MIN??
// 	SHORT sCurrentAvg;	//??????,0.1A??
// 	WORD wBatVolt;	  	//?????,0.1v??
// 	WORD wMinCellVolt;	//??????,0.001V??
// 	char cMaxCellTemp;	//??????,1???	
// 	BYTE bySoc;			//SOC		
// }HisModuleMinRcd;

// //模组日记录
// typedef struct
// {
// 	MDDateYMD stDateDay;  //DAY??
// 	BYTE bySoh;			  //SOH
// 	BYTE byChgKwh;		  //????
// 	BYTE byDisKwh;		  //????
// 	BYTE byMaxSoc;		  //??SOC
// 	BYTE byMinSoc;		  //??SOC
// 	INT8 cMaxCondiTemp;	  //??????
// 	INT8 cMinCondiTemp;	  //??????
// 	INT8 cMaxCellTemp;	  //??????
// 	INT8 cMinCellTemp;	  //??????
// 	BYTE byMinBatt;		  //???????
// }HisModuleDayRcd;

// //模组告警记录
// typedef struct
// {
// 	BYTE 	byYear;			     //??:??????
// 	BYTE 	byMonth;
// 	BYTE 	byDay;	
// 	BYTE 	byHour;

// 	BYTE 	byMin;
// 	BYTE 	bySec;	
// 	BYTE 	byAlmClass;			 //????
// 	BYTE 	byAlmType;			 //????

// 	WORD 	wPackVolt;			 //PACK??,0.1V??
// 	WORD 	wBattVolt;			 //BATT??,0.1V??

// 	SHORT 	sCurrent;			 //??,0.1A??
// 	BYTE 	bySoc;				 //SOC
// 	BYTE	bySoh;				 //SOH

// 	WORD  	wMaxCellVolt;		 //??????
// 	WORD 	wMinCellVolt;		 //??????

// 	INT8	cCondiTemp;			 //????
// 	INT8	cMaxCellTemp;		 //??????
// 	INT8	cMinCellTemp;		 //??????
// 	BYTE 	bitChgMos:2;		 //??MOS??,0x01???,0x02???
// 	BYTE 	bitDisMos:2;		 //??MOS??,0x01???,0x02???
// 	BYTE 	bitRev:4;			 //???
// }HisModuleAlmRcd;

// /*读取缓冲结构体*/
// //簇分钟读取缓冲
// typedef struct
// {
//    WORD  wCount;
//    WORD  wIndex;
//    WORD  wArrayCount;
//    HisClusterMinRcd   Rcd[20];
// }TReadClusterMinRecTemp;

// //模组告警读取缓冲
// typedef struct
// {
//    WORD  wCount;
//    WORD  wIndex;
//    WORD  wArrayCount;
//    HisModuleAlmRcd   Rcd[20];
// }TReadHisRecTemp;

// //模组日记录读取缓冲
// typedef struct
// {
//    WORD  wCount;
//    WORD  wIndex;
//    WORD  wArrayCount;
//    HisModuleDayRcd   Rcd[20];
// }TReadDayRecTemp;

// //模组分钟记录读取缓冲
// typedef struct
// {
//    WORD  wCount;
//    WORD  wIndex;
//    WORD  wArrayCount;
//    HisModuleMinRcd   Rcd[20];
// }TReadMinRecTemp;




#endif

//End Of File.
