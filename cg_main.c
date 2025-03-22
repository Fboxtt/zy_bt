/***********************************************************************************************************************
* File Name    : cg_main.c
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements main function.
* Creation Date: 2022/1/28
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "cg_macrodriver.h"
#include <stdio.h>
#include "BAT32G137.h"
#include "cg_cgc.h"
#include "cg_port.h"
#include "cg_tma.h"
#include "cg_wdt.h"
#include "cg_sci.h"
#include "clk.h"
/* Start user code for include. Do not edit comment generated here */
#include <stdio.h>
#include "boot_core.h"
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"

#include "boot.h"
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/*
    P50->RX0
    P51->TX0
    初始波特率->9600
    系统主频->48M
    RST->使能
*/
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
// volatile uint32_t g_ticks;
int32_t P71FlushCount = 0;
uint32_t g_uartWaitTime = 0;
uint8_t g_bWholeSysShutdown = 0;
uint8_t g_boot100MsCount = 0;
uint32_t g_vbOffWaitTime = 0;
typedef struct 
{
	PORT_TypeDef	emGPIOx;		//refer to PORT_TypeDef
	PIN_TypeDef 	emPin;			//refer to PIN_TypeDef
	PIN_ModeDef		emMode;			//refer to PIN_ModeDef
	uint8_t 		value;			//output TRUE: high, FALSE: low
}TGPIO;

extern TGPIO PIN_SW; 	
extern TGPIO PIN_HEATE_N;	
extern TGPIO PIN_ALERT;	

extern TGPIO PIN_VBCTL; 
extern TGPIO PIN_CDEN; 	
extern TGPIO PIN_CEN; 	
extern TGPIO PIN_GREEN; 
extern TGPIO PIN_RED; 	
//extern TGPIO PIN_485DE; 
extern TGPIO PIN_FUSE_EN; 	
extern TGPIO PIN_WAKE; 		
//extern TGPIO PIN_PACKADC_EN; 

extern TGPIO PIN_COM3V3_EN;	 
extern TGPIO PIN_COM5V_EN;	 
extern TGPIO PIN_REGOUT_EN;	 
//extern void toggle();
TGPIO PIN_SW 	= {PORT1,PIN6,PULLUP_INPUT};

TGPIO PIN_VBCTL = {PORT1,PIN5,OUTPUT};		//ok
TGPIO PIN_RED   = {PORT12,PIN0,OUTPUT};		//ok
TGPIO PIN_GREEN = {PORT4,PIN1,OUTPUT};		//ok

/********************************************************************************
GPIO操作定义,所有引脚电平需要定义
********************************************************************************/
//定义电源控制
#define  	VB_ON		(PORT_SetBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))	 
#define		VB_OFF		(PORT_ClrBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))
#define 	IS_VB_ON	(PORT_GetBit(PIN_VBCTL.emGPIOx,PIN_VBCTL.emPin))

//定义按键输入
#define		IS_SWITCH_PUSH	((PORT_GetBit(PIN_SW.emGPIOx,PIN_SW.emPin)))

//绿灯LED
#define		GREEN_ON		(PORT_SetBit  (PIN_GREEN.emGPIOx,	PIN_GREEN.emPin))  //PORT_ClrBit
#define		GREEN_OFF		(PORT_ClrBit(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin))   //PORT_SetBit
#define		IS_GREEN_ON		(!PORT_GetBit(PIN_GREEN.emGPIOx,PIN_GREEN.GPIO_Pin))
#define		GREEN_REVERSE	(PORT_ToggleBit(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin))	

//红灯LED
#define		RED_ON			(PORT_SetBit(PIN_RED.emGPIOx,	PIN_RED.emPin))
#define		RED_OFF			(PORT_ClrBit(PIN_RED.emGPIOx,	PIN_RED.emPin))
#define		IS_RED_ON		(!PORT_GetBit(PIN_RED.emGPIOx,PIN_RED.emPin))
#define		RED_REVERSE		(PORT_ToggleBit(PIN_RED.emGPIOx,	PIN_RED.emPin))	

TUartData g_tUartData;

void GPIO_Config(void)
{
	//输入
	PORT_Init(PIN_SW.emGPIOx,		PIN_SW.emPin,		PIN_SW.emMode);	
//	PORT_Init(PIN_ALERT.emGPIOx,	PIN_ALERT.emPin,	PIN_ALERT.emMode);	
	PORT_Init(PORT5,PIN1,INPUT);    //485唤醒
	PORT_Init(PORT14,PIN0,PULLUP_INPUT);
	//PORT_Init(PIN_REV.emGPIOx,		PIN_REV.emPin,		PIN_REV.emMode);
  	
	//输出
	PORT_Init(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin,	PIN_VBCTL.emMode);
	VB_ON;
   
// 	PORT_Init(PIN_COM5V_EN.emGPIOx,PIN_COM5V_EN.emPin,PIN_COM5V_EN.emMode);
//   PIN_COM5V_OFF;
	
// 	PORT_Init(PIN_COM3V3_EN.emGPIOx,PIN_COM3V3_EN.emPin,PIN_COM3V3_EN.emMode);
// 	PIN_COM3V3_OFF; 
	
//   PORT_Init(PIN_HEATE_N.emGPIOx,	PIN_HEATE_N.emPin,	PIN_HEATE_N.emMode);
//   HEAT_OFF;
	
// 	PORT_Init(PIN_CDEN.emGPIOx,		PIN_CDEN.emPin,		PIN_CDEN.emMode);
// 	CD_OFF;
	
// 	PORT_Init(PIN_CEN.emGPIOx,		PIN_CEN.emPin,		PIN_CEN.emMode);
// 	C_OFF;
	
	PORT_Init(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin,	PIN_GREEN.emMode);
	GREEN_OFF;
	
	PORT_Init(PIN_RED.emGPIOx,		PIN_RED.emPin,		PIN_RED.emMode);
	RED_ON;
	
// 	PORT_Init(PIN_DEBUG.emGPIOx,		PIN_DEBUG.emPin,		PIN_DEBUG.emMode);
// 	DEBUG_LED_ON;
	

// 	PORT_Init(PIN_FUSE_EN.emGPIOx,	PIN_FUSE_EN.emPin,	PIN_FUSE_EN.emMode);
// 	FUSE_ON;
	
	PORT_Init(PORT5,PIN0,OUTPUT);    //CTLD
	// PORT_SetBit(PORT5,PIN0);
	
	//PORT_Init(PIN_HEATFUSE_EN.emGPIOx,PIN_HEATFUSE_EN.emPin,PIN_HEATFUSE_EN.emMode);
	//HEATFUSE_ON;
    
	//PORT_Init(PIN_PACKADC_EN.emGPIOx,	PIN_PACKADC_EN.emPin,	PIN_PACKADC_EN.emMode);
	//PACKADC_ENABLE;
	
	//PORT_Init(PIN_485DE.emGPIOx,	PIN_485DE.emPin,	PIN_485DE.emMode);
	//RS485_SEND_DISABLE;
	
// 	//未使用管脚配置
// 	PORT_Init(PORT13,PIN6,OUTPUT);
// 	PORT_Init(PORT7,PIN5,OUTPUT); 
// 	PORT_Init(PORT7,PIN4,OUTPUT); 
// 	PORT_Init(PORT3,PIN0,OUTPUT);
// 	PORT_Init(PORT1,PIN2,OUTPUT);
//   PORT_Init(PORT1,PIN0,OUTPUT);  
// 	PORT_Init(PORT2,PIN0,OUTPUT);
// 	PORT_Init(PORT2,PIN1,OUTPUT);
// 	PORT_Init(PORT2,PIN2,OUTPUT);
//   PORT_Init(PORT2,PIN3,OUTPUT);
//   PORT_Init(PORT2,PIN4,OUTPUT);
//   PORT_Init(PORT2,PIN5,OUTPUT);
// 	PORT_Init(PORT13,PIN0,OUTPUT); 
	
}

void delay_ms(uint32_t n)
{
    // g_ticks = n;
    // while(g_ticks);
}

void system_tick_init()
{
    uint32_t msCnt; 	// count value of 1ms
    // g_ticks = 1000; 	// 1000ms
	// SystemCoreClockUpdate();
	msCnt = SystemCoreClock / (1000 / TIME_UNIT);
	SysTick_Config(msCnt); 
}
void HardFault_Handler()
{
    //printf("hardfault!");
    while(1);
}


void SysTick_Handler(void)
{
	P71FlushCount++;
	if(P71FlushCount  % 2 == 0) {
		PORT->P7 |= _02_Pn1_OUTPUT_1;
	} else {
		PORT->P7 &= (~_02_Pn1_OUTPUT_1);
	}
//	toggle();
	// g_ticks--;
	if(CmmuReadNumber != 0) {
		g_uartWaitTime++;
	}
	g_bootWaitTime++;
	// g_vbOffWaitTime++;
	g_boot100MsCount++;
}

void toggle_Init(void)
{
    // PORT->P7 = _04_Pn2_OUTPUT_1 | _02_Pn1_OUTPUT_1;
    PORT->P7 |= _02_Pn1_OUTPUT_1;
    PORT->PU7 |= _01_PUn0_PULLUP_ON;
    PORT->POM7 &= (~_02_POMn1_NCH_ON);
    // PORT->PM7 = _00_PMn2_MODE_OUTPUT | _00_PMn1_MODE_OUTPUT | _01_PMn0_MODE_INPUT;
    PORT->PM7 &= (~_02_PMn1_MODE_INPUT);
}
void toggle(void)
{
	// PORT->P7 = _04_Pn2_OUTPUT_1 | _02_Pn1_OUTPUT_1;
	// PORT->P7 = _00_Pn2_OUTPUT_0 | _00_Pn1_OUTPUT_0;
    PORT->P7 |= _02_Pn1_OUTPUT_1;
	PORT->P7 &= (~_02_Pn1_OUTPUT_1);
	PORT->P7 |= _02_Pn1_OUTPUT_1;
}

void Clock_Config(void)
{	
	uint32_t msCnt = 0;
	
	CLK_Osc_Setting(OSC_OSCILLATOR, OSC_OSCILLATOR); /* MainOSC/SubOSC enable */
	CLK_MainOsc_Setting(OSC_OSCILLATOR,OSC_OVER_10M);
	CLK_Fclk_Select(MAINCLK_FMX);//select FMX
	while((CGC->CKC & CGC_CKC_MCS_Msk) == 0);
	
	SystemCoreClock = 12000000;  		//12000000 外部晶振输入12M
	msCnt = SystemCoreClock / 1000;; 	// count value of 1ms
	SysTick_Config(msCnt); 				//系统计数器初始化
}

void HardDriveInit(void)
{
	Clock_Config();		//OK
	system_tick_init();
	GPIO_Config();		//OK
	UART1_Init(SystemCoreClock, UartBaud);
}

//uint8_t g_bWholeSysShutdown = 0;

void CheckSwitch(void)
{
	static BYTE	Switch_Count = 0;
	static BYTE NO_Siwtch_Count = 0;
	static uint16_t LED_CHANGE_COUNT = 0;
	if (!IS_SWITCH_PUSH)		//非高电平，等于按下按键
	{
		if(Switch_Count == 0) {
		}
		NO_Siwtch_Count = 0;
		Switch_Count++;
		if(Switch_Count > 20)
		{
			RED_OFF;
			GREEN_OFF;
		}
	} else {

		NO_Siwtch_Count++;
		if(NO_Siwtch_Count > 5) {
			Switch_Count = 0;
			NO_Siwtch_Count = 0;
		}
		if(Switch_Count > 20)
		{
			g_bWholeSysShutdown = 1;	
			// Switch_Count = 52;
			// SetBmsEventAct(xEVENT_MANNULA_DOWN,TRUE);
		} else {
			LED_CHANGE_COUNT++;
			if(LED_CHANGE_COUNT / 20 % 2 == 0) {
				RED_ON;
				GREEN_OFF;
			} else {
				GREEN_ON;
				RED_OFF;
			}
		}
	}

	//系统关机
	if(g_bWholeSysShutdown)
	{
//		g_stSBS.ulFaultStatus |= FAULT_SWITCH_OFF;	
		g_bWholeSysShutdown = 0;
		VB_OFF;
	}
}

void CmdSendFunc(uint8_t *sBuff, uint32_t lenth)
{
	uint32_t i;
	for(i = 0; i < lenth; i++) {
		UartSendOneByte(*(sBuff + i));
	}
}

uint8_t CompareArray() 
{
	uint8_t BLENotConnectCmd[3] = {0x45,0x52,0x52};
	int i = 0;
	while(i < 3) {
		if(BLENotConnectCmd[i] != CommuData[i]) {
			return 0;
		}
		i++;
	}
	return 1;
}
// 重置向量表
void __set_VECTOR_ADDR(uint32_t addr)
{
	SCB->VTOR = addr;
}
#ifdef BMS_BT_DEVICE
void IAPEnterApp()
{
	BaseTimeSystemInit(BOOT_DISABLE);	//关闭定时器
	SCI0->ST0   = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
	CGC->PER0 &= ~CGC_PER0_SCI0EN_Msk;
	INTC_DisableIRQ(SR0_IRQn);
	__set_VECTOR_ADDR(APP_VECTOR_ADDR); // 需要配置向量表，因为实测发现app发生中断依然会跳到bt的systick
	__set_MSP(*(__IO uint32_t*) APP_ADDR);

	((void (*)()) (*(volatile unsigned long *)(APP_ADDR+0x04)))();//to APP
    NVIC_SystemReset();					//如果无法进入APP则复位
}
#endif

void CheckAndEnterApp(void)
{
	if(CheckSumCheck(APROM_AREA) == 1) { // 如果时间到，校验App数据，正确则进入APP
		IAPEnterApp();
	} else if(CheckSumCheck(APROM_BUFF_AREA) == 1) {
		// 如果因为意外使APP损坏，将缓冲区APP复制过来
		IAP_Erase_Some(BUFFER_RESTORE_ADDRESS, 4);
		uint32ValWrite(RESTORE_BUFF, BUFFER_RESTORE_ADDRESS);
	} else if(CheckSumCheck(APROM_BACKUP_AREA) == 1) {
		IAP_Erase_Some(BACKUP_RESTORE_ADDRESS, 4);
		uint32ValWrite(RESTORE_BKP, BACKUP_RESTORE_ADDRESS);
	}
}
/********************************************************************************************************
**函数原型		:   WORD CalCRC (BYTE *ptr,int count)
**输入参数		: 	*ptr：待计算CRC的数据的首地址；   count：长度
**输出参数      :   无
**返回值		:	CRC计算结果
**说明		    :	
*********************************************************************************************************/
WORD CalCRC (BYTE *ptr,int count)
{
  WORD crc = 0;
  BYTE  i;
  while(--count>=0)
  {
    crc = crc^(int)*ptr++<<8;
    i = 8;
    do
   {
      if(crc & 0x8000)
        crc = crc<<1^0x1021;
      else
        crc = crc<<1;
    }while(--i);
  }
  return (crc)	 ;
}


int main(void)
{
    /* Start user code. Do not edit comment generated here */
	SCB->VTOR = 0x0000;    
	uint8_t openBootCmd[9] = {0x00,0x00,0x05,0x01,0x7B,0x55,0xAA,0x00,0x80};
	
	HardDriveInit();
    BootInit();
	toggle_Init();
	CmdSendFunc(openBootCmd, sizeof(openBootCmd));

    while (1U)
    {
		WDT->WDTE = 0xAC;
		if(g_boot100MsCount / TICK_100MS_COUNT > 0) {
			g_boot100MsCount = 0;
			CheckSwitch();
			AppRestore();
			if(g_waitFlag == SHORT_WAIT) {
				if(g_bootWaitTime > NO_CMD_BOOT_WAIT_LIMIT) {
					g_bootWaitTime = 0;
					CheckAndEnterApp();
				}
			} else if(g_waitFlag == LONG_WAIT) {
				if(g_bootWaitTime > YES_CMD_BOOT_WAIT_LIMIT) {
					g_bootWaitTime = 0;
					CheckAndEnterApp();
					g_waitFlag = SHORT_WAIT;
				}
			} else if(g_waitFlag == VB_WAIT) {
				if(g_bootWaitTime > VB_OFF_WAIT_TIME) {
					g_bootWaitTime = 0;
					RED_OFF;
					VB_OFF;
					g_waitFlag = SHORT_WAIT;
				}
			}
		}
		if(g_uartWaitTime > DELAY_RETURN_COUNT) {
			if(CmmuReadNumber < (3 + CommuData[1] * 0x100 + CommuData[2] + 1) && CmmuReadNumber >= 5) {
				if(CmmuReadNumber == 7 && CompareArray() == 1) {
					//蓝牙模块未连接时会发送ERROR字符串，这个判断是为了隔离这个影响
				} else {
					fillbackFunc(CmdSendAll, NULL, CmdSendData[4] | 0x80, 0, 0x01);
					CmdSendFunc(CmdSendAll, 9);
				}
			} else if (CmmuReadNumber >= 8){
				g_tUartData.pbuf = CommuData;
				g_tUartData.wLen = CmmuReadNumber;
				DownloadProcess(&g_tUartData,0);
			}
			ClearCommu();
			g_uartWaitTime = 0;
		}
    }

	
    /* End user code. Do not edit comment generated here */
}
