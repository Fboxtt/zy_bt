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
volatile uint32_t g_ticks;
int32_t P71FlushCount = 0;
uint32_t g_uartWaitTime = 0;

void delay_ms(uint32_t n)
{
    g_ticks = n;
    while(g_ticks);
}

void system_tick_init()
{
    uint32_t msCnt; 	// count value of 1ms
    g_ticks = 1000; 	// 1000ms
	// SystemCoreClockUpdate();
	msCnt = SystemCoreClock / 1000;
	SysTick_Config(msCnt); 
}
void HardFault_Handler()
{
    //printf("hardfault!");
    while(1);
}


void SysTick_Handler(void)
{
	WDT->WDTE = 0xAC;
	P71FlushCount++;
	if(P71FlushCount / 100 % 2 == 1) {
		PORT->P7 |= _02_Pn1_OUTPUT_1;
	} else {
		PORT->P7 &= (~_02_Pn1_OUTPUT_1);
	}
	g_ticks--;
	if(g_uartWaitTime > 10) {
		if(CmmuReadNumber < (3 + CommuData[1] * 0x100 + CommuData[2] + 1) && CmmuReadNumber >= 5) {
			fillbackFunc(CmdSendAll, NULL, CmdSendData[4] | 0x80, 0, 0x01);
			CmdSendFunc(CmdSendAll, 9);
			ClearCommu();
		}
	}
	g_uartWaitTime++;
	g_bootWaitTime++;
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
//	GPIO_Config();		//OK
	UART1_Init(SystemCoreClock, UartBaud);
}

int main(void)
{
    /* Start user code. Do not edit comment generated here */
	SCB->VTOR = 0x0000;
	HardDriveInit();
    BootInit();
	toggle_Init();
	toggle();
	toggle();
	BootWaitTimeInit();    
//	ReplyEnterBoot();       
	
    while (1U)
    {
		BootProcess();
    }
    /* End user code. Do not edit comment generated here */
}
