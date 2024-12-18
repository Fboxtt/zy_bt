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
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: main
* Description  : This function implements main function.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//typedef enum {
//	PORT0 = 0,
//	PORT1,
//	PORT2,
//	PORT3,
//	PORT4,
//	PORT5,
//	PORT6,
//	PORT7,
//	PORT8,
//	PORT9,
//	PORT10,
//	PORT11,
//	PORT12,
//	PORT13,
//	PORT14,
//	
//}PORT_TypeDef;

//typedef enum {
//	PIN0 = 0,
//	PIN1,
//	PIN2,
//	PIN3,
//	PIN4,
//	PIN5,
//	PIN6,
//	PIN7,
//	
//}PIN_TypeDef;

//typedef enum {
//	INPUT = 0,
//	PULLUP_INPUT,
//	TTL_INPUT,
//	ANALOG_INPUT,
//	OUTPUT,
//	OPENDRAIN_OUTPUT,
//	
//}PIN_ModeDef;
//typedef struct 
//{
//	PORT_TypeDef	emGPIOx;		//refer to PORT_TypeDef
//	PIN_TypeDef 	emPin;			//refer to PIN_TypeDef
//	PIN_ModeDef		emMode;			//refer to PIN_ModeDef
//	uint8_t 		value;			//output TRUE: high, FALSE: low
//}TGPIO;
// TGPIO PIN_VBCTL = {PORT1,PIN5,OUTPUT};		//ok
// void PORT_SetBit(PORT_TypeDef PORTx,PIN_TypeDef PINx)
// {
// 	uint8_t pos = 1<<PINx;
// 	*((volatile uint8_t*)(&PORT->P0+PORTx)) |= pos;
// }
// #define  	VB_ON		(PORT_SetBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))	 
// #define		VB_OFF		(PORT_ClrBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))
// #define 	IS_VB_ON	(PORT_GetBit(PIN_VBCTL.emGPIOx,PIN_VBCTL.emPin))
// void P_Init(PORT_TypeDef PORTx,PIN_TypeDef PINx,PIN_ModeDef MODEx)
// {
//   	uint8_t mode = MODEx;
// 	uint8_t pos = 1<<PINx;
	
// 	switch(mode)
// 	{
// 		case INPUT:
// 			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PM0+PORTx)) |= pos;
// 			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PU0+PORTx)) &= ~pos;
// 			break;
// 		case PULLUP_INPUT:
// 			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PM0+PORTx)) |= pos;
// 			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PU0+PORTx)) |= pos;
// 			break;
// 		case TTL_INPUT:
// 			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PM0+PORTx)) |= pos;
// 			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) |= pos;
// 			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PU0+PORTx)) &= ~pos;
// 			break;
// 		case ANALOG_INPUT:
// 			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) |= pos;
// 			break;
// 		case OUTPUT:
// 			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
// 			break;
// 		case OPENDRAIN_OUTPUT:
// 			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
// 			*((volatile uint8_t*)(&PORT->POM0+PORTx)) |= pos;
// 			break;
// 	}
// }
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

int main(void)
{
    /* Start user code. Do not edit comment generated here */
	SCB->VTOR = 0x0000;
	HardDriveInit();
    BootInit();
	toggle_Init();
	toggle();
	toggle();
	// Port_Init(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin,	PIN_VBCTL.emMode); // 配置p16 p17
//	P_Init(PORT7,PIN2,OUTPUT); // 
//	P_Init(PORT7,PIN3,PULLUP_INPUT); // 
	// Port_Init(PORT2,PIN3,OUTPUT); // 配置P23引脚
	// VB_ON; // 打开VB使能RS485
	BootWaitTimeInit();    
//	ReplyEnterBoot();       
	
	// UART1_BaudRate();
    while (1U)
    {
		BootProcess();
    }
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
int32_t P71FlushCount = 0;
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
	BootWaitTime++;
}
/* End user code. Do not edit comment generated here */
