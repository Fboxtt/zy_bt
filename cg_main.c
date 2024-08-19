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
/* Start user code for include. Do not edit comment generated here */
#include <stdio.h>
#include "boot_core.h"
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"

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
uint8_t result_cmd;
extern volatile uint8_t ACK;
void delay_ms(uint32_t n)
{
    g_ticks = n;
    while(g_ticks);
}

void system_tick_init()
{
    uint32_t msCnt; 	// count value of 1ms
    g_ticks = 1000; 	// 1000ms
	SystemCoreClockUpdate();
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
int main(void)
{
    /* Start user code. Do not edit comment generated here */
    system_tick_init();
    BootInit();
    while (1U)
    {
        if(UartReceFlag)
        {
            CMDBuff = AnalysisData();           
            ClearCommu();
            if (ACK == ERR_NO) {
							result_cmd = BootCmdRun(CMDBuff); 
            }

            CommuSendCMD(result_cmd,CmmuSendLength,CmdSendData);
//                if(CMDBuff==SET_BAUD)//设置新的波特率
//                {
//                    UartInit(NewBaud);
//                }
            result_cmd = BOOT_BOOL_FALSE;

            CMDBuff = 0; 
        }
        BootCheckReset();
    }
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
void SysTick_Handler(void)
{
	WDT->WDTE = 0xAC;
	g_ticks--;
}
/* End user code. Do not edit comment generated here */
