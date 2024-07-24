/***********************************************************************************************************************
* File Name    : cg_wdt.c
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements device driver for WDT module.
* Creation Date: 2022/1/28
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "cg_macrodriver.h"
#include "cg_wdt.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: WDT_Init
* Description  : This function initializes the watchdogtimer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void WDT_Init(void)
{
    INTC_DisableIRQ(NonMaskableInt_IRQn);/* disable INTWDT interrupt */
    INTC_ClearPendingIRQ(NonMaskableInt_IRQn);/* clear INTWDT interrupt flag */
    NVIC_SetPriority(NonMaskableInt_IRQn, 3);/* Set INTWDT low priority */
    INTC_EnableIRQ(NonMaskableInt_IRQn);/* enable INTWDT interrupt */
}
/***********************************************************************************************************************
* Function Name: WDT_Restart
* Description  : This function restarts the watchdog timer.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void WDT_Restart(void)
{
    WDT->WDTE = 0xACU;     /* restart watchdog timer */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
