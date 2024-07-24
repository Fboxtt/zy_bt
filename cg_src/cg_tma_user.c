/***********************************************************************************************************************
* File Name    : cg_tma_user.c
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements device driver for TMA module.
* Creation Date: 2022/1/28
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "cg_macrodriver.h"
#include "cg_tma.h"
/* Start user code for include. Do not edit comment generated here */
#include "base_time_system.h"
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
void IRQ26_Handler(void) __attribute__((alias("tma0_interrupt")));
/* Start user code for pragma. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: tma0_interrupt
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void tma0_interrupt(void)
{
    INTC_ClearPendingIRQ(TMA_IRQn); /* clear INTTMA interrupt flag */
    /* Start user code. Do not edit comment generated here */
//    static uint16_t t_ms = 0;
//    t_ms++;
//    if(t_ms>=1000)
//    {
//        PORT->P7 ^= (1<<2); 	// Toggle P72
//        t_ms = 0;
//    }
    BaseTimeSystemScan();
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
