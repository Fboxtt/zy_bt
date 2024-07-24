/***********************************************************************************************************************
* File Name    : cg_cgc.c
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements device driver for CGC module.
* Creation Date: 2022/1/28
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "cg_macrodriver.h"
#include "cg_cgc.h"
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
* Function Name: CLK_Init
* Description  : This function initializes the clock generator.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void CLK_Init(void)
{
    CGC->CMC = _00_CGC_HISYS_PORT | _00_CGC_SUB_PORT | _00_CGC_SYSOSC_UNDER10M;
    /* Set fMX */
    CGC->CSC |= _80_CGC_HISYS_STOP;
    /* Set fSUB */
    CGC->CSC |= _40_CGC_FSUB_STOP;
    CGC->OSMC = _10_CGC_TMA_IT_CLK_FIL;
    /* Set fCLK */
    CGC->CKC &= (uint8_t)~_40_CGC_MAINCLK_FSUB;
    /* Set fMAIN */
    CGC->CKC &= (uint8_t)~_10_CGC_MAINCLK_SELHISYS;
    /* Set fIH */
    CGC->CSC &= (uint8_t)~_01_CGC_HIO_STOP;
    /* Set fRTC */
    MISC->RTCCL = _01_RTC_FRTC_FIL;

}



/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
