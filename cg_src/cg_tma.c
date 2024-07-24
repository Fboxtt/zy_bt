/***********************************************************************************************************************
* File Name    : cg_tma.c
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
* Function Name: TMA0_Init
* Description  : This function initializes the TMA module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void TMA0_Init(void)
{
    CGC->PER1 |= CGC_PER1_TMAEN_Msk;     /* enables input clock supply */
    TMA->TACR0 &= (uint8_t)~_01_TMA_COUNT_START;
    INTC_DisableIRQ(TMA_IRQn);/* disable INTTMA interrupt */
    INTC_ClearPendingIRQ(TMA_IRQn);/* clear INTTMA interrupt flag */
    TMA->TAMR0 = _10_TMA_COUNT_SOURCE_FCLK8 | _00_TMA_MODE_TIMER;
    TMA->TA0 = _9C3F_TMA_TA0_VALUE;
}
/***********************************************************************************************************************
* Function Name: TMA0_Start
* Description  : This function starts TMA counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void TMA0_Start(void)
{
    INTC_ClearPendingIRQ(TMA_IRQn);/* clear INTTMA interrupt flag */
    INTC_EnableIRQ(TMA_IRQn);/* enable INTTMA interrupt */
    TMA->TACR0 |= _01_TMA_COUNT_START;
}
/***********************************************************************************************************************
* Function Name: TMA0_Stop
* Description  : This function stops TMRJ0 counter.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void TMA0_Stop(void)
{
    TMA->TACR0 &= (uint8_t)~_01_TMA_COUNT_START;
    INTC_DisableIRQ(TMA_IRQn);/* disable INTTMA interrupt */
    INTC_ClearPendingIRQ(TMA_IRQn);/* clear INTTMA interrupt flag */
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
