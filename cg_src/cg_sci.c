/***********************************************************************************************************************
* File Name    : cg_sci.c
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements device driver for SCI module.
* Creation Date: 2022/1/28
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "cg_macrodriver.h"
#include "cg_sci.h"
/* Start user code for include. Do not edit comment generated here */
#include "stdlib.h"
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"
#include "boot.h"
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
const uint16_t sps_tbl[16] = {1, 2, 4, 8, 16, 32, 64, 128, 256, 512, 1024, 2048, 4096, 8192, 16384, 32768};
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/* Start user code for global. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: SCI0_Init
* Description  : This function initializes the SAU0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void SCI0_Init(void)
{
	CGC->PER0 |= CGC_PER0_SCI0EN_Msk;
    NOP();
    NOP();
    NOP();
    NOP();
    SCI0->SPS0 = _0040_SCI_CK01_fCLK_4 | _0004_SCI_CK00_fCLK_4;
    UART0_Init();
}

/* ToDo: You can allocate the TXD0 to P51, P17, P40 or P12 with PIOR35, PIOR43 and PIOR01 register */
#define TXD0_PORT_SETTING() do{ \
        PORT->PIOR0 |=  (1 << 1);    /* allocate TXD0 to P17 */ \
        PORT->P1    |=  (1 << 7);    /* P17 output high level */ \
        PORT->PM1   &= ~(1 << 7);    /* P17 is used as TXD0 output */ \
        PORT->POM1  &= ~(1 << 7);    /* P17 is push-pull output mode */ \
}while(0)

/* ToDo: You can allocate the RXD0 to P50, P16, P137 or P11 with PIOR35, PIOR43 and PIOR01 register */
#define RXD0_PORT_SETTING() do{ \
        PORT->PIOR0 |=  (1 << 1);    /* allocate RXD0 to P16 */ \
        PORT->PM1   |=  (1 << 6);    /* P16 is used as RXD0 input */ \
}while(0)

/***********************************************************************************************************************
* Function Name: UART0_Init
* Description  : This function initializes the UART0 module.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void UART0_Init(void)
{
    SCI0->ST0 |= _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
    INTC_DisableIRQ(ST0_IRQn);/* disable INTST0 interrupt */
    INTC_ClearPendingIRQ(ST0_IRQn);/* clear INTST0 interrupt flag */
    INTC_DisableIRQ(SR0_IRQn);/* disable INTSR0 interrupt */
    INTC_ClearPendingIRQ(SR0_IRQn);/* clear INTSR0 interrupt flag */
    INTC_DisableIRQ(SRE0_IRQn);/* disable INTSRE0 interrupt */
    INTC_ClearPendingIRQ(SRE0_IRQn);/* clear INTSRE0 interrupt flag */
    NVIC_SetPriority(SR0_IRQn, 3);/* Set INTSR0 low priority */
    NVIC_SetPriority(ST0_IRQn, 3);/* Set INTST0 low priority */
    SCI0->SMR00 = _0020_SMR00_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS | _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR00 = _0004_SCR00_DEFAULT_VALUE | _8000_SCI_TRANSMISSION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK | _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR00 = _CE00_SCI0_CH0_BAUDRATE_DIVISOR;
    MISC->NFEN0 |= _01_SCI_RXD0_FILTER_ON;
    SCI0->SIR01 = _0004_SCI_SIRMN_FECTMN | _0002_SCI_SIRMN_PECTMN | _0001_SCI_SIRMN_OVCTMN;
    SCI0->SMR01 = _0020_SMR01_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS | _0100_SCI_TRIGGER_RXD | _0000_SCI_EDGE_FALL | _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR01 = _0004_SCR01_DEFAULT_VALUE | _4000_SCI_RECEPTION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK | _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR01 = _CE00_SCI0_CH1_BAUDRATE_DIVISOR;
    SCI0->SO0 |= _0001_SCI_CH0_DATA_OUTPUT_1;
    SCI0->SOL0 &= (uint16_t)~_0001_SCI_CHANNEL0_INVERTED;
    SCI0->SOE0 |= _0001_SCI_CH0_OUTPUT_ENABLE;
    /* Set RxD0 pin */
	RXD0_PORT_SETTING();
	TXD0_PORT_SETTING();
}
/***********************************************************************************************************************
* Function Name: UART0_Start
* Description  : This function starts the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void UART0_Start(void)
{
    SCI0->SO0 |= _0001_SCI_CH0_DATA_OUTPUT_1;
    SCI0->SOE0 |= _0001_SCI_CH0_OUTPUT_ENABLE;
    SCI0->SS0 |= _0002_SCI_CH1_START_TRG_ON | _0001_SCI_CH0_START_TRG_ON;
    INTC_ClearPendingIRQ(ST0_IRQn);/* clear INTST0 interrupt flag */
    INTC_ClearPendingIRQ(SR0_IRQn);/* clear INTSR0 interrupt flag */
    INTC_EnableIRQ(ST0_IRQn);/* enable INTST0 interrupt */
    INTC_EnableIRQ(SR0_IRQn);/* enable INTSR0 interrupt */
}
/***********************************************************************************************************************
* Function Name: UART0_Stop
* Description  : This function stops the UART0 module operation.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void UART0_Stop(void)
{
    INTC_DisableIRQ(ST0_IRQn);/* disable INTST0 interrupt */
    INTC_DisableIRQ(SR0_IRQn);/* disable INTSR0 interrupt */
    SCI0->ST0 |= _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
    SCI0->SOE0 &= (uint16_t)~_0001_SCI_CH0_OUTPUT_ENABLE;
    INTC_ClearPendingIRQ(ST0_IRQn);/* clear INTST0 interrupt flag */
    INTC_ClearPendingIRQ(SR0_IRQn);/* clear INTSR0 interrupt flag */
}
/***********************************************************************************************************************
* Function Name: UART0_Receive
* Description  : This function receives UART0 data.
* Arguments    : rx_buf -
*                    receive buffer pointer
*                rx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS UART0_Receive(uint8_t * const rx_buf, uint16_t rx_num)
{
    MD_STATUS status = MD_OK;

    if (rx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        g_uart0_rx_count = 0U;
        g_uart0_rx_length = rx_num;
        gp_uart0_rx_address = rx_buf;
    }

    return (status);
}
/***********************************************************************************************************************
* Function Name: UART0_Send
* Description  : This function sends UART0 data.
* Arguments    : tx_buf -
*                    transfer buffer pointer
*                tx_num -
*                    buffer size
* Return Value : status -
*                    MD_OK or MD_ARGERROR
***********************************************************************************************************************/
MD_STATUS UART0_Send(uint8_t * const tx_buf, uint16_t tx_num)
{
    MD_STATUS status = MD_OK;

    if (tx_num < 1U)
    {
        status = MD_ARGERROR;
    }
    else
    {
        gp_uart0_tx_address = tx_buf;
        g_uart0_tx_count = tx_num;
		INTC_DisableIRQ(ST0_IRQn);    /* disable INTST0 interrupt */
        SCI0->TXD0 = *gp_uart0_tx_address;
        gp_uart0_tx_address++;
        g_uart0_tx_count--;
		INTC_EnableIRQ(ST0_IRQn);    /* enable INTST0 interrupt */
    }

    return (status);
}

/* Start user code for adding. Do not edit comment generated here */
/***********************************************************************************************************************
* Function Name: UART_BaudRateCal
* @brief  This function search the setting value for specified freq and baud of UART
* @param  fclk_freq
*             - the frequency of fCLK clock. unit Hz.
* @param  baud
*             - the target baud rate, unit bps.
* @param  pvalue
*             - the pointer of calculated result
* @return MD_OK, MD_ERROR
***********************************************************************************************************************/
MD_STATUS UART_BaudRateCal(uint32_t fclk_freq, uint32_t baud, uart_baud_t *pvalue)
{
    MD_STATUS status = MD_ERROR;

    int32_t baud_err;
    uint32_t baud_cal;
    uint32_t fmck_freq;
    unsigned char i, j;

    for (i = 0; i < 16; i++)
    {
        fmck_freq = fclk_freq / sps_tbl[i];
        for (j = 2; j < 128; j++)
        {
            baud_cal = fmck_freq / (j + 1) / 2;
            baud_err = 10000 * baud_cal / baud - 10000;  /* n ten thousandths */
            if (abs(baud_err) < 20)  /* 0.2% */
            {
                pvalue->prs = i;
                pvalue->sdr = j;
                //printf("fclk_freq = %10dHz, baud = %6dbps, prs = %2d, sdr = %3d, errors = %3d\n", fclk_freq, baud, pvalue->prs, pvalue->sdr, baud_err);
                status = MD_OK;
                return (status);
            }
        }
    }

    return (status);
}
MD_STATUS UART0_BaudRate(uint32_t fclk_freq, uint32_t baud)
{
    MD_STATUS status;
    uart_baud_t pvalue;

#ifndef RTL_SIMULATION
    status = UART_BaudRateCal(fclk_freq, baud, &pvalue);
#else
    pvalue.prs = 0x4;
    pvalue.sdr = 0x4D;
    status = MD_OK;
#endif

    if (status == MD_OK)
    {
        SCI0->ST0 = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
        SCI0->SPS0 &= ~SCI0_SPS0_PRS00_Msk;
        SCI0->SPS0 |= pvalue.prs;
        SCI0->SDR00 = pvalue.sdr << 9;
        SCI0->SDR01 = pvalue.sdr << 9;
        SCI0->SS0 |= _0002_SCI_CH1_START_TRG_ON | _0001_SCI_CH0_START_TRG_ON;
    }

    return (status);
}
/* End user code. Do not edit comment generated here */
