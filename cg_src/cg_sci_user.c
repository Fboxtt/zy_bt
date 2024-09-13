/***********************************************************************************************************************
* File Name    : cg_sci_user.c
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
#include <stdio.h>
#include "communication_protocol.h"
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/



//void IRQ11_Handler(void) __attribute__((alias("uart1_interrupt_receive")));
//void IRQ11_Handler(void) __attribute__((alias("uart2_interrupt_receive")));
/* Start user code for pragma. Do not edit comment generated here */

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
extern volatile uint8_t * gp_uart0_tx_address;         /* uart0 send buffer address */
extern volatile uint16_t  g_uart0_tx_count;            /* uart0 send data number */
extern volatile uint8_t * gp_uart0_rx_address;         /* uart0 receive buffer address */
extern volatile uint16_t  g_uart0_rx_count;            /* uart0 receive data number */
extern volatile uint16_t  g_uart0_rx_length;           /* uart0 receive data length */
/* Start user code for global. Do not edit comment generated here */

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: uart0_interrupt_receive
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/



/***********************************************************************************************************************
* Function Name: uart0_interrupt_send
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void uart0_interrupt_send(void)
//{
//    INTC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
//    if (g_uart0_tx_count > 0U)
//    {
//		SCI0->TXD0 = *gp_uart0_tx_address;
//        gp_uart0_tx_address++;
//        g_uart0_tx_count--;
//    }
//    else
//    {
//        uart0_callback_sendend();
//    }
//}
/***********************************************************************************************************************
* Function Name: uart0_callback_receiveend
* Description  : This function is a callback function when UART0 finishes reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: uart0_callback_softwareoverrun
* Description  : This function is a callback function when UART0 receives an overflow data.
* Arguments    : rx_data -
*                    receive data
* Return Value : None
***********************************************************************************************************************/

/***********************************************************************************************************************
* Function Name: uart0_callback_sendend
* Description  : This function is a callback function when UART0 finishes transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
//static void uart0_callback_sendend(void)
//{
//    /* Start user code. Do not edit comment generated here */
//    UartSendFlag=1; 	 //BootLoader∑¢ÀÕ±Í÷æ
//    uart_send_flag=1;
//    /* End user code. Do not edit comment generated here */
//}
/***********************************************************************************************************************
* Function Name: uart0_callback_error
* Description  : This function is a callback function when UART0 reception error occurs.
* Arguments    : err_type -
*                    error type value
* Return Value : None
***********************************************************************************************************************/


/* Start user code for adding. Do not edit comment generated here */
//int fputc(int ch, FILE *f)
//{
//    uint8_t  tx_buf[1];
//    tx_buf[0]=ch;
//    UART0_Send(tx_buf,1);	
//    while(uart_send_flag==0);
//    uart_send_flag = 0;
//    return ch;
//}
/* End user code. Do not edit comment generated here */
