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
void IRQ10_Handler(void) __attribute__((alias("uart0_interrupt_send")));
void IRQ11_Handler(void) __attribute__((alias("uart0_interrupt_receive")));


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
uint8_t uart_send_flag = 0;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: uart0_interrupt_receive
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/


static void uart0_interrupt_receive(void)
{
    INTC_ClearPendingIRQ(SR0_IRQn); /* clear INTSR0 interrupt flag */
    uartId id = UART0;
	// interrupt_receive(UART0);

        volatile uint8_t rx_data;
    volatile uint8_t err_type;
    
    if(id == UART0) {
        err_type = (uint8_t)(SCI0->SSR01 & 0x0007U);
        SCI0->SIR01 = (uint16_t)err_type;
        rx_data = SCI0->RXD0;
    } else if(id == UART1) {
        err_type = (uint8_t)(SCI0->SSR03 & 0x0007U);
        SCI0->SIR03 = (uint16_t)err_type;
        rx_data = SCI0->RXD1;
    } else if(id == UART2) {
        err_type = (uint8_t)(SCI1->SSR11 & 0x0007U);
        SCI1->SIR11 = (uint16_t)err_type;
        rx_data = SCI1->RXD2;
    }

    // if (err_type != 0U)
    // {
    //     uart0_callback_error(err_type);
    // }
    

    UartReceData(id);
}
/***********************************************************************************************************************
* Function Name: uart0_interrupt_send
* Description  : None
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void uart0_interrupt_send(void)
{
    INTC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
    if (g_uart0_tx_count > 0U)
    {
		SCI0->TXD0 = *gp_uart0_tx_address;
        gp_uart0_tx_address++;
        g_uart0_tx_count--;
    }
    else
    {
        uart0_callback_sendend();
    }
}
/***********************************************************************************************************************
* Function Name: uart0_callback_receiveend
* Description  : This function is a callback function when UART0 finishes reception.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void uart0_callback_receiveend(void)
{
    /* Start user code. Do not edit comment generated here */
     
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: uart0_callback_softwareoverrun
* Description  : This function is a callback function when UART0 receives an overflow data.
* Arguments    : rx_data -
*                    receive data
* Return Value : None
***********************************************************************************************************************/
static void uart0_callback_softwareoverrun(uint16_t rx_data)
{
    /* Start user code. Do not edit comment generated here */
//     UartReceData();
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: uart0_callback_sendend
* Description  : This function is a callback function when UART0 finishes transmission.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
static void uart0_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    UartSendFlag=1; 	 //BootLoader∑¢ÀÕ±Í÷æ
    uart_send_flag=1;
    /* End user code. Do not edit comment generated here */
}
/***********************************************************************************************************************
* Function Name: uart0_callback_error
* Description  : This function is a callback function when UART0 reception error occurs.
* Arguments    : err_type -
*                    error type value
* Return Value : None
***********************************************************************************************************************/
static void uart0_callback_error(uint8_t err_type)
{
    /* Start user code. Do not edit comment generated here */
    /* End user code. Do not edit comment generated here */
}

/* Start user code for adding. Do not edit comment generated here */
int fputc(int ch, FILE *f)
{
    uint8_t  tx_buf[1];
    tx_buf[0]=ch;
    UART0_Send(tx_buf,1);	
    while(uart_send_flag==0);
    uart_send_flag = 0;
    return ch;
}
/* End user code. Do not edit comment generated here */
