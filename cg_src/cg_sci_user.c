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

#include "includes.h"

void UartReceData(uartId id)//?????
{
	if(!UartReceFlag)
	{		
		// if(id == UART0) {
		// 	CommuData[CmmuReadNumber] = SCI0->RXD0;
		// }else if(id == UART1) {
		// 	CommuData[CmmuReadNumber] = SCI0->RXD1;
		// }else if(id == UART2) {
		// 	CommuData[CmmuReadNumber] = SCI1->RXD2;
		// }
		if(id == UART1) {
			CommuData[CmmuReadNumber] = SCI0->RXD1;
		}
		CmmuReadNumber++;
		g_uartWaitTime = 0;

		if(CmmuReadNumber >= 3) {
			if(CmmuReadNumber>=(3 + CommuData[1] * 0x100 + CommuData[2] + 1)) //CmmuReadNumber??
			{
				
				CmmuLength = 3 + CommuData[1] * 0x100 + CommuData[2] + 1;
				UartReceFlag = 1;
			}
		}

	}
}
