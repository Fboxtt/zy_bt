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

void UartReceData(uartId id)
{
	if(CmmuReadNumber >= ReceiveLength1) {
		CmmuReadNumber = 0;
	}
	if(id == UART1) {
		CommuData[CmmuReadNumber] = SCI0->RXD1;
	}
	CmmuReadNumber++;
	g_uartWaitTime = 0;
}
