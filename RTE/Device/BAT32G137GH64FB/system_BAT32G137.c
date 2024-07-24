/***********************************************************************************************************************
* File Name    : system_BAT32G137.c
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements system initializing function.
* Creation Date: 2022/1/28
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "cg_macrodriver.h"
#include <stdint.h>
#include "BAT32G137.h"
#include "cg_cgc.h"
#include "cg_port.h"
#include "cg_tma.h"
#include "cg_wdt.h"
#include "cg_sci.h"
/* Start user code for include. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#include "cg_userdefine.h"

/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/
/* Start user code for pragma. Do not edit comment generated here */
typedef enum {
  HOCO_FREQ_64MHZ = 0xF8,   /*!< fHOCO = 64MHz, fIH = 32MHz     */
  HOCO_FREQ_48MHZ = 0xF0,   /*!< fHOCO = 48MHz, fIH = 48MHz     */
  HOCO_FREQ_32MHZ = 0xE8,   /*!< fHOCO = 32MHz, fIH = 32MHz     */
  HOCO_FREQ_24MHZ = 0xE0,   /*!< fHOCO = 24MHz, fIH = 24MHz     */
  HOCO_FREQ_16MHZ = 0xE9,   /*!< fHOCO = 32MHz, fIH = 16MHz     */
  HOCO_FREQ_12MHZ = 0xE1,   /*!< fHOCO = 24MHz, fIH = 12MHz     */
  HOCO_FREQ_8MHZ  = 0xEA,   /*!< fHOCO = 32MHz, fIH =  8MHz     */
  HOCO_FREQ_6MHZ  = 0xE2,   /*!< fHOCO = 24MHz, fIH =  6MHz     */
  HOCO_FREQ_4MHZ  = 0xEB,   /*!< fHOCO = 32MHz, fIH =  4MHz     */
  HOCO_FREQ_3MHZ  = 0xE3,   /*!< fHOCO = 24MHz, fIH =  3MHz     */
  HOCO_FREQ_2MHZ  = 0xEC,   /*!< fHOCO = 32MHz, fIH =  2MHz     */
  HOCO_FREQ_1MHZ  = 0xED    /*!< fHOCO = 32MHz, fIH =  1MHz     */
} hoco_freq_t;
/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/
const uint8_t user_opt_data[4] __attribute__((at(0x000000C0))) =
{
	0xFF,
	0xFF,
	0xF0,
	0xFF
};
/* Start user code for global. Do not edit comment generated here */
uint32_t SystemCoreClock;       /* System Clock Frequency (Core Clock)*/
/*----------------------------------------------------------------------------
  Clock functions
 *----------------------------------------------------------------------------*/
__WEAK uint32_t CLK_GetHocoFreq(void)
{

  uint32_t freq;
  uint8_t  frqsel   = (*(uint8_t *)0x000000C2U);
           frqsel  &= 0x18;  /* Mask the higher and lower 3 bits */
           frqsel >>= 3;     /* right shift 3 bit */
           
  switch(frqsel)
  {
      case 0x03: 
          freq = 64000000U;     /* fHOCO = 64MHz    */
          break;
      case 0x02: 
          freq = 48000000U;     /* fHOCO = 48MHz    */
          break;
      case 0x01: 
          freq = 32000000U;     /* fHOCO = 32MHz    */
          break;
      case 0x00: 
          freq = 24000000U;     /* fHOCO = 24MHz    */
          break;
  }

  return(freq);
}

__WEAK uint32_t CLK_GetCoreFreq(void)
{

  uint32_t freq;
  uint8_t  frqsel  = (*(uint8_t *)0x000000C2U);
           frqsel &= 0xF8;          /* Mask the lower 3 bits */
           frqsel |= CGC->HOCODIV;  /* Refer the value of HOCODIV */ 
           
  freq = 1000000U;  /* fIH = 1MHz except for the following cases */
        
  switch(frqsel)
  {
      case HOCO_FREQ_64MHZ: 
          freq = 32000000U;     /* fIH = 32MHz  */
          break;
      case HOCO_FREQ_48MHZ: 
          freq = 48000000U;     /* fIH = 48MHz  */
          break;
      case HOCO_FREQ_32MHZ: 
          freq = 32000000U;     /* fIH = 32MHz  */
          break;
      case HOCO_FREQ_24MHZ: 
          freq = 24000000U;     /* fIH = 24MHz  */
          break;
      case HOCO_FREQ_16MHZ: 
          freq = 16000000U;     /* fIH = 16MHz  */
          break;
      case HOCO_FREQ_12MHZ: 
          freq = 12000000U;     /* fIH = 12MHz  */
          break;
      case HOCO_FREQ_8MHZ: 
          freq = 8000000U;      /* fIH = 8MHz   */
          break;
      case HOCO_FREQ_6MHZ: 
          freq = 6000000U;      /* fIH = 6MHz   */
          break;
      case HOCO_FREQ_4MHZ: 
          freq = 4000000U;      /* fIH = 4MHz   */
          break;
      case HOCO_FREQ_3MHZ: 
          freq = 3000000U;      /* fIH = 3MHz   */
          break;
      case HOCO_FREQ_2MHZ: 
          freq = 2000000U;      /* fIH = 2MHz   */
          break;
      case HOCO_FREQ_1MHZ: 
          freq = 1000000U;      /* fIH = 1MHz   */
          break;
  }

  return(freq);
}

__WEAK void SystemCoreClockUpdate (void)  /* Get Core Clock Frequency */
{
/* ToDo: add code to calculate the system frequency based upon the current
         register settings.
         This function can be used to retrieve the system core clock frequeny
         after user changed register sittings. */
  SystemCoreClock = CLK_GetCoreFreq();
}

/* End user code. Do not edit comment generated here */

/***********************************************************************************************************************
* Function Name: SystemInit
* Description  : This function initializes every macro.
* Arguments    : None
* Return Value : None
***********************************************************************************************************************/
void SystemInit (void)
{
    /* RAM Parity Reset disable */
    SAF->RPECTL = 0x80U;
    /* NVIC Clear Pending IRQs */
    NVIC->ICPR[0U] = 0xFFFFFFFF;
    /* NVIC Enable IRQs */
    NVIC->ISER[0U] = 0xFFFFFFFF;
    /* NVIC Lower Priority */
    NVIC->IP[0U] = 0xC0C0C0C0;
    NVIC->IP[1U] = 0xC0C0C0C0;
    NVIC->IP[2U] = 0xC0C0C0C0;
    NVIC->IP[3U] = 0xC0C0C0C0;
    NVIC->IP[4U] = 0xC0C0C0C0;
    NVIC->IP[5U] = 0xC0C0C0C0;
    NVIC->IP[6U] = 0xC0C0C0C0;
    NVIC->IP[7U] = 0xC0C0C0C0;
    PORT->PIOR0 = 0x00U;
    PORT->PIOR1 = 0x00U;
    PORT->PIOR2 = 0x00U;
    PORT->PIOR3 = 0x00U;
    CLK_Get_ResetSource();
    PORT_Init();
    CLK_Init();
    WDT_Init();
    SAF->SFRGD = 0x00U;
}

/* Start user code for adding. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
