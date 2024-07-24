/***********************************************************************************************************************
* File Name    : cg_tma.h
* Device(s)    : BAT32G137GH64FB
* Tool-Chain   : ARMCC
* Description  : This file implements device driver for TMA module.
* Creation Date: 2022/1/28
***********************************************************************************************************************/
#ifndef TMA_H
#define TMA_H

/***********************************************************************************************************************
Macro definitions (Register bit)
***********************************************************************************************************************/
/*
    Timer A Control Register 0 (TACR0) 
*/
/* Timer A underflow flag (TUNDF) */
#define _00_TMA_UNDERFLOW_NOT_OCCUR                  (0x00U) /* no underflow */
#define _20_TMA_UNDERFLOW_OCCUR                      (0x20U) /* underflow */
/* Active edge judgment flag (TEDGF) */
#define _00_TMA_ACTIVE_EDGE_RECEIVED                 (0x00U) /* active edge not received */
#define _10_TMA_ACTIVE_EDGE_UNRECEIVED               (0x10U) /* active edge received (end of measurement period) */
/* Timer A count forcible stop bit (TSTOP) */
#define _00_TMA_FORCIBLE_STOP_DISABLE                (0x00U) /* the count is not forcibly stopped */
#define _04_TMA_FORCIBLE_STOP_ENABLE                 (0x04U) /* the count is forcibly stopped */
/* Timer A count status flag (TCSTF) */
#define _00_TMA_STATUS_STOP                          (0x00U) /* count stops */
#define _02_TMA_STATUS_COUNT                         (0x02U) /* during count */
/* Timer A count start bit (TSTART) */
#define _00_TMA_COUNT_STOP                           (0x00U) /* count stops */
#define _01_TMA_COUNT_START                          (0x01U) /* count starts */

/*
    Timer A I/O Control Register 0 (TAIOC0) 
*/
/* TAIO event input control bit (TIOGT1,TIOGT0) */
#define _00_TMA_EVENT_ENABLE_ALWAYS                  (0x00U) /* event is counted */
#define _40_TMA_EVENT_ENABLE_INTP4                   (0x40U) /* event is counted during INTP4 specified period */
#define _80_TMA_EVENT_ENABLE_TIMEROUTSIGNAL          (0x80U) /* event is counted during polarity period specified for timer output signal */
/* TAIO input filter select bit (TIPF1,TIPF0) */
#define _00_TMA_TAIO_FILTER_UNUSED                   (0x00U) /* no filter */
#define _10_TMA_TAIO_FILTER_FCLK                     (0x10U) /* filter with fCLK sampling */
#define _20_TMA_TAIO_FILTER_FCLK8                    (0x20U) /* filter with fCLK/8 sampling */
#define _30_TMA_TAIO_FILTER_FCLK32                   (0x30U) /* filter with fCLK/32 sampling */
/* TAO output enable bit (TOENA) */
#define _00_TMA_TAO_OUTPUT_DISABLE                   (0x00U) /* TAO output disable */
#define _04_TMA_TAO_OUTPUT_ENABLE                    (0x04U) /* TAO output */
/* TAIO polarity switch bit (TEDGSEL) */
#define _00_TMA_TAIO_POLARITY_0                      (0x00U) /* TAIO polarity switch bit = 0 */
#define _01_TMA_TAIO_POLARITY_1                      (0x01U) /* TAIO polarity switch bit = 1 */

/*
    Timer A Mode Register 0 (TAMR0) 
*/
/* Timer A count source select bit (TCK2,TCK1,TCK0) */
#define _00_TMA_COUNT_SOURCE_FCLK                    (0x00U) /* fCLK */
#define _10_TMA_COUNT_SOURCE_FCLK8                   (0x10U) /* fCLK/8 */
#define _30_TMA_COUNT_SOURCE_FCLK2                   (0x30U) /* fCLK/2 */
#define _40_TMA_COUNT_SOURCE_FIL                     (0x40U) /* fIL */
#define _50_TMA_COUNT_SOURCE_FELC                    (0x50U) /* event input from event link controller (ELC) */
#define _60_TMA_COUNT_SOURCE_FSUB                    (0x60U) /* fSUB */
/* TAIO input polarity select bit (TEDGPL) */
#define _00_TMA_TAIO_POLARITY_ONE                    (0x00U) /* one edge */
#define _08_TMA_TAIO_POLARITY_BOTH                   (0x08U) /* both edges */
/* Timer A operating mode select bit (TMOD2,TMOD1,TMOD0) */
#define _00_TMA_MODE_TIMER                           (0x00U) /* timer mode */
#define _01_TMA_MODE_PULSE_OUTPUT                    (0x01U) /* pulse output mode */
#define _02_TMA_MODE_EVENT_COUNTER                   (0x02U) /* event counter mode */
#define _03_TMA_MODE_PULSE_WIDTH                     (0x03U) /* pulse width measurement mode */
#define _04_TMA_MODE_PULSE_PERIOD                    (0x04U) /* pulse period measurement mode */

/*
    Timer A event pin selection register 0 (TAISR0) 
*/
/* PWM signal and INTP4 polarity selection (RCCPSEL2) */
#define _00_TMA_PWM_POLARITY_L                       (0x00U) /* L period is counted */
#define _04_TMA_PWM_POLARITY_H                       (0x04U) /* H period is counted */
/* PWM signal selection (RCCPSEL1,RCCPSEL0) */
#define _00_TMA_PWM_TRDIOD1                          (0x00U) /* TRDIOD1 */
#define _01_TMA_PWM_TRDIOC1                          (0x01U) /* TRDIOC1 */
#define _02_TMA_PWM_TO02                             (0x02U) /* TO02 */
#define _03_TMA_PWM_TO03                             (0x03U) /* TO03 */


/***********************************************************************************************************************
Macro definitions
***********************************************************************************************************************/
#define _7CFF_TMA_TA0_VALUE                          (0x7CFFU) /* Timer A counter register 0 */
#define _9C3F_TMA_TA0_VALUE                          (0x9C3FU) /* Timer A counter register 0 */
/***********************************************************************************************************************
Typedef definitions
***********************************************************************************************************************/

/***********************************************************************************************************************
Global functions
***********************************************************************************************************************/
void TMA0_Init(void);
void TMA0_Start(void);
void TMA0_Stop(void);
/* Start user code for function. Do not edit comment generated here */
/* End user code. Do not edit comment generated here */
#endif
