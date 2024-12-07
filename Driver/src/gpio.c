/***********************************************************************************************************************
* Copyright (C) All rights reserved.
***********************************************************************************************************************/

/***********************************************************************************************************************
* @file    gpio.c
* @brief   This file implements device driver for GPIO module.
* @version V1.0.0
* @date    2019/12/24
***********************************************************************************************************************/

/***********************************************************************************************************************
Includes
***********************************************************************************************************************/
#include "BAT32G137.h"
#include "includes.h"
/***********************************************************************************************************************
Pragma directive
***********************************************************************************************************************/

/***********************************************************************************************************************
Global variables and functions
***********************************************************************************************************************/

/**
 * @brief Set specified GPIO as output function. 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_Output_Enable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2A0) &= ~pinMsk;  /*!< PMC=0: Digital Function   */
    *(port + 0x020) &= ~pinMsk;  /*!< PM =0: Output Function    */
}

/**
 * @brief Set specified GPIO as input function. 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_Input_Enable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2A0) &= ~pinMsk;  /*!< PMC=0: Digital Function   */
    *(port + 0x020) |=  pinMsk;  /*!< PM =1: Input Function     */
}

/**
 * @brief Enable pull up resister of input GPIO . 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_PullUp_Enable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2D0) |=  pinMsk;  /*!< PU =1: Pull Up enable         */
}

/**
 * @brief Disable pull up resister of input GPIO . 
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param pinMsk 
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_PullUp_Disable(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2D0) &=  ~pinMsk;  /*!< PU =0: Pull Up disable        */
}

/**
 * @brief Nch Open Drain Output mode
 *
 * @param port address, such as &P0, &P1, &P3, &P5, &P7
 * @param pinMsk
 *             e.g., bit0: 0x01, bit1: 0x02, bit0~3: 0x0F, bit0~7: 0xFF
 */
void GPIO_Nch_OpenDrain(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2B0) |=  pinMsk;  /*!< POM =1: Nch OpenDrain Output */
}

void GPIO_Ttl_Input(__IO uint8_t *port, uint8_t pinMsk)
{
    *(port - 0x2C0) |=  pinMsk;  /*!< PIM =1: TTL input */
}

/**
 * @brief Set specified value to GPIO output
 *
 * @param port port address, such as &P0, &P1, &P2...
 * @param value 
 */
void GPIO_Set_Value(__IO uint8_t *port, uint8_t value)
{
    *port = value;           /*!< PL = value */
}

/**
 * @brief Get value from GPIO input 
 *
 * @param port port address, such as &P0, &P1, &P2...
 *
 * @return 
 */
uint8_t GPIO_Get_Value(__IO uint8_t *port)
{
    //PORT->PMS = 0x01;        /*!< Digital output level of the pin is read */ 
    return (*port);          /*!< PL = value                              */
}


void PORT_Init(PORT_TypeDef PORTx,PIN_TypeDef PINx,PIN_ModeDef MODEx)
{
  	uint8_t mode = MODEx;
	uint8_t pos = 1<<PINx;
	
	switch(mode)
	{
		case INPUT:
			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PM0+PORTx)) |= pos;
			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PU0+PORTx)) &= ~pos;
			break;
		case PULLUP_INPUT:
			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PM0+PORTx)) |= pos;
			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PU0+PORTx)) |= pos;
			break;
		case TTL_INPUT:
			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PM0+PORTx)) |= pos;
			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) |= pos;
			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PU0+PORTx)) &= ~pos;
			break;
		case ANALOG_INPUT:
			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) |= pos;
			break;
		case OUTPUT:
			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->POM0+PORTx)) &= ~pos;
			break;
		case OPENDRAIN_OUTPUT:
			*((volatile uint8_t*)(&PORT->PMC0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->PIM0+PORTx)) &= ~pos;
			*((volatile uint8_t*)(&PORT->POM0+PORTx)) |= pos;
			break;
	}
}


/**
  * @brief  Set the PORTx bit
  * @param  PORTx: where x can be 0~14
  * @param  PINx: where x can be 0~7    
	*
  * @retval None
  */
void PORT_SetBit(PORT_TypeDef PORTx,PIN_TypeDef PINx)
{
	uint8_t pos = 1<<PINx;
	*((volatile uint8_t*)(&PORT->P0+PORTx)) |= pos;
}

/**
  * @brief  Clear the PORTx bit
  * @param  PORTx: where x can be 0~14
  * @param  PINx: where x can be 0~7    
	*
  * @retval None
  */
void PORT_ClrBit(PORT_TypeDef PORTx,PIN_TypeDef PINx)
{
	uint8_t pos = 1<<PINx;
	*((volatile uint8_t*)(&PORT->P0+PORTx)) &= ~pos;
}

/**
  * @brief  Toggle the PORTx bit
  * @param  PORTx: where x can be 0~14
  * @param  PINx: where x can be 0~7    
	*
  * @retval None
  */
void PORT_ToggleBit(PORT_TypeDef PORTx,PIN_TypeDef PINx)
{
	uint8_t pos = 1<<PINx;
	*((volatile uint8_t*)(&PORT->P0+PORTx)) ^= pos;
}

/**
  * @brief  Get the PORTx bit
  * @param  PORTx: where x can be 0~14
  * @param  PINx: where x can be 0~7    
	*
  * @retval None
  */
uint8_t PORT_GetBit(PORT_TypeDef PORTx,PIN_TypeDef PINx)
{
	uint8_t pos = 1<<PINx;
	//PORT->PMS = 0x01;        /*!< Digital output level of the pin is read */
	return *((volatile uint8_t*)(&PORT->P0+PORTx))&pos;
}

/********************************************************************************
GPIO操作定义,所有引脚电平需要定义
********************************************************************************/
//定义电源控制
TGPIO PIN_SW 	= {PORT1,PIN6,PULLUP_INPUT};		//
#define  	VB_ON		(PORT_SetBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))	 
#define		VB_OFF		(PORT_ClrBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))
#define 	IS_VB_ON	(PORT_GetBit(PIN_VBCTL.emGPIOx,PIN_VBCTL.emPin))

void GPIO_Config(void)
{
	//输入
	PORT_Init(PIN_SW.emGPIOx,		PIN_SW.emPin,		PIN_SW.emMode);	
// 	PORT_Init(PIN_ALERT.emGPIOx,	PIN_ALERT.emPin,	PIN_ALERT.emMode);	
// 	PORT_Init(PORT5,PIN1,INPUT);    //485唤醒
//   PORT_Init(PORT14,PIN0,PULLUP_INPUT);
// 	//PORT_Init(PIN_REV.emGPIOx,		PIN_REV.emPin,		PIN_REV.emMode);
  	
// 	//输出
// 	PORT_Init(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin,	PIN_VBCTL.emMode);
// 	VB_ON;
   
// 	PORT_Init(PIN_COM5V_EN.emGPIOx,PIN_COM5V_EN.emPin,PIN_COM5V_EN.emMode);
//   PIN_COM5V_OFF;
	
// 	PORT_Init(PIN_COM3V3_EN.emGPIOx,PIN_COM3V3_EN.emPin,PIN_COM3V3_EN.emMode);
// 	PIN_COM3V3_OFF; 
	
//   PORT_Init(PIN_HEATE_N.emGPIOx,	PIN_HEATE_N.emPin,	PIN_HEATE_N.emMode);
//   HEAT_OFF;
	
// 	PORT_Init(PIN_CDEN.emGPIOx,		PIN_CDEN.emPin,		PIN_CDEN.emMode);
// 	CD_OFF;
	
// 	PORT_Init(PIN_CEN.emGPIOx,		PIN_CEN.emPin,		PIN_CEN.emMode);
// 	C_OFF;
	
// 	PORT_Init(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin,	PIN_GREEN.emMode);
// 	GREEN_OFF;
	
// 	PORT_Init(PIN_RED.emGPIOx,		PIN_RED.emPin,		PIN_RED.emMode);
// 	RED_OFF;
	
// 	PORT_Init(PIN_DEBUG.emGPIOx,		PIN_DEBUG.emPin,		PIN_DEBUG.emMode);
// 	DEBUG_LED_ON;
	

// 	PORT_Init(PIN_FUSE_EN.emGPIOx,	PIN_FUSE_EN.emPin,	PIN_FUSE_EN.emMode);
// 	FUSE_ON;
	
	PORT_Init(PORT5,PIN0,OUTPUT);    //CTLD
// 	PORT_SetBit(PORT5,PIN0);
	
// 	//PORT_Init(PIN_HEATFUSE_EN.emGPIOx,PIN_HEATFUSE_EN.emPin,PIN_HEATFUSE_EN.emMode);
// 	//HEATFUSE_ON;
    
// 	//PORT_Init(PIN_PACKADC_EN.emGPIOx,	PIN_PACKADC_EN.emPin,	PIN_PACKADC_EN.emMode);
// 	//PACKADC_ENABLE;
	
// 	//PORT_Init(PIN_485DE.emGPIOx,	PIN_485DE.emPin,	PIN_485DE.emMode);
// 	//RS485_SEND_DISABLE;
	
// 	//未使用管脚配置
// 	PORT_Init(PORT13,PIN6,OUTPUT);
// 	PORT_Init(PORT7,PIN5,OUTPUT); 
// 	PORT_Init(PORT7,PIN4,OUTPUT); 
// 	PORT_Init(PORT3,PIN0,OUTPUT);
// 	PORT_Init(PORT1,PIN2,OUTPUT);
//   PORT_Init(PORT1,PIN0,OUTPUT);  
// 	PORT_Init(PORT2,PIN0,OUTPUT);
// 	PORT_Init(PORT2,PIN1,OUTPUT);
// 	PORT_Init(PORT2,PIN2,OUTPUT);
//   PORT_Init(PORT2,PIN3,OUTPUT);
//   PORT_Init(PORT2,PIN4,OUTPUT);
//   PORT_Init(PORT2,PIN5,OUTPUT);
// 	PORT_Init(PORT13,PIN0,OUTPUT); 
	
}
