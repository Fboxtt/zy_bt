//************************************************************
//  Copyright (c) 
//	�ļ�����	: boot.c
//	ģ�鹦��	: boot��Ҫ�����ļ�
//  ��������	: 2024/9/12
// 	�汾		: V1.0
//************************************************************
#include "includes.h"

// uint32_t* g_restoreBufferFlag = (uint32_t*)0x20000000; // ��ǿ�ƻָ���־λ������sram�ڵ���ʼ��ַ
// uint32_t* g_restoreBackupFlag = (uint32_t*)0x20000004; // ��ǿ�ƻָ���־λ������sram�ڵ���ʼ��ַ

uint32_t CmmuReadNumber;	//ͨѶ��ǰ��ȡ����Ϊһ֡�еĵڼ�����
uint8_t UartReceFlag;				//UART0������һ֡��־λ
uint8_t UartSendFlag;				//UART0������һByte��־λ



uint8_t CommunicationCheckNumber;			//У��λ
commu_length_t CmmuLength;						//�������ݳ���
uint8_t CMDBuff;								//����洢����
commu_data_t CommuData[ReceiveLength1];	//ͨѶ���ջ���

commu_data_t CmdSendData[SendLength1];	//��������
commu_length_t CmmuSendLength;		    //�������ݳ���

commu_data_t CmdSendAll[SendLength1];	//���ͻ���
commu_length_t CmdSendAllLenth;			//���ͻ��泤��

uint8_t CRCchecksum[4];

uint8_t uart_send_flag = 0;

typedef enum {
	NO_DOWNLOADING = 0x0,
	DOWNLOADING_BUFF	= 0x55AA55AA,
	DOWNLOADING_BKP		= 0x0A555AAA,
	DOWNLOADED_BUFF		= 0x5A5A5555,
	DOWNLOADED_BKP		= 0x0A5AAAAA,
	RESTORE_BUFF		= 0x5AA56699,
	RESTORE_BKP 		= 0x69695A5A,
}DOWNLOAD_STATUS;

DOWNLOAD_STATUS g_downLoadStatus = NO_DOWNLOADING;
#ifndef BMS_APP_DEVICE
volatile uint8_t * gp_uart0_tx_address;         /* uart0 send buffer address */
volatile uint16_t  g_uart0_tx_count;            /* uart0 send data number */
volatile uint8_t * gp_uart0_rx_address;         /* uart0 receive buffer address */
volatile uint16_t  g_uart0_rx_count;            /* uart0 receive data number */
volatile uint16_t  g_uart0_rx_length;           /* uart0 receive data length */
#endif
typedef struct {
	uint16_t majorVer;			// ���汾��
	uint16_t minorVer;			// �ΰ汾��
	uint16_t revisionVer;		// �޶��汾
	uint16_t year;				// ��
	uint8_t month;				// ��
	uint8_t day;				// ��
	uint8_t reserved[2];		// ����������
}VerStru;

#ifndef IN_APP
const VerStru btVersion __attribute((at(BOOT_VER_ADDR)))= {
	1,2,0,2024,11,12
};
#endif

#ifndef BMS_APP_DEVICE

TUartData g_tUartData;

#ifdef IN_APP
const TVER g_stVersion __attribute__((at(APP_VER_ADDR)))= {
	1,1,1,2024,10,11
};
#endif
#endif
uint8_t* g_sendArray;

uint8_t result_cmd;
// extern volatile uint8_t ACK;

uint32_t BootWaitTime = 0;
uint32_t BootWaitTimeLimit = 0;



int g_FLSTSMaxCount = 0;

__asm uint32_t get_pc(void) {
	mov r0, pc
	bx lr
}
#ifndef BMS_APP_DEVICE



void UartInit(uint32_t baud)
{
    SCI0_Init();
    /* UART0 Start, Setting baud rate */
    UART0_BaudRate(Fsoc, baud);
    UART0_Start();
}

void UartSendOneByte(uint8_t input_data)
{
    SCI0->TXD1 = input_data;
	//�������ȵ���־λ���㼴�ɷ����¸�����
	while (SCI0->SSR02 & (_0040_SCI_UNDER_EXECUTE | _0020_SCI_VALID_STORED))
    {
        ;
    }
}

#endif 
#define SLAVE_ADDRESS 0x00//�豸��ַ
//void UartReceData(uartId id)//��������֡
//{
//	if(!UartReceFlag)
//	{		
//		if(id == UART0) {
//			CommuData[CmmuReadNumber] = SCI0->RXD0;		//�������������뻺��
//		}else if(id == UART1) {
//			CommuData[CmmuReadNumber] = SCI0->RXD1;		//�������������뻺��
//		}else if(id == UART2) {
//			CommuData[CmmuReadNumber] = SCI1->RXD2;		//�������������뻺��
//		}
//		CmmuReadNumber++;
//		// if(CommuData[0] == SLAVE_ADDRESS)
//		// {
//			
//		// }
//		if(CmmuReadNumber >= 3) {
//			if(CmmuReadNumber>=(3 + CommuData[1] * 0x100 + CommuData[2] + 1)) //������������256�Ļ���Ҫ�޸�CmmuReadNumber����
//			{
//				/* �������Ź����幷 */
//				CmmuLength = 3 + CommuData[1] * 0x100 + CommuData[2] + 1;
//				UartReceFlag = 1;	  //��ʾ���յ�һ֡����
//			}
//		}

//	}
//}
void ClearCommu()
{
    CommuData[0] = 0; //�������������ͷ��׼���´δ������ݵ���
    CmmuReadNumber = 0; //���¼�����׼���´δ������ݵ���
    UartReceFlag = 0; //���������ɱ�־
	CmdSendAllLenth = 0;
}

//#ifndef BMS_APP_DEVICE

//void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data)
//{
//	uint8_t i;
//	uint8_t check_sum = 0;
//	UartSendOneByte(SEND_ADDRESS);	//����֡ͷ
//	
//	UartSendOneByte((Data_len + 5) >> 8);		 				 		//���������򳤶ȸ�8λ
//	UartSendOneByte(Data_len + 5);		 			 	//���������򳤶ȵ�8λ
//	UartSendOneByte(SEND_BMS_TYPE);					//���͵���������
//	UartSendOneByte(Command);					 	//���Ϳ�����
//	UartSendOneByte(SEND_SHAKE_1);					//������1
//	UartSendOneByte(SEND_SHAKE_2);					//������1
//	UartSendOneByte(ACK);							//����Ӧ����
//	check_sum = ((Data_len + 5) >> 8) + (Data_len + 5) + SEND_BMS_TYPE + Command + SEND_SHAKE_1 + SEND_SHAKE_2 + ACK;
//	for(i=0;i<Data_len;i++)	  					 	//����������
//	{
//		UartSendOneByte(*(Data+i));
//		check_sum+=	*(Data+i);
//	}	
//	UartSendOneByte(check_sum);						//����У��λ��8λ
//	// UartSendOneByte(CommunicationCommandEnd);		//����֡β   
//}
//#endif

void fillbackFunc(commu_data_t* pBuff, commu_data_t* Data,commu_cmd_t Command,commu_cmd_t dataLen, commu_data_t Ack)
{
	uint8_t i;
	uint8_t check_sum = 0;

	pBuff[0] = SEND_ADDRESS;	//����֡ͷ
	pBuff[1] = (dataLen + 5) >> 8;		 				 		//���������򳤶ȸ�8λ
	pBuff[2] = dataLen + 5;		 			 	//���������򳤶ȵ�8λ
	pBuff[3] = SEND_BMS_TYPE;					//���͵���������
	pBuff[4] = Command;					 	//���Ϳ�����
	pBuff[5] = SEND_SHAKE_1;					//������1
	pBuff[6] = SEND_SHAKE_2;					//������1
	pBuff[7] = Ack;							//����Ӧ����
	check_sum = ((dataLen + 5) >> 8) + (dataLen + 5) + SEND_BMS_TYPE + Command + SEND_SHAKE_1 + SEND_SHAKE_2 + Ack;
	for(i=0;i<dataLen;i++)	  					 	//����������
	{
		pBuff[8+i] = *(Data+i);
		check_sum+=	*(Data+i);
	}
	pBuff[8 + dataLen] = check_sum;						//����У��λ��8λ
	// UartSendOneByte(CommunicationCommandEnd);		//����֡β  
}

uint8_t AnalysisData(uint8_t* pBuff, uint32_t wholeLen,uint32_t* noPackNumLen, volatile uint8_t* pAck)//��������֡������
{
	volatile uint8_t cmd = NO_CMD;
    uint32_t calLen;
	uint32_t i;
	uint8_t check_sum = 0;
	calLen = pBuff[1] * 0x100 + pBuff[2];
	cmd = pBuff[4];

	*pAck = ERR_NO;
	//���㵥�����͵�����λ��У���
	for(i=1; i < calLen + 3; i++)
	{
	   check_sum+=pBuff[i];
	}
	if((cmd & 0x80) != 0) {
		*pAck = ERR_CMD_ID;
	} else {
		if(wholeLen != calLen + 4) {
			*pAck = ERR_CMD_LEN;
		}
	}
	if(cmd != PC_SET_WRITE_FLASH && cmd != PC_SET_ALL_CHECKSUM && wholeLen != 8) {
		*pAck = ERR_CMD_LEN;
	}
	//У��ɹ�,��ȡ������
	if(check_sum != (pBuff[3 + calLen]))
	{
        *pAck = ERR_CHKSUM;
	}
	// if(cmd == PC_SET_WRITE_FLASH) {
	// 	*noPackNumLen = calLen - TYPE_TO_DATA_LENTH;//ȡ����
	// } else {
	*noPackNumLen = calLen - TYPE_TO_SHAKE_LENTH;//ȡ����
	// }

    return cmd;
}

#ifndef BMS_APP_DEVICE
void IRQ10_Handler(void) __attribute__((alias("uart0_interrupt_send")));
void IRQ11_Handler(void) __attribute__((alias("uart0_interrupt_receive")));

static void uart0_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    UartSendFlag=1; 	 //BootLoader��???����??
    uart_send_flag=1;
    /* End user code. Do not edit comment generated here */
}
static void uart1_callback_sendend(void)
{
    /* Start user code. Do not edit comment generated here */
    UartSendFlag=1; 	 //BootLoader��???����??
    uart_send_flag=1;
    /* End user code. Do not edit comment generated here */
}
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

static void uart0_interrupt_receive(void)
{
	uartId id = UART0;

    
	// interrupt_receive(UART0);

    volatile uint8_t rx_data;
    volatile uint8_t err_type;
    INTC_ClearPendingIRQ(SR0_IRQn); /* clear INTSR0 interrupt flag */
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
#endif
/*flash_operate*/
const unsigned char  IapCheckNum[IAP_CHECK_LENGTH]={IAP_CHECK_NUMBER};	//APP����������״̬��
const unsigned char  BuffCheckNum[IAP_CHECK_LENGTH] = {BUFF_CHECK_NUMBER};	//���뻺�����������״̬��
uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area)//д���ֽ�IAP����
{
    uint8_t *ptr;
    
    ptr = (uint8_t *) IAP_IapAddr;
    
    FMC->FLPROT = 0xF1;
    
    FMC->FLOPMD1 = 0xAA;
    FMC->FLOPMD2 = 0x55;  
    *ptr = Write_IAP_IapData;    
    // polling OVER Flag
    while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0);
    FMC->FLSTS |= FMC_FLSTS_OVF_Msk;

    FMC->FLPROT = 0x00;
	
    if(IAP_ReadOneByte(IAP_IapAddr,area) == Write_IAP_IapData)
    {
        return 1;	//д��׼ȷ
    }
    else
    {
        return 0;	//д������
    }
}

uint8_t IAP_WriteOneByte_Check(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area)//д���ֽ�IAP����
{
    uint8_t *ptr;
    int FLSTS_flagCount = 0;
    ptr = (uint8_t *) IAP_IapAddr;
    
    FMC->FLPROT = 0xF1;
    FMC->FLOPMD1 = 0xAA;
    FMC->FLOPMD2 = 0x55;  
    *ptr = Write_IAP_IapData;    
    // polling OVER Flag
	// ����ж�FLSTSֵ��ѭ��һ����7�����ָ��
    while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0 && FLSTS_flagCount < g_FLSTSMaxCount) {
		FLSTS_flagCount++;
	};
    FMC->FLSTS |= FMC_FLSTS_OVF_Msk;

    FMC->FLPROT = 0x00;
	if(FLSTS_flagCount >= g_FLSTSMaxCount) {
		return 0;
	}
    if(IAP_ReadOneByte(IAP_IapAddr,area) == Write_IAP_IapData)
    {
        return 1;	//д��׼ȷ
    }
    else
    {
        return 0;	//д������
    }
}

uint8_t IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area)//����һ���飨512B��
{
	int FLSTS_flagCount = 0;
    FMC->FLERMD = 0x10;
    FMC->FLPROT = 0xF1;
    FMC->FLOPMD1 = 0x55;
    FMC->FLOPMD2 = 0xAA;  
    // Write data to start address of sector to trigger Erase Operation
    *(uint32_t *) IAP_IapAddr = 0xFFFFFFFF;
    
    // polling Erase Over Flag
//    while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0);
	while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0 && FLSTS_flagCount < 120000) {
		FLSTS_flagCount++;
	};
    FMC->FLSTS |= FMC_FLSTS_OVF_Msk;
    FMC->FLERMD = 0x00;
    FMC->FLPROT = 0x00;

    if(FMC->FLSTS & FMC_FLSTS_EVF_Msk)
    {
        //printf("\nerror\n");
    }
	if(FLSTS_flagCount >= g_FLSTSMaxCount) {
		return 0;
	}
	return 1;
    
}
void IAP_Erase_Some(uint32_t IAP_IapAddr, uint32_t lenth)// ��������¼�������ݣ�������ÿռ�
{
	uint8_t buff[512] = {0};
	uint32_t sectorAddr = IAP_IapAddr & 0xfffffe00;
	uint32_t lowLenth = IAP_IapAddr - sectorAddr;
	uint32_t hignLenth = 512 - lenth - lowLenth;
	int i = 0;
	if(lenth > 512) {
		return;
	}
	for(i = 0; i < 512; i++) {
		buff[i] = *((uint8_t *)sectorAddr + i);
	}

    FMC->FLERMD = 0x10;
    FMC->FLPROT = 0xF1;
    FMC->FLOPMD1 = 0x55;
    FMC->FLOPMD2 = 0xAA;  
    // Write data to start address of sector to trigger Erase Operation
    *(uint32_t *) IAP_IapAddr = 0xFFFFFFFF;
    
    // polling Erase Over Flag
    while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0);
    FMC->FLSTS |= FMC_FLSTS_OVF_Msk;
    FMC->FLERMD = 0x00;
    FMC->FLPROT = 0x00;
    
    if(FMC->FLSTS & FMC_FLSTS_EVF_Msk)
    {
        //printf("\nerror\n");
    }
	IAP_WriteMultiByte(sectorAddr, &buff[0], lowLenth, IAP_CHECK_AREA);
	IAP_WriteMultiByte(sectorAddr + lowLenth + lenth, &buff[lowLenth + lenth], hignLenth, IAP_CHECK_AREA);
}
uint8_t IAP_Erase_ALL(uint8_t area)
{
    uint16_t i;
	uint16_t k = 0;
	uint32_t begin_addr = 0;
	if(area==APROM_AREA)
	{
		k = (APP_SIZE/ONE_PAGE_SIZE);
		begin_addr = APP_ADDR;
	}
	else if(area==APROM_BACKUP_AREA)
	{
		k = (BACKUP_SIZE/ONE_PAGE_SIZE);
		begin_addr = BACKUP_ADDR;
		area = APROM_AREA;
	}
	#ifdef FLASH_BUFF_ENABLE
	else if(area==APROM_BUFF_AREA)
	{
		k = (APP_BUFF_SIZE/ONE_PAGE_SIZE);
		begin_addr = APP_BUFF_ADDR;
		area = APROM_AREA;
	}
	#endif
	else if(area==DATA_AREA)
	{
		k = (DATA_SIZE/ONE_PAGE_SIZE);
		begin_addr = DATA_ADDR;
	}
	#ifdef ENCRYPT_UID_ENABLE
	else if(area==UID_ENC_AREA)
	{
		k = 1;
		begin_addr = UID_ENC_ADRESS;
		area = APROM_AREA;
	}
	#endif
    for(i=0;i<k;i++)
    {
		if(IAP_Erase_512B(i*ONE_PAGE_SIZE+begin_addr,area) == 0) {
			return 0;
		}
    }
	return 1;
}

void __set_VECTOR_ADDR(uint32_t addr)
{
	SCB->VTOR = addr;
}

void IAP_Reset()
{	
    SCI0->ST0   = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
	CGC->PER0 &= ~CGC_PER0_SCI0EN_Msk;
	INTC_DisableIRQ(SR0_IRQn);
	NVIC_SystemReset();
}

#ifdef BMS_BT_DEVICE
void IAPEnterApp()
{
	BaseTimeSystemInit(BOOT_DISABLE);	//�رն�ʱ��
	SCI0->ST0   = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
	CGC->PER0 &= ~CGC_PER0_SCI0EN_Msk;
	INTC_DisableIRQ(SR0_IRQn);
	__set_VECTOR_ADDR(APP_VECTOR_ADDR); // ��Ҫ������������Ϊʵ�ⷢ��app�����ж���Ȼ������bt��systick
	__set_MSP(*(__IO uint32_t*) APP_ADDR);
	((void (*)()) (*(volatile unsigned long *)(APP_ADDR+0x04)))();//to APP
    NVIC_SystemReset();					//����޷�����APP��λ
}
#endif

uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area)	//д���ֽ�IAP����
{
	uint32_t i;
	uint8_t Write_IAP_IapData;
	for(i=0;i<len;i++)
	{
		Write_IAP_IapData = buff[i];
        if(IAP_WriteOneByte(IAP_IapAddr+i,Write_IAP_IapData,area)==0)//�ж�д���Ƿ���ȷ
		{
			return 0;
		}			
	}
	return 1;
}

uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area)	//�����ֽ�IAP����
{
    uint8_t IAP_IapData; 
    IAP_IapData = *(uint32_t *)IAP_IapAddr;
	return IAP_IapData;
}

void IAP_ReadMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint16_t len,uint8_t area)
{
    uint16_t i;
    for(i=0;i<len;i++)
    {               
        (*buff) = IAP_ReadOneByte(IAP_IapAddr+i,area);
        buff++;
    }  
}
void IAP_FlagWrite(uint8_t flag)
{
    unsigned char i;
    IAP_Erase_Some(IAP_CHECK_ADRESS,ALL_FLAG_LENTH);
    if(flag==1)
    {        
        for(i=0;i<IAP_CHECK_LENGTH;i++)
        {
           IAP_WriteOneByte(IAP_CHECK_ADRESS+i,IapCheckNum[i],IAP_CHECK_AREA);
        }
    }
	else if(flag==2)
	{
		for(i=0;i<IAP_CHECK_LENGTH;i++)
        {
           IAP_WriteOneByte(IAP_CHECK_ADRESS+i,BuffCheckNum[i],IAP_CHECK_AREA);
        }
	}
}

uint8_t IAP_CheckAPP()
{
    unsigned char i;
	volatile uint8_t temp = 1;
    for(i=0;i<IAP_CHECK_LENGTH;i++)
    {
        if(IAP_ReadOneByte(IAP_CHECK_ADRESS+i,IAP_CHECK_AREA)!=IapCheckNum[i])
        {
            temp = 0;
			break;
        }
    }
	if(temp)
	{
		return temp;
	}
	#ifdef FLASH_BUFF_ENABLE
	for(i=0;i<IAP_CHECK_LENGTH;i++)
    {
		if(IAP_ReadOneByte(IAP_CHECK_ADRESS+i,IAP_CHECK_AREA)!=BuffCheckNum[i])
        {
			break;
        }		
    }
	if(i>=IAP_CHECK_LENGTH)
	{
		temp = 2;
	}
	#endif
    return temp;
}

void IAP_ReadEncUID(uint8_t* buff)
{
	uint8_t i;
	for(i=0;i<UID_ENC_SIZE;i++)
	{
		buff[i] = IAP_ReadOneByte(UID_ENC_ADRESS+i,UID_ENC_AREA_AREA);
	}
}
#ifdef FLASH_BUFF_ENABLE
uint8_t IAP_Remap()//���������Ĵ���װ����������
{
	uint16_t i;
	IAP_Erase_ALL(APROM_AREA);//����APP����������
	for(i=0;i<APP_BUFF_SIZE;i++)
	{
		if(IAP_WriteOneByte(APP_ADDR+i,IAP_ReadOneByte(APP_BUFF_ADDR+i,APROM_AREA),APROM_AREA) == 0) {
			return 0;
		}
	}
	return 1;
}
#endif

uint8_t IAP_BkpRemap()//���������Ĵ���װ����������
{
	uint16_t i;
	IAP_Erase_ALL(APROM_AREA);//����APP����������
	for(i=0;i<APP_BUFF_SIZE;i++)
	{
		if(IAP_WriteOneByte(APP_ADDR+i,IAP_ReadOneByte(BACKUP_ADDR+i,APROM_AREA),APROM_AREA) == 0) {
			return 0;
		}
	}
	return 1;
}
/*flash_operate*/
/*flash_operate*/
/*flash_operate*/


/*boot_core.c*/
/*boot_core.c*/
/*boot_core.c*/
uint8_t g_BkpFlag = 0;								//����������У��״̬
uint8_t ResetFlag = 0;								//��ʾ��λ�������
uint8_t CurrState = 0;								//��ǰоƬ��״̬
uint32_t ReadFlashLength = 0;                       //��Flash�ĳ���        
uint32_t ReadFlashAddr = 0;							//��Flash����ʼ��ַ

uint32_t g_packetTotalNum = 0;								//��¼�ļ����ݰ�������

uint32_t CheckSum = 0;
// uint8_t CheckSum[2] = {0x0, 0x0};
const uint8_t Boot_Inf_Buff[IC_TYPE_LENTH] = IC_TYPE_128KB_NAME;//�汾�Ŵ洢
boot_addr_t BeginAddr = APP_ADDR;				    //��ʼ��ַ�洢
uint32_t NewBaud = UartBaud;						//�洢�²����ʵı���
extern commu_data_t CmdSendData[SendLength1];
uint32_t NextPacketNumber = 0;

const uint8_t IC_INF_BUFF[IC_TYPE_LENTH] = IC_TYPE_128KB_NAME; // оƬ�ͺŴ洢
// volatile uint8_t *Ack =  0x00;


WritableFlag g_flashWritableFlag = {0};

/* boot��ʼ�����Ӻ������뽫��ʼ������д��ú��� */
void BootInit()
{
	// UartInit(UartBaud);
	g_FLSTSMaxCount = 24 * SystemCoreClock / ONE_DISASSEMBLE_COUNT / 1000000 * 2;
	if(CheckAreaWritable(APP_ADDR + APP_SIZE - 512) == 1) { // ȷ������APP�Ƿ��д
		g_flashWritableFlag.bit.appArea = 1;
	}
	if(CheckAreaWritable(APP_BUFF_ADDR + APP_BUFF_SIZE - 512) == 1) { // ȷ������BUFF�Ƿ��д
		g_flashWritableFlag.bit.bufferArea = 1;
	}
	if(CheckAreaWritable(BACKUP_ADDR + BACKUP_SIZE - 512) == 1) { // ȷ������BACKUP�Ƿ��д
		g_flashWritableFlag.bit.backupArea = 1;
	}
	// CurrState = IAP_CheckAPP();
    if(CurrState==1)//�ж�APP�Ƿ�����������������ʱ
    {
//        BaseTimeSystemInit(BOOT_ENABLE);
    }
	#ifdef FLASH_BUFF_ENABLE
	else if(CurrState==2)//�����������ص���������ֱ������APP
	{
		// IAP_Remap();//�����뻺���������ݼ��������������
		// IAP_FlagWrite(1);//����ΪAPP������̬
//        IAP_Reset();
	}
	#endif
}

//void Decrypt_Fun(uint8_t* buff)
//{
//	uint8_t i;
//	uint8_t k;
//	uint32_t first_chunk;
//	uint32_t second_chunk;
//    union  
//	{
//		uint32_t temp_uint32[16];
//		uint8_t temp_uint8[64];
//	}temp; 
//	if(CmmuLength>64)
//	{
//		return;
//	}
//    //ARMΪС��ģʽ��Ҫ��ÿ���ֵĸ�λ�͵�λ�Ե�
//    for(i=0;i<(CmmuLength/4);i=i+1)
//	{		
//        for(k=0;k<4;k++)
//        {
//            temp.temp_uint8[i*4+(3-k)] = buff[i*4+k];
//        }
//	}
//	for(i=0;i<(CmmuLength/4);i=i+2)
//	{
//		first_chunk = temp.temp_uint32[i];
//		second_chunk = temp.temp_uint32[i+1];
//		DecryptTEA(&first_chunk,&second_chunk);
//		temp.temp_uint32[i] = first_chunk;
//		temp.temp_uint32[i+1] = second_chunk;
//        for(k=0;k<4;k++)
//        {
//            buff[i*4+k] = temp.temp_uint8[i*4+(3-k)] ;
//            buff[(i+1)*4+k] = temp.temp_uint8[(i+1)*4+(3-k)] ;
//        }
//	}
//}

void All_CheckSum_Write(uint32_t checkSum, uint32_t addr)
{
    unsigned char i;
	for(i=0;i<CHECKSUM_LENGTH;i++)
	{
		IAP_WriteOneByte(addr+i,(checkSum >> (8 * i)),IAP_CHECK_AREA);
	}

}

void uint32ValWrite(uint32_t packetTotalNum, uint32_t addr)
{
	int i = 0;
	for(i = 0;i < TOTAL_NUM_LENGTH; i++)
	{
		IAP_WriteOneByte(addr + i, (packetTotalNum >> (8 * i)), IAP_CHECK_AREA);
	}
}

uint32_t All_CheckSum_Read(uint32_t addr)
{
    unsigned char i;
	volatile uint8_t temp = 0;
	uint32_t checkSum = 0;
    for(i=0;i<CHECKSUM_LENGTH;i++)
    {
		checkSum += (IAP_ReadOneByte(addr+i,IAP_CHECK_AREA) << (i * 8));

    }
	return checkSum;
}

uint32_t PacketTotalNumRead(uint32_t addr)
{
    uint32_t packetTotalNum = 0;
	int i;
    for(i = 0; i<TOTAL_NUM_LENGTH; i++)
    {
        packetTotalNum += (IAP_ReadOneByte(addr+i,IAP_CHECK_AREA) << (8 * i));
    }
	if(packetTotalNum == 0xffffffff) {
		return 0;
	}
	return packetTotalNum;
}

typedef struct {
	uint32_t checkAddr;
	uint32_t numAddr;
	uint32_t hexAddr;
} CheckSumStruct;

static uint32_t checkAddr = 0, numAddr = 0, hexAddr = 0; 
void getCheckPara(int area)
{
	if(area == APROM_AREA) {
		checkAddr = APP_CHECKSUM_ADRESS;
		numAddr = APP_TOTAL_NUM_ADRESS;
		hexAddr = APP_ADDR;
	} else if(area == APROM_BUFF_AREA) {
		checkAddr = BUFFER_CHECKSUM_ADRESS;
		numAddr = BUFFER_TOTAL_NUM_ADRESS;
		hexAddr = APP_BUFF_ADDR;
	} else if(area == APROM_BACKUP_AREA) {
		checkAddr = BACKUP_CHECKSUM_ADRESS;
		numAddr = BACKUP_TOTAL_NUM_ADRESS;
		hexAddr = BACKUP_ADDR;
	}
}

void CheckSumWrite(uint32_t totalNum, uint32_t chkSum, int area)
{
	getCheckPara(area);
	IAP_Erase_Some(numAddr,ALL_FLAG_LENTH);
	uint32ValWrite(totalNum, numAddr); // appУ��Ϳ���ȡbuffer����backup��buffer���������룬backup����������
	All_CheckSum_Write(chkSum, checkAddr);
}
uint8_t CheckSumCheck(int area)
{
	uint32_t packetTatolSize = 0;
	uint16_t calCheckSum = 0;
	int i;
	getCheckPara(area);
	packetTatolSize = PACKET_SIZE * PacketTotalNumRead(numAddr);
	
	if(packetTatolSize == 0 || packetTatolSize > MAX_PACK_NUM) {
		return 0;
	}
	for(i = 0; i < packetTatolSize; i++) {
		calCheckSum += IAP_ReadOneByte(hexAddr+i,IAP_CHECK_AREA);
	}

	if(calCheckSum == All_CheckSum_Read(checkAddr)) {
		return 1;
	} else {
		return 0;
	}
}
#ifdef ENCRYPT_UID_ENABLE
uint8_t CheckUID()
{
	uint8_t i;	
	uint8_t * uid_point = (uint8_t *)UID_BASE;
	uint8_t * buff_point = CommuData;
	IAP_ReadEncUID(buff_point);
	CmmuLength = UID_ENC_SIZE;
	Decrypt_Fun(CommuData);
	for(i=0;i<UID_SIZE;i++)
	{
		if(CommuData[i]!=*(uid_point+i))
		{
			return 0;
		}
	}
	return 1;
}
#endif

void ReplyEnterBoot(void)
{
	CmmuSendLength = 0;
//	CommuSendCMD(result_cmd,CmmuSendLength,CmdSendData); // ��Ӧ��λ��
}

void AppRestore()
{
	#ifdef BMS_BT_DEVICE
	if(ReadInt(BUFFER_RESTORE_ADDRESS) == RESTORE_BUFF) {
		if(IAP_Remap() == 1) {
			CheckSumWrite(PacketTotalNumRead(BUFFER_TOTAL_NUM_ADRESS), All_CheckSum_Read(BUFFER_CHECKSUM_ADRESS), APROM_AREA);
			if(CheckSumCheck(APROM_AREA) == 1)
			{
				IAP_Erase_Some(BUFFER_RESTORE_ADDRESS, 4);
				// *Ack =  ERR_NO; //��Ӧ�˳���Bootloader
			} else {
				// *Ack =  ERR_ALL_CHECK;
			}
		} else {
			// *Ack =  ERR_REMAP;
		}
		result_cmd = BMS_SHAKE_ENTER_APP;
	} else if(ReadInt(BACKUP_RESTORE_ADDRESS) == RESTORE_BKP) {
		if(IAP_BkpRemap() == 1) {
			// �ӱ������ж�ȡУ������ݣ���д�뵽APP������
			CheckSumWrite(PacketTotalNumRead(BACKUP_TOTAL_NUM_ADRESS), All_CheckSum_Read(BACKUP_CHECKSUM_ADRESS), APROM_AREA);
			if(CheckSumCheck(APROM_AREA) == 1)
			{
				IAP_Erase_Some(BACKUP_RESTORE_ADDRESS, 4); // �ɹ��ָ����ݺ�Ż������־λ���������������ɹ�������ɷ������룬������ҪAPP�в���λ
				// *Ack =  ERR_NO; //��Ӧ�˳���Bootloader
			} else {
				// *Ack =  ERR_ALL_CHECK;
			}
		} else {
			// *Ack =  ERR_REMAP;
		}
	} else if(BootWaitTime > BootWaitTimeLimit) {
		if(CheckSumCheck(APROM_AREA) == 1) { // ���ʱ�䵽��У��App���ݣ���ȷ�����APP
			IAPEnterApp();
		} else if(CheckSumCheck(APROM_BUFF_AREA) == 1) {
			// �����Ϊ����ʹAPP�𻵣���������APP���ƹ���
			IAP_Erase_Some(BUFFER_RESTORE_ADDRESS, 4);
			ReadInt(BUFFER_RESTORE_ADDRESS) = RESTORE_BUFF;
		} else if(CheckSumCheck(APROM_BACKUP_AREA) == 1) {
			IAP_Erase_Some(BACKUP_RESTORE_ADDRESS, 4);
			ReadInt(BACKUP_RESTORE_ADDRESS) = RESTORE_BKP;
		}
		BootWaitTime = 0;
	}
	#endif
}
void BootCheckReset()
{
    if(ResetFlag==1)
    {
        ResetFlag = 0;	
        IAP_Reset();//��λ����APP
    }
}

// void logDebug(char* data, int lenth) {
// 	UartSendOneByte('\n');
// 	UartSendOneByte('d');
// 	UartSendOneByte('b');
// 	UartSendOneByte('=');
// 	for(int i = 0; i < lenth; i++)
// 	{
// 		UartSendOneByte(*(data + i));
// 	}
// 	UartSendOneByte('\n');
// }

void GetVer(uint32_t addr, int lenth)
{
	uint8_t* p_addr = (uint8_t*)addr;
	uint32_t area = 0;
	int i;
	switch(addr) {
		case APP_VER_ADDR: 		area 	= APROM_AREA; 			break;
		case APP_BUFF_VER_ADDR: area 	= APROM_BUFF_AREA;		break;
		case BACKUP_VER_ADDR: 	area 	= APROM_BACKUP_AREA; 	break;
	}
	// ��¼����Ҫ�жϼ����
	// ���У��Ͳ���ȷ���򲻷��ذ汾�ţ������Ժ��ܿ��á�
	if(area == 0 || CheckSumCheck(area) == 1) {
		for(i = 0; i < lenth; i++) {
			CmdSendData[CmmuSendLength + i] = *(p_addr + i);
		}
	} else {
		for(i = 0; i < lenth; i++) {
			CmdSendData[CmmuSendLength + i] = 0xff;
		}
	}

	CmmuSendLength += lenth;
}


typedef enum {
	ENTER_CMD = 0xA,
	BUFFER_CMD = 0xB,
	BACKUP_CMD = 0XC,
	BUFFER_FLAG = 0xAAAB,
	BACKUP_FLAG  = 0xACCC,
}SHAKE_FLAG;

static uint16_t g_shakehandFlag = 0x0;
void SetShakehandFlag(SHAKE_FLAG flag)
{
	g_shakehandFlag = g_shakehandFlag << 4;
	g_shakehandFlag |= flag;
}


uint8_t temp = APROM_AREA;
boot_cmd_t BootCmdRun(uint8_t *rBuff, uint32_t dataLen, boot_cmd_t cmd, uint8_t *Ack)
{
    // boot_cmd_t cmd_buff = BOOT_BOOL_FALSE;//����ִ�н������
	VerStru* hexVer = 0x0;
	int i = 0;
    CmmuSendLength = 0;	
	*Ack = ERR_NO;
	
	switch(cmd)
	{
		case PC_SHAKE_ENTER_BOOTMODE:
		case PC_SET_DOWNLOAD_BUFFER:
		case PC_SET_DOWNLOAD_BACKUP:
		{
			// �������HEX�г�������ָ�����Ҫ��������
			if(g_shakehandFlag == BUFFER_FLAG || g_shakehandFlag == BACKUP_FLAG) {
				g_shakehandFlag = 0;
				g_downLoadStatus =  NO_DOWNLOADING;
			}
		}break;
	}
    switch(cmd)//��������ִ����Ӧ�Ķ���
    {
//        case READ_BOOT_CODE_INF: // ��ȡ�汾��
//        {
//			
//			// BeginAddr = APP_ADDR; // ��ַ�޸ĳɻ�������ַΪwriteflash��׼��
//			IAP_Erase_ALL(APROM_AREA);
//            *Ack = ERR_NO;
//        }break;
		case PC_GET_VER:
		{
			hexVer = (VerStru*)(APP_VER_ADDR);
			if(g_flashWritableFlag.bit.appArea == 1) {
				if(CheckSumCheck(APROM_AREA) == 1) {
					memcpy(&CmdSendData[0], hexVer, sizeof(VerStru));
					CmmuSendLength = sizeof(VerStru);
					*Ack = ERR_NO;
				} else {
					*Ack = ERR_ALL_CHECK;
				}
			} else {
				*Ack = ERR_AREA_NOT_WRITABLE;
			}
		}
		break;
		// case PC_GET_VER_BACKUP:
		// {
		// 	hexVer = (TVER*)(BACKUP_VER_ADDR);
		// 	if(g_flashWritableFlag.bit.backupArea == 1) {
		// 		if(CheckSumCheck(APROM_BACKUP_AREA) == 1) {
		// 			memcpy(&CmdSendData[0], hexVer, sizeof(TVER));
		// 			CmmuSendLength = sizeof(TVER);
		// 			*Ack = ERR_NO;
		// 		} else {
		// 			*Ack = ERR_ALL_CHECK;
		// 		}
		// 	} else {
		// 		*Ack = ERR_AREA_NOT_WRITABLE;
		// 	}
		// }
		case PC_GET_INF:
		{
			// BT�汾�Ż�ȡ
			volatile uint32_t pcValue = get_pc();
			GetVer(BOOT_VER_ADDR,					sizeof(VerStru));
			GetVer(APP_VER_ADDR,					sizeof(VerStru));
			GetVer(APP_BUFF_VER_ADDR, 				sizeof(VerStru));
			GetVer(BACKUP_VER_ADDR, 				sizeof(VerStru));
			GetVer((uint32_t)IC_INF_BUFF, 			IC_TYPE_LENTH);
			GetVer((uint32_t)(&g_flashWritableFlag),sizeof(g_flashWritableFlag));
			GetVer((uint32_t)&pcValue, 				sizeof(pcValue));
			*Ack = ERR_NO;
			// logDebug((char*)&DBG->DBGSTOPCR, sizeof(DBG->DBGSTOPCR));
		}
		break;
        // case READ_IC_INF: // ��ȡоƬ�ͺ�
        // {
        //     for(i=0;i < IC_TYPE_LENTH;i++)
        //     {
        //         CmdSendData[i] = IC_INF_BUFF[i];                
        //     }
        //     CmmuSendLength = IC_TYPE_LENTH;
        //     *Ack =  ERR_NO;
        // }break;
        // case HEX_INFO:
        // {
        //     *Ack =  ERR_NO;
        // }break;
		// case PC_GET_VER_BT:
		// {
		// 	g_sendArray = (uint8_t*)(&btVersion);
		// 	for(int i = 0; i < sizeof(VerStru); i++){
		// 		CmdSendData[i] = *(g_sendArray + i);
		// 	}
		// 	CmmuSendLength = sizeof(VerStru);
		// 	*Ack = ERR_NO;
		// }
        case PC_SHAKE_ENTER_BOOTMODE: // �������μ��ɿ�ʼ��¼
        {
			SetShakehandFlag(ENTER_CMD);
			/* �ر�ʱ�� */
//          BaseTimeSystemInit(BOOT_DISABLE);
			#ifndef FLASH_BUFF_ENABLE
			IAP_FlagWrite(0);//��APP��ɱ�־ȥ����������¹���ʧ�����´��ϵ�һֱά����BOOT�ȴ�����
			#endif
            *Ack = ERR_NO;
        }break;

//        case SET_BAUD:
//        {
//            cmd_buff = DEAL_SUCCESS;
//			NewBaud = (((uint32_t)rBuff[4])<<24)+(((uint32_t)rBuff[5])<<16)+(((uint32_t)rBuff[6])<<8)+((uint32_t)rBuff[7]);
//        }break;

        case PC_SET_DOWNLOAD_BUFFER:	//����APROM��������
        {
			SetShakehandFlag(BUFFER_CMD);
			if(g_shakehandFlag != BUFFER_FLAG) {
				*Ack = ERR_NO;
				break;
			}
			if(IAP_Erase_ALL(APROM_BUFF_AREA) == 0) {
				*Ack = ERR_ERASE;
				break;
			}
			BeginAddr = APP_BUFF_ADDR; // ��ַ�޸ĳɻ�������ַΪwriteflash��׼��
			g_downLoadStatus = DOWNLOADING_BUFF;
			NextPacketNumber = 1;
			*Ack = ERR_NO_SHAKE_SUCCESS;
        }break;
		case PC_SET_DOWNLOAD_BACKUP:	//����APROM��������
        {
			SetShakehandFlag(BACKUP_CMD);
			if(g_shakehandFlag != BACKUP_FLAG) {
				*Ack = ERR_NO;
				break;
			}
			if(BACKUP_ADDR < (88 * 1024) || BACKUP_SIZE > MAX_PACK_NUM) { // ���ݵ�ַ����С��88KB������Ӱ�컺������app����
				*Ack = ERR_OPERATE;
				break;
			}
			if(IAP_Erase_ALL(APROM_BACKUP_AREA) == 0) {
				*Ack = ERR_ERASE;
				break;
			}
			BeginAddr = BACKUP_ADDR; // ��ַ�޸ĳɻ�������ַΪwriteflash��׼��
			g_downLoadStatus = DOWNLOADING_BKP;
			NextPacketNumber = 1;
			*Ack = ERR_NO_SHAKE_SUCCESS;
        }break;
		case PC_SET_WRITE_FLASH:// д��app���ɹ������app
		{
			if(g_shakehandFlag != BUFFER_FLAG && g_shakehandFlag != BACKUP_FLAG) {
				*Ack = ERR_SHAKEHAND;
				break;
			}
			if(dataLen != PACKET_SIZE + RECEIVE_PACKET_LENTH) {
				*Ack = ERR_CMD_LEN;
				break;
			}
			#ifdef ENCRYPT_ENABLE
			if(temp==UID_ENC_AREA)
			{
				temp = UID_ENC_AREA_AREA;
			}
			else
			{
				Decrypt_Fun(rBuff+4);
			}
			#endif
			if((rBuff[0] + (uint32_t)rBuff[1] * 0x100) != (NextPacketNumber)) {
				*Ack = ERR_PACKET_NUMBER;
			}

			if(IAP_WriteMultiByte(BeginAddr,(rBuff+DATA_OFFSET),PACKET_SIZE,temp))
			{
				BeginAddr = BeginAddr+PACKET_SIZE;
				NextPacketNumber++;
				*Ack = ERR_NO;
				g_packetTotalNum = 0;
				for(i = 0; i < PACKET_ID_LENTH; i++) {
					g_packetTotalNum += rBuff[i + PACKET_ID_LENTH] << (i * 8);
				}
			}
			else
			{
				*Ack = ERR_OPERATE;
			}
			for(i = 0; i < PACKET_ID_LENTH; i++) {
				CmdSendData[i] = rBuff[i];
			}
			CmmuSendLength = PACKET_ID_LENTH;
		}break;        
		case PC_SET_ALL_CHECKSUM: //����hex�ļ�У���
        {
			for(i = 0; i < PACKET_ID_LENTH; i++) {
				CmdSendData[i] = rBuff[i];
			}
			// CheckSum[0] = CommuData[7];
			// CheckSum[1] = CommuData[8];
			CheckSum = rBuff[0] + rBuff[1] * 0x100;
			// All_CheckSum_Write(CheckSum, APP_CHECKSUM_ADRESS);
			// uint32ValWrite(g_packetTotalNum, APP_TOTAL_NUM_ADRESS);
			// if(AppCheckSumCheck() == 1)
			// {
			// 	*Ack = ERR_NO; //��Ӧ�˳���Bootloader
			// } else {
			// 	*Ack = ERR_ALL_CHECK;
			// }
			if(g_downLoadStatus == DOWNLOADING_BUFF) {
				CheckSumWrite(g_packetTotalNum, CheckSum, APROM_BUFF_AREA);
				uint32ValWrite(RESTORE_BUFF, BUFFER_RESTORE_ADDRESS);
				g_packetTotalNum = 0;
				if(CheckSumCheck(APROM_BUFF_AREA) == 1 && ReadInt(BUFFER_RESTORE_ADDRESS) == RESTORE_BUFF)
				{
					*Ack = ERR_NO; //��Ӧ�˳���Bootloader
					
//					CheckSumWrite(0, 0, APROM_AREA); 	// ��app�������óɷǷ�
					ReadInt(BUFFER_RESTORE_ADDRESS) = RESTORE_BUFF;	// ���ûָ���������־λ,�ȴ�����bt��
					g_downLoadStatus = DOWNLOADED_BUFF;	// �޸�����״̬
					g_shakehandFlag = 0x0;				// ������ֳɹ���־λ
				} else {
					*Ack = ERR_ALL_CHECK;
				}
			} else if(g_downLoadStatus == DOWNLOADING_BKP){
				CheckSumWrite(g_packetTotalNum, CheckSum, APROM_BACKUP_AREA);
				g_packetTotalNum = 0;
				if(CheckSumCheck(APROM_BACKUP_AREA) == 1)
				{
					*Ack = ERR_NO; //��Ӧ�˳���Bootloader
					g_downLoadStatus = DOWNLOADED_BUFF;
					g_shakehandFlag = 0x0;
				} else {
					*Ack = ERR_ALL_CHECK;
				}
			} else if(g_downLoadStatus == DOWNLOADED_BUFF || g_downLoadStatus == DOWNLOADED_BKP) {
				*Ack = ERR_DOWNLOAD_DONE;
			}

        }break;        
       case BMS_SHAKE_ENTER_APP: //�����û�����
       {
			*Ack = ERR_NO;
		   IAP_Reset();
//           	g_restoreBufferFlag = RESTORE_BUFF;
       }break;        
        case NO_CMD://�޲���
        {
            *Ack = ERR_CMD_ID;
        }break;
        case PC_GET_READ_FLASH: // ��ȡflash����δʹ�ô˹���
        {            
			//ReadFlashAddr = rBuff[6]*256+rBuff[7];
            ReadFlashAddr = (((uint32_t)rBuff[0])<<24)+(((uint32_t)rBuff[1])<<16)+(((uint32_t)rBuff[2])<<8)+((uint32_t)rBuff[3]);
			ReadFlashLength = (rBuff[4]<<24)+(rBuff[5]<<16)+(rBuff[6]<<8)+rBuff[7];            
            // if((rBuff[4])==RETURN_FLASH_UID)
            // {
            //     temp = APROM_AREA;
			// 	ReadFlashAddr = UID_BASE;
            // }
            // else
            // {
            //     temp = APROM_AREA;
            // }
			IAP_ReadMultiByte(ReadFlashAddr,CmdSendData,ReadFlashLength,temp);								

            CmmuSendLength = ReadFlashLength;
        }break;
		case PC_SET_RESTORE_BACKUP:
		// �ָ����������� 1���� 2ǿ�ƻָ����� 3��ת��bt 4�ָ� 5��ת��app
		{
			if(IAP_ReadOneByte(BACKUP_ADDR,IAP_CHECK_AREA) == 0x0) { // �ж�BACKUP�����Ƿ�������
				*Ack = ERR_AREA_BLANK;
				break;
			}
			uint32ValWrite(RESTORE_BKP, BACKUP_RESTORE_ADDRESS);
			if(ReadInt(BACKUP_RESTORE_ADDRESS) != RESTORE_BKP) {
				*Ack = ERR_AREA_NOT_WRITABLE;
				break;
			}
			if(CheckSumCheck(APROM_BACKUP_AREA) == 1) { // У��BACKUP����У���
				ReadInt(BACKUP_RESTORE_ADDRESS) = RESTORE_BKP;		// ���ñ�־λ������bt��ʼ�ָ�backup��
				*Ack = ERR_NO;
			} else {
				*Ack = ERR_ALL_CHECK;
			}
		}
		break;
        default:
        {
            // CmdSendData[0] = ERR_CHKSUM;
            CmmuSendLength = 0;
            *Ack = ERR_CMD_ID;
        }
        break;
    }
    if(*Ack != ERR_CMD_ID) {
#ifndef BMS_APP_DEVICE
		BootWaitTime = 0;
		BootWaitTimeLimit = YES_CMD_BOOT_WAIT_LIMIT;
#endif
        return (cmd | 0x80);
    } else {
        return cmd;
    }
}

/*boot_core.c*/
/*boot_core.c*/
/*boot_core.c*/

//main
//main

void BootWaitTimeInit(void)
{
	BootWaitTimeLimit = NO_CMD_BOOT_WAIT_LIMIT; // ����APP�ȴ���ʼ��ʱ
	BootWaitTime = 0;
}

#ifndef BMS_APP_DEVICE
void BootProcess(void)
{
	AppRestore();
	
	
	if(UartReceFlag)
	{
		UartReceFlag = 0;
		g_tUartData.pbuf = CommuData;
		g_tUartData.wLen = CmmuLength;
		DownloadProcess(&g_tUartData,0);
	}
	
	BootCheckReset(); // ��ת�������������㼴����ת��app
}
#endif

#ifndef BMS_APP_DEVICE
void UART1_Start(void)
{
    SCI0->SO0 |= _0004_SCI_CH2_DATA_OUTPUT_1;
    SCI0->SOE0 |= _0004_SCI_CH2_OUTPUT_ENABLE;
    SCI0->SS0 |= _0008_SCI_CH3_START_TRG_ON | _0004_SCI_CH2_START_TRG_ON;
    INTC_ClearPendingIRQ(ST1_IRQn); /* clear INTST1 interrupt flag */
    INTC_ClearPendingIRQ(SR1_IRQn); /* clear INTSR1 interrupt flag */
    NVIC_ClearPendingIRQ(ST1_IRQn); /* clear INTST1 interrupt flag */
    NVIC_ClearPendingIRQ(SR1_IRQn); /* clear INTSR1 interrupt flag */
    INTC_DisableIRQ(ST1_IRQn);       /* enable INTST1 interrupt */	// ȡ�������ж�
    INTC_EnableIRQ(SR1_IRQn);       /* enable INTSR1 interrupt */
}

MD_STATUS UART1_BaudRate(uint32_t fclk_freq, uint32_t baud)
{
    MD_STATUS status;
    uart_baud_t pvalue;

    status = UART_BaudRateCal(fclk_freq, baud, &pvalue);

    if (status == MD_OK)
    {
        SCI0->ST0 = _0008_SCI_CH3_STOP_TRG_ON | _0004_SCI_CH2_STOP_TRG_ON;
        SCI0->SPS0 = _0000_SCI_CK01_fCLK_0 | pvalue.prs;
        SCI0->SDR02 = pvalue.sdr << 9;
        SCI0->SDR03 = pvalue.sdr << 9;
        SCI0->SS0 |= _0008_SCI_CH3_START_TRG_ON | _0004_SCI_CH2_START_TRG_ON;
    }

    return (status);
}

MD_STATUS UART1_Init(uint32_t freq, uint32_t baud)
{
    MD_STATUS status;
    CGC->PER0 |= CGC_PER0_SCI0EN_Msk;
	
	SCI0->SPS0 &= ~SCI0_SPS0_PRS00_Msk;	//ѡ��ͨ��0�Ĵ���ʱ�ӣ�
	//SCI0->SPS0 &= ~SCI0_SPS0_PRS00_Msk;
    
    SCI0->ST0 |= _0008_SCI_CH3_STOP_TRG_ON | _0004_SCI_CH2_STOP_TRG_ON;
    INTC_DisableIRQ(ST1_IRQn);       /* disable INTST1 interrupt */
    INTC_DisableIRQ(SR1_IRQn);       /* disable INTSR1 interrupt */
    INTC_DisableIRQ(SRE1_IRQn);      /* disable INTSRE1 interrupt */
    INTC_ClearPendingIRQ(ST1_IRQn);  /* clear INTST1 interrupt flag */
    INTC_ClearPendingIRQ(SR1_IRQn);  /* clear INTSR1 interrupt flag */
    INTC_ClearPendingIRQ(SRE1_IRQn); /* clear INTSRE1 interrupt flag */

    /* transmission channel */
    SCI0->SMR02 = _0020_SMRMN_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS |
                  _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR02 = _0004_SCRMN_DEFAULT_VALUE | _8000_SCI_TRANSMISSION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK |
                  _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR02 = _CE00_SCI_BAUDRATE_DIVISOR;
    /* reception channel */
    MISC->NFEN0 |= _04_SCI_RXD1_FILTER_ON;
    SCI0->SIR03 = _0004_SCI_SIRMN_FECTMN | _0002_SCI_SIRMN_PECTMN | _0001_SCI_SIRMN_OVCTMN;
    SCI0->SMR03 = _0020_SMRMN_DEFAULT_VALUE | _0000_SCI_CLOCK_SELECT_CK00 | _0000_SCI_CLOCK_MODE_CKS |
                  _0100_SCI_TRIGGER_RXD | _0000_SCI_EDGE_FALL | _0002_SCI_MODE_UART | _0000_SCI_TRANSFER_END;
    SCI0->SCR03 = _0004_SCRMN_DEFAULT_VALUE | _4000_SCI_RECEPTION | _0000_SCI_TIMING_1 | _0000_SCI_INTSRE_MASK |
                  _0000_SCI_PARITY_NONE | _0080_SCI_LSB | _0010_SCI_STOP_1 | _0003_SCI_LENGTH_8;
    SCI0->SDR03 = _CE00_SCI_BAUDRATE_DIVISOR;
    /* output enable */
    SCI0->SO0 |= _0004_SCI_CH2_DATA_OUTPUT_1;
    SCI0->SOL0 &= (uint16_t)~_0004_SCI_CHANNEL2_INVERTED;
    SCI0->SOE0 |= _0004_SCI_CH2_OUTPUT_ENABLE;
    /* Set TxD1 pin */
    TXD1_PORT_SETTING();	//�ض�λ��P72/P73
    /* Set RxD1 pin */
    RXD1_PORT_SETTING();
    /* UART1 Start, Setting baud rate */
    status = UART1_BaudRate(freq, baud);
    UART1_Start();

    return (status);
}

// #define USE_SCI_UART1_TX
// #define USE_SCI_UART1_RX

#if defined USE_SCI_UART1_TX
void IRQ13_Handler(void) __attribute__((alias("uart1_interrupt_send")));
#elif defined USE_SCI_SPI10
void IRQ13_Handler(void) __attribute__((alias("spi10_interrupt")));
#elif defined USE_SCI_IIC10
void IRQ13_Handler(void) __attribute__((alias("iic10_interrupt")));
#endif

#if defined USE_SCI_UART1_RX
void IRQ14_Handler(void) __attribute__((alias("uart1_interrupt_receive")));
#elif defined USE_SCI_SPI11
void IRQ14_Handler(void) __attribute__((alias("spi11_interrupt")));
#elif defined USE_SCI_IIC11
void IRQ14_Handler(void) __attribute__((alias("iic11_interrupt")));
#endif

/***********************************************************************************************************************
* Function Name: uart1_interrupt_receive
* @brief  UART1 Receive interrupt service routine
* @param  None
* @return None
***********************************************************************************************************************/

void uart1_callback_error(void)
{
	
}
void uart1_interrupt_receive(void)
{
    volatile uint8_t rx_data;
    volatile uint8_t err_type;
    uartId id = UART1;


    INTC_ClearPendingIRQ(SR1_IRQn);
    err_type = (uint8_t)(SCI0->SSR03 & 0x0007U);
    SCI0->SIR03 = (uint16_t)err_type;
    // INTC_ClearPendingIRQ(SR1_IRQn);
    // SCI0->SIR03 = (uint16_t)err_type;
    if (err_type != 0U)
    {
        uart1_callback_error();
    }
    // rx_data = SCI0->RXD1;






    // if(id == UART0) {
    //     err_type = (uint8_t)(SCI0->SSR01 & 0x0007U);
    //     SCI0->SIR01 = (uint16_t)err_type;
    //     rx_data = SCI0->RXD0;
    // } else if(id == UART1) {
    //     err_type = (uint8_t)(SCI0->SSR03 & 0x0007U);
    //     SCI0->SIR03 = (uint16_t)err_type;
    //     rx_data = SCI0->RXD1;
    // } else if(id == UART2) {
    //     err_type = (uint8_t)(SCI1->SSR11 & 0x0007U);
    //     SCI1->SIR11 = (uint16_t)err_type;
    //     rx_data = SCI1->RXD2;
    // }

    UartReceData(id);
}

void uart1_interrupt_send(void)
{
    INTC_ClearPendingIRQ(ST0_IRQn); /* clear INTST0 interrupt flag */
	uart1_callback_sendend();

}


/*************************************************************************
***Module	:	GPIO code module
***brief 	:
***
*************************************************************************/
TGPIO PIN_SW 	= {PORT1,PIN6,PULLUP_INPUT};		//
TGPIO PIN_HEATE_N 	= {PORT1,PIN3,OUTPUT};		//
TGPIO PIN_ALERT	= {PORT3,PIN1,PULLUP_INPUT};		//	

TGPIO PIN_VBCTL = {PORT1,PIN5,OUTPUT};		//ok
TGPIO PIN_CDEN 	= {PORT1,PIN4,OUTPUT};		//ok
TGPIO PIN_CEN 	= {PORT1,PIN7,OUTPUT};		//ok

TGPIO PIN_RED   = {PORT12,PIN0,OUTPUT};		//ok
TGPIO PIN_GREEN = {PORT4,PIN1,OUTPUT};		//ok

TGPIO PIN_FUSE_EN 	= {PORT1,PIN1,OUTPUT};	//δ����




TGPIO PIN_COM3V3_EN = {PORT6,PIN2,OUTPUT};	//OK PIN_AMP_EN
TGPIO PIN_COM5V_EN = {PORT6,PIN3,OUTPUT};	//OK

TGPIO PIN_DEBUG ={PORT14,PIN6,OUTPUT};



/********************************************************************************
GPIO��������,�������ŵ�ƽ��Ҫ����
********************************************************************************/
//�����Դ����
#define  	VB_ON		(PORT_SetBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))	 
#define		VB_OFF		(PORT_ClrBit(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin))
#define 	IS_VB_ON	(PORT_GetBit(PIN_VBCTL.emGPIOx,PIN_VBCTL.emPin))

//���尴������
#define		IS_SWITCH_PUSH	((PORT_GetBit(PIN_SW.emGPIOx,PIN_SW.emPin)))

//������������ر�
#define  	HEAT_ON		(PORT_SetBit(PIN_HEATE_N.emGPIOx,	PIN_HEATE_N.emPin))	 
#define		HEAT_OFF	(PORT_ClrBit(PIN_HEATE_N.emGPIOx,	PIN_HEATE_N.emPin))
#define		IS_HEAT_EN	((PORT_GetBit(PIN_HEATE_N.emGPIOx,PIN_HEATE_N.emPin)))

//�̵�LED
#define		GREEN_ON		(PORT_ClrBit  (PIN_GREEN.emGPIOx,	PIN_GREEN.emPin))
#define		GREEN_OFF		(PORT_SetBit(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin))
#define		IS_GREEN_ON		(!PORT_GetBit(PIN_GREEN.emGPIOx,PIN_GREEN.GPIO_Pin))
#define		GREEN_REVERSE	(PORT_ToggleBit(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin))	

//���LED
#define		RED_ON			(PORT_ClrBit(PIN_RED.emGPIOx,	PIN_RED.emPin))
#define		RED_OFF			(PORT_SetBit(PIN_RED.emGPIOx,	PIN_RED.emPin))
#define		IS_RED_ON		(!PORT_GetBit(PIN_RED.emGPIOx,PIN_RED.emPin))
#define		RED_REVERSE		(PORT_ToggleBit(PIN_RED.emGPIOx,	PIN_RED.emPin))		
#define		LED_ALL_REVERSE	{GREEN_REVERSE;RED_REVERSE}	

//����LED
#define		DEBUG_LED_ON		(PORT_SetBit(PIN_DEBUG.emGPIOx,	PIN_DEBUG.emPin))
#define		DEBUG_LED_OFF		(PORT_ClrBit(PIN_DEBUG.emGPIOx,	PIN_DEBUG.emPin))

//����������˿���ƣ�����Ч
//#define 	HEATFUSE_ON		(PORT_ClrBit(PIN_HEATFUSE_EN.emGPIOx,	PIN_HEATFUSE_EN.emPin))
//#define 	HEATFUSE_OFF	(PORT_SetBit(PIN_HEATFUSE_EN.emGPIOx,	PIN_HEATFUSE_EN.emPin))
//#define		IS_HEATFUSE_OFF  ((PORT_GetBit(PIN_HEATFUSE_EN.emGPIOx,PIN_HEATFUSE_EN.emPin)))

//���˱���˿����,��λ�۶�
#define	  FUSE_OFF	(PORT_SetBit(PIN_FUSE_EN.emGPIOx,	PIN_FUSE_EN.emPin))
#define		FUSE_ON	    (PORT_ClrBit(PIN_FUSE_EN.emGPIOx,	PIN_FUSE_EN.emPin))
#define		IS_FUSE_OFF	(PORT_GetBit(PIN_FUSE_EN.emGPIOx,PIN_FUSE_EN.emPin))

//485����ʹ��
//#define		RS485_SEND_ENABLE	(PORT_SetBit(PIN_485DE.emGPIOx,	PIN_485DE.emPin))
//#define		RS485_SEND_DISABLE	(PORT_ClrBit(PIN_485DE.emGPIOx,	PIN_485DE.emPin))

//PCAK
//#define		PACKADC_DISABLE	(PORT_ClrBit(PIN_PACKADC_EN.emGPIOx,	PIN_PACKADC_EN.emPin))
//#define		PACKADC_ENABLE	(PORT_SetBit(PIN_PACKADC_EN.emGPIOx,	PIN_PACKADC_EN.emPin))

//�������
#define 	C_ON		(PORT_SetBit(PIN_CEN.emGPIOx,	PIN_CEN.emPin))
#define 	C_OFF		(PORT_ClrBit(PIN_CEN.emGPIOx,	PIN_CEN.emPin))
#define		IS_C_ON		(PORT_GetBit(PIN_CEN.emGPIOx,PIN_CEN.emPin))

//�ŵ�����
#define 	CD_ON		(PORT_SetBit(PIN_CDEN.emGPIOx,	PIN_CDEN.emPin))
#define 	CD_OFF		(PORT_ClrBit(PIN_CDEN.emGPIOx,	PIN_CDEN.emPin))
#define		IS_CD_ON	(PORT_GetBit(PIN_CDEN.emGPIOx,PIN_CDEN.emPin))

//3V3 485��Դʹ��
#define 	PIN_COM3V3_ON		(PORT_SetBit(PIN_COM3V3_EN.emGPIOx,	PIN_COM3V3_EN.emPin))  //��Դ�ر�
#define 	PIN_COM3V3_OFF		(PORT_ClrBit(PIN_COM3V3_EN.emGPIOx,	PIN_COM3V3_EN.emPin))  //��Դ����
#define		IS_COM3V3_ON	((PORT_GetBit(PIN_COM3V3_EN.emGPIOx,PIN_COM3V3_EN.emPin)))

//5V �����Դʹ��
#define 	PIN_COM5V_ON		(PORT_SetBit(PIN_COM5V_EN.emGPIOx,	PIN_COM5V_EN.emPin))
#define 	PIN_COM5V_OFF		(PORT_ClrBit(PIN_COM5V_EN.emGPIOx,	PIN_COM5V_EN.emPin))
#define		IS_PIN_COM5V_ON		((PORT_GetBit(PIN_COM5V_EN.emGPIOx,PIN_COM5V_EN.emPin)))

//AFE����ģʽ
//#define		REGOUT_ON		(PORT_ClrBit(PIN_REGOUT_EN.emGPIOx,	PIN_REGOUT_EN.emPin))	
//#define		REGOUT_OFF		(PORT_SetBit(PIN_REGOUT_EN.emGPIOx,	PIN_REGOUT_EN.emPin))
//#define		IS_REGOUT_ON	(!(PORT_GetBit(PIN_REGOUT_EN.emGPIOx,PIN_REGOUT_EN.emPin)))

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Config(void)
{
	//����
	PORT_Init(PIN_SW.emGPIOx,		PIN_SW.emPin,		PIN_SW.emMode);	
	PORT_Init(PIN_ALERT.emGPIOx,	PIN_ALERT.emPin,	PIN_ALERT.emMode);	
	PORT_Init(PORT5,PIN1,INPUT);    //485����
  PORT_Init(PORT14,PIN0,PULLUP_INPUT);
	//PORT_Init(PIN_REV.emGPIOx,		PIN_REV.emPin,		PIN_REV.emMode);
  	
	//���
	PORT_Init(PIN_VBCTL.emGPIOx,	PIN_VBCTL.emPin,	PIN_VBCTL.emMode);
	VB_ON;
   
	PORT_Init(PIN_COM5V_EN.emGPIOx,PIN_COM5V_EN.emPin,PIN_COM5V_EN.emMode);
  PIN_COM5V_OFF;
	
	PORT_Init(PIN_COM3V3_EN.emGPIOx,PIN_COM3V3_EN.emPin,PIN_COM3V3_EN.emMode);
	PIN_COM3V3_OFF; 
	
  PORT_Init(PIN_HEATE_N.emGPIOx,	PIN_HEATE_N.emPin,	PIN_HEATE_N.emMode);
  HEAT_OFF;
	
	PORT_Init(PIN_CDEN.emGPIOx,		PIN_CDEN.emPin,		PIN_CDEN.emMode);
	CD_OFF;
	
	PORT_Init(PIN_CEN.emGPIOx,		PIN_CEN.emPin,		PIN_CEN.emMode);
	C_OFF;
	
	PORT_Init(PIN_GREEN.emGPIOx,	PIN_GREEN.emPin,	PIN_GREEN.emMode);
	GREEN_OFF;
	
	PORT_Init(PIN_RED.emGPIOx,		PIN_RED.emPin,		PIN_RED.emMode);
	RED_OFF;
	
	PORT_Init(PIN_DEBUG.emGPIOx,		PIN_DEBUG.emPin,		PIN_DEBUG.emMode);
	DEBUG_LED_ON;
	

	PORT_Init(PIN_FUSE_EN.emGPIOx,	PIN_FUSE_EN.emPin,	PIN_FUSE_EN.emMode);
	FUSE_ON;
	
	PORT_Init(PORT5,PIN0,OUTPUT);    //CTLD
	PORT_SetBit(PORT5,PIN0);
	
	//PORT_Init(PIN_HEATFUSE_EN.emGPIOx,PIN_HEATFUSE_EN.emPin,PIN_HEATFUSE_EN.emMode);
	//HEATFUSE_ON;
    
	//PORT_Init(PIN_PACKADC_EN.emGPIOx,	PIN_PACKADC_EN.emPin,	PIN_PACKADC_EN.emMode);
	//PACKADC_ENABLE;
	
	//PORT_Init(PIN_485DE.emGPIOx,	PIN_485DE.emPin,	PIN_485DE.emMode);
	//RS485_SEND_DISABLE;
	
	//δʹ�ùܽ�����
	PORT_Init(PORT13,PIN6,OUTPUT);
	PORT_Init(PORT7,PIN5,OUTPUT); 
	PORT_Init(PORT7,PIN4,OUTPUT); 
	PORT_Init(PORT3,PIN0,OUTPUT);
	PORT_Init(PORT1,PIN2,OUTPUT);
  PORT_Init(PORT1,PIN0,OUTPUT);  
	PORT_Init(PORT2,PIN0,OUTPUT);
	PORT_Init(PORT2,PIN1,OUTPUT);
	PORT_Init(PORT2,PIN2,OUTPUT);
  PORT_Init(PORT2,PIN3,OUTPUT);
  PORT_Init(PORT2,PIN4,OUTPUT);
  PORT_Init(PORT2,PIN5,OUTPUT);
	PORT_Init(PORT13,PIN0,OUTPUT); 
	

}

//void Clock_Config(void)
//{	
//	CLK_Osc_Setting(OSC_PORT, OSC_PORT); /* MainOSC/SubOSC enable */
//	CLK_MainOsc_Setting(OSC_PORT,OSC_OVER_10M);   //OSC_PORT  OSC_OSCILLATOR
//	CLK_Fclk_Select(MAINCLK_FIH);//select FMX   MAINCLK_FIH   MAINCLK_FMX
////	while((CGC->CKC & CGC_CKC_MCS_Msk) == 0);
//	SystemCoreClock = 8000000;  //12000000
//}
/********************************************************************************************************
**????		:   void HardDriveInit(void)
**????      :   ?
**????      :   ?                  
**???		:	?   
**??		    :	??????????
**??          :  zml
**??          :  2022-07-05
*********************************************************************************************************/
void HardDriveInit(void)
{
	Clock_Config();		//OK
#ifdef BMS_BT_DEVICE
	system_tick_init();
#endif 
	
//	GPIO_Config();		//OK
	//Ext_INT_Config();	//?????
	//TimeTick_Config();	//OK
	// TIM_Config();		//OK		
	// USART_Config();		//OK
	//CAN_Config();		//CAN OK?
	// ADC_Config();		//OK
	//sEE_Config();		//OK
	UART1_Init(SystemCoreClock, UartBaud);

}
#endif


uint8_t CheckAreaWritable(uint32_t addr)
{
	uint8_t ok = 0;
	uint8_t CheckFlashBuff[512] = {0};
	int i = 0;
	for(i = 0; i < 512; i++)
	{
		CheckFlashBuff[i] = IAP_ReadOneByte(addr + i,IAP_CHECK_AREA);
	}
	IAP_Erase_512B(addr & 0xffffff00,IAP_CHECK_AREA);
	ok = IAP_WriteOneByte_Check(addr,(0x55),IAP_CHECK_AREA);

	if(ok == 1) {
		IAP_Erase_512B(addr & 0xffffff00,IAP_CHECK_AREA);
		for(i=0;i<512;i++)
		{
			IAP_WriteOneByte_Check(addr + i, CheckFlashBuff[i], IAP_CHECK_AREA);
		}
	}
	return ok;
}
#ifdef BMS_BT_DEVICE
void CmdSendFunc(uint8_t *sBuff, uint32_t lenth)
{
	uint32_t i;
	for(i = 0; i < lenth; i++) {
		UartSendOneByte(*(sBuff + i));
	}
}
#endif

void DownloadTermination(void)
{
	if(g_downLoadStatus == DOWNLOADING_BUFF) {
		IAP_Erase_ALL(APROM_BUFF_AREA); // ����������ݻ�����
		CheckSumWrite(0xffffffff, 0xffffffff, APROM_BUFF_AREA); // ���У��Ͱ�����
		uint32ValWrite(0x0, BUFFER_RESTORE_ADDRESS); // ����ָ���־λ

	} else if(g_downLoadStatus == DOWNLOADING_BKP) {
		IAP_Erase_ALL(APROM_BACKUP_AREA);
		CheckSumWrite(0xffffffff, 0xffffffff, APROM_BACKUP_AREA);
		uint32ValWrite(0x0, BACKUP_RESTORE_ADDRESS);
	}
	// ������ֱ�־λ, ���û��g_downLoadStatus˵���������֣�Ҳ��Ҫ������ֱ�־λ
	g_shakehandFlag = 0x0;

}

uint32_t g_errTime = 0;

void DownloadProcess(void *p,UCHAR ucComPort)
{
	uint8_t  *rBuff, cmd, Ack;                         //????????????
	uint32_t  wholeDataLen, unitDataLen;	                          //???????????
	rBuff 		= 	((TUartData *)(p))->pbuf;
	wholeDataLen	=	((TUartData *)(p))->wLen;

	cmd = AnalysisData(rBuff, wholeDataLen, &unitDataLen,&Ack);  // �������жϺ����ܻ�ȡ�����ݰ��� ����cmd

	if (Ack == ERR_NO) {
		result_cmd = BootCmdRun(&rBuff[7], unitDataLen, cmd, &Ack);  // ����cmd������Ӧ����
	}
	if(Ack != ERR_NO && Ack != ERR_NO_SHAKE_SUCCESS) {
		if(++g_errTime > 3) {
			g_errTime = 0;
			DownloadTermination();
		} 
	} else {
		g_errTime = 0;
	}
	// fillbackFunc
#ifdef BMS_APP_DEVICE
	fillbackFunc(SysSendUart[g_byRecComChn].pSendBuff	, CmdSendData, result_cmd, CmmuSendLength, Ack);
	SysSendUart[g_byRecComChn].EndPos += 9 + CmmuSendLength;
#else
	fillbackFunc(CmdSendAll								, CmdSendData, result_cmd, CmmuSendLength, Ack);
	CmdSendAllLenth 				  += 9 + CmmuSendLength;
	CmdSendFunc(CmdSendAll, CmdSendAllLenth);
	ClearCommu();
#endif

	if(ReadInt(BUFFER_RESTORE_ADDRESS) == RESTORE_BUFF || ReadInt(BACKUP_RESTORE_ADDRESS) == RESTORE_BKP) {	// ���ûָ���������־λ,�ȴ�����bt��)
	// �������if��֤��bt������޷����BUFFER_RESTORE_ADDRESS��־λ�����������ѭ��
		if(g_downLoadStatus == DOWNLOADED_BUFF || g_downLoadStatus == DOWNLOADED_BKP) {
#ifdef BMS_APP_DEVICE
			SetDelayTask(IAP_Reset, NULL, 1000);
#else
			ResetFlag = 1;
#endif
		}
	}
}
