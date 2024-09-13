//************************************************************
//  Copyright (c) 
//	�ļ�����	: boot.c
//	ģ�鹦��	: boot��Ҫ�����ļ�
//  ��������	: 2024/9/12
// 	�汾		: V1.0
//************************************************************
#include "boot.h"

/*���ڽ��շ���*/
uint32_t CmmuReadNumber;	//ͨѶ��ǰ��ȡ����Ϊһ֡�еĵڼ�����
uint8_t UartReceFlag;				//UART0������һ֡��־λ
uint8_t UartSendFlag;				//UART0������һByte��־λ

uint8_t CommunicationCheckNumber;			//У��λ
commu_length_t CmmuLength;						//�������ݳ���
uint8_t CMDBuff;								//����洢����
commu_data_t CommuData[ReceiveLength1];	//ͨѶ���ջ���
commu_data_t CmdSendData[SendLength1];	//���ͻ���
commu_length_t CmmuSendLength;		            //�������ݳ���
uint8_t CRCchecksum[4];

uint8_t uart_send_flag = 0;


volatile uint8_t * gp_uart0_tx_address;         /* uart0 send buffer address */
volatile uint16_t  g_uart0_tx_count;            /* uart0 send data number */
volatile uint8_t * gp_uart0_rx_address;         /* uart0 receive buffer address */
volatile uint16_t  g_uart0_rx_count;            /* uart0 receive data number */
volatile uint16_t  g_uart0_rx_length;           /* uart0 receive data length */

/*���ڽ��շ���*/

/*�������ݷ���*/
uint8_t result_cmd;
extern volatile uint8_t ACK;
/*�������ݷ���*/

uint32_t BootWaitTime = 0;
uint32_t BootWaitTimeLimit = 0;


// ���ڳ�ʼ�������ں���
// ���ڽ��շ���
// �������ݷ���

// flash����

void IRQ10_Handler(void) __attribute__((alias("uart0_interrupt_send")));
void IRQ11_Handler(void) __attribute__((alias("uart0_interrupt_receive")));

void UartInit(uint32_t baud)
{
    SCI0_Init();
    /* UART0 Start, Setting baud rate */
    UART0_BaudRate(Fsoc, baud);
    UART0_Start();
}

void UartSendOneByte(uint8_t input_data)
{
    SCI0->TXD0 = input_data;
    while(!UartSendFlag);
    UartSendFlag = 0;
//    while (SCI0->SSR00 & (_0040_SCI_UNDER_EXECUTE | _0020_SCI_VALID_STORED))
//    {
//        ;
//    }
}
#define SLAVE_ADDRESS 0x00//�豸��ַ
void UartReceData(uartId id)//��������֡
{
	if(!UartReceFlag)
	{		
		if(id == UART0) {
			CommuData[CmmuReadNumber] = SCI0->RXD0;		//�������������뻺��
		}else if(id == UART1) {
			CommuData[CmmuReadNumber] = SCI0->RXD1;		//�������������뻺��
		}else if(id == UART2) {
			CommuData[CmmuReadNumber] = SCI1->RXD2;		//�������������뻺��
		}

		if(CommuData[0] == SLAVE_ADDRESS)
		{
			CmmuReadNumber++;
		}
		if(CmmuReadNumber >= 3) {
			if(CmmuReadNumber>=(3 + CommuData[1] * 0x100 + CommuData[2] + 1)) //������������256�Ļ���Ҫ�޸�CmmuReadNumber����
			{
				/* �������Ź����幷 */
				UartReceFlag = 1;	  //��ʾ���յ�һ֡����
			}
		}

	}
}
void ClearCommu()
{
    CommuData[0] = 0; //�������������ͷ��׼���´δ������ݵ���
    CmmuReadNumber = 0; //���¼�����׼���´δ������ݵ���
    UartReceFlag = 0; //���������ɱ�־
}

void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data)
{
	uint8_t i;
	uint8_t check_sum = 0;
	UartSendOneByte(SEND_ADDRESS);	//����֡ͷ
	
	UartSendOneByte((Data_len + 5) >> 8);		 				 		//���������򳤶ȸ�8λ
	UartSendOneByte(Data_len + 5);		 			 	//���������򳤶ȵ�8λ
	UartSendOneByte(SEND_BMS_TYPE);					//���͵���������
	UartSendOneByte(Command);					 	//���Ϳ�����
	UartSendOneByte(SEND_SHAKE_1);					//������1
	UartSendOneByte(SEND_SHAKE_2);					//������1
	UartSendOneByte(ACK);							//����Ӧ����
	check_sum = ((Data_len + 5) >> 8) + (Data_len + 5) + SEND_BMS_TYPE + Command + SEND_SHAKE_1 + SEND_SHAKE_2 + ACK;
	for(i=0;i<Data_len;i++)	  					 	//����������
	{
		UartSendOneByte(*(Data+i));
		check_sum+=	*(Data+i);
	}	
	UartSendOneByte(check_sum);						//����У��λ��8λ
	// UartSendOneByte(CommunicationCommandEnd);		//����֡β   
}
static void uart0_callback_sendend(void)
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
uint8_t AnalysisData()//��������֡������
{
	volatile uint8_t cmd = NO_CMD;
    uint32_t data_len;
	uint32_t i;
	uint8_t check_sum = 0;
	data_len = CommuData[1] * 0x100 + CommuData[2];
	cmd = CommuData[4];

	ACK = ERR_NO;
	//���㵥�����͵�����λ��У���
	for(i=1; i<data_len + 3; i++)
	{
	   check_sum+=CommuData[i];
	}
	if((cmd&0x80) != 0) {
		ACK = ERR_CMD_ID;
	}
	if(cmd != WRITE_FLASH && cmd != REC_ALL_CHECKSUM && CmmuReadNumber != 8) {
		ACK = ERR_CMD_LEN;
	}
	//У��ɹ�,��ȡ������
	if(check_sum!=(CommuData[3 + data_len]))
	{
        ACK = ERR_CHECK;
	}
	if(cmd == WRITE_FLASH) {
		CmmuLength = data_len - TYPE_TO_DATA_LENTH;//ȡ����
	} else {
		CmmuLength = data_len - TYPE_TO_SHAKE_LENTH;//ȡ����
	}

    return cmd;
}
/*flash_operate*/
/*flash_operate*/
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


void IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area)//����һ���飨512B��
{
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
    
}

void IAP_Erase_ALL(uint8_t area)
{
    uint16_t i;
	uint16_t k = 0;
	uint32_t begin_addr = 0;
	if(area==APROM_AREA)
	{
		k = (APP_SIZE/ONE_PAGE_SIZE);
		begin_addr = APP_ADDR;
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
		IAP_Erase_512B(i*ONE_PAGE_SIZE+begin_addr,area);
    }
}


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
//		toggle();
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
    IAP_Erase_512B(IAP_CHECK_ADRESS,IAP_CHECK_AREA);
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
void IAP_Remap()//���������Ĵ���װ����������
{
	uint16_t i;
	IAP_Erase_ALL(APROM_AREA);//����APP����������
	for(i=0;i<APP_BUFF_SIZE;i++)
	{
		 IAP_WriteOneByte(APP_ADDR+i,IAP_ReadOneByte(APP_BUFF_ADDR+i,APROM_AREA),APROM_AREA);
	}
}
#endif

//����������
void __set_VECTOR_ADDR(uint32_t addr)
{
	SCB->VTOR = addr;
}

void IAP_Reset()
{	
    #ifdef ENCRYPT_UID_ENABLE		
	if(!CheckUID())
	{
		return;
	}
	#endif
    SCI0->ST0   = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
	CGC->PER0 &= ~CGC_PER0_SCI0EN_Msk;
	INTC_DisableIRQ(SR0_IRQn);
	__set_VECTOR_ADDR(APP_VECTOR_ADDR); // ��Ҫ������������Ϊʵ�ⷢ��app�����ж���Ȼ������bt��systick
    __set_MSP(*(__IO uint32_t*) APP_ADDR);
    ((void (*)()) (*(volatile unsigned long *)(APP_ADDR+0x04)))();//to APP
    
    /* Trap the CPU */
    while(1);
}
/*flash_operate*/
/*flash_operate*/
/*flash_operate*/


/*boot_core.c*/
/*boot_core.c*/
/*boot_core.c*/
uint8_t HandShakeValue;	                            //���ִ����洢����
uint8_t ResetFlag = 0;								//��ʾ��λ�������
uint8_t CurrState = 0;								//��ǰоƬ��״̬
uint32_t ReadFlashLength = 0;                       //��Flash�ĳ���        
uint32_t ReadFlashAddr = 0;							//��Flash����ʼ��ַ
uint32_t g_packetTotalNum = 0;								//��¼�ļ����ݰ�������
uint32_t CheckSum = 0;
const uint8_t Boot_Inf_Buff[EditionLength] = Edition;//�汾�Ŵ洢
const uint8_t IC_INF_BUFF[IC_EDITION_LENTH] = IC_EDITION; // оƬ�ͺŴ洢
boot_addr_t BeginAddr = APP_ADDR;				    //��ʼ��ַ�洢
// uint32_t NewBaud = UartBaud;						//�洢�²����ʵı���
uint32_t NextPacketNumber = 0;
volatile uint8_t ACK = 0x00;


/* boot��ʼ�����Ӻ������뽫��ʼ������д��ú��� */
void BootInit()
{
	UartInit(UartBaud);
	CurrState = IAP_CheckAPP();
//    if(CurrState==1)//�ж�APP�Ƿ�����������������ʱ
//    {
//        BaseTimeSystemInit(BOOT_ENABLE);
//    }
	#ifdef FLASH_BUFF_ENABLE
	else if(CurrState==2)//�����������ص���������ֱ������APP
	{
		IAP_Remap();//�����뻺���������ݼ��������������
		IAP_FlagWrite(1);//����ΪAPP������̬
        IAP_Reset();
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

void All_CheckSum_Write(uint32_t checkSum)
{
    unsigned char i;
    IAP_Erase_512B(IAP_CHECK_ADRESS,IAP_CHECK_AREA);
	for(i=0;i<TOTAL_CHECKSUM_LENGTH;i++)
	{
		IAP_WriteOneByte(TOTAL_CHECKSUM_ADRESS+i,(checkSum >> (8 * i)),IAP_CHECK_AREA);
	}

}

void PacketTotalNumWrite(uint32_t packetTotalNum)
{
	for(uint32_t i = 0;i < PACKET_TOTAL_NUM_LENGTH; i++)
	{
		IAP_WriteOneByte(PACKET_TOTAL_NUM_ADRESS+i, (packetTotalNum >> (8 * i)), IAP_CHECK_AREA);
	}
}

uint8_t All_CheckSum_Read(uint8_t* checkSum)
{
    unsigned char i;
	volatile uint8_t temp = 0;
    for(i=0;i<TOTAL_CHECKSUM_LENGTH;i++)
    {
		temp = IAP_ReadOneByte(TOTAL_CHECKSUM_ADRESS+i,IAP_CHECK_AREA);
        if(temp != *(checkSum + i))
        {
			return 0;
        }
    }
	return 1;
}

uint32_t PacketTotalNumRead(void)
{
    uint32_t packetTotalNum = 0;
    for(uint32_t i = 0; i<PACKET_TOTAL_NUM_LENGTH; i++)
    {
        packetTotalNum += (IAP_ReadOneByte(PACKET_TOTAL_NUM_ADRESS+i,IAP_CHECK_AREA) << (8 * i));
    }
	if(packetTotalNum == 0xffffffff) {
		return 0;
	}
	return packetTotalNum;
}
uint8_t AllCheckSumCheck(void)
{
	uint32_t packetTatolSize = PACKET_SIZE * PacketTotalNumRead();
	uint16_t calCheckSum = 0;
	
	for(uint32_t i = 0; i < packetTatolSize; i++) {
		calCheckSum += IAP_ReadOneByte(APP_ADDR+i,IAP_CHECK_AREA);
	}

	if(All_CheckSum_Read((uint8_t*)(&calCheckSum)) == 1) {
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




void BootCheckReset()
{
    if(ResetFlag==1)
    {
        ResetFlag = 0;	
//        BaseTimeSystemInit(BOOT_DISABLE);//�رն�ʱ��

		#ifdef FLASH_BUFF_ENABLE
		if(CurrState == 2)
		{
			IAP_FlagWrite(2);//���뻺����������־
			//IAP_Remap();
		}
		if(CurrState == 0)
		{
			IAP_FlagWrite(2);//APP���������б�־
		}
		#else
        // IAP_FlagWrite(1);//APP���������б�־
		#endif	
					
        IAP_Reset();//��λ����APP
    }
}


uint8_t temp = APROM_AREA;
boot_cmd_t BootCmdRun(boot_cmd_t cmd)
{
    uint8_t i;	
    // boot_cmd_t cmd_buff = BOOT_BOOL_FALSE;//����ִ�н������
    CmmuSendLength = 0;	
    ACK = ERR_NO;
    switch(cmd)//��������ִ����Ӧ�Ķ���
    {
        case READ_BOOT_CODE_INF: // ��ȡ�汾��
        {
            for(i=0;i<EditionLength;i++)
            {
                CmdSendData[i] = Boot_Inf_Buff[i];                
            }
            CmmuSendLength = EditionLength;
            ACK = ERR_NO;
        }break;
        case READ_IC_INF: // ��ȡоƬ�ͺ�
        {
            for(i=0;i<EditionLength;i++)
            {
                CmdSendData[i] = IC_INF_BUFF[i];                
            }
            CmmuSendLength = IC_EDITION_LENTH;
            ACK = ERR_NO;
        }break;
        case HEX_INFO:
        {
            ACK = ERR_NO;
        }break;
        case ENTER_BOOTMODE: // �������μ��ɿ�ʼ��¼
        {
            HandShakeValue++;
            if(HandShakeValue>=HandShakes)
            {
               /* �ر�ʱ�� */
//               BaseTimeSystemInit(BOOT_DISABLE);
			   #ifndef FLASH_BUFF_ENABLE
               IAP_FlagWrite(0);//��APP��ɱ�־ȥ����������¹���ʧ�����´��ϵ�һֱά����BOOT�ȴ�����
			   #endif
            }
            ACK = ERR_NO;
        }break;
//        case SET_BAUD:
//        {
//            cmd_buff = DEAL_SUCCESS;
//			NewBaud = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
//        }break;

//        case SET_ADDRESS://���û���ַ��������
//        {
//            //BeginAddr = CommuData[6]*256+CommuData[7];
//			BeginAddr = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
//            //if((CommuData[4])==RETURN_FLASH_UID)
//            if(BeginAddr==UID_ENC_ADRESS)
//            {
//                temp = UID_ENC_AREA;
//            }
//            else
//            {
//                temp = APROM_AREA;
//                #ifdef FLASH_BUFF_ENABLE
//                BeginAddr = APP_BUFF_ADDR;//�����봫��������ȥ
//                #endif		
//            }            			
//            cmd_buff = DEAL_SUCCESS;			
//        }break;
        case EARSE_ALL:	//����APROM��������
        {
			#ifdef FLASH_BUFF_ENABLE
			if(temp==APROM_AREA)
			{
				IAP_Erase_ALL(APROM_BUFF_AREA);
			}
			else
			{
				IAP_Erase_ALL(temp);
			}
			#else
			IAP_Erase_ALL(temp);
			#endif
            ACK = ERR_NO;
			BeginAddr = APP_ADDR;
        }break;
		case WRITE_FLASH:// д��app���ɹ������app
		{
			#ifdef ENCRYPT_ENABLE
			if(temp==UID_ENC_AREA)
			{
				temp = UID_ENC_AREA_AREA;
			}
			else
			{
				Decrypt_Fun(CommuData+4);
			}
			#endif
			if((CommuData[7] + (uint32_t)CommuData[8] * 0x100) != (NextPacketNumber)) {
				ACK = ERR_PACKET_NUMBER;
			}
			if(IAP_WriteMultiByte(BeginAddr,(CommuData+DATA_OFFSET),PACKET_SIZE,temp))
			{
				BeginAddr = BeginAddr+CmmuLength;
				NextPacketNumber++;
				ACK = ERR_NO;
				g_packetTotalNum = 0;
				for(int i = 0; i < PACKET_ID_LENTH; i++) {
					g_packetTotalNum += CommuData[i + 7 + PACKET_ID_LENTH] << (i * 8);
				}
			}
			else
			{
				ACK = ERR_OPERATE;
			}
			for(int i = 0; i < PACKET_ID_LENTH; i++) {
				CmdSendData[i] = CommuData[i + 7];
			}
			CmmuSendLength = PACKET_ID_LENTH;
		}break;        
		case REC_ALL_CHECKSUM: //�����û�����
        {
			for(int i = 0; i < PACKET_ID_LENTH; i++) {
				CmdSendData[i] = CommuData[i + 7];
			}
			// CheckSum[0] = CommuData[7];
			// CheckSum[1] = CommuData[8];
			CheckSum = CommuData[7] + CommuData[8] * 0x100;
			All_CheckSum_Write(CheckSum);
			PacketTotalNumWrite(g_packetTotalNum);
			if(AllCheckSumCheck() == 1)
			{
				ACK = ERR_NO; //��Ӧ�˳���Bootloader
				ResetFlag = 1;
			} else {
				ACK = ERR_ALL_CHECK;
			}
        }break;        
//        case ENTER_APPMODE: //�����û�����
//        {
//            cmd_buff = DEAL_SUCCESS; //��Ӧ�˳���Bootloader 
//            #ifdef FLASH_BUFF_ENABLE            
//			CurrState = 2;//��ʾ���뻺�����������
//            #endif
//            ResetFlag = 1;
//        }break;        
        case NO_CMD://�޲���
        {
            ACK = ERR_CMD_ID;
        }break;
        case READ_FLASH: // ��ȡflash����δʹ�ô˹���
        {            
			//ReadFlashAddr = CommuData[6]*256+CommuData[7];
            ReadFlashAddr = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
			ReadFlashLength = (CommuData[8]<<24)+(CommuData[9]<<16)+(CommuData[10]<<8)+CommuData[11];            
            if((CommuData[4])==RETURN_FLASH_UID)
            {
                temp = APROM_AREA;
				ReadFlashAddr = UID_BASE;
            }
            else
            {
                temp = APROM_AREA;
            }
			IAP_ReadMultiByte(ReadFlashAddr,CmdSendData,ReadFlashLength,temp);								

            CmmuSendLength = ReadFlashLength;
        }break;
        default:
        {
            CmdSendData[0] = ERR_CHECK;
            CmmuSendLength = 0;
            ACK = ERR_CMD_ID;
        }
        break;
    }
    if(ACK != ERR_CMD_ID) {
		BootWaitTime = 0;
		BootWaitTimeLimit = YES_CMD_BOOT_WAIT_LIMIT;
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


void BootProcess(void)
{
	if(UartReceFlag)
	{
		CMDBuff = AnalysisData();  // �������жϺ����ܻ�ȡ�����ݰ��� ����cmd       
		ClearCommu();
		if (ACK == ERR_NO) {
			result_cmd = BootCmdRun(CMDBuff);  // ����cmd������Ӧ����
		}
		CommuSendCMD(result_cmd,CmmuSendLength,CmdSendData); // ��Ӧ��λ��
		result_cmd = BOOT_BOOL_FALSE;
		CMDBuff = 0; 
	}
	if(BootWaitTime > BootWaitTimeLimit) {
		if(AllCheckSumCheck() == 1) {
			ResetFlag = 1;
		} else {
			ResetFlag = 0;
			BootWaitTime = 0;
		}
	}
	BootCheckReset(); // ��ת�������������㼴����ת��app
}
