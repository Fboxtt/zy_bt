//************************************************************
//  Copyright (c) 
//	�ļ�����	: boot.c
//	ģ�鹦��	: boot��Ҫ�����ļ�
//  ��������	: 2024/9/12
// 	�汾		: V1.0
//************************************************************
#include "boot.h"
#include "gpio.h"
#include "cg_sci.h"
#include "communication_protocol.h"
#include "base_time_system.h"
#include "clk.h"

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


typedef struct {
	uint16_t majorVer;
	uint16_t minorVer;
	uint16_t revisionVer;
	uint16_t year;
	uint8_t month;
	uint8_t day;
	uint8_t reserved[2];
}BtVersion;

BtVersion btVersion = {
	1,0,0,2024,10,11
};

uint8_t* g_sendArray;

uint8_t result_cmd;
extern volatile uint8_t ACK;

uint32_t BootWaitTime = 0;
uint32_t BootWaitTimeLimit = 0;

int g_FLSTSMaxCount = 0;

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
//    while(!UartSendFlag);
//    UartSendFlag = 0;
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
	else if(area==APROM_BACKUP_AREA)
	{
		k = (BACKUP_SIZE/ONE_PAGE_SIZE);
		begin_addr = BACKUP_SIZE;
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
		IAP_Erase_512B(i*ONE_PAGE_SIZE+begin_addr,area);
    }
}

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
uint8_t HandShakeValue;	                            //���ִ����洢����
uint8_t BufferFlag = 0;								//��������У��״̬
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
volatile uint8_t ACK = 0x00;

typedef union { // ȷ�������Ƿ��д�ı�־λ
	uint8_t value;
	struct {
		uint8_t firstArea:1;
		uint8_t secondArea:1;
	}bit;
}WritableFlag;

WritableFlag g_flashWritableFlag = {0};

/* boot��ʼ�����Ӻ������뽫��ʼ������д��ú��� */
void BootInit()
{
	// UartInit(UartBaud);
	g_FLSTSMaxCount = 24 / ONE_DISASSEMBLE_COUNT * SystemCoreClock / 1000000 * 2;
	if(CheckAreaWritable(0x20000 - 512) == 1) { // ȷ������0-0x20000�Ƿ��д
		g_flashWritableFlag.bit.firstArea = 1;
	}
	if(CheckAreaWritable(0x40000 - 512) == 1) { // ȷ������0-0x40000�Ƿ��д
		g_flashWritableFlag.bit.secondArea = 1;
	}
	
	CurrState = IAP_CheckAPP();
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
    IAP_Erase_512B(addr & 0xff00,IAP_CHECK_AREA);
	for(i=0;i<CHECKSUM_LENGTH;i++)
	{
		IAP_WriteOneByte(addr+i,(checkSum >> (8 * i)),IAP_CHECK_AREA);
	}

}

void PacketTotalNumWrite(uint32_t packetTotalNum, uint32_t addr)
{
	for(uint32_t i = 0;i < TOTAL_NUM_LENGTH; i++)
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
    for(uint32_t i = 0; i<TOTAL_NUM_LENGTH; i++)
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
	PacketTotalNumWrite(totalNum, numAddr); // appУ��Ϳ���ȡbuffer����backup��buffer���������룬backup����������
	All_CheckSum_Write(chkSum, checkAddr);
}
uint8_t CheckSumCheck(int area)
{
	uint32_t packetTatolSize = 0;
	uint16_t calCheckSum = 0;

	getCheckPara(area);
	packetTatolSize = PACKET_SIZE * PacketTotalNumRead(numAddr);
	
	if(packetTatolSize == 0 || packetTatolSize > MAX_PACK_NUM) {
		return 0;
	}
	for(uint32_t i = 0; i < packetTatolSize; i++) {
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
	CommuSendCMD(result_cmd,CmmuSendLength,CmdSendData); // ��Ӧ��λ��
}

void BufferExchange()
{
	if(BufferFlag == 1) {
		BufferFlag = 0;
		if(IAP_Remap() == 1) {
			CheckSumWrite(PacketTotalNumRead(BUFFER_TOTAL_NUM_ADRESS), All_CheckSum_Read(BUFFER_CHECKSUM_ADRESS), APROM_BUFF_AREA);
			if(CheckSumCheck(APROM_AREA) == 1)
			{
				ACK = ERR_NO; //��Ӧ�˳���Bootloader
				ResetFlag = 1;
			} else {
				ACK = ERR_ALL_CHECK;
			}
		} else {
			ACK = ERR_REMAP;
		}
		result_cmd = ENTER_APP;
	} else if(g_BkpFlag == 1) {
		g_BkpFlag = 0;
		if(IAP_BkpRemap() == 1) {
			CheckSumWrite(PacketTotalNumRead(BACKUP_TOTAL_NUM_ADRESS), All_CheckSum_Read(BACKUP_CHECKSUM_ADRESS), APROM_BACKUP_AREA);
			if(CheckSumCheck(APROM_AREA) == 1)
			{
				ACK = ERR_NO; //��Ӧ�˳���Bootloader
				ResetFlag = 1;
			} else {
				ACK = ERR_ALL_CHECK;
			}
		} else {
			ACK = ERR_REMAP;
		}
	}
}
void BootCheckReset()
{
    if(ResetFlag==1)
    {
        ResetFlag = 0;	
		BaseTimeSystemInit(BOOT_DISABLE);//�رն�ʱ��

		// #ifdef FLASH_BUFF_ENABLE
		// if(CurrState == 2)
		// {
		// 	IAP_FlagWrite(2);//���뻺����������־
		// 	//IAP_Remap();
		// }
		// if(CurrState == 0)
		// {
		// 	IAP_FlagWrite(2);//APP���������б�־
		// }
		// #else
        // IAP_FlagWrite(1);//APP���������б�־
		// #endif	
					
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
	static char downBkpCount = 0;
    switch(cmd)//��������ִ����Ӧ�Ķ���
    {
//        case READ_BOOT_CODE_INF: // ��ȡ�汾��
//        {
//			
//			// BeginAddr = APP_ADDR; // ��ַ�޸ĳɻ�������ַΪwriteflash��׼��
//			IAP_Erase_ALL(APROM_AREA);
//            ACK = ERR_NO;
//        }break;
        case READ_IC_INF: // ��ȡоƬ�ͺ�
        {
            for(i=0;i < IC_TYPE_LENTH;i++)
            {
                CmdSendData[i] = IC_INF_BUFF[i];                
            }
            CmmuSendLength = IC_TYPE_LENTH;
            ACK = ERR_NO;
        }break;
        case HEX_INFO:
        {
            ACK = ERR_NO;
        }break;
		case GET_BT_VERSION:
		{
			g_sendArray = (uint8_t*)(&btVersion);
			for(int i = 0; i < sizeof(BtVersion); i++){
				CmdSendData[i] = *(g_sendArray + i);
			}
			CmmuSendLength = sizeof(BtVersion);
			ACK = ERR_NO;
		}
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
		case GET_WRITABLE_AREA:
		{
			CmdSendData[0] = g_flashWritableFlag.value;
			CmmuSendLength = sizeof(WritableFlag);
			ACK = ERR_NO;
		}
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
        case DOWNLOAD_BUFFER:	//����APROM��������
        {
			IAP_Erase_ALL(APROM_BUFF_AREA);
			BeginAddr = APP_BUFF_ADDR; // ��ַ�޸ĳɻ�������ַΪwriteflash��׼��
            ACK = ERR_NO;
        }break;
		case DOWNLOAD_BACKUP:	//����APROM��������
        {
			downBkpCount++;
			if(downBkpCount < 3) {
				ACK = ERR_NO;
				break;
			} else {
				downBkpCount = 0;
			}
			if(BACKUP_ADDR < (88 * 1024)) { // ���ݵ�ַ����С��88KB������Ӱ�컺������app����
				if(BACKUP_SIZE > MAX_PACK_NUM) {
					ACK = ERR_OPERATE;
				} else {
					IAP_Erase_ALL(APROM_BACKUP_AREA);
					BeginAddr = BACKUP_ADDR; // ��ַ�޸ĳɻ�������ַΪwriteflash��׼��
					ACK = ERR_NO;
				}
			} else {
				ACK = ERR_OPERATE;
			}
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
		case REC_ALL_CHECKSUM: //����hex�ļ�У���
        {
			for(int i = 0; i < PACKET_ID_LENTH; i++) {
				CmdSendData[i] = CommuData[i + 7];
			}
			// CheckSum[0] = CommuData[7];
			// CheckSum[1] = CommuData[8];
			CheckSum = CommuData[7] + CommuData[8] * 0x100;
			// All_CheckSum_Write(CheckSum, APP_CHECKSUM_ADRESS);
			// PacketTotalNumWrite(g_packetTotalNum, APP_TOTAL_NUM_ADRESS);
			// if(AppCheckSumCheck() == 1)
			// {
			// 	ACK = ERR_NO; //��Ӧ�˳���Bootloader
			// } else {
			// 	ACK = ERR_ALL_CHECK;
			// }
			CheckSumWrite(g_packetTotalNum, CheckSum, APROM_BUFF_AREA);
			g_packetTotalNum = 0;
			if(CheckSumCheck(APROM_BUFF_AREA) == 1)
			{
				ACK = ERR_NO; //��Ӧ�˳���Bootloader
				BufferFlag = 1;
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
		case RESTORE_BACKUP:
		{
			if(IAP_ReadOneByte(BACKUP_ADDR,IAP_CHECK_AREA) == 0x0) { // �ж�BACKUP�����Ƿ�������
				ACK = ERR_AREA_BLANK;
				break;
			}
			if(CheckSumCheck(APROM_BACKUP_AREA) == 1) { // У��BACKUP����У���
				g_BkpFlag = 1;							// У�����ȷ�����Կ�ʼǨ��HEX����
			} else {
				ACK = ERR_ALL_CHECK;
			}
		}
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
	BufferExchange();
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
		if(CheckSumCheck(APROM_AREA) == 1) { // ���ʱ�䵽��У��App���ݣ���ȷ�����APP
			ResetFlag = 1;
		} else if(CheckSumCheck(APROM_BUFF_AREA) == 1) {
			// �����Ϊ����ʹAPP�𻵣���������APP���ƹ���
			BufferFlag = 1;
			ResetFlag = 0;
		// } else if(CheckSumCheck(APROM_BACKUP_AREA) == 1) {
		// 	g_BkpFlag = 1;
		// 	ResetFlag = 0;
		}
		BootWaitTime = 0;
	}
	BootCheckReset(); // ��ת�������������㼴����ת��app
}

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
void uart1_interrupt_receive(void)
{
    volatile uint8_t rx_data;
    volatile uint8_t err_type;
    uartId id = UART1;

    INTC_ClearPendingIRQ(SR1_IRQn);
    SCI0->SIR03 = (uint16_t)err_type;
    // if (err_type != 0U)
    // {
    //     uart1_callback_error(err_type);
    // }
    rx_data = SCI0->RXD1;

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

void Clock_Config(void)
{	
	CLK_Osc_Setting(OSC_PORT, OSC_PORT); /* MainOSC/SubOSC enable */
	CLK_MainOsc_Setting(OSC_PORT,OSC_OVER_10M);   //OSC_PORT  OSC_OSCILLATOR
	CLK_Fclk_Select(MAINCLK_FIH);//select FMX   MAINCLK_FIH   MAINCLK_FMX
//	while((CGC->CKC & CGC_CKC_MCS_Msk) == 0);
	SystemCoreClock = 8000000;  //12000000
}
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
	if(DEVICE == BMS_DEVICE) {
		Clock_Config();		//OK
	} else if (DEVICE == DEBUG_DEVICE) {
		system_tick_init();
	}
	
//	GPIO_Config();		//OK
	//Ext_INT_Config();	//?????
	//TimeTick_Config();	//OK
	// TIM_Config();		//OK		
	// USART_Config();		//OK
	//CAN_Config();		//CAN OK?
	// ADC_Config();		//OK
	//sEE_Config();		//OK
	uint32_t uart_statu = UART1_Init(SystemCoreClock, UartBaud);

}

uint8_t CheckAreaWritable(uint32_t addr)
{
	uint8_t ok = 0;
	uint8_t CheckFlashBuff[512] = {0};
	for(int i=0;i<512;i++)
	{
		CheckFlashBuff[i] = IAP_ReadOneByte(addr + i,IAP_CHECK_AREA);
	}
	IAP_Erase_512B(addr & 0xff00,IAP_CHECK_AREA);
	ok = IAP_WriteOneByte_Check(addr,(0x55),IAP_CHECK_AREA);

	if(ok == 1) {
		IAP_Erase_512B(addr & 0xff00,IAP_CHECK_AREA);
		for(int i=0;i<512;i++)
		{
			IAP_WriteOneByte_Check(addr + i, CheckFlashBuff[i], IAP_CHECK_AREA);
		}
	}
	return ok;
}
