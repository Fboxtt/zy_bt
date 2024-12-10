//************************************************************
//  Copyright (c) 
//	文件名称	: boot.c
//	模块功能	: boot主要功能文件
//  更正日期	: 2024/9/12
// 	版本		: V1.0
//************************************************************
#include "includes.h"

uint32_t CmmuReadNumber;	//通讯当前读取数据为一帧中的第几个数
uint8_t UartReceFlag;				//UART0接收完一帧标志位


commu_length_t CmmuLength;						//接收数据长度
commu_data_t CommuData[ReceiveLength1];	//通讯接收缓存

commu_data_t CmdSendData[SendLength1];	//发送数据
commu_length_t CmmuSendLength;		    //发送数据长度

commu_data_t CmdSendAll[SendLength1];	//发送缓存
commu_length_t CmdSendAllLenth;			//发送缓存长度


// 表示烧录状态宏定义
typedef enum {
	NO_DOWNLOADING = 0x0,
	DOWNLOADING_BUFF	= 0x55AA55AA,
	DOWNLOADING_BKP		= 0x0A555AAA,
	DOWNLOADED_BUFF		= 0x5A5A5555,
	DOWNLOADED_BKP		= 0x0A5AAAAA,
	RESTORE_BUFF		= 0x5AA56699, // 恢复缓冲区到APP区域
	RESTORE_BKP 		= 0x69695A5A,
}DOWNLOAD_STATUS;

DOWNLOAD_STATUS g_downLoadStatus = NO_DOWNLOADING;

#ifdef BMS_BT_DEVICE
// 预设版本号
const TVER btVersion __attribute((at(BOOT_VER_ADDR)))= {
	vMAIN,
	vREV,
	vFIX,
	vYEAR,
	vMONTH,
	vDAY,
	vHW,
	vFW,
};

TUartData g_tUartData;

#endif

uint8_t* g_sendArray;

uint8_t result_cmd;

uint32_t g_bootWaitTime = 0;						// 在boot中的已等待时间
uint32_t g_bootWaitTimeLimit = 0;					// 在boot中的等待时间上限

int g_flashStatusCount = 0; // 保证读写FLASH时不会卡死

uint8_t g_BkpFlag = 0;								//代表备份区的校验状态
uint8_t ResetFlag = 0;								//表示复位条件达成
uint8_t CurrState = 0;								//当前芯片的状态
uint32_t ReadFlashLength = 0;                       //读Flash的长度        
uint32_t ReadFlashAddr = 0;							//读Flash的起始地址

uint32_t g_packetTotalNum = 0;						//烧录文件数据包的数量

uint32_t CheckSum = 0;

const uint8_t Boot_Inf_Buff[IC_TYPE_LENTH] = IC_TYPE_128KB_NAME;//版本号存储
boot_addr_t BeginAddr = APP_ADDR;				    //起始地址存储
uint32_t NewBaud = UartBaud;						//存储新波特率的变量
extern commu_data_t CmdSendData[SendLength1];
uint32_t NextPacketNumber = 0;

const uint8_t IC_INF_BUFF[IC_TYPE_LENTH] = IC_TYPE_128KB_NAME; // 芯片型号存储


WritableFlag g_flashWritableFlag = {0};


//表示握手状态
typedef enum {
	ENTER_CMD = 0xA,
	BUFFER_CMD = 0xB,
	BACKUP_CMD = 0XC,
	BUFFER_FLAG = 0xAAAB,
	BACKUP_FLAG  = 0xACCC,
}SHAKE_FLAG;

// 记录握手顺序变量
static uint16_t g_shakehandFlag = 0x0; 

//记录握手顺序函数
void SetShakehandFlag(SHAKE_FLAG flag)
{
	g_shakehandFlag = g_shakehandFlag << 4;
	g_shakehandFlag |= flag;
}

// 无用函数
uint8_t temp = APROM_AREA;

/*
跳转相关函数
*/

__asm uint32_t get_pc(void) {
	mov r0, pc
	bx lr
}

// 重置向量表
void __set_VECTOR_ADDR(uint32_t addr)
{
	SCB->VTOR = addr;
}

// 复位
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
	BaseTimeSystemInit(BOOT_DISABLE);	//关闭定时器
	SCI0->ST0   = _0002_SCI_CH1_STOP_TRG_ON | _0001_SCI_CH0_STOP_TRG_ON;
	CGC->PER0 &= ~CGC_PER0_SCI0EN_Msk;
	INTC_DisableIRQ(SR0_IRQn);
	__set_VECTOR_ADDR(APP_VECTOR_ADDR); // 需要配置向量表，因为实测发现app发生中断依然会跳到bt的systick
	__set_MSP(*(__IO uint32_t*) APP_ADDR);
	((void (*)()) (*(volatile unsigned long *)(APP_ADDR+0x04)))();//to APP
    NVIC_SystemReset();					//如果无法进入APP则复位
}
#endif


/*
通讯模块
*/

#ifndef BMS_APP_DEVICE

void UartSendOneByte(uint8_t input_data)
{
    SCI0->TXD1 = input_data;
	//阻塞，等到标志位清零即可发送下个数据
	while (SCI0->SSR02 & (_0040_SCI_UNDER_EXECUTE | _0020_SCI_VALID_STORED))
    {
        ;
    }
}

#endif 

// 清除通讯数据，为下次通讯做准备
void ClearCommu()
{
    CommuData[0] = 0; //清除缓冲区数据头，准备下次串口数据到来
    CmmuReadNumber = 0; //重新计数，准备下次串口数据到来
    UartReceFlag = 0; //清除传输完成标志
	CmdSendAllLenth = 0;
}

//分析接收帧的数据
uint8_t AnalysisData(uint8_t* pBuff, uint32_t wholeLen,uint32_t* noPackNumLen, volatile uint8_t* pAck)
{
	volatile uint8_t cmd = NO_CMD;
    uint32_t calLen;
	uint32_t i;
	uint8_t check_sum = 0;
	calLen = pBuff[1] * 0x100 + pBuff[2];
	cmd = pBuff[4];

	*pAck = ERR_NO;
	//计算单板类型到数据位的校验和
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
	//校验成功,提取控制码
	if(check_sum != (pBuff[3 + calLen]))
	{
        *pAck = ERR_CHKSUM;
	}
	// if(cmd == PC_SET_WRITE_FLASH) {
	// 	*noPackNumLen = calLen - TYPE_TO_DATA_LENTH;//取长度
	// } else {
	*noPackNumLen = calLen - TYPE_TO_SHAKE_LENTH;//取长度
	// }

    return cmd;
}

void fillbackFunc(commu_data_t* pBuff, commu_data_t* Data,commu_cmd_t Command,commu_cmd_t dataLen, commu_data_t Ack)
{
	uint8_t i;
	uint8_t check_sum = 0;

	pBuff[0] = SEND_ADDRESS;	//发送帧头
	pBuff[1] = (dataLen + 5) >> 8;		 				 		//发送数据域长度高8位
	pBuff[2] = dataLen + 5;		 			 	//发送数据域长度低8位
	pBuff[3] = SEND_BMS_TYPE;					//发送单板类型码
	pBuff[4] = Command;					 	//发送控制码
	pBuff[5] = SEND_SHAKE_1;					//握手字1
	pBuff[6] = SEND_SHAKE_2;					//握手字1
	pBuff[7] = Ack;							//发送应答码
	check_sum = ((dataLen + 5) >> 8) + (dataLen + 5) + SEND_BMS_TYPE + Command + SEND_SHAKE_1 + SEND_SHAKE_2 + Ack;
	for(i=0;i<dataLen;i++)	  					 	//发送数据域
	{
		pBuff[8+i] = *(Data+i);
		check_sum+=	*(Data+i);
	}
	pBuff[8 + dataLen] = check_sum;						//发送校验位低8位
	// UartSendOneByte(CommunicationCommandEnd);		//发送帧尾  
}


/*
flash 操作相关函数
重写这些函数，超时跳出，解决操作不成功卡死问题
*/

uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area)//写单字节IAP操作
{
	int FLSTS_flagCount = 0;
    uint8_t *ptr;
    ptr = (uint8_t *) IAP_IapAddr;
    
    FMC->FLPROT = 0xF1;
    
    FMC->FLOPMD1 = 0xAA;
    FMC->FLOPMD2 = 0x55;  
    *ptr = Write_IAP_IapData;    
    // 超时跳出，避免卡死
    while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0 && FLSTS_flagCount < g_flashStatusCount) {
		FLSTS_flagCount++;
	};
    FMC->FLSTS |= FMC_FLSTS_OVF_Msk;

    FMC->FLPROT = 0x00;
	
    if(IAP_ReadOneByte(IAP_IapAddr,area) == Write_IAP_IapData)
    {
        return 1;	//写入准确
    }
    else
    {
        return 0;	//写入有误
    }
}

// 写单字节并校验
uint8_t IAP_WriteOneByte_Check(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area)//写单字节IAP操作
{
	int FLSTS_flagCount = 0;
    uint8_t *ptr;
    ptr = (uint8_t *) IAP_IapAddr;
    
    FMC->FLPROT = 0xF1;
    FMC->FLOPMD1 = 0xAA;
    FMC->FLOPMD2 = 0x55;  
    *ptr = Write_IAP_IapData;    
    // polling OVER Flag
	// 这个判断FLSTS值的循环一共有7条汇编指令
    while((FMC->FLSTS & FMC_FLSTS_OVF_Msk) == 0 && FLSTS_flagCount < g_flashStatusCount) {
		FLSTS_flagCount++;
	};
    FMC->FLSTS |= FMC_FLSTS_OVF_Msk;

    FMC->FLPROT = 0x00;
	if(FLSTS_flagCount >= g_flashStatusCount) {
		return 0;
	}
    if(IAP_ReadOneByte(IAP_IapAddr,area) == Write_IAP_IapData)
    {
        return 1;	//写入准确
    }
    else
    {
        return 0;	//写入有误
    }
}

uint8_t IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area)//擦除一个块（512B）
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
	if(FLSTS_flagCount >= g_flashStatusCount) {
		return 0;
	}
	return 1;
    
}

// 擦除部分flash数据，
void IAP_Erase_Some(uint32_t IAP_IapAddr, uint32_t lenth)// 擦除并记录部分数据，充分利用空间
{
	int FLSTS_flagCount = 0;
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
	else if(area==APROM_BUFF_AREA)
	{
		k = (APP_BUFF_SIZE/ONE_PAGE_SIZE);
		begin_addr = APP_BUFF_ADDR;
		area = APROM_AREA;
	}
	else if(area==DATA_AREA)
	{
		k = (DATA_SIZE/ONE_PAGE_SIZE);
		begin_addr = DATA_ADDR;
	}

    for(i=0;i<k;i++)
    {
		if(IAP_Erase_512B(i*ONE_PAGE_SIZE+begin_addr,area) == 0) {
			return 0;
		}
    }
	return 1;
}



uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area)	//写多字节IAP操作
{
	uint32_t i;
	uint8_t Write_IAP_IapData;
	for(i=0;i<len;i++)
	{
		Write_IAP_IapData = buff[i];
        if(IAP_WriteOneByte(IAP_IapAddr+i,Write_IAP_IapData,area)==0)//判断写入是否正确
		{
			return 0;
		}			
	}
	return 1;
}

uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area)	//读单字节IAP操作
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

/*
	HEX文件读写相关函数
*/
// 把校验和记录到flash中
void All_CheckSum_Write(uint32_t checkSum, uint32_t addr)
{
    unsigned char i;
	for(i=0;i<CHECKSUM_LENGTH;i++)
	{
		IAP_WriteOneByte(addr+i,(checkSum >> (8 * i)),IAP_CHECK_AREA);
	}

}

// 把一些标志位写入到固定区域
void uint32ValWrite(uint32_t packetTotalNum, uint32_t addr)
{
	int i = 0;
	for(i = 0;i < TOTAL_NUM_LENGTH; i++)
	{
		IAP_WriteOneByte(addr + i, (packetTotalNum >> (8 * i)), IAP_CHECK_AREA);
	}
}

// 读取校验和出来
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

// 读取包长度出来
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

// 恢复数据到APP中
uint8_t IAP_Remap()//将缓存区的代码装载如运行区
{
	uint16_t i;
	IAP_Erase_ALL(APROM_AREA);//擦除APP运行区代码
	for(i=0;i<APP_BUFF_SIZE;i++)
	{
		if(IAP_WriteOneByte(APP_ADDR+i,IAP_ReadOneByte(APP_BUFF_ADDR+i,APROM_AREA),APROM_AREA) == 0) {
			return 0;
		}
	}
	return 1;
}

// 恢复备份区数据到APP中
uint8_t IAP_BkpRemap()//将缓存区的代码装载如运行区
{
	uint16_t i;
	IAP_Erase_ALL(APROM_AREA);//擦除APP运行区代码
	for(i=0;i<APP_BUFF_SIZE;i++)
	{
		if(IAP_WriteOneByte(APP_ADDR+i,IAP_ReadOneByte(BACKUP_ADDR+i,APROM_AREA),APROM_AREA) == 0) {
			return 0;
		}
	}
	return 1;
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

// 把烧录标志位相关数据写入到flash固定区域中
void CheckSumWrite(uint32_t totalNum, uint32_t chkSum, int area)
{
	getCheckPara(area);
	IAP_Erase_Some(numAddr,ALL_FLAG_LENTH);
	uint32ValWrite(totalNum, numAddr); // app校验和靠读取buffer或者backup，buffer靠外面输入，backup靠外面输入
	All_CheckSum_Write(chkSum, checkAddr);
}

// 计算flash内存储的校验和是否正确
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

// 开机时向主机发送命令
void ReplyEnterBoot(void)
{
	CmmuSendLength = 0;
//	CommuSendCMD(result_cmd,CmmuSendLength,CmdSendData); // 回应上位机
}

// 恢复APP
void AppRestore()
{
	#ifdef BMS_BT_DEVICE
	if(ReadInt(BUFFER_RESTORE_ADDRESS) == RESTORE_BUFF) {
		if(IAP_Remap() == 1) {
			CheckSumWrite(PacketTotalNumRead(BUFFER_TOTAL_NUM_ADRESS), All_CheckSum_Read(BUFFER_CHECKSUM_ADRESS), APROM_AREA);
			if(CheckSumCheck(APROM_AREA) == 1)
			{
				IAP_Erase_Some(BUFFER_RESTORE_ADDRESS, 4);
				// *Ack =  ERR_NO; //回应退出了Bootloader
			} else {
				// *Ack =  ERR_ALL_CHECK;
			}
		} else {
			// *Ack =  ERR_REMAP;
		}
		result_cmd = BMS_SHAKE_ENTER_APP;
	} else if(ReadInt(BACKUP_RESTORE_ADDRESS) == RESTORE_BKP) {
		if(IAP_BkpRemap() == 1) {
			// 从备份区中读取校验和数据，并写入到APP区域中
			CheckSumWrite(PacketTotalNumRead(BACKUP_TOTAL_NUM_ADRESS), All_CheckSum_Read(BACKUP_CHECKSUM_ADRESS), APROM_AREA);
			if(CheckSumCheck(APROM_AREA) == 1)
			{
				IAP_Erase_Some(BACKUP_RESTORE_ADDRESS, 4); // 成功恢复数据后才会清楚标志位，但是如果清楚不成功可能造成反复进入，所以需要APP中不复位
				// *Ack =  ERR_NO; //回应退出了Bootloader
			} else {
				// *Ack =  ERR_ALL_CHECK;
			}
		} else {
			// *Ack =  ERR_REMAP;
		}
	} else if(g_bootWaitTime > g_bootWaitTimeLimit) {
		if(CheckSumCheck(APROM_AREA) == 1) { // 如果时间到，校验App数据，正确则进入APP
			IAPEnterApp();
		} else if(CheckSumCheck(APROM_BUFF_AREA) == 1) {
			// 如果因为意外使APP损坏，将缓冲区APP复制过来
			IAP_Erase_Some(BUFFER_RESTORE_ADDRESS, 4);
			uint32ValWrite(RESTORE_BUFF, BUFFER_RESTORE_ADDRESS);
		} else if(CheckSumCheck(APROM_BACKUP_AREA) == 1) {
			IAP_Erase_Some(BACKUP_RESTORE_ADDRESS, 4);
			uint32ValWrite(RESTORE_BKP, BACKUP_RESTORE_ADDRESS);
		}
		g_bootWaitTime = 0;
	}
	#endif
}

// 重启
void BootCheckReset()
{
    if(ResetFlag==1)
    {
        ResetFlag = 0;	
        IAP_Reset();//复位进入APP
    }
}

// 获取版本号函数
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
	// 烧录区需要判断检验和
	// 如果校验和不正确，则不返回版本号，经测试后功能可用。
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



// 命令执行函数
boot_cmd_t BootCmdRun(uint8_t *rBuff, uint32_t dataLen, boot_cmd_t cmd, uint8_t *Ack)
{
    // boot_cmd_t cmd_buff = BOOT_BOOL_FALSE;//命令执行结果缓存
	TVER* hexVer = 0x0;
	int i = 0;
    CmmuSendLength = 0;	
	*Ack = ERR_NO;
	
	switch(cmd)
	{
		case PC_SHAKE_ENTER_BOOTMODE:
		case PC_SET_DOWNLOAD_BUFFER:
		case PC_SET_DOWNLOAD_BACKUP:
		{
			// 如果下载HEX中出现握手指令，则需要重新握手
			if(g_shakehandFlag == BUFFER_FLAG || g_shakehandFlag == BACKUP_FLAG) {
				g_shakehandFlag = 0;
				g_downLoadStatus =  NO_DOWNLOADING;
			}
		}break;
	}
    switch(cmd)//根据命令执行相应的动作
    {
		case PC_GET_VER:
		{
			hexVer = (TVER*)(APP_VER_ADDR); //使用TVER结构体而不是TVER，节省空间发送
			if(g_flashWritableFlag.bit.appArea == 1) {
				if(CheckSumCheck(APROM_AREA) == 1) {
					memcpy(&CmdSendData[0], hexVer, SIMPLE_VER_LENGTH);
					CmmuSendLength = SIMPLE_VER_LENGTH;
					*Ack = ERR_NO;
				} else {
					*Ack = ERR_ALL_CHECK;
				}
			} else {
				*Ack = ERR_AREA_NOT_WRITABLE;
			}
		}
		break;
		case PC_GET_INF:
		{
			// BT版本号获取
			volatile uint32_t pcValue = get_pc();
			GetVer(BOOT_VER_ADDR,					sizeof(TVER));
			GetVer(APP_VER_ADDR,					SIMPLE_VER_LENGTH);
			GetVer(APP_BUFF_VER_ADDR, 				SIMPLE_VER_LENGTH);
			GetVer(BACKUP_VER_ADDR, 				SIMPLE_VER_LENGTH);
			GetVer((uint32_t)IC_INF_BUFF, 			IC_TYPE_LENTH);
			GetVer((uint32_t)(&g_flashWritableFlag),sizeof(g_flashWritableFlag));
			GetVer((uint32_t)&pcValue, 				sizeof(pcValue));
			*Ack = ERR_NO;
		}
		break;
        case PC_SHAKE_ENTER_BOOTMODE: // 握手三次即可开始烧录
        {
			SetShakehandFlag(ENTER_CMD);

            *Ack = ERR_NO;
        }break;
        case PC_SET_DOWNLOAD_BUFFER:	//擦除APROM所有内容
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
			BeginAddr = APP_BUFF_ADDR; // 地址修改成缓冲区地址为writeflash做准备
			g_downLoadStatus = DOWNLOADING_BUFF;
			NextPacketNumber = 1;
			*Ack = ERR_NO_SHAKE_SUCCESS;
        }break;
		case PC_SET_DOWNLOAD_BACKUP:	//擦除APROM所有内容
        {
			SetShakehandFlag(BACKUP_CMD);
			if(g_shakehandFlag != BACKUP_FLAG) {
				*Ack = ERR_NO;
				break;
			}
			if(BACKUP_ADDR < (88 * 1024) || BACKUP_SIZE > MAX_PACK_NUM) { // 备份地址不能小于88KB，不能影响缓冲区和app区域
				*Ack = ERR_OPERATE;
				break;
			}
			if(IAP_Erase_ALL(APROM_BACKUP_AREA) == 0) {
				*Ack = ERR_ERASE;
				break;
			}
			BeginAddr = BACKUP_ADDR; // 地址修改成缓冲区地址为writeflash做准备
			g_downLoadStatus = DOWNLOADING_BKP;
			NextPacketNumber = 1;
			*Ack = ERR_NO_SHAKE_SUCCESS;
        }break;
		case PC_SET_WRITE_FLASH:// 写入app，成功后进入app
		{
			if(g_shakehandFlag != BUFFER_FLAG && g_shakehandFlag != BACKUP_FLAG) {
				*Ack = ERR_SHAKEHAND;
				break;
			}
			if(dataLen != PACKET_SIZE + RECEIVE_PACKET_LENTH) {
				*Ack = ERR_CMD_LEN;
				break;
			}
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
		case PC_SET_ALL_CHECKSUM: //接受hex文件校验和
        {
			for(i = 0; i < PACKET_ID_LENTH; i++) {
				CmdSendData[i] = rBuff[i];
			}
			CheckSum = rBuff[0] + rBuff[1] * 0x100;

			if(g_downLoadStatus == DOWNLOADING_BUFF) {
				CheckSumWrite(g_packetTotalNum, CheckSum, APROM_BUFF_AREA);
				g_packetTotalNum = 0;
				IAP_Erase_Some(BUFFER_RESTORE_ADDRESS, sizeof(uint32_t));
				if(CheckSumCheck(APROM_BUFF_AREA) == 1)
				{
					*Ack = ERR_NO; //回应退出了Bootloader
					uint32ValWrite(RESTORE_BUFF, BUFFER_RESTORE_ADDRESS); // 设置恢复缓冲区标志位,等待跳入bt中
					g_downLoadStatus = DOWNLOADED_BUFF;	// 修改下载状态
					g_shakehandFlag = 0x0;				// 清除握手成功标志位
				} else {
					uint32ValWrite(0xffffffff, BUFFER_RESTORE_ADDRESS); // 设置恢复缓冲区标志位,等待跳入bt中
					*Ack = ERR_ALL_CHECK;
				}
			} else if(g_downLoadStatus == DOWNLOADING_BKP){
				CheckSumWrite(g_packetTotalNum, CheckSum, APROM_BACKUP_AREA);
				g_packetTotalNum = 0;
				if(CheckSumCheck(APROM_BACKUP_AREA) == 1)
				{
					*Ack = ERR_NO; //回应退出了Bootloader
					g_downLoadStatus = DOWNLOADED_BUFF;
					g_shakehandFlag = 0x0;
				} else {
					*Ack = ERR_ALL_CHECK;
				}
			} else if(g_downLoadStatus == DOWNLOADED_BUFF || g_downLoadStatus == DOWNLOADED_BKP) {
				*Ack = ERR_DOWNLOAD_DONE;
			}

        }break;        
       case BMS_SHAKE_ENTER_APP: //运行用户代码
       {
			*Ack = ERR_NO;
		   IAP_Reset();
//           	g_restoreBufferFlag = RESTORE_BUFF;
       }break;        
        case NO_CMD://无操作
        {
            *Ack = ERR_CMD_ID;
        }break;
        case PC_GET_READ_FLASH: // 读取flash，暂未使用此功能
        {            
            ReadFlashAddr = (((uint32_t)rBuff[0])<<24)+(((uint32_t)rBuff[1])<<16)+(((uint32_t)rBuff[2])<<8)+((uint32_t)rBuff[3]);
			ReadFlashLength = (rBuff[4]<<24)+(rBuff[5]<<16)+(rBuff[6]<<8)+rBuff[7];            
			IAP_ReadMultiByte(ReadFlashAddr,CmdSendData,ReadFlashLength,temp);								
            CmmuSendLength = ReadFlashLength;
        }break;
		case PC_SET_RESTORE_BACKUP:
		// 恢复备份区流程 1下载 2强制恢复命令 3跳转到bt 4恢复 5跳转到app
		{
			if(IAP_ReadOneByte(BACKUP_ADDR,IAP_CHECK_AREA) == 0xffffffff) { // 判断BACKUP区域是否有数据
				*Ack = ERR_AREA_BLANK;
				break;
			}
			IAP_Erase_Some(BACKUP_RESTORE_ADDRESS, sizeof(uint32_t));
			if(CheckSumCheck(APROM_BACKUP_AREA) == 1) { // 校验BACKUP区域校验和
				uint32ValWrite(RESTORE_BKP, BACKUP_RESTORE_ADDRESS); // 设置标志位，进入bt后开始恢复backup区
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
		g_bootWaitTime = 0;
		g_bootWaitTimeLimit = YES_CMD_BOOT_WAIT_LIMIT;
#endif
        return (cmd | 0x80);
    } else {
        return cmd;
    }
}

/*boot_core.c*/

//main

// 重置Bt中等待时间
void BootWaitTimeInit(void)
{
	g_bootWaitTimeLimit = NO_CMD_BOOT_WAIT_LIMIT; // 进入APP等待开始计时
	g_bootWaitTime = 0;
}



// 检查flash区域是否可写
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

// 发生错误时清除烧录，为重新烧录做准备
void DownloadStop(void)
{
	if(g_downLoadStatus == DOWNLOADING_BUFF) {
		IAP_Erase_ALL(APROM_BUFF_AREA); // 清除接受数据缓冲区
		CheckSumWrite(0xffffffff, 0xffffffff, APROM_BUFF_AREA); // 清除校验和包数量
		uint32ValWrite(0x0, BUFFER_RESTORE_ADDRESS); // 清除恢复标志位

	} else if(g_downLoadStatus == DOWNLOADING_BKP) {
		IAP_Erase_ALL(APROM_BACKUP_AREA);
		CheckSumWrite(0xffffffff, 0xffffffff, APROM_BACKUP_AREA);
		uint32ValWrite(0x0, BACKUP_RESTORE_ADDRESS);
	}
	// 清除握手标志位, 如果没有g_downLoadStatus说明正在握手，也需要清除握手标志位
	g_shakehandFlag = 0x0;

}

uint32_t g_errTime = 0;

// 烧录程序，包含命令校验，命令执行，命令恢复功能
void DownloadProcess(void *p,UCHAR ucComPort)
{
	uint8_t  *rBuff, cmd, Ack;
	uint32_t  wholeDataLen, unitDataLen;
	rBuff 		= 	((TUartData *)(p))->pbuf;
	wholeDataLen	=	((TUartData *)(p))->wLen;

	cmd = AnalysisData(rBuff, wholeDataLen, &unitDataLen,&Ack);  // 分析从中断函数总获取的数据包， 返回cmd

	if (Ack == ERR_NO) {
		result_cmd = BootCmdRun(&rBuff[7], unitDataLen, cmd, &Ack);  // 根据cmd运行响应函数
	}
	if(Ack != ERR_NO && Ack != ERR_NO_SHAKE_SUCCESS) {
		if(++g_errTime > 3) {
			g_errTime = 0;
			DownloadStop();
		} 
	} else {
		g_errTime = 0;
	}
#ifdef BMS_APP_DEVICE
	fillbackFunc(SysSendUart[g_byRecComChn].pSendBuff	, CmdSendData, result_cmd, CmmuSendLength, Ack);
	SysSendUart[g_byRecComChn].EndPos += 9 + CmmuSendLength;
#else
	fillbackFunc(CmdSendAll								, CmdSendData, result_cmd, CmmuSendLength, Ack);
	CmdSendAllLenth 				  += 9 + CmmuSendLength;
	CmdSendFunc(CmdSendAll, CmdSendAllLenth);
	ClearCommu();
#endif

	if(ReadInt(BUFFER_RESTORE_ADDRESS) == RESTORE_BUFF || ReadInt(BACKUP_RESTORE_ADDRESS) == RESTORE_BKP) {	// 设置恢复缓冲区标志位,等待跳入bt中)
	// 下面这个if保证在bt中如果无法清除BUFFER_RESTORE_ADDRESS标志位，不会进入死循环
		if(g_downLoadStatus == DOWNLOADED_BUFF || g_downLoadStatus == DOWNLOADED_BKP) {
#ifdef BMS_APP_DEVICE
			SetDelayTask(IAP_Reset, NULL, 1000);
#else
			ResetFlag = 1;
#endif
		}
	}
}

/* boot初始化函数，会判断那些区域可写 */
void BootInit()
{
	// UartInit(UartBaud);
	g_flashStatusCount = 24 * SystemCoreClock / ONE_DISASSEMBLE_COUNT / 1000000 * 2;
	if(CheckAreaWritable(APP_ADDR + APP_SIZE - 512) == 1) { // 确认区域APP是否可写
		g_flashWritableFlag.bit.appArea = 1;
	}
	if(CheckAreaWritable(APP_BUFF_ADDR + APP_BUFF_SIZE - 512) == 1) { // 确认区域BUFF是否可写
		g_flashWritableFlag.bit.bufferArea = 1;
	}
	if(CheckAreaWritable(BACKUP_ADDR + BACKUP_SIZE - 512) == 1) { // 确认区域BACKUP是否可写
		g_flashWritableFlag.bit.backupArea = 1;
	}
}
// BootLoader使用的主程序
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
	
	BootCheckReset(); // 跳转函数，条件满足即可跳转入app
}

#endif
