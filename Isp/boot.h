#ifndef USE_BOOT
#define BOOT

#include "BAT32G137.h"
#include "userdefine.h"
#include "gpio.h"

#define DEBUG_DEVICE 1
#define BMS_BT_DEVICE 2
// #define BMS_APP_DEVICE 3
#define DEVICE BMS_APP_DEVICE



#ifdef BMS_APP_DEVICE

#include "sci.h"

#elif BMS_BT_DEVICE

#include "cg_sci.h"
#include "cg_macrodriver.h"
#include "Typedefs.h"

// <<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<BootLoader中需要修改的参数

#define vMAIN		1 //主版本号
#define vMINOR		2 //次版本号
#define vFIX		0 //修复版本号

#define vYEAR		2024 //版本生成年
#define vMONTH		12   //版本生成月
#define vDAY		9    //版本生成日

#define vHW			"T12100-V1.1-1OZ"  //BMS24200-H 带加热器
#define vFW			"V1"				//功能版本号

typedef struct
{
   BYTE  *pbuf; 
   WORD  wLen;  
}TUartData;	 

void CmdSendFunc(uint8_t *sBuff, uint32_t lenth); // BOOT专用的串口发送函数

#endif

#define IC_TYPE_LENTH					15
#define IC_TYPE_128KB_NAME				"BAT32G137GH48"
#define IC_TYPE_256KB_NAME				"BAT32G139GH48"

#define UNIQUE_NUM_LENTH				4

#define SIMPLE_VER_LENGTH 12


/*************************通讯协议相关宏定义*******************************/
//帧格式：帧头+控制码+数据域长度(2Byte)+数据域+校验位(1Byte)+帧尾
/**************************************************************************/

#define UartBaud				19200		    	 //初始默认波特率
#define	Fsoc					48000000	    	//主频选择
extern uint8_t UartReceFlag;		  			//UART0接收完一帧标志位
extern uint32_t CmmuReadNumber;
void UartInit(uint32_t baud);
void UartSendOneByte(uint8_t input_data);

typedef enum {
    UART0,
    UART1,
    UART2,
}uartId;


#define CommunicationCommandHeader   0X68		//命令帧头
#define CommunicationCommandEnd		 0x16		//命令帧尾
#define SEND_PACKET_LENTH           2
#define SendLength1                (128+SEND_PACKET_LENTH+8)

#define PACKET_ID_LENTH            2
#define RECEIVE_PACKET_LENTH        (PACKET_ID_LENTH+PACKET_ID_LENTH)
#define DATA_OFFSET					(RECEIVE_PACKET_LENTH)
#define PACKET_SIZE                 372
#define MAX_PACK_NUM				(80 * 1024)
#define ReceiveLength1              (PACKET_SIZE+RECEIVE_PACKET_LENTH+8)  //帧数据 + 包号 + 总包号 + 其他通讯内容
#define TYPE_TO_SHAKE_LENTH         4
#define TYPE_TO_DATA_LENTH          (TYPE_TO_SHAKE_LENTH + RECEIVE_PACKET_LENTH)
/* 依据芯片特性设计合适的数据类型 */
#define commu_bool_t uint8_t                //bool型数据类型
#define commu_data_t uint8_t           //数据对应的数据类型
#define commu_addr_t uint16_t           //地址对应的数据类型
#define commu_length_t uint16_t         //数据长度对应的数据类型
#define commu_cmd_t  uint8_t                //命令的数据类型

extern commu_length_t CmmuLength;		                //接收数据长度
extern commu_data_t CommuData[ReceiveLength1];	//通讯接收缓存
extern commu_data_t CmdSendData[SendLength1];  //发送缓存
extern commu_length_t CmmuSendLength;		            //接收数据长度

void UartReceData(uartId id);
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t dataLen,commu_data_t* Data, commu_data_t Ack);


#define TIME_UNIT				10 						// 10ms
#define DELAY_RETURN_COUNT 		(10 / TIME_UNIT)		// 10ms
#define TICK_100MS_COUNT		(100 / TIME_UNIT)		// 100ms
#define NO_CMD_BOOT_WAIT_LIMIT  (1000 / TIME_UNIT)		// 1000ms
#define YES_CMD_BOOT_WAIT_LIMIT (20000 / TIME_UNIT)		// 1000ms
#define VB_OFF_WAIT_TIME		(1000 / TIME_UNIT * 60 * 10) // 无操作2分钟触发一次boot关机

#define SHORT_WAIT 0 //开机后在boot总需要停留多久的标志位
#define LONG_WAIT 1  //在boot中等待20s的标志位
#define VB_WAIT 2    //无操作，需要关机标志位

extern uint32_t g_bootWaitTime;
extern uint32_t g_bootWaitTimeLimit;
extern uint32_t g_vbOffWaitTime;
extern uint8_t g_waitFlag;
extern uint32_t g_uartWaitTime;


/* boot core.h*/
/* 依据芯片特性设计合适的数据类型 */
#define boot_bool_t 	uint8_t         	//bool型数据类型
#define boot_data_t 	uint8_t    	        //数据对应的数据类型
#define boot_addr_t 	uint32_t    	    //地址对应的数据类型
#define boot_length_t 	uint16_t  		    //数据长度对应的数据类型
#define boot_cmd_t  	uint8_t         	//命令的数据类型
#define boot_flag_t 	uint8_t    	        //外部标志类型 

#define PC_GET_READ_FLASH_ENABLE						//使能后允许执行读FLASH操作

typedef enum {
	TYPE_128KB,
	TYPE_256KB,
} IC_TYPE_ENUM;


#define HANDLE0						0x55				
#define HANDLE1						0xAA	

//主站发送来的控制码类型 私有协议修改内容
#define NO_CMD						0x00		//表示无命令
#define PC_GET_VER_BOOT				0x16		// 获取APP的版本号

#define PC_GET_INF					0x71		// 获取BT版本号，APP版本号，BUFFER版本号，BACKUP版本号，芯片型号，芯片可写区域
#define PC_GET_BT_INF				0x72		// 获取BT详细版本号

#define PC_SET_DOWNLOAD_BUFFER		0x75		// 擦除所有APROM
#define PC_SHAKE_ENTER_BOOTMODE 	0x76		// 进入更新模式，即握手信号
#define PC_SET_WRITE_FLASH			0x77		// 更新程序命令
#define PC_SET_ALL_CHECKSUM        	0x78		// 发送校验和
#define PC_GET_READ_FLASH           0x79        // 读FLASH指定地址
#define BMS_RESET					0x7A        // 软复位
#define BMS_MCU_OPEN				0x7B		// 向主机表示开机了
#define PC_SET_DOWNLOAD_BACKUP		0x7C		// 下载备份
#define PC_SET_RESTORE_BACKUP		0x7D		// 将备份恢复到APP中

#define ERR_NO                  	0x00        // 无异常
#define ERR_CMD_LEN             	0x02        // 从机接收到的包长度和命令长度不对
#define ERR_CMD_ID             	 	0x04        // 没有命令
#define ERR_HANDLE					0x05		// 握手字错误
#define ERR_CHKSUM               	0x06        // 主机某个包校验和错误
#define ERR_OPERATE             	0x07        // 未能完成主机要求的操作
#define ERR_SHAKEHAND				0x20 		// 握手次数错误
#define ERR_PACKET_NUMBER       	0x21        // 主机包的序号跳错误
#define ERR_MEM_NOT_ENOUGH      	0x22        // 主机hex文件过大无法写入
#define ERR_ALL_CHECK				0x23		// 总包校验和错误
#define ERR_REMAP			    	0x24		// 重映射错误
#define ERR_AREA_BLANK				0x25		// 区域内数据为0
#define ERR_AREA_NOT_WRITABLE		0x26		// 区域不可写
#define ERR_DOWNLOAD_DONE			0x27		// 烧录已完成，请重新开始
#define ERR_ERASE					0x28		// 擦除错误
#define ERR_NO_SHAKE_SUCCESS		0x29		// 握手成功

#define BOOT_ENABLE        1
#define BOOT_DISABLE       0

/*communication_protocol.h*/

#define SEND_ADDRESS 				0x01
#define SEND_BMS_TYPE 				0x01
#define SEND_SHAKE_1 				0x55
#define SEND_SHAKE_2 				0xAA
extern volatile uint8_t ACK;

void BootCmdRun(uint8_t *rBuff, uint32_t dataLen, boot_cmd_t cmd, uint8_t *Ack);
void ClearCommu(void);
void fillbackFunc(commu_data_t* pBuff, commu_data_t* Data,commu_cmd_t Command,commu_cmd_t dataLen, commu_data_t Ack);
uint8_t AnalysisData(uint8_t *pBuff, uint32_t wholeLen,uint32_t* noPackNumLen, volatile uint8_t* pAck);
extern commu_data_t CmdSendAll[SendLength1];	//发送缓存



typedef union { // 确认区域是否可写的标志位
	uint8_t value;
	struct {
		uint8_t appArea:1;
		uint8_t bufferArea:1;
		uint8_t backupArea:1;
	}bit;
}WritableFlag;

#define ONE_DISASSEMBLE_COUNT 7 // 判断一次FLSTS的值需要7个汇编指令

#define APP_VER_OFFSET			0xD0

#define BOOT_ADDR				0x0000
#define BOOT_VTOR_ADDR			0x0000

#define BOOT_VER_ADDR			(BOOT_ADDR + APP_VER_OFFSET)

#define APP_ADDR                0X3000							// APP的起始位置
#define APP_SIZE                (58 * 1024)						// APP代码最大长度
#define APP_VER_ADDR			(APP_ADDR + APP_VER_OFFSET) 	// 存储app版本号的地址
#define APP_UNIQUE_ADDR			(APP_VER_ADDR + 0x100)


#define APP_BUFF_ADDR           (APP_ADDR + APP_SIZE)		        // APP缓存区的起始位置
#define APP_BUFF_SIZE           APP_SIZE						// APP缓存区最大长度
#define APP_BUFF_VER_ADDR		(APP_BUFF_ADDR + APP_VER_OFFSET) 	// 存储app版本号的地址
#define APP_VECTOR_ADDR         APP_ADDR

#define BACKUP_ADDR				(APP_ADDR + APP_SIZE * 2)
#define BACKUP_SIZE				APP_SIZE						// 60 * 1024 = 0xF000
#define BACKUP_VER_ADDR			(BACKUP_ADDR + APP_VER_OFFSET) 	// 存储backup版本号的地址

#define DATA_ADDR				0x500200						// 程序状态标志DATA Flash的起始位置
#define DATA_SIZE				0x500							// 程序状态标志DATA的大小

#define ONE_PAGE_SIZE           512                 			// 一页的长度

#define IAP_CHECK_AREA			APROM_AREA			// 标志所处区域
#define	IAP_CHECK_NUMBER		0XAA,0X55,0X55,0XAA // 表示APP代码区程序正常的数字码，最大14Byte

#define CHECKSUM_LENGTH         4
#define TOTAL_NUM_LENGTH        4
#define IAP_CHECK_LENGTH		4		  			//更新成功数字码长度,最大14Byte

#define ALL_FLAG_LENTH			(CHECKSUM_LENGTH + TOTAL_NUM_LENGTH + IAP_CHECK_LENGTH)

#define IAP_CHECK_ADRESS 		0x2E00     		    //更新成功数字码存储的起始地址
#define APP_TOTAL_NUM_ADRESS	(IAP_CHECK_ADRESS + 4)     		    //上位机发送校验和存储地址
#define APP_CHECKSUM_ADRESS		(APP_TOTAL_NUM_ADRESS + 4)     		//hex文件大小存储


#define BUFFER_CHECK_ADRESS 	(IAP_CHECK_ADRESS + 0x80)     		    //更新成功数字码存储的起始地址
#define BUFFER_TOTAL_NUM_ADRESS	(APP_TOTAL_NUM_ADRESS + 0x80)     		    //上位机发送校验和存储地址
#define BUFFER_CHECKSUM_ADRESS	(BUFFER_TOTAL_NUM_ADRESS + 4)            //缓冲区hex文件大小存储
#define BUFFER_RESTORE_ADDRESS	(BUFFER_CHECKSUM_ADRESS + 4)

#define BACKUP_CHECK_ADRESS 	(BUFFER_CHECK_ADRESS + 0x80)     		    //更新成功数字码存储的起始地址
#define BACKUP_TOTAL_NUM_ADRESS	(BUFFER_TOTAL_NUM_ADRESS + 0x80)     		    //上位机发送校验和存储地址
#define BACKUP_CHECKSUM_ADRESS	(BACKUP_TOTAL_NUM_ADRESS + 4)            //缓冲区hex文件大小存储
#define BACKUP_RESTORE_ADDRESS	(BACKUP_CHECKSUM_ADRESS + 4)

#define	BUFF_CHECK_NUMBER		0X55,0XAA,0XAA,0X55 //表示APP缓存区装载完备的数字码，最大14Byte

#define APP_TO_BOOT             0x55
#define BOOT_TO_APP             0xAA
#define UID_ENC_AREA			0x22				//UID密文存储区
#define LDROM_AREA	            0X96				//LDROM区
#define DATA_AREA               0xAA				//DATA区

#define BOOT_AREA				0x9A				//BOOT区
#define	APROM_AREA	            0x55				//APROM区
#define APROM_BUFF_AREA			0x69				//APP缓存区
#define APROM_BACKUP_AREA		0x5A				//备份区

uint8_t AppCheckSumCheck(void);
extern uint8_t IAP_Erase_Some(uint32_t IAP_IapAddr, uint32_t lenth);
extern void uint32ValWrite(uint32_t packetTotalNum, uint32_t addr);
uint8_t CheckSumCheck(int area);
void CheckSumWrite(uint32_t totalNum, uint32_t chkSum, int area);

extern WritableFlag g_flashWritableFlag;
#define ReadInt(x) *(uint32_t*)(x)



extern uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area);//写多字节IAP操作
extern void IAP_ReadMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint16_t len,uint8_t area); //读多字节IAP操作
extern uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area);  //读单字节IAP操作
extern void MCU_Reset(void);			 		                    //复位启动								
extern uint8_t IAP_Erase_ALL(uint8_t area);						    //将目标区域全擦
extern uint8_t IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area);   //擦除一个块（512B）
extern uint8_t IAP_Remap(void);//将缓存区的代码装载如运行区
extern uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area); //写单字节IAP操作
extern void BootProcess(void);
MD_STATUS UART1_Init(uint32_t freq, uint32_t baud);

extern void ADC_Config(void);
extern void GPIO_Config(void);
extern void Ext_INT_Config(void);
extern void Clock_Config(void);
extern void TimeTick_Config(void);
extern void RTC_Config(void);
extern void RTC_GetDateAndTime(void *p);
extern void WDT_feed(void);
extern void ADC_ClearChnValue(char ADCx);
extern void TimingDelay_Decrement(void);
extern void TIM_Config(void);
extern void PORT_Init(PORT_TypeDef PORTx,PIN_TypeDef PINx,PIN_ModeDef MODEx);
extern void system_tick_init(void);
uint8_t CheckAreaWritable(uint32_t addr);




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

extern WORD CalCRC (BYTE *ptr,int count);

extern boot_bool_t ResetFlag;
extern void BootCheckReset(void);		//检测是否有复位信号
extern void CheckAndEnterApp(void);
extern void AppRestore(void);
void BootInit(void);
void DownloadProcess(void *p,UCHAR ucComPort);
#endif
