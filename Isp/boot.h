#ifndef USE_BOOT
#define BOOT
/*communication_protocol.h*/
/*communication_protocol.h*/
/*communication_protocol.h*/

#include "serial_port_config.h"//串口通讯底层驱动文件
/*************************通讯协议相关宏定义*******************************/
//帧格式：帧头+控制码+数据域长度(2Byte)+数据域+校验位(1Byte)+帧尾
/**************************************************************************/
typedef enum {
    UART0,
    UART1,
    UART2,
}uartId;

#define CommunicationCommandHeader   0X68		//命令帧头
#define CommunicationCommandEnd		 0x16		//命令帧尾
#define SEND_PACKET_LENTH           2
#define SendLength1                (64+SEND_PACKET_LENTH+8)

#define PACKET_ID_LENTH            2
#define RECEIVE_PACKET_LENTH        (PACKET_ID_LENTH+PACKET_ID_LENTH)
#define DATA_OFFSET					(7+RECEIVE_PACKET_LENTH)
#define PACKET_SIZE                 512
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
extern commu_cmd_t CMDBuff;		                        //命令存储缓存
extern commu_data_t CommuData[ReceiveLength1];	//通讯接收缓存
extern commu_data_t CmdSendData[SendLength1];  //发送缓存
extern commu_length_t CmmuSendLength;		            //接收数据长度
void CommuSendCMD(commu_cmd_t Command,commu_cmd_t Data_len,commu_data_t* Data);    
uint8_t AnalysisData(void);
void ClearCommu(void);
void UartReceData(uartId id);

/*communication_protocol.h*/
/*communication_protocol.h*/
/*communication_protocol.h*/


#define SEND_ADDRESS 0x01
#define SEND_BMS_TYPE 0x01
#define SEND_SHAKE_1 0x55
#define SEND_SHAKE_2 0xAA
extern volatile uint8_t ACK;


#define BIT7 0x80
#define BIT6 0x40
#define BIT5 0x20
#define BIT4 0x10
#define BIT3 0x08
#define BIT2 0x04
#define BIT1 0x02
#define BIT0 0x01

/* boot core.h*/
/* boot core.h*/
/* boot core.h*/
/* 依据芯片特性设计合适的数据类型 */
#define boot_bool_t 	uint8_t         	//bool型数据类型
#define boot_data_t 	uint8_t    	        //数据对应的数据类型
#define boot_addr_t 	uint32_t    	    //地址对应的数据类型
#define boot_length_t 	uint16_t  		    //数据长度对应的数据类型
#define boot_cmd_t  	uint8_t         	//命令的数据类型
#define boot_flag_t 	uint8_t    	        //外部标志类型 

#define READ_FLASH_ENABLE						//使能后允许执行读FLASH操作
//#define ENCRYPT_ENABLE						    //使能通讯加密，使能后会对接收的更新数据进行解密操作	
//#define ENCRYPT_UID_ENABLE					    //使能UID加密功能，使能后会在跳转至APP前进行一次UID解密判断，若不一致就拒绝跳转到APP工作
//#define FLASH_BUFF_ENABLE						//Flash缓存功能开关，使能后会在FLASH区域开辟一个代码缓存区用于存储传输到来的新代码数据

#define IC_EDITION              "BAT32G13701"
#define IC_EDITION_LENTH              11

#define Edition                 "BAT32G13701"	//BOOT代码的版本号
#define EditionLength           11				//版本号长度
//私有协议新增内容
#define TYPE_FAIL 0xDE//主机命令类型错误
#define CHECK_FAIL 0xDF //校验错误

// //主站发送来的控制码类型
// #define ENTER_BOOTMODE 			0x01		//进入更新模式，即握手信号
// #define ENTER_APPMODE			0x0f		//跳转执行用户程序
// #define WRITE_FLASH				0x22		//更新程序命令
// #define SET_ADDRESS			    0x30		//设置MCU开始更新的地址
// #define	SET_BAUD				0x25		//设置波特率
// #define	READ_IC_INF				0x03		//读取芯片型号
// #define	READ_BOOT_CODE_INF		0x04		//读取Boot代码版本号
// #define EARSE_ALL				0x06		//擦除所有APROM
// #define READ_FLASH              0x23        //读FLASH指定地址
//主站发送来的控制码类型 私有协议修改内容
#define	READ_BOOT_CODE_INF		0x10		//读取Boot代码版本号
#define	READ_IC_INF				0x51		//读取芯片型号
#define HEX_INFO                0x52        //接收HEX文件信息
#define ENTER_BOOTMODE 			0x53		//进入更新模式，即握手信号
// #define ENTER_APPMODE			0x0f		//跳转执行用户程序

// #define SET_ADDRESS			    0x30		//设置MCU开始更新的地址
// #define	SET_BAUD				0x25		//设置波特率
#define EARSE_ALL				0x54		//擦除所有APROM
#define WRITE_FLASH				0x55		//更新程序命令
#define READ_FLASH              0x56        //读FLASH指定地址
#define REC_ALL_CHECKSUM        0x57		// 发送校验和

#define ERR_NO                  0x00        // 无异常
#define ERR_CMD_LEN             0x02        // 从机接收到的包长度和命令长度不对
#define ERR_CMD_ID              0x04        // 没有命令
#define ERR_CHECK               0x06        // 主机某个包校验和错误
#define ERR_OPERATE             0x07        // 未能完成主机要求的操作
#define ERR_PACKET_NUMBER       0x21        // 主机包的序号跳错误
#define ERR_MEM_NOT_ENOUGH      0x22        // 主机hex文件过大无法写入
#define ERR_ALL_CHECK			0x23		// 总包校验和错误

#define NO_CMD_BOOT_WAIT_LIMIT  4500
#define YES_CMD_BOOT_WAIT_LIMIT 5000

extern uint32_t BootWaitTime;
extern uint32_t BootWaitTimeLimit;

//从站回应控制码类型
#define DEAL_SUCCESS 			0X9F		//回应操作成功
#define DEAL_FAIL				0xDF		//回应操作失败
#define	RETURN_IC_INF			0xA3		//回应芯片型号
#define	RETURN_BOOT_CODE_INF	0XA4		//回应BOOT程序版本号
#define RETURN_FLASH            0xA5        //回应读出的FLASH信息
//错误类型
// #define	ERROR_CHECK_FAIL		0x01		//表示通讯校验失败
// #define	ERROR_BURN_FAIL			0x02		//表示烧写校验错误
// #define	ERROR_CMD_FAIL			0x04		//表示命令错误
//空闲
#define NO_CMD					0x00		//表示无命令

#define  RETURN_FLASH_APROM     0x00		//选择APROM
#define  RETURN_FLASH_DATA      0x01		//选择DATA Flash
#define  RETURN_FLASH_LDROM     0x02		//选择APROM
#define  RETURN_FLASH_XDATA     0x03		//选择XDATA
#define  RETURN_FLASH_SFR       0x04		//选择SFR
#define  RETURN_FLASH_RAM	    0x05		//选择RAM
#define  RETURN_FLASH_UID       0x06		//选择UID

#define HandShakes				3			 //握手次数设置

#define BOOT_BOOL_TRUE     1
#define BOOT_BOOL_FALSE    0
#define BOOT_ENABLE        1
#define BOOT_DISABLE       0


/*     此处为通讯相关接口，需要在通讯协议文件中定义此部分内容      */
// #define CommunicationLength1    (64+2+8)
extern boot_length_t CmmuLength;		             //接收数据长度
extern boot_cmd_t CMDBuff;		                     //命令存储缓存
extern boot_data_t CommuData[ReceiveLength1];	 //通讯接收缓存
extern boot_data_t CmdSendData[SendLength1];//发送缓存
extern boot_length_t CmmuSendLength;		         //接收数据长度
extern uint32_t NewBaud;							 //新波特率存储													
extern uint8_t CurrState;							 //存储当前芯片的状态,0:BOOT模式  1:APP运行态     2:代码缓存就绪态
extern boot_bool_t ResetFlag;
extern void BootCheckReset(void);		//检测是否有复位信号
extern void BufferExchange(void);
extern uint8_t CheckUID(void);
void BootInit(void);
boot_cmd_t BootCmdRun(boot_cmd_t cmd);


uint8_t AppCheckSumCheck(void);
/* boot core.h*/
/* boot core.h*/
/* boot core.h*/

/*cg_sci_user.c*/
/*cg_sci_user.c*/
/*cg_sci_user.c*/

/*cg_sci_user.c*/
/*cg_sci_user.c*/
/*cg_sci_user.c*/

// flash_operate.h

#define APP_ADDR                0X2000				//APP的起始位置
#define APP_BUFF_ADDR           0x11000		        //APP缓存区的起始位置
#define APP_SIZE                (APP_BUFF_ADDR - 0X2000)	//APP代码最大长度

// #define BUFFER_ADDR             0x11000
// #define BUFFER_SIZE                (0x20000 - BUFFER_ADDR)	//APP代码最大长度
//#define VECTOR_OFFSET           0x1c
#define APP_VECTOR_ADDR         APP_ADDR

#define LDROM_ADDR				0X0000				//LDROM的起始位置
#define LDROM_SIZE				0x2000				//LDROM的大小

#define DATA_ADDR				0x500200			//程序状态标志DATA Flash的起始位置
#define DATA_SIZE				0x500				//程序状态标志DATA的大小

#define ONE_PAGE_SIZE           512                 //一页的长度

#define IAP_CHECK_ADRESS 		0x1C00     		    //更新成功数字码存储的起始地址
#define IAP_CHECK_LENGTH		4		  			//更新成功数字码长度,最大14Byte
#define IAP_CHECK_AREA			APROM_AREA			//标志所处区域
#define	IAP_CHECK_NUMBER		0XAA,0X55,0X55,0XAA //表示APP代码区程序正常的数字码，最大14Byte

#define APP_CHECKSUM_ADRESS     0x1C04     		    //上位机发送校验和存储地址
#define CHECKSUM_LENGTH         4

#define APP_TOTAL_NUM_ADRESS    (APP_CHECKSUM_ADRESS + 4)     		//hex文件大小存储
#define TOTAL_NUM_LENGTH        4

#define BUFFER_CHECK_ADRESS 	0x1E00     		    //更新成功数字码存储的起始地址
#define BUFFER_CHECKSUM_ADRESS  0x1E04     		    //上位机发送校验和存储地址
#define BUFFER_TOTAL_NUM_ADRESS (BUFFER_CHECKSUM_ADRESS + 4)            //缓冲区hex文件大小存储


#define APP_BUFF_SIZE           (0x20000 - APP_BUFF_ADDR)	//APP缓存区最大长度
#define FLASH_BUFF_ENABLE
#define	BUFF_CHECK_NUMBER		0X55,0XAA,0XAA,0X55 //表示APP缓存区装载完备的数字码，最大14Byte

#define UID_ENC_ADRESS			0x1FE00		        //UID密文存储地址
#define UID_ENC_SIZE			16					//UID密文长度
#define UID_SIZE				(128/8)				//UID有效长度
#define UID_ENC_AREA_AREA		APROM_AREA			//UID密文所在的存储区域

#define APP_TO_BOOT             0x55
#define BOOT_TO_APP             0xAA
#define	APROM_AREA	            0x55				//APROM区
#define DATA_AREA               0xAA				//DATA区
#define LDROM_AREA	            0X96				//LDROM区
#define APROM_BUFF_AREA			0x69				//APP缓存区
#define UID_ENC_AREA			0x22				//UID密文存储区

extern uint8_t IAP_IapLength;	        //用于IAP操作数据长度缓存

extern uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area);//写多字节IAP操作
extern void IAP_ReadMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint16_t len,uint8_t area); //读多字节IAP操作
extern uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area);  //读单字节IAP操作
extern void IAP_Reset(void);			 		                    //复位启动								
extern void IAP_Erase_ALL(uint8_t area);						    //将目标区域全擦
extern void IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area);   //擦除一个块（512B）
extern void IAP_FlagWrite(uint8_t flag);
extern uint8_t IAP_CheckAPP(void);
extern void IAP_ReadEncUID(uint8_t* buff);
extern void IAP_Remap(void);//将缓存区的代码装载如运行区
extern uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area); //写单字节IAP操作

// flash_operate.h

extern volatile uint8_t * gp_uart0_tx_address;        /* uart0 transmit buffer address */
extern volatile uint16_t  g_uart0_tx_count;           /* uart0 transmit data number */
extern volatile uint8_t * gp_uart0_rx_address;        /* uart0 receive buffer address */
extern volatile uint16_t  g_uart0_rx_count;           /* uart0 receive data number */
extern volatile uint16_t  g_uart0_rx_length;          /* uart0 receive data length */

extern void BootWaitTimeInit(void);
extern void BootProcess(void);
#endif
