#ifndef USE_BOOT
#define BOOT
/*communication_protocol.h*/
/*communication_protocol.h*/
/*communication_protocol.h*/

#include "serial_port_config.h"//����ͨѶ�ײ������ļ�
#include "gpio.h"
/*************************ͨѶЭ����غ궨��*******************************/
//֡��ʽ��֡ͷ+������+�����򳤶�(2Byte)+������+У��λ(1Byte)+֡β
/**************************************************************************/
typedef enum {
    UART0,
    UART1,
    UART2,
}uartId;

#define DEBUG_DEVICE 1
#define BMS_DEVICE 2
#define DEVICE DEBUG_DEVICE
#define CommunicationCommandHeader   0X68		//����֡ͷ
#define CommunicationCommandEnd		 0x16		//����֡β
#define SEND_PACKET_LENTH           2
#define SendLength1                (64+SEND_PACKET_LENTH+8)

#define PACKET_ID_LENTH            2
#define RECEIVE_PACKET_LENTH        (PACKET_ID_LENTH+PACKET_ID_LENTH)
#define DATA_OFFSET					(7+RECEIVE_PACKET_LENTH)
#define PACKET_SIZE                 512
#define MAX_PACK_NUM				(80 * 1024)
#define ReceiveLength1              (PACKET_SIZE+RECEIVE_PACKET_LENTH+8)  //֡���� + ���� + �ܰ��� + ����ͨѶ����
#define TYPE_TO_SHAKE_LENTH         4
#define TYPE_TO_DATA_LENTH          (TYPE_TO_SHAKE_LENTH + RECEIVE_PACKET_LENTH)
/* ����оƬ������ƺ��ʵ��������� */
#define commu_bool_t uint8_t                //bool����������
#define commu_data_t uint8_t           //���ݶ�Ӧ����������
#define commu_addr_t uint16_t           //��ַ��Ӧ����������
#define commu_length_t uint16_t         //���ݳ��ȶ�Ӧ����������
#define commu_cmd_t  uint8_t                //�������������

extern commu_length_t CmmuLength;		                //�������ݳ���
extern commu_cmd_t CMDBuff;		                        //����洢����
extern commu_data_t CommuData[ReceiveLength1];	//ͨѶ���ջ���
extern commu_data_t CmdSendData[SendLength1];  //���ͻ���
extern commu_length_t CmmuSendLength;		            //�������ݳ���
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
/* ����оƬ������ƺ��ʵ��������� */
#define boot_bool_t 	uint8_t         	//bool����������
#define boot_data_t 	uint8_t    	        //���ݶ�Ӧ����������
#define boot_addr_t 	uint32_t    	    //��ַ��Ӧ����������
#define boot_length_t 	uint16_t  		    //���ݳ��ȶ�Ӧ����������
#define boot_cmd_t  	uint8_t         	//�������������
#define boot_flag_t 	uint8_t    	        //�ⲿ��־���� 

#define READ_FLASH_ENABLE						//ʹ�ܺ�����ִ�ж�FLASH����
//#define ENCRYPT_ENABLE						    //ʹ��ͨѶ���ܣ�ʹ�ܺ��Խ��յĸ������ݽ��н��ܲ���	
//#define ENCRYPT_UID_ENABLE					    //ʹ��UID���ܹ��ܣ�ʹ�ܺ������ת��APPǰ����һ��UID�����жϣ�����һ�¾;ܾ���ת��APP����
//#define FLASH_BUFF_ENABLE						//Flash���湦�ܿ��أ�ʹ�ܺ����FLASH���򿪱�һ�����뻺�������ڴ洢���䵽�����´�������

typedef enum {
	TYPE_128KB,
	TYPE_256KB,
} IC_TYPE_ENUM;

#define IC_TYPE_LENTH					13

#define IC_TYPE_128KB_NAME				"BAT32G137GH48"
#define IC_TYPE_256KB_NAME				"BAT32G139GH48"


//˽��Э����������
#define TYPE_FAIL 0xDE//�����������ʹ���
#define CHECK_FAIL 0xDF //У�����

// //��վ�������Ŀ���������
// #define ENTER_BOOTMODE 			0x01		//�������ģʽ���������ź�
// #define ENTER_APPMODE			0x0f		//��תִ���û�����
// #define WRITE_FLASH				0x22		//���³�������
// #define SET_ADDRESS			    0x30		//����MCU��ʼ���µĵ�ַ
// #define	SET_BAUD				0x25		//���ò�����
// #define	READ_IC_INF				0x03		//��ȡоƬ�ͺ�
// #define	READ_BOOT_CODE_INF		0x04		//��ȡBoot����汾��
// #define EARSE_ALL				0x06		//��������APROM
// #define READ_FLASH              0x23        //��FLASHָ����ַ
//��վ�������Ŀ��������� ˽��Э���޸�����
#define PC_GET_VER_APP			0x16		// ��ȡAPP�İ汾��
// #define PC_GET_VER_BACKUP		0X18		// ��ȡBACKUP�İ汾��
// #define	READ_IC_INF				0x51		// ��ȡоƬ�ͺ�
// // #define HEX_INFO                0x52        // ����HEX�ļ���Ϣ
// #define PC_GET_VER_BT			0x53        // ��ѯBT�İ汾
// #define GET_WRITABLE_AREA		0x54		// ��ѯоƬ��д����

#define PC_GET_INF				0x51		// ��ȡBT�汾�ţ�APP�汾�ţ�BUFFER�汾�ţ�BACKUP�汾�ţ�оƬ�ͺţ�оƬ��д����

#define DOWNLOAD_BUFFER			0x55		// ��������APROM
#define ENTER_BOOTMODE 			0x56		// �������ģʽ���������ź�
#define WRITE_FLASH				0x57		// ���³�������
#define REC_ALL_CHECKSUM        0x58		// ����У���
#define READ_FLASH              0x59        // ��FLASHָ����ַ
#define ENTER_APP               0x5A        // ����APP

#define DOWNLOAD_BACKUP			0x5C		// ���ر���
#define RESTORE_BACKUP			0x5D		// �����ݻָ���APP��

#define ERR_NO                  0x00        // ���쳣
#define ERR_CMD_LEN             0x02        // �ӻ����յ��İ����Ⱥ�����Ȳ���
#define ERR_CMD_ID              0x04        // û������
#define ERR_CHECK               0x06        // ����ĳ����У��ʹ���
#define ERR_OPERATE             0x07        // δ���������Ҫ��Ĳ���
#define ERR_PACKET_NUMBER       0x21        // �����������������
#define ERR_MEM_NOT_ENOUGH      0x22        // ����hex�ļ������޷�д��
#define ERR_ALL_CHECK			0x23		// �ܰ�У��ʹ���
#define ERR_REMAP			    0x24		// ��ӳ�����
#define ERR_AREA_BLANK			0x25		// ����������Ϊ0
#define ERR_AREA_NOT_WRITABLE	0x26		// ���򲻿�д
#define NO_CMD_BOOT_WAIT_LIMIT  4500
#define YES_CMD_BOOT_WAIT_LIMIT 5000

extern uint32_t BootWaitTime;
extern uint32_t BootWaitTimeLimit;

//��վ��Ӧ����������
#define DEAL_SUCCESS 			0X9F		//��Ӧ�����ɹ�
#define DEAL_FAIL				0xDF		//��Ӧ����ʧ��
#define	RETURN_IC_INF			0xA3		//��ӦоƬ�ͺ�
#define	RETURN_BOOT_CODE_INF	0XA4		//��ӦBOOT����汾��
#define RETURN_FLASH            0xA5        //��Ӧ������FLASH��Ϣ
//��������
// #define	ERROR_CHECK_FAIL		0x01		//��ʾͨѶУ��ʧ��
// #define	ERROR_BURN_FAIL			0x02		//��ʾ��дУ�����
// #define	ERROR_CMD_FAIL			0x04		//��ʾ�������
//����
#define NO_CMD					0x00		//��ʾ������

#define  RETURN_FLASH_APROM     0x00		//ѡ��APROM
#define  RETURN_FLASH_DATA      0x01		//ѡ��DATA Flash
#define  RETURN_FLASH_LDROM     0x02		//ѡ��APROM
#define  RETURN_FLASH_XDATA     0x03		//ѡ��XDATA
#define  RETURN_FLASH_SFR       0x04		//ѡ��SFR
#define  RETURN_FLASH_RAM	    0x05		//ѡ��RAM
#define  RETURN_FLASH_UID       0x06		//ѡ��UID

#define HandShakes				3			 //���ִ�������

#define BOOT_BOOL_TRUE     1
#define BOOT_BOOL_FALSE    0
#define BOOT_ENABLE        1
#define BOOT_DISABLE       0


/*     �˴�ΪͨѶ��ؽӿڣ���Ҫ��ͨѶЭ���ļ��ж���˲�������      */
// #define CommunicationLength1    (64+2+8)
extern boot_length_t CmmuLength;		             //�������ݳ���
extern boot_cmd_t CMDBuff;		                     //����洢����
extern boot_data_t CommuData[ReceiveLength1];	 //ͨѶ���ջ���
extern boot_data_t CmdSendData[SendLength1];//���ͻ���
extern boot_length_t CmmuSendLength;		         //�������ݳ���
extern uint32_t NewBaud;							 //�²����ʴ洢													
extern uint8_t CurrState;							 //�洢��ǰоƬ��״̬,0:BOOTģʽ  1:APP����̬     2:���뻺�����̬
extern boot_bool_t ResetFlag;
extern void BootCheckReset(void);		//����Ƿ��и�λ�ź�
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

#define APP_ADDR                0X2000							// APP����ʼλ��
#define APP_SIZE                (40 * 1024)						// APP������󳤶�
#define APP_VER_ADDR			(APP_ADDR + APP_SIZE - 1024) 	// �洢app�汾�ŵĵ�ַ

#define APP_BUFF_ADDR           (0x2000 + APP_SIZE)		        // APP����������ʼλ��
#define APP_BUFF_SIZE           APP_SIZE						// APP��������󳤶�
#define APP_BUFF_VER_ADDR		(APP_BUFF_ADDR + APP_BUFF_SIZE - 1024) 	// �洢app�汾�ŵĵ�ַ
#define APP_VECTOR_ADDR         APP_ADDR

#define BACKUP_ADDR				(0x2000 + APP_SIZE * 2)
#define BACKUP_SIZE				APP_SIZE						// 60 * 1024 = 0xF000
#define BACKUP_VER_ADDR			(BACKUP_ADDR + BACKUP_SIZE - 1024) 	// �洢backup�汾�ŵĵ�ַ

#define DATA_ADDR				0x500200						// ����״̬��־DATA Flash����ʼλ��
#define DATA_SIZE				0x500							// ����״̬��־DATA�Ĵ�С

#define ONE_PAGE_SIZE           512                 			// һҳ�ĳ���





#define IAP_CHECK_AREA			APROM_AREA			// ��־��������
#define	IAP_CHECK_NUMBER		0XAA,0X55,0X55,0XAA // ��ʾAPP���������������������룬���14Byte

#define CHECKSUM_LENGTH         4
#define TOTAL_NUM_LENGTH        4
#define IAP_CHECK_LENGTH		4		  			//���³ɹ������볤��,���14Byte

#define ALL_FLAG_LENTH			(CHECKSUM_LENGTH + TOTAL_NUM_LENGTH + IAP_CHECK_LENGTH)

#define IAP_CHECK_ADRESS 		0x1E00     		    //���³ɹ�������洢����ʼ��ַ
#define APP_TOTAL_NUM_ADRESS	0x1E04     		    //��λ������У��ʹ洢��ַ
#define APP_CHECKSUM_ADRESS		(APP_TOTAL_NUM_ADRESS + 4)     		//hex�ļ���С�洢

#define BUFFER_CHECK_ADRESS 	0x1E80     		    //���³ɹ�������洢����ʼ��ַ
#define BUFFER_TOTAL_NUM_ADRESS	0x1E84     		    //��λ������У��ʹ洢��ַ
#define BUFFER_CHECKSUM_ADRESS	(BUFFER_TOTAL_NUM_ADRESS + 4)            //������hex�ļ���С�洢

#define BACKUP_CHECK_ADRESS 	0x1F00     		    //���³ɹ�������洢����ʼ��ַ
#define BACKUP_TOTAL_NUM_ADRESS	0x1F04     		    //��λ������У��ʹ洢��ַ
#define BACKUP_CHECKSUM_ADRESS	(BACKUP_TOTAL_NUM_ADRESS + 4)            //������hex�ļ���С�洢

#define FLASH_BUFF_ENABLE
#define	BUFF_CHECK_NUMBER		0X55,0XAA,0XAA,0X55 //��ʾAPP������װ���걸�������룬���14Byte

#define UID_ENC_ADRESS			0x1FE00		        //UID���Ĵ洢��ַ
#define UID_ENC_SIZE			16					//UID���ĳ���
#define UID_SIZE				(128/8)				//UID��Ч����
#define UID_ENC_AREA_AREA		APROM_AREA			//UID�������ڵĴ洢����

#define APP_TO_BOOT             0x55
#define BOOT_TO_APP             0xAA
#define UID_ENC_AREA			0x22				//UID���Ĵ洢��
#define LDROM_AREA	            0X96				//LDROM��
#define DATA_AREA               0xAA				//DATA��

#define	APROM_AREA	            0x55				//APROM��
#define APROM_BUFF_AREA			0x69				//APP������
#define APROM_BACKUP_AREA		0x5A				//������

extern uint8_t IAP_IapLength;	        //����IAP�������ݳ��Ȼ���

extern uint8_t IAP_WriteMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint32_t len,uint8_t area);//д���ֽ�IAP����
extern void IAP_ReadMultiByte(uint32_t IAP_IapAddr,uint8_t * buff,uint16_t len,uint8_t area); //�����ֽ�IAP����
extern uint8_t IAP_ReadOneByte(uint32_t IAP_IapAddr,uint8_t area);  //�����ֽ�IAP����
extern void IAP_Reset(void);			 		                    //��λ����								
extern void IAP_Erase_ALL(uint8_t area);						    //��Ŀ������ȫ��
extern void IAP_Erase_512B(uint32_t IAP_IapAddr,uint8_t area);   //����һ���飨512B��
extern void IAP_FlagWrite(uint8_t flag);
extern uint8_t IAP_CheckAPP(void);
extern void IAP_ReadEncUID(uint8_t* buff);
extern uint8_t IAP_Remap(void);//���������Ĵ���װ����������
extern uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area); //д���ֽ�IAP����

// flash_operate.h

extern volatile uint8_t * gp_uart0_tx_address;        /* uart0 transmit buffer address */
extern volatile uint16_t  g_uart0_tx_count;           /* uart0 transmit data number */
extern volatile uint8_t * gp_uart0_rx_address;        /* uart0 receive buffer address */
extern volatile uint16_t  g_uart0_rx_count;           /* uart0 receive data number */
extern volatile uint16_t  g_uart0_rx_length;          /* uart0 receive data length */

extern void BootWaitTimeInit(void);
extern void BootProcess(void);
extern void ReplyEnterBoot(void);
MD_STATUS UART1_Init(uint32_t freq, uint32_t baud);













//typedef enum {
//	PORT0 = 0,
//	PORT1,
//	PORT2,
//	PORT3,
//	PORT4,
//	PORT5,
//	PORT6,
//	PORT7,
//	PORT8,
//	PORT9,
//	PORT10,
//	PORT11,
//	PORT12,
//	PORT13,
//	PORT14,
//	
//}PORT_TypeDef;

//typedef enum {
//	PIN0 = 0,
//	PIN1,
//	PIN2,
//	PIN3,
//	PIN4,
//	PIN5,
//	PIN6,
//	PIN7,
//	
//}PIN_TypeDef;

//typedef enum {
//	INPUT = 0,
//	PULLUP_INPUT,
//	TTL_INPUT,
//	ANALOG_INPUT,
//	OUTPUT,
//	OPENDRAIN_OUTPUT,
//	
//}PIN_ModeDef;


//ADC ����
#define	 	ADC_PACK_V		ADC_CHANNEL_6
#define     ADC_TEMP_T3     ADC_CHANNEL_0  
#define     ADC_TEMP_MOS    ADC_CHANNEL_2
#define     ADC_TEMP_T5     ADC_CHANNEL_1
#define     ADC_TEMP_HEAT   ADC_CHANNEL_7

//GPIO��������ڱ�������

typedef struct 
{
	PORT_TypeDef	emGPIOx;		//refer to PORT_TypeDef
	PIN_TypeDef 	emPin;			//refer to PIN_TypeDef
	PIN_ModeDef		emMode;			//refer to PIN_ModeDef
	uint8_t 		value;			//output TRUE: high, FALSE: low
}TGPIO;

extern TGPIO PIN_SW; 	
extern TGPIO PIN_HEATE_N;	
extern TGPIO PIN_ALERT;	

extern TGPIO PIN_VBCTL; 
extern TGPIO PIN_CDEN; 	
extern TGPIO PIN_CEN; 	
extern TGPIO PIN_GREEN; 
extern TGPIO PIN_RED; 	
//extern TGPIO PIN_485DE; 
extern TGPIO PIN_FUSE_EN; 	
extern TGPIO PIN_WAKE; 		
//extern TGPIO PIN_PACKADC_EN; 

extern TGPIO PIN_COM3V3_EN;	 
extern TGPIO PIN_COM5V_EN;	 
extern TGPIO PIN_REGOUT_EN;	 



extern void ADC_Config(void);
//extern uint16_t ADC_GetChnValue(adc_channel_t Chn);
extern void GPIO_Config(void);
extern void Ext_INT_Config(void);
extern void Clock_Config(void);
extern void TimeTick_Config(void);
extern void RTC_Config(void);
extern void RTC_GetDateAndTime(void *p);
//extern void RTC_SetDateAndTime(MDate *pDate);
extern void WDT_feed(void);
extern void ADC_ClearChnValue(char ADCx);

extern void TimingDelay_Decrement(void);


extern void TIM_Config(void);
//extern ULONG	FLASH_Write(ULONG ulDstAddr, ULONG ulSrcAddr, ULONG ulLen);

extern void PORT_Init(PORT_TypeDef PORTx,PIN_TypeDef PINx,PIN_ModeDef MODEx);

//extern void PORT_ClrBit(PORT_TypeDef PORTx,PIN_TypeDef PINx);
extern void HardDriveInit(void);
extern void system_tick_init(void);

uint8_t CheckSumCheck(int area);
void CheckSumWrite(uint32_t totalNum, uint32_t chkSum, int area);

uint8_t CheckAreaWritable(uint32_t addr);

#define ONE_DISASSEMBLE_COUNT 7 // �ж�һ��FLSTS��ֵ��Ҫ7�����ָ��

typedef struct tagVersion
{
	uint16_t	usMajorVer;				    	 
	uint16_t	usMinorVer;				    	 
	uint16_t	usRevision;						 
	uint16_t	usCompileYear;					 
	uint8_t		ucCompileMonth;			    	 
	uint8_t		ucCompileDay;					  
	int8_t    cHWversion[30];					 
	int8_t    cFuncVersion[40];				 
}TVER;

typedef union { // ȷ�������Ƿ��д�ı�־λ
	uint8_t value;
	struct {
		uint8_t appArea:1;
		uint8_t bufferArea:1;
		uint8_t backupArea:1;
	}bit;
}WritableFlag;

extern WritableFlag g_flashWritableFlag;

#endif
