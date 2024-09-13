#ifndef USE_BOOT
#define BOOT
/*communication_protocol.h*/
/*communication_protocol.h*/
/*communication_protocol.h*/
#include "serial_port_config.h"//����ͨѶ�ײ������ļ�
/*************************ͨѶЭ����غ궨��*******************************/
//֡��ʽ��֡ͷ+������+�����򳤶�(2Byte)+������+У��λ(1Byte)+֡β
/**************************************************************************/
typedef enum {
    UART0,
    UART1,
    UART2,
}uartId;

#define CommunicationCommandHeader   0X68		//����֡ͷ
#define CommunicationCommandEnd		 0x16		//����֡β
#define SEND_PACKET_LENTH           2
#define SendLength1                (64+SEND_PACKET_LENTH+8)

#define PACKET_ID_LENTH            2
#define RECEIVE_PACKET_LENTH        (PACKET_ID_LENTH+PACKET_ID_LENTH)
#define DATA_OFFSET					(7+RECEIVE_PACKET_LENTH)
#define PACKET_SIZE                 512
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

#define IC_EDITION              "BAT32G13701"
#define IC_EDITION_LENTH              11

#define Edition                 "BAT32G13701"	//BOOT����İ汾��
#define EditionLength           11				//�汾�ų���
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
#define	READ_BOOT_CODE_INF		0x10		//��ȡBoot����汾��
#define	READ_IC_INF				0x51		//��ȡоƬ�ͺ�
#define HEX_INFO                0x52        //����HEX�ļ���Ϣ
#define ENTER_BOOTMODE 			0x53		//�������ģʽ���������ź�
// #define ENTER_APPMODE			0x0f		//��תִ���û�����

// #define SET_ADDRESS			    0x30		//����MCU��ʼ���µĵ�ַ
// #define	SET_BAUD				0x25		//���ò�����
#define EARSE_ALL				0x54		//��������APROM
#define WRITE_FLASH				0x55		//���³�������
#define READ_FLASH              0x56        //��FLASHָ����ַ
#define REC_ALL_CHECKSUM        0x57		// ����У���

#define ERR_NO                  0x00        // ���쳣
#define ERR_CMD_LEN             0x02        // �ӻ����յ��İ����Ⱥ�����Ȳ���
#define ERR_CMD_ID              0x04        // û������
#define ERR_CHECK               0x06        // ����ĳ����У��ʹ���
#define ERR_OPERATE             0x07        // δ���������Ҫ��Ĳ���
#define ERR_PACKET_NUMBER       0x21        // �����������������
#define ERR_MEM_NOT_ENOUGH      0x22        // ����hex�ļ������޷�д��
#define ERR_ALL_CHECK			0x23		// �ܰ�У��ʹ���

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
extern uint8_t CheckUID(void);
void BootInit(void);
boot_cmd_t BootCmdRun(boot_cmd_t cmd);


uint8_t AllCheckSumCheck(void);
/* boot core.h*/
/* boot core.h*/
/* boot core.h*/

/*cg_sci_user.c*/
/*cg_sci_user.c*/
/*cg_sci_user.c*/

/*cg_sci_user.c*/
/*cg_sci_user.c*/
/*cg_sci_user.c*/

#include "BAT32G137.h"
#include "boot_core.h"

#define APP_ADDR                0X2000				//APP����ʼλ��
#define APP_SIZE                (0x10000 - 0X2000)	//APP������󳤶�

//#define VECTOR_OFFSET           0x1c
#define APP_VECTOR_ADDR         APP_ADDR

#define LDROM_ADDR				0X0000				//LDROM����ʼλ��
#define LDROM_SIZE				0x2000				//LDROM�Ĵ�С

#define DATA_ADDR				0x500200			//����״̬��־DATA Flash����ʼλ��
#define DATA_SIZE				0x500				//����״̬��־DATA�Ĵ�С

#define ONE_PAGE_SIZE           512                 //һҳ�ĳ���

#define IAP_CHECK_ADRESS 		0x1C00     		//���³ɹ�������洢����ʼ��ַ
#define IAP_CHECK_LENGTH		4		  			//���³ɹ������볤��,���14Byte
#define IAP_CHECK_AREA			APROM_AREA			//��־��������
#define	IAP_CHECK_NUMBER		0XAA,0X55,0X55,0XAA //��ʾAPP���������������������룬���14Byte

#define TOTAL_CHECKSUM_ADRESS   0x1C04     		//��λ������У��ʹ洢��ַ
#define TOTAL_CHECKSUM_LENGTH   4

#define PACKET_TOTAL_NUM_ADRESS (TOTAL_CHECKSUM_ADRESS + 4)     		//��λ������У��ʹ洢��ַ
#define PACKET_TOTAL_NUM_LENGTH 4

#define APP_BUFF_ADDR           0X10000		        //APP����������ʼλ��
#define APP_BUFF_SIZE           (0x10000-0X2000)	//APP��������󳤶�
#define	BUFF_CHECK_NUMBER		0X55,0XAA,0XAA,0X55 //��ʾAPP������װ���걸�������룬���14Byte

#define UID_ENC_ADRESS			0x1FE00		        //UID���Ĵ洢��ַ
#define UID_ENC_SIZE			16					//UID���ĳ���
#define UID_SIZE				(128/8)				//UID��Ч����
#define UID_ENC_AREA_AREA		APROM_AREA			//UID�������ڵĴ洢����

#define APP_TO_BOOT             0x55
#define BOOT_TO_APP             0xAA
#define	APROM_AREA	            0x55				//APROM��
#define DATA_AREA               0xAA				//DATA��
#define LDROM_AREA	            0X96				//LDROM��
#define APROM_BUFF_AREA			0x69				//APP������
#define UID_ENC_AREA			0x22				//UID���Ĵ洢��

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
extern void IAP_Remap(void);//���������Ĵ���װ����������
extern uint8_t IAP_WriteOneByte(uint32_t IAP_IapAddr,uint8_t Write_IAP_IapData,uint8_t area); //д���ֽ�IAP����



extern volatile uint8_t * gp_uart0_tx_address;        /* uart0 transmit buffer address */
extern volatile uint16_t  g_uart0_tx_count;           /* uart0 transmit data number */
extern volatile uint8_t * gp_uart0_rx_address;        /* uart0 receive buffer address */
extern volatile uint16_t  g_uart0_rx_count;           /* uart0 receive data number */
extern volatile uint16_t  g_uart0_rx_length;          /* uart0 receive data length */

extern void BootWaitTimeInit(void);
extern void BootProcess(void);
#endif
