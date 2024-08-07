//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: boot_core.h
//	ģ�鹦��	: BootLoader����
//  ��������	: 2022/1/1
// 	�汾		: V1.0
//************************************************************
#ifndef _BOOT_CORE_H_
#define _BOOT_CORE_H_
/* �뽫��Ҫ��ͷ�ļ����� */
#include "BAT32G137.h"
#include "communication_protocol.h"//ͨѶЭ��ͷ�ļ�
#include "flash_operate.h"//Flash����ͷ�ļ�
#include "serial_port_config.h"//����ͨѶ�ײ������ļ�
#include "base_time_system.h"
#include "tea.h"

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


#define ERR_NO                  0x00        // ���쳣
#define ERR_CMD_LEN             0x02        // �ӻ����յ��İ����Ⱥ�����Ȳ���
#define ERR_CMD_ID              0x04        // û������
#define ERR_CHECK               0x06        // ����ĳ����У��ʹ���
#define ERR_OPERATE             0x07        // δ���������Ҫ��Ĳ���
#define ERR_PACKET_NUMBER       0x21        // �����������������
#define ERR_MEM_NOT_ENOUGH      0x22        // ����hex�ļ������޷�д��

//��վ��Ӧ����������
#define DEAL_SUCCESS 			0X9F		//��Ӧ�����ɹ�
#define DEAL_FAIL				0xDF		//��Ӧ����ʧ��
#define	RETURN_IC_INF			0xA3		//��ӦоƬ�ͺ�
#define	RETURN_BOOT_CODE_INF	0XA4		//��ӦBOOT����汾��
#define RETURN_FLASH            0xA5        //��Ӧ������FLASH��Ϣ
//��������
#define	ERROR_CHECK_FAIL		0x01		//��ʾͨѶУ��ʧ��
#define	ERROR_BURN_FAIL			0x02		//��ʾ��дУ�����
#define	ERROR_CMD_FAIL			0x04		//��ʾ�������
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
#define CommunicationLength1    (64+7)
extern boot_length_t CmmuLength;		             //�������ݳ���
extern boot_cmd_t CMDBuff;		                     //����洢����
extern boot_data_t CommuData[CommunicationLength1];	 //ͨѶ���ջ���
extern boot_data_t CmdSendData[CommunicationLength1];//���ͻ���
extern boot_length_t CmmuSendLength;		         //�������ݳ���
extern uint32_t NewBaud;							 //�²����ʴ洢													
extern uint8_t CurrState;							 //�洢��ǰоƬ��״̬,0:BOOTģʽ  1:APP����̬     2:���뻺�����̬
extern boot_bool_t ResetFlag;
extern void BootCheckReset(void);		//����Ƿ��и�λ�ź�
extern uint8_t CheckUID(void);
void BootInit(void);
boot_cmd_t BootCmdRun(boot_cmd_t cmd);

#endif
