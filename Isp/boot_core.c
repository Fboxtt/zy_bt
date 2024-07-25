//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: boot_core.c
//	ģ�鹦��	: BootLoader����
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************

#include "boot_core.h"

uint8_t HandShakeValue;	                            //���ִ����洢����
uint8_t ResetFlag = 0;								//��ʾ��λ�������
uint8_t CurrState = 0;								//��ǰоƬ��״̬
uint32_t ReadFlashLength = 0;                       //��Flash�ĳ���        
uint32_t ReadFlashAddr = 0;							//��Flash����ʼ��ַ
const uint8_t Boot_Inf_Buff[EditionLength] = Edition;//�汾�Ŵ洢
boot_addr_t BeginAddr = APP_ADDR;				    //��ʼ��ַ�洢
uint32_t NewBaud = UartBaud;						//�洢�²����ʵı���

/* boot��ʼ�����Ӻ������뽫��ʼ������д��ú��� */
void BootInit()
{
	UartInit(UartBaud);
	CurrState = IAP_CheckAPP();
    if(CurrState==1)//�ж�APP�Ƿ�����������������ʱ
    {
        BaseTimeSystemInit(BOOT_ENABLE);
    }
	#ifdef FLASH_BUFF_ENABLE
	else if(CurrState==2)//�����������ص���������ֱ������APP
	{
		IAP_Remap();//�����뻺���������ݼ��������������
		IAP_FlagWrite(1);//����ΪAPP������̬
        IAP_Reset();
	}
	#endif
}

void Decrypt_Fun(uint8_t* buff)
{
	uint8_t i;
	uint8_t k;
	uint32_t first_chunk;
	uint32_t second_chunk;
    union  
	{
		uint32_t temp_uint32[16];
		uint8_t temp_uint8[64];
	}temp; 
	if(CmmuLength>64)
	{
		return;
	}
    //ARMΪС��ģʽ��Ҫ��ÿ���ֵĸ�λ�͵�λ�Ե�
    for(i=0;i<(CmmuLength/4);i=i+1)
	{		
        for(k=0;k<4;k++)
        {
            temp.temp_uint8[i*4+(3-k)] = buff[i*4+k];
        }
	}
	for(i=0;i<(CmmuLength/4);i=i+2)
	{
		first_chunk = temp.temp_uint32[i];
		second_chunk = temp.temp_uint32[i+1];
		DecryptTEA(&first_chunk,&second_chunk);
		temp.temp_uint32[i] = first_chunk;
		temp.temp_uint32[i+1] = second_chunk;
        for(k=0;k<4;k++)
        {
            buff[i*4+k] = temp.temp_uint8[i*4+(3-k)] ;
            buff[(i+1)*4+k] = temp.temp_uint8[(i+1)*4+(3-k)] ;
        }
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
        BaseTimeSystemInit(BOOT_DISABLE);//�رն�ʱ��

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
        IAP_FlagWrite(1);//APP���������б�־
		#endif	
					
        IAP_Reset();//��λ����APP
    }
}


uint8_t temp = 0;
boot_cmd_t BootCmdRun(boot_cmd_t cmd)
{
    uint8_t i;	
    boot_cmd_t cmd_buff = BOOT_BOOL_FALSE;//����ִ�н������
    CmmuSendLength = 0;	
    switch(cmd)//��������ִ����Ӧ�Ķ���
    {
        case ENTER_BOOTMODE:
        {
            HandShakeValue++;
            if(HandShakeValue>=HandShakes)
            {
               /* �ر�ʱ�� */
               BaseTimeSystemInit(BOOT_DISABLE);
			   #ifndef FLASH_BUFF_ENABLE
               IAP_FlagWrite(0);//��APP��ɱ�־ȥ����������¹���ʧ�����´��ϵ�һֱά����BOOT�ȴ�����
			   #endif
            }
            cmd_buff = DEAL_SUCCESS;
        }break;
        case SET_BAUD:
        {
            cmd_buff = DEAL_SUCCESS;
			NewBaud = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
        }break;
        case READ_BOOT_CODE_INF:
        {
            cmd_buff = RETURN_BOOT_CODE_INF;
            for(i=0;i<EditionLength;i++)
            {
                CmdSendData[i] = Boot_Inf_Buff[i];                
            }
            CmmuSendLength = EditionLength;
        }break;
        case SET_ADDRESS://���û���ַ��������
        {
            //BeginAddr = CommuData[6]*256+CommuData[7];
			BeginAddr = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
            //if((CommuData[4])==RETURN_FLASH_UID)
            if(BeginAddr==UID_ENC_ADRESS)
            {
                temp = UID_ENC_AREA;
            }
            else
            {
                temp = APROM_AREA;
                #ifdef FLASH_BUFF_ENABLE
                BeginAddr = APP_BUFF_ADDR;//�����봫��������ȥ
                #endif		
            }            			
            cmd_buff = DEAL_SUCCESS;			
        }break;
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
            cmd_buff = DEAL_SUCCESS;
        }break;
        case WRITE_FLASH://IAPд������ɹ�
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
			
			if(IAP_WriteMultiByte(BeginAddr,CommuData,CmmuLength,temp))
			{
				cmd_buff = DEAL_SUCCESS;
			}				
            else
			{
				cmd_buff = DEAL_FAIL;
			}
            BeginAddr = BeginAddr+CmmuLength;
        }break;        
        case ENTER_APPMODE: //�����û�����
        {
            cmd_buff = DEAL_SUCCESS; //��Ӧ�˳���Bootloader 
            #ifdef FLASH_BUFF_ENABLE            
			CurrState = 2;//��ʾ���뻺�����������
            #endif
            ResetFlag = 1;
        }break;        
        case NO_CMD://�޲���
        {
            return BOOT_BOOL_FALSE;
        }
        case READ_FLASH:
        {            
			//ReadFlashAddr = CommuData[6]*256+CommuData[7];
            ReadFlashAddr = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
			ReadFlashLength = (CommuData[8]<<24)+(CommuData[9]<<16)+(CommuData[10]<<8)+CommuData[11];            
            if((CommuData[4])==RETURN_FLASH_UID)
            {
                temp = APROM_AREA;ReadFlashAddr = UID_BASE;
            }
            else
            {
                temp = APROM_AREA;
            }
			IAP_ReadMultiByte(ReadFlashAddr,CmdSendData,ReadFlashLength,temp);								
            cmd_buff = RETURN_FLASH;
            CmmuSendLength = ReadFlashLength;
			 			
        }break;      
        default:
        {
            cmd_buff = DEAL_FAIL;
            CmdSendData[0] = ERROR_CMD_FAIL;
            CmmuSendLength = 1;
        }
        break;
    }
    return cmd_buff;
}

