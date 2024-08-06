//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: boot_core.c
//	模块功能	: BootLoader核心
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************

#include "boot_core.h"

uint8_t HandShakeValue;	                            //握手次数存储变量
uint8_t ResetFlag = 0;								//表示复位条件达成
uint8_t CurrState = 0;								//当前芯片的状态
uint32_t ReadFlashLength = 0;                       //读Flash的长度        
uint32_t ReadFlashAddr = 0;							//读Flash的起始地址
const uint8_t Boot_Inf_Buff[EditionLength] = Edition;//版本号存储
const uint8_t IC_INF_BUFF[IC_EDITION_LENTH] = IC_EDITION; // 芯片型号存储
boot_addr_t BeginAddr = APP_ADDR;				    //起始地址存储
uint32_t NewBaud = UartBaud;						//存储新波特率的变量

/* boot初始化钩子函数，请将初始化代码写入该函数 */
void BootInit()
{
	UartInit(UartBaud);
	CurrState = IAP_CheckAPP();
    if(CurrState==1)//判断APP是否完整，完整则开启定时
    {
        BaseTimeSystemInit(BOOT_ENABLE);
    }
	#ifdef FLASH_BUFF_ENABLE
	else if(CurrState==2)//将缓存区加载到运行区后直接运行APP
	{
		IAP_Remap();//将代码缓存区的内容加载入程序运行区
		IAP_FlagWrite(1);//设置为APP可运行态
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
    //ARM为小端模式需要将每个字的高位和低位对调
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
        BaseTimeSystemInit(BOOT_DISABLE);//关闭定时器

		#ifdef FLASH_BUFF_ENABLE
		if(CurrState == 2)
		{
			IAP_FlagWrite(2);//代码缓存区就绪标志
			//IAP_Remap();
		}
		if(CurrState == 0)
		{
			IAP_FlagWrite(2);//APP可正常运行标志
		}
		#else
        IAP_FlagWrite(1);//APP可正常运行标志
		#endif	
					
        IAP_Reset();//复位进入APP
    }
}


uint8_t temp = 0;
boot_cmd_t BootCmdRun(boot_cmd_t cmd)
{
    uint8_t i;	
    boot_cmd_t cmd_buff = BOOT_BOOL_FALSE;//命令执行结果缓存
    CmmuSendLength = 0;	
    switch(cmd)//根据命令执行相应的动作
    {
        case READ_BOOT_CODE_INF:
        {
            cmd_buff = RETURN_BOOT_CODE_INF;
            for(i=0;i<EditionLength;i++)
            {
                CmdSendData[i] = Boot_Inf_Buff[i];                
            }
            CmmuSendLength = EditionLength;
        }break;
        case READ_IC_INF:
        {
            cmd_buff = READ_IC_INF;
            for(i=0;i<EditionLength;i++)
            {
                CmdSendData[i] = Boot_Inf_Buff[i];                
            }
            CmmuSendLength = EditionLength;
        }break;
        case HEX_INFO:
        {
            cmd_buff = HEX_INFO;

        }break;
        case ENTER_BOOTMODE:
        {
            HandShakeValue++;
            if(HandShakeValue>=HandShakes)
            {
               /* 关闭时钟 */
               BaseTimeSystemInit(BOOT_DISABLE);
			   #ifndef FLASH_BUFF_ENABLE
               IAP_FlagWrite(0);//将APP完成标志去掉，如果更新过程失败则下次上电一直维持在BOOT等待更新
			   #endif
            }
            cmd_buff = DEAL_SUCCESS;
        }break;
//        case SET_BAUD:
//        {
//            cmd_buff = DEAL_SUCCESS;
//			NewBaud = (((uint32_t)CommuData[4])<<24)+(((uint32_t)CommuData[5])<<16)+(((uint32_t)CommuData[6])<<8)+((uint32_t)CommuData[7]);
//        }break;

//        case SET_ADDRESS://设置基地址配置命令
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
//                BeginAddr = APP_BUFF_ADDR;//将代码传到缓存区去
//                #endif		
//            }            			
//            cmd_buff = DEAL_SUCCESS;			
//        }break;
        case EARSE_ALL:	//擦除APROM所有内容
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
        case WRITE_FLASH://IAP写入操作成功
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
//        case ENTER_APPMODE: //运行用户代码
//        {
//            cmd_buff = DEAL_SUCCESS; //回应退出了Bootloader 
//            #ifdef FLASH_BUFF_ENABLE            
//			CurrState = 2;//表示代码缓存已下载完成
//            #endif
//            ResetFlag = 1;
//        }break;        
        case NO_CMD://无操作
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

