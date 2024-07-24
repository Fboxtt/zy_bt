//************************************************************
//  Copyright (c) 中微半导体（深圳）股份有限公司
//	文件名称	: tea.c
//	模块功能	: TEA算法加解密C文件
//  更正日期	: 2022/1/10
// 	版本		: V1.0
//************************************************************
#include "tea.h"

typedef union 
{
	uint32_t Key_uint32[4];
	uint16_t Key_uint16[8];
}Tea_key_t;
Tea_key_t TeaKey ={TEA_KEY_0,TEA_KEY_1,TEA_KEY_2,TEA_KEY_3};

//void EncryptTEA(uint32_t* firstChunk,uint32_t* secondChunk)
//{
//    uint32_t y = *firstChunk;
//    uint32_t z = *secondChunk;
//    uint32_t sum = 0;
//	uint32_t delta = 0x9e3779b9;
//	uint8_t i;
//    

//    for (i = 0; i < 32; i++)//32轮运算(需要对应下面的解密核心函数的轮数一样)
//    {
//        sum += delta;
//        y += ((z << 4) + TeaKey.Key_uint32[0]) ^ (z + sum) ^ ((z >> 5) + TeaKey.Key_uint32[1]);
//        z += ((y << 4) + TeaKey.Key_uint32[2]) ^ (y + sum) ^ ((y >> 5) + TeaKey.Key_uint32[3]);
//    }

//    *firstChunk = y;
//    *secondChunk = z;
//}
void DecryptTEA(uint32_t * firstChunk, uint32_t * secondChunk)
{
    uint32_t  sum = 0;
    uint32_t  y = *firstChunk;
    uint32_t  z = *secondChunk;
    uint32_t  delta = 0x9e3779b9;
	uint8_t i;
	
    sum = delta << 5; //32轮运算，所以是2的5次方；16轮运算，所以是2的4次方；8轮运算，所以是2的3次方

    for (i = 0; i < 32; i++) //32轮运算
    {
        z -= (y << 4) + TeaKey.Key_uint32[2] ^ y + sum ^ (y >> 5) + TeaKey.Key_uint32[3];
        y -= (z << 4) + TeaKey.Key_uint32[0] ^ z + sum ^ (z >> 5) + TeaKey.Key_uint32[1];
        sum -= delta;
    }

    *firstChunk = y;
    *secondChunk = z;
}



