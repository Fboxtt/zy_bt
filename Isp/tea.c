//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: tea.c
//	ģ�鹦��	: TEA�㷨�ӽ���C�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
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

//    for (i = 0; i < 32; i++)//32������(��Ҫ��Ӧ����Ľ��ܺ��ĺ���������һ��)
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
	
    sum = delta << 5; //32�����㣬������2��5�η���16�����㣬������2��4�η���8�����㣬������2��3�η�

    for (i = 0; i < 32; i++) //32������
    {
        z -= (y << 4) + TeaKey.Key_uint32[2] ^ y + sum ^ (y >> 5) + TeaKey.Key_uint32[3];
        y -= (z << 4) + TeaKey.Key_uint32[0] ^ z + sum ^ (z >> 5) + TeaKey.Key_uint32[1];
        sum -= delta;
    }

    *firstChunk = y;
    *secondChunk = z;
}



