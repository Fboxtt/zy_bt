//************************************************************
//  Copyright (c) ��΢�뵼�壨���ڣ��ɷ����޹�˾
//	�ļ�����	: tea.h
//	ģ�鹦��	: TEA�㷨�ӽ���h�ļ�
//  ��������	: 2022/1/10
// 	�汾		: V1.0
//************************************************************

#ifndef _TEA_H_
#define _TEA_H_

#include "BAT32G137.h"

#define TEA_KEY_0		0X12345678
#define TEA_KEY_1		0X9ABCDEF0
#define TEA_KEY_2		0X55555555
#define TEA_KEY_3		0XAAAAAAAA

void EncryptTEA(uint32_t * firstChunk,uint32_t * secondChunk);
void DecryptTEA(uint32_t * firstChunk, uint32_t * secondChunk);

#endif
