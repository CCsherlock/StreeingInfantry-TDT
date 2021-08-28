/*****************************************************************************
File name: TDT_Alg\src\crc.h
Description: CRC校验算法
Author: 肖银河
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef _CRC_H
#define _CRC_H

#include "stdint.h"

#ifdef __cplusplus
extern "C"{
#endif	/*__cplusplus*/
#include "stm32f4xx.h"


#define NULL 0                   /* see <stddef.h> */

extern const unsigned char CRC8_TAB[256];
extern const uint16_t wCRC_Table[256];

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength, unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength, uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);


#ifdef __cplusplus
}
#endif	/*__cplusplus*/
#endif 
