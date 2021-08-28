/*****************************************************************************
File name: TDT_Bsp\src\usart.h
Description: 串口
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
*****************************************************************************/
#ifndef __USART_H__
#define __USART_H__

#include "board.h"


#define CONTROL_RX_BUF_NUM 10u      //上层主控板发送字节数的2倍
#define CONTROL_FRAME_LENGTH 5u      //上层主控板发送字节数    
#define CONTROL_TX_BUF_NUM 9u      //发送给上层主控板的数据字节数

extern  uint8_t CONTROL_tx_tran_buf[CONTROL_TX_BUF_NUM];   //向外设传输数据的中转站   tx_bufdata到tx_buf到USART1->DR


void TDT_Communicate_Init(void);
void Communicate_SendChar(void); 
void Communicate_Update(void);  
static void Communicate_Handle_data( uint8_t *comm_buf);


#endif
