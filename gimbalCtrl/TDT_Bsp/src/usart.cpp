/******************************
File name: TDT_Bsp\src\usart.cpp
Description: 串口
function:
	——————————————————————————————————————————————————————————————————————————
	
	——————————————————————————————————————————————————————————————————————————
Author: 祖传
Version: 1.1.1.191112_alpha
Date: 19.11.12
History: 
	——————————————————————————————————————————————————————————————————————————
	19.11.12 首次完成
	——————————————————————————————————————————————————————————————————————————
****************************  */
#include "usart.h"
#include "crc.h"

 uint8_t CONTROL_rx_buf[2][CONTROL_RX_BUF_NUM];    //上层主控板发送的数据，二维数组
static uint8_t CONTROL_tx_buf[CONTROL_TX_BUF_NUM];     //向外设传送数据的地址
static uint8_t CONTROL_tx_tran_buf[CONTROL_TX_BUF_NUM];   //向外设传输数据的中转站   tx_bufdata到tx_buf到USART1->DR
 
void TDT_Communicate_Init(void)
{
        /* -------------- Enable Module Clock Source ----------------------------*/
        RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA | RCC_AHB1Periph_DMA2, ENABLE);
        RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);

        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
        RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);

        GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1); //PA9  usart1 rx
        GPIO_PinAFConfig(GPIOA, GPIO_PinSource10,GPIO_AF_USART1); //PA10  usart1 Tx

        /* -------------- Configure GPIO ---------------------------------------*/
        {
                GPIO_InitTypeDef GPIO_InitStructure;
                USART_InitTypeDef USART_InitStructure;
            
                GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 |GPIO_Pin_10;
                GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
                GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
                GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
                GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
                GPIO_Init(GPIOA, &GPIO_InitStructure);

                USART_DeInit(USART1);

                USART_InitStructure.USART_BaudRate = 115200;
                USART_InitStructure.USART_WordLength = USART_WordLength_8b;
                USART_InitStructure.USART_StopBits = USART_StopBits_1;
                USART_InitStructure.USART_Parity = USART_Parity_No;
                USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
                USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
                USART_Init(USART1, &USART_InitStructure);

                USART_DMACmd(USART1, USART_DMAReq_Rx|USART_DMAReq_Tx, ENABLE);

                USART_ClearFlag(USART1, USART_FLAG_IDLE);
                USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);

                USART_Cmd(USART1, ENABLE);
        }

        /* -------------- Configure NVIC ---------------------------------------*/
        {
                NVIC_InitTypeDef NVIC_InitStructure;
            
                NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
                NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
                NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
                NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
                NVIC_Init(&NVIC_InitStructure);
        }

        //DMA2 stream5 ch4  or (DMA2 stream2 ch4)    !!!!!!! P206 of the datasheet
        /* -------------- Configure RX DMA -----------------------------------------*/
        {
                DMA_InitTypeDef DMA_InitStructure;
            
                DMA_DeInit(DMA2_Stream5);

                DMA_InitStructure.DMA_Channel = DMA_Channel_4;
                DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART1->DR);
                DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_rx_buf[0];
                DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
                DMA_InitStructure.DMA_BufferSize = CONTROL_RX_BUF_NUM;
                DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
                DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
                DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
                DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
                DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
                DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
                DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
                DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
                DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
                DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
                DMA_Init(DMA2_Stream5, &DMA_InitStructure);
            
                DMA_DoubleBufferModeConfig(DMA2_Stream5, (uint32_t)CONTROL_rx_buf[1], DMA_Memory_0);
                DMA_DoubleBufferModeCmd(DMA2_Stream5, ENABLE);
                DMA_Cmd(DMA2_Stream5, DISABLE); //Add a disable
                DMA_Cmd(DMA2_Stream5, ENABLE);
        }
        /* -------------- Configure TX DMA -----------------------------------------*/
        {
        	DMA_InitTypeDef DMA_InitStructure;
            NVIC_InitTypeDef NVIC_InitStructure;
                
            DMA_DeInit(DMA2_Stream7);
    
            DMA_DeInit(DMA2_Stream7);
            DMA_InitStructure.DMA_Channel= DMA_Channel_4;
            DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
            DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)CONTROL_tx_buf;
            DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;
            DMA_InitStructure.DMA_BufferSize = CONTROL_TX_BUF_NUM;
            DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
            DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
            DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
            DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
            DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
            DMA_InitStructure.DMA_Priority = DMA_Priority_High;
            DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
            DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
            DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
            DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
            DMA_Init(DMA2_Stream7,&DMA_InitStructure);
            
            DMA_ITConfig(DMA2_Stream7,DMA_IT_TC,ENABLE);
            
            NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
            NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
            NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
            NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
            NVIC_Init(&NVIC_InitStructure);
            
            DMA_Cmd(DMA2_Stream7,DISABLE);
            }
}

 int Flag_Tx_Gsm_Busy;

//串口中断
void USART1_IRQHandler(void)
{
    if (USART_GetITStatus(USART1,  USART_IT_TC) != RESET)
    {
        /* 关闭发送完成中断  */ 
        USART_ITConfig(USART1,USART_IT_TC,DISABLE);  
        Flag_Tx_Gsm_Busy = 0; 
        USART_ClearITPendingBit(USART1, USART_IT_TC);           
    }
    else if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET)
    {
        USART_ReceiveData(USART1);
    }   
    
    else if (USART_GetITStatus(USART1, USART_IT_IDLE) != RESET)
    {
        static uint16_t this_time_rx_len = 0;
        USART_ReceiveData(USART1);

        if(DMA_GetCurrentMemoryTarget(DMA2_Stream5) == 0)
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream5, DISABLE);
            this_time_rx_len = CONTROL_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream5);
            DMA_SetCurrDataCounter(DMA2_Stream5, CONTROL_RX_BUF_NUM);
            DMA2_Stream5->CR |= DMA_SxCR_CT;
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA2_Stream5, ENABLE);
            if(this_time_rx_len == CONTROL_FRAME_LENGTH)
            {
                //处理遥控器数据
                Communicate_Handle_data(CONTROL_rx_buf[0]);
            }
        }
        else
        {
            //重新设置DMA
            DMA_Cmd(DMA2_Stream5, DISABLE);
            this_time_rx_len = CONTROL_RX_BUF_NUM - DMA_GetCurrDataCounter(DMA2_Stream5);
            DMA_SetCurrDataCounter(DMA2_Stream5, CONTROL_RX_BUF_NUM);
            DMA2_Stream5->CR &= ~(DMA_SxCR_CT);
            //清DMA中断标志
            DMA_ClearFlag(DMA2_Stream5, DMA_FLAG_TCIF5 | DMA_FLAG_HTIF5);
            DMA_Cmd(DMA2_Stream5, ENABLE);
            if(this_time_rx_len == CONTROL_FRAME_LENGTH)
            {
                //处理遥控器数据
                Communicate_Handle_data(CONTROL_rx_buf[1]);
            }
        }
    }
}


 void Communicate_SendChar(void)  
{  

    CONTROL_tx_tran_buf[0]=0xFF;
    CONTROL_tx_tran_buf[1]=0xFF;
    CONTROL_tx_tran_buf[2]=0xFF;
    CONTROL_tx_tran_buf[3]=0xFF;
    CONTROL_tx_tran_buf[4]=0xFF;
    CONTROL_tx_tran_buf[5]=0xFF;
    CONTROL_tx_tran_buf[6]=0xFF;
	
	Append_CRC16_Check_Sum(CONTROL_tx_tran_buf,CONTROL_TX_BUF_NUM);
	
    //等待空闲
    while (Flag_Tx_Gsm_Busy);
    Flag_Tx_Gsm_Busy = 1;

    //复制数据  
    memcpy(CONTROL_tx_buf,CONTROL_tx_tran_buf,CONTROL_TX_BUF_NUM);  
    //设置传输数据长度  
    DMA_SetCurrDataCounter(DMA2_Stream7,CONTROL_TX_BUF_NUM);  
    //打开DMA,开始发送  
    DMA_Cmd(DMA2_Stream7,ENABLE); 
    DelayUs(100);   
}  

void DMA2_Stream7_IRQHandler(void)
{
    if(DMA_GetITStatus(DMA2_Stream7,DMA_IT_TCIF7) != RESET)   
    {  
        /* 清除标志位 */
        DMA_ClearFlag(DMA2_Stream7,DMA_IT_TCIF7);  
        /* 关闭DMA */
        DMA_Cmd(DMA2_Stream7,DISABLE);
        /* 打开发送完成中断,确保最后一个字节发送成功 */
        USART_ITConfig(USART1,USART_IT_TC,ENABLE);  
    }  
}


static void Communicate_Handle_data( uint8_t *comm_buf)
{
	
    if(comm_buf[0]==0xFF &&  (Verify_CRC16_Check_Sum(comm_buf,CONTROL_FRAME_LENGTH)))
    {    
      //      Claw_Singal = (u8)(comm_buf[1]>>4);//爪子状态
    }
}

















