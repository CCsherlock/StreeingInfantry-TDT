#ifndef _GY_53_H_
#define _GY_53_H_
#ifdef __cplusplus

#endif	/*__cplusplus*/
#include "board.h"
#include "crc.h"

class gy_53
{
private:
	/* data */
	uint32_t baudRatePrescaler = 115200;
	bool usartInitFlag[3] = {0};
	
public:
	
	gy_53(USART_TypeDef * USART,uint32_t BaudRatePrescaler);
	USART_TypeDef * ThisUSART;
	void gy_53_Init();
	void send_com(u8 data);
	void USART_Send(uint8_t *Buffer, uint8_t Length);
	void USART_Send_bytes(uint8_t *Buffer, uint8_t Length);
	void USARTx_send_byte(uint8_t byte);
	struct gy_53_Recv_Struct_t
	{
			uint8_t frameHeader1;           //0xA5
			uint8_t frameHeader2;           //0xA5
			uint8_t datatype;				//数据类型	0x15
			uint8_t	datasize;				//数据量	0x03
			uint16_t ditance;			//距离低8位
			uint8_t	measure_mode;			//测量模式
			uint8_t	sum_check;				//校验和
	};
	gy_53_Recv_Struct_t gy_53_Recv_Struct;

	
};
extern uint16_t GY_53distence[3];
extern "C"{
	void USART1_IRQHandler(void);
	void USART3_IRQHandler(void);
	void USART6_IRQHandler(void);	
#ifdef __cplusplus
}
#endif	/*__cplusplus*/
#endif
