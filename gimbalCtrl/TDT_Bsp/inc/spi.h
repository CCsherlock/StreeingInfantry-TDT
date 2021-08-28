#ifndef _SPI_H
#define _SPI_H

#include "board.h"
struct CsPin
{
	GPIO_TypeDef* port;
	uint16_t pin;
};
class Spi
{
	private:
		SPI_TypeDef* spix;//SPIx，用于初始化时操作不用IO
		int baud;//波特率（暂时不可用）
		CsPin csPin;
		static bool initFlag[3]; 
	public:
		/*SPI构造器*/
		Spi(SPI_TypeDef* spix,int baud);
		/*SPI初始化*/
		void init();
	
		/*CS脚初始化*/
		void csInit(CsPin csPin);
		/*CS片选操作*/
		virtual void csOn();
		virtual void csOff();
		/*读写一个字节*/
		uint16_t readWriteByte(uint16_t TxData);
		/*读一个字节*/
		u8 readByte(u8 regAddr);
		/*读多个字节*/
		void readBytes(u8 regAddr, u8 len, u8* data);
		/*写一个字节*/
		void writeByte(u8 regAddr, u8 data);
		/*写多个字节*/
		void writeBytes(u8 regAddr, u8 len, u8* data);

};


#endif