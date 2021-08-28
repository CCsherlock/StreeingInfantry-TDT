#include "spi.h"


//三个SPI的默认CS脚
namespace SPI{
	enum SPIX{spi1=0,spi2,spi3};
	CsPin defaultCsPin[3] ={{GPIOA,GPIO_Pin_4} , {0,0} , {GPIOB,GPIO_Pin_6}};	
}
//静态成员申明和定义
bool Spi::initFlag[3] ={false,false,false};
/**
  * @brief SPI构造器
  * @note 暂时只有SPI1，全功能SPUI预计下版本完善
  */
Spi::Spi(SPI_TypeDef* spix,int baud)
{
	this->spix=spix;
	this->baud =baud;
}



/**
  * @brief SPI初始化
  * @note 暂时只有SPI1，全功能SPI预计下版本完善
  */
void Spi::init(void)
{
	GPIO_InitTypeDef  gpioInitStructure;
	SPI_InitTypeDef  SPI_InitStructure;
	gpioInitStructure.GPIO_Mode = GPIO_Mode_AF;//复用功能
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	
	
	//对应的SPI首次初始化判断
	if(spix == SPI1 && initFlag[SPI::spi1] == false)
	{
		Spi::initFlag[SPI::spi1] = true;
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//使能GPIOA时钟
		RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1, ENABLE);//使能SPI1时钟
		gpioInitStructure.GPIO_Pin =  GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7;//PA5~7复用功能输出
		GPIO_Init(GPIOA, &gpioInitStructure);//初始化	
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PA5复用为 SPI1  CLK//PB3
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PA6复用为 SPI1  MISO//PB4
		GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PA7复用为 SPI1  MOSI//PB5
		
		csInit(SPI::defaultCsPin[SPI::spi1]);
		//这里只针对SPI口初始化
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,ENABLE);//复位SPI1
		RCC_APB2PeriphResetCmd(RCC_APB2Periph_SPI1,DISABLE);//停止复位SPI1
			
	}
	else if(spix == SPI2 && initFlag[SPI::spi2] == false)
	{
		initFlag[SPI::spi2] = true;
		
		
	}
	else if(spix == SPI3 && initFlag[SPI::spi3] == false)
	{
		initFlag[SPI::spi3] = true;
		csInit(SPI::defaultCsPin[SPI::spi3]);
		RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB, ENABLE);//使能GPIOA时钟
		RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI3, ENABLE);//使能SPI2,3时钟
		gpioInitStructure.GPIO_Pin =  GPIO_Pin_3|GPIO_Pin_4|GPIO_Pin_5;//PA5~7复用功能输出	
		GPIO_Init(GPIOB, &gpioInitStructure);//初始化	
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource3,GPIO_AF7_SPI3); //PA5复用为 SPI1  CLK//PB3
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource4,GPIO_AF7_SPI3); //PA6复用为 SPI1  MISO//PB4
		GPIO_PinAFConfig(GPIOB,GPIO_PinSource5,GPIO_AF7_SPI3); //PA7复用为 SPI1  MOSI//PB5
		
		//这里只针对SPI口初始化
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,ENABLE);//复位SPI1
		RCC_APB1PeriphResetCmd(RCC_APB1Periph_SPI3,DISABLE);//停止复位SPI1
			
	}
	
	SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;  //设置SPI单向或者双向的数据模式:SPI设置为双线双向全双工
	SPI_InitStructure.SPI_Mode = SPI_Mode_Master;		//设置SPI工作模式:设置为主SPI
	SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;		//设置SPI的数据大小:SPI发送接收8位帧结构
	SPI_InitStructure.SPI_CPOL = SPI_CPOL_Low;		//串行同步时钟的空闲状态为高电平
	SPI_InitStructure.SPI_CPHA = SPI_CPHA_1Edge;	//串行同步时钟的第二个跳变沿（上升或下降）数据被采样
	SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;		//NSS信号由硬件（NSS管脚）还是软件（使用SSI位）管理:内部NSS信号有SSI位控制
	SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_64;		//定义波特率预分频的值:波特率预分频值为8
	SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;	//指定数据传输从MSB位还是LSB位开始:数据传输从MSB位开始
	SPI_InitStructure.SPI_CRCPolynomial = 7;	//CRC值计算的多项式

	SPI_Init(spix, &SPI_InitStructure);  //根据SPI_InitStruct中指定的参数初始化外设SPIx寄存器
	SPI_Cmd(spix, ENABLE); //使能SPI外设
	readWriteByte(0xff);//启动传输	
}

void Spi::csInit(CsPin cs)
{
	this->csPin.port = cs.port;
	this->csPin.pin = cs.pin;
	GPIO_InitTypeDef  gpioInitStructure;
	
	gpioInitStructure.GPIO_Pin = csPin.pin;//PB5~7复用功能输出	
	gpioInitStructure.GPIO_Mode = GPIO_Mode_OUT;//复用功能
	gpioInitStructure.GPIO_OType = GPIO_OType_PP;//推挽输出
	gpioInitStructure.GPIO_Speed = GPIO_Speed_100MHz;//100MHz
	gpioInitStructure.GPIO_PuPd = GPIO_PuPd_UP;//上拉
	GPIO_Init(csPin.port, &gpioInitStructure);//初始化
}


/**
  * @brief CS选中
  * @note 暂时只有SPI1，全功能SPUI预计下版本完善
  */
 void Spi::csOn()
{
	GPIO_ResetBits(csPin.port, csPin.pin);
}
void Spi::csOff()
{
	GPIO_SetBits(csPin.port, csPin.pin);
}



/**
  * @brief 读写一个字节
  * @param TxData:要写入的字节
  * @return 
  * @note 通过for循环避免死循环
  */
uint16_t Spi::readWriteByte(uint16_t TxData)//SPI1_SendByte
{
	for(int i=0;i<0xfffffff && SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_TXE) == RESET;i++);//等待发送区空  
	SPI_I2S_SendData(spix, TxData); //通过外设SPIx发送一个byte  数据
	for(int i=0;i<0xfffffff && SPI_I2S_GetFlagStatus(spix, SPI_I2S_FLAG_RXNE) == RESET;i++);//等待接收完一个byte  
	return SPI_I2S_ReceiveData(spix); //返回通过SPIx最近接收的数据	
}



/**
  * @brief 读单字节
  * @param regAddr:寄存器地址
  * @return 读取到的字节
  */
u8 Spi::readByte(u8 regAddr)
{
	csOn();
	u8 val;
	readWriteByte(regAddr | 0x80); //Register. MSB 1 is read instruction.
	val = readWriteByte(0x00); //Send DUMMY to read data
	csOff();
	return val;
}



/**
  * @brief 读多字节
  * @param regAddr:寄存器地址
  *			len:长度
  *			data：数据地址
  */
void Spi::readBytes(u8 regAddr, u8 len, u8* data)
{
	csOn();
	readWriteByte(regAddr | 0x80); //Register. MSB 1 is read instruction.
	for(u8 i=0;i<len;i++)
	{
		data[i] = readWriteByte(0x00); //Send DUMMY to read data
	}
	csOff();
}



/**
  * @brief 写单字节
  * @param regAddr:寄存器地址
  * 		data:写入的数据
  */
void Spi::writeByte(u8 regAddr, u8 data)
{
	csOn();
	readWriteByte(regAddr & 0x7F); //Register. MSB 0 is write instruction.
	readWriteByte(data); //Send Data to write
	csOff();
}



/**
  * @brief 写多字节
  * @param 	regAddr:寄存器地址
  *			len:长度
  *			data：数据地址
  */
void Spi::writeBytes(u8 regAddr, u8 len, u8* data)
{
	csOn();
	readWriteByte(regAddr & 0x7F); //Register. MSB 0 is write instruction.
	for(u8 i=0;i<len;i++)
	{
		readWriteByte(data[i++]); //Send Data to write
	}
	csOff();
}



