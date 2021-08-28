#include "judgement.h"
#include "crc.h"
#include "FreeRTOS.h"
#include "timers.h"

#if USE_JUDGEMENT
Judgement judgement; //裁判系统结构体


static const u16 RED_BLUE_ID_DIFF = 100;	 //红蓝ID差
static const u16 ROBOT_CLIENT_ID_DIFF = 256; //机器人与客户端ID差

#define CRC_NO_CALC_STATUS 0	 //不计算CRC16和CRC8
#define CRC8_CRC16_CALC_STATUS 1 //同时计算CRC16和CRC8
#define CRC16_CALC_STATUS 2		 //仅计算CRC16

#define USE_DMA_TRANSFER_DATA 0 //使用DMA进行内存拷贝

static u16 jgmtOfflineCheck = 0;

/**
 * @brief 队列处理
 *
 * @param xTimer
 */
void ringQueue(TimerHandle_t xTimer)
{
	if(jgmtOfflineCheck > 500)//500ms
		judgement.jgmtOffline = 1;
	else
		jgmtOfflineCheck++;
	judgement.ringQueue();
}

/**
 * @brief 初始化
 *
 */
void Judgement::init()
{
	uart6Config(); //初始化串口6以及发送接收的DMA

	xTimerStart(xTimerCreate("", pdMS_TO_TICKS(1), pdTRUE, (void *)5, ::ringQueue), 0); //创建软件定时器，1ms调用
}

/**
 * @brief 串口6初始化
 *
 */
void Judgement::uart6Config()
{
	USART_InitTypeDef USART_InitStructure;
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	DMA_InitTypeDef DMA_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

	GPIO_PinAFConfig(GPIOC, GPIO_PinSource6, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource7, GPIO_AF_USART6);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7 | GPIO_Pin_6;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_Init(GPIOC, &GPIO_InitStructure);

	USART_DeInit(USART6);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);
	USART_Cmd(USART6, ENABLE);

	USART_DMACmd(USART6, USART_DMAReq_Rx | USART_DMAReq_Tx, ENABLE);

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);

	DMA_DeInit(DMA2_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judgeDataBuffer;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = JUDGE_BUFFER_LENGTH;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull;
	DMA_InitStructure.DMA_MemoryBurst = DMA_Mode_Normal;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1, &DMA_InitStructure);

	DMA_ITConfig(DMA2_Stream1, DMA_IT_TC, ENABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
}


/**
 * @brief 接收DMA定义
 * @note  由于裁判系统发送一帧的中间会停止发送，故不能用空闲中断+DMA进行读取
 * 		  而使用DMA循环接收+各个字节轮询遍历的方法
 */
void DMA2_Stream1_IRQHandler(void)
{
	if (DMA_GetITStatus(DMA2_Stream1, DMA_IT_TCIF1))
	{
		jgmtOfflineCheck = 0;
		judgement.jgmtOffline = 0;
		judgement.addFullCount(); //说明到末尾了，需要进行过圈处理

		DMA_ClearFlag(DMA2_Stream1, DMA_IT_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}
}

static u8 CRC8_Check = 0xff;		  //参与CRC8校验的计算变量
static uint16_t CRC16_Check = 0xffff; //参与CRC16校验的计算变量
static u8 crcCalcState = 0;			  //CRC8判断到底进行什么校验

static int64_t read_len;	  //已经处理的长度
static int64_t rx_len;		  //已经读取的长度
static uint16_t wholePackLen; //当前解算的帧的总长度
static int32_t index = 0;	  //当前解算的帧在fullDataBuffer的索引
static u8 lastSeq;			  //上一次包序号

void Judgement::ringQueue()
{
	rx_len = JUDGE_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1) + judgementFullCount * JUDGE_BUFFER_LENGTH; //获取裁判当前已经接收的长度
	if (rx_len > read_len + JUDGE_BUFFER_LENGTH + 1)
		wrongStatusCnt.CYCLE_Wrong_cnt++; //如果读取的长度比接收的长度短一圈（少JUDGE_BUFFER_LENGTH），认为已经过圈，过圈计数器加一

	while (rx_len > read_len + 1) //当没读满时
	{
		int read_arr = read_len % JUDGE_BUFFER_LENGTH; //当前应该读取的字节在judgeDataBuffer的位置
		u8 byte = judgeDataBuffer[read_arr];		   //将该字节取出

		u8 dataLenFromCmdid;							//根据命令id获取长度

		/* 独立计算CRC，平摊算力 */
		if (crcCalcState == CRC8_CRC16_CALC_STATUS)
		{
			CRC8_Check = CRC8_TAB[CRC8_Check ^ (byte)];
			CRC16_Check = ((uint16_t)(CRC16_Check) >> 8) ^ wCRC_Table[((uint16_t)(CRC16_Check) ^ (uint16_t)(byte)) & 0x00ff];
		}

		if (crcCalcState == CRC16_CALC_STATUS)
			CRC16_Check = ((uint16_t)(CRC16_Check) >> 8) ^ wCRC_Table[((uint16_t)(CRC16_Check) ^ (uint16_t)(byte)) & 0x00ff];

		read_len++; //认为该字节已经读取，位置往后移

		switch (judgementStep)
		{
		case STEP_HEADER: //帧头
			if (index == 0)
			{
				if (byte != 0xA5)//错位
				{
					index = 0;
					crcCalcState = CRC8_CRC16_CALC_STATUS;
					CRC8_Check = 0xff;
					CRC16_Check = 0xffff;
					break;
				}
				fullDataBuffer[index++] = byte;
			}
			else if (index < 4)
			{
				fullDataBuffer[index++] = byte;
				if (index == 4) //包序号
				{
					lastSeq++;
					if (((FrameHeader *)fullDataBuffer)->seq != lastSeq) //上一帧的包序号不等于这一帧的包序号+1
					{
						wrongStatusCnt.SEQ_Wrong_cnt++; //认为包序号错误，计数器+1
						lastSeq = byte;
					}
					//包序号错误仍进行处理
					crcCalcState = CRC16_CALC_STATUS; //下一帧起不再计算CRC8
					judgementStep = STEP_HEADER_CRC8;
				}
			}
			break;
		case STEP_HEADER_CRC8:
			fullDataBuffer[index++] = byte;
			if (((FrameHeader *)fullDataBuffer)->crc8 != CRC8_Check) //CRC8校验正确
			{
				wrongStatusCnt.CRC8_Wrong_cnt++;	   //CRC8校验错误计数器+1
				judgementStep = STEP_HEADER;		   //从头开始
				index = 0;							   //清空fullDataBuffer索引
				crcCalcState = CRC8_CRC16_CALC_STATUS; //CRC8与CRC16都计算
				CRC8_Check = 0xff;					   //初始化CRC8校验
				CRC16_Check = 0xffff;				   //初始化CRC16校验
				break;
			}
			judgementStep = STEP_CMDID_GET; //获取命令码
			CRC8_Check = 0xff;				//初始化CRC8校验
			break;
		case STEP_CMDID_GET:
			fullDataBuffer[index++] = byte;
			if (index != 7) //未获取完命令码
				break;

			judgementStep = STEP_DATA_TRANSFER;
			dataLenFromCmdid= getLength((FrameHeader *)fullDataBuffer);

			wholePackLen = 7 + getLength((FrameHeader *)fullDataBuffer) + 2;
			if (dataLenFromCmdid == 0xFF || dataLenFromCmdid != ((FrameHeader *)fullDataBuffer)->dataLength) //命令错误
			{
				judgementStep = STEP_HEADER;		   //从头开始
				index = 0;							   //清空fullDataBuffer索引
				crcCalcState = CRC8_CRC16_CALC_STATUS; //CRC8与CRC16都计算
				CRC8_Check = 0xff;					   //初始化CRC8校验
				CRC16_Check = 0xffff;				   //初始化CRC16校验
			}
			break;
		case STEP_DATA_TRANSFER:
			fullDataBuffer[index++] = byte;
			if (index == (wholePackLen - 2)) //数据传输完，待校验
			{
				crcCalcState = CRC_NO_CALC_STATUS; //下帧起不计算CRC16，避免将帧尾的校验位也计算
				judgementStep = STEP_DATA_CRC16;
			}
			break;
		case STEP_DATA_CRC16:
			fullDataBuffer[index++] = byte;
			if (index == (wholePackLen))
			{
				uint8_t CRC16_char[2];
				CRC16_char[0] = (u8)(CRC16_Check & 0x00ff);
				CRC16_char[1] = (u8)((CRC16_Check >> 8) & 0x00ff);
				if (CRC16_char[0] == fullDataBuffer[index - 2] && CRC16_char[1] == fullDataBuffer[index - 1]) //CRC16校验通过
					getJudgeData();
				else
					wrongStatusCnt.CRC16_Wrong_cnt++; //CRC16校验失败则计数器+1

				//无论是否通过都重置
				judgementStep = STEP_HEADER;		   //从头开始
				index = 0;							   //清空fullDataBuffer索引
				crcCalcState = CRC8_CRC16_CALC_STATUS; //CRC8与CRC16都计算
				CRC8_Check = 0xff;					   //初始化CRC8校验
				CRC16_Check = 0xffff;				   //初始化CRC16校验
			}
			break;
		default:
			judgementStep = STEP_HEADER;		   //从头开始
			index = 0;							   //清空fullDataBuffer索引
			crcCalcState = CRC8_CRC16_CALC_STATUS; //CRC8与CRC16都计算
			CRC8_Check = 0xff;					   //初始化CRC8校验
			CRC16_Check = 0xffff;				   //初始化CRC16校验
			break;
		}
		rx_len = JUDGE_BUFFER_LENGTH - DMA_GetCurrDataCounter(DMA2_Stream1) +
			judgementFullCount * JUDGE_BUFFER_LENGTH; //重新获取已经接收的长度
	}
	if (rx_len % JUDGE_BUFFER_LENGTH > (JUDGE_BUFFER_LENGTH / 3) &&
		rx_len % JUDGE_BUFFER_LENGTH < (2 * JUDGE_BUFFER_LENGTH / 3)) //防止judgementFullCount溢出，过早清除与过晚清除都可能会导致包圈
	{
		read_len -= JUDGE_BUFFER_LENGTH * judgementFullCount;
		judgementFullCount = 0;
	}
}

/**
 * @brief 获取当前帧对应的长度
 *
 * @param frameHeader 帧头结构体
 * @return uint8_t 长度
 */
uint8_t Judgement::getLength(FrameHeader *frameHeader)
{
	switch (frameHeader->cmdid)
	{
	case STATUS_DATA:
		return sizeof(GameStatus);
	case RESULT_DATA:
		return sizeof(GameResult);
	case ROBOT_HP_DATA:
		return sizeof(GameRobotHP);
	case DART_STATUS:
		return sizeof(DartStatus);
	case ICRA_BUFF_DEBUFF_ZONE_STATUS:
		return sizeof(ICRA_BuffDebuffZoneStatus_t);
	case EVENT_DATA:
		return sizeof(EventData);
	case SUPPLY_PROJECTILE_ACTION:
		return sizeof(SupplyProjectileAction);
	case ROBOT_WARNING_DATA:
		return sizeof(RefereeWarning);
	case DART_REMAINING_TIME:
		return sizeof(DartRemainingTime);
	case GAME_ROBOT_STATUS:
		return sizeof(GameRobotStatus);
	case POWER_HEAT_DATA:
		return sizeof(PowerHeatData);
	case GAME_ROBOT_POS:
		return sizeof(GameRobotPos);
	case BUFF:
		return sizeof(Buff);
	case AERIAL_ROBOT_ENERGY:
		return sizeof(AerialRobotEnergy);
	case ROBOT_HURT:
		return sizeof(RobotHurt);
	case SHOOT_DATA:
		return sizeof(ShootData);
	case BULLET_REMAINING:
		return sizeof(BulletRemaining);
	case RFID_STATUS:
		return sizeof(RfidStatus);
	case DART_CLIENT_CMD:
		return sizeof(DartClientCmd);

	case STUDENT_INTERACTIVE_HEADER_DATA:
		return frameHeader->dataLength;
	}
	return 0xff;
}

/**
 * @brief 结构体复制
 *
 */
void Judgement::getJudgeData()
{
	unsigned char *judgeData_ADD;
#if USE_DMA_TRANSFER_DATA
	static NVIC_InitTypeDef NVIC_InitStructure;
	static DMA_InitTypeDef DMA_InitStructure;
	static u8 firstLoad = 1; //首次加载时加载配置缺省值
	if (firstLoad == 1)
	{
		firstLoad = 0;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)fullDataBuffer + sizeof(FrameHeader); //外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToMemory;										   //数据传输方向：外设到内存
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Enable;							   //外设地址递增
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;									   //内置地址递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;					   //数据宽度为八位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;							   //数据宽度为八位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;											   //不执行循环模式
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;										   //dma通道拥有高优先级

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream2_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		DMA_ITConfig(DMA2_Stream2, DMA_IT_TC, DISABLE); //失能传输完成中断中断
	}
#endif
	u16 cmdid = ((FrameHeader *)fullDataBuffer)->cmdid;
	switch (cmdid)
	{
	case STATUS_DATA:
		judgeData_ADD = (u8 *)&gameStatus;
		break; //10Hz
	case RESULT_DATA:
		judgeData_ADD = (u8 *)&gameResult;
		break;
	case ROBOT_HP_DATA:
		judgeData_ADD = (u8 *)&gameRobotHP;
		break;
	case DART_STATUS:
		judgeData_ADD = (u8 *)&dartStatus;
		break;
	case ICRA_BUFF_DEBUFF_ZONE_STATUS:
		judgeData_ADD = (u8 *)&ICRA_BuffDebuffZoneStatus;
		break;
	case EVENT_DATA:
		judgeData_ADD = (u8 *)&eventData;
		break; //50hZ
	case SUPPLY_PROJECTILE_ACTION:
		judgeData_ADD = (u8 *)&supplyProjectileAction;
		break; //10hz
	case ROBOT_WARNING_DATA:
		judgeData_ADD = (u8 *)&refereeWarning;
		break;
	case DART_REMAINING_TIME:
		judgeData_ADD = (u8 *)&dartRemainingTime;
		break;
	case GAME_ROBOT_STATUS:
		judgeData_ADD = (u8 *)&gameRobotStatus;
		break;
	case POWER_HEAT_DATA:
		judgeData_ADD = (u8 *)&powerHeatData;
		break; //50hz
	case GAME_ROBOT_POS:
		judgeData_ADD = (u8 *)&gameRobotPos;
		break;
	case BUFF:
		judgeData_ADD = (u8 *)&buff;
		break;
	case AERIAL_ROBOT_ENERGY:
		judgeData_ADD = (u8 *)&aerialRobotEnergy;
		break;
	case ROBOT_HURT:
		judgeData_ADD = (u8 *)&robotHurt;
		break;
	case SHOOT_DATA:
		judgeData_ADD = (u8 *)&shootData;
		break;
	case BULLET_REMAINING:
		judgeData_ADD = (u8 *)&bulletRemaining;
		break;
	case RFID_STATUS:
		judgeData_ADD = (u8 *)&rfidStatus;
		break;
	case DART_CLIENT_CMD:
		judgeData_ADD = (u8 *)&dartClientCmd;
		break;
	case STUDENT_INTERACTIVE_HEADER_DATA:
		judgeData_ADD = (u8 *)&studentRecviveData;
		break;
	default:
		return;
	}
#if USE_DMA_TRANSFER_DATA
	DMA_DeInit(DMA2_Stream2);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)judgeData_ADD;			 //内存基地址
	DMA_InitStructure.DMA_BufferSize = getLength((FrameHeader *)fullDataBuffer); //dma缓存大小
	DMA_Init(DMA2_Stream2, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream2, ENABLE);
	NVIC_Init(&NVIC_InitStructure);
#else
	memmove(judgeData_ADD, fullDataBuffer + sizeof(FrameHeader), getLength((FrameHeader *)fullDataBuffer));
#endif
	switch (cmdid)
	{
	case ROBOT_HURT:
		hurtCount++;
		break;
	case SHOOT_DATA:
		shootNum[shootData.shooterId-1]++;
	default:
		break;
	}

}

/**
 * @brief 针对0x301的联合体数据进行发送
 *
 * @param count
 */
void Judgement::customSend(u8 count)
{
	sendUnionData.frameHeader.sof = 0xa5;									//帧头
	sendUnionData.frameHeader.dataLength = count - sizeof(FrameHeader) - 2; //数据长度
	Append_CRC8_Check_Sum((unsigned char *)&sendUnionData, 5);				//CRC8校验
	sendUnionData.senderId = gameRobotStatus.robotId;						//发送方ID

	if (sendUnionData.receiverId != ClientId())						   //不是客户端
		sendUnionData.receiverId = IdToMate(sendUnionData.receiverId); //转化为友方

	Append_CRC16_Check_Sum((unsigned char *)&sendUnionData, count);

	uart6SendBytes(&sendUnionData, count);
}

/**
 * @brief 发送机器人交互数据
 *
 * @param dataLength 交互内容的长度
 */
void Judgement::robotsCommunication(uint16_t dataCmdId, RobotIdDef robotIdDef, u8 dataLength)
{
	sendUnionData.dataCmdId = dataCmdId;
	sendUnionData.receiverId = robotIdDef;
	int count = sizeof(FrameHeader) + 6 + dataLength + 2;
	sendUnionData.frameHeader.cmdid = 0x0301;
	//    unsigned char send_Buffer[17];
	customSend(count);
}

/**
 * @brief 发送客户端UI命令
 *
 * @param sendGraphics 一次发送的图形个数
 */
void Judgement::graphicDraw(u8 sendGraphics)
{
	if (sendGraphics <= 1)
	{
		sendUnionData.dataCmdId = 0x101;
		sendGraphics = 1;
	}
	else if (sendGraphics <= 2)
	{
		sendUnionData.dataCmdId = 0x102;
		sendGraphics = 2;
	}
	else if (sendGraphics <= 5)
	{
		sendUnionData.dataCmdId = 0x103;
		sendGraphics = 5;
	}
	else if (sendGraphics <= 7)
	{
		sendUnionData.dataCmdId = 0x104;
		sendGraphics = 7;
	}
	sendUnionData.frameHeader.cmdid = 0x0301;

	int count = sizeof(FrameHeader) + 6 + sendGraphics * sizeof(GraphicDataStruct) + 2;
	sendUnionData.receiverId = ClientId();
	sendUnionData.senderId = gameRobotStatus.robotId;

	customSend(count);
}

/**
 * @brief 字符绘制
 *
 */
void Judgement::characterDraw()
{
	sendUnionData.frameHeader.cmdid = 0x0301;
	sendUnionData.dataCmdId = 0x110;
	int count = sizeof(FrameHeader) + 6 + sizeof(GraphicDataStruct) + 30 + 2;
	sendUnionData.receiverId = ClientId();
	sendUnionData.senderId = gameRobotStatus.robotId;

	customSend(count);
}

/**
 * @brief 删除图形
 *
 */
void Judgement::graphicDel()
{
	sendUnionData.frameHeader.cmdid = 0x0301;
	sendUnionData.dataCmdId = 0x100;
	int count = sizeof(FrameHeader) + 6 + sizeof(ClientCustomGraphicDelete) + 2;
	sendUnionData.receiverId = ClientId();
	sendUnionData.senderId = gameRobotStatus.robotId;

	customSend(count);
}

/**
 * @brief 发送地图命令
 *
 */
void Judgement::mapCommandSend()
{
	mapCommandData.frameHeader.cmdid = 0x0303;
	int count = sizeof(FrameHeader) + sizeof(MapCommand) + 2;

	mapCommandData.frameHeader.sof = 0xa5;
	mapCommandData.frameHeader.dataLength = count - sizeof(FrameHeader) - 2;
	Append_CRC8_Check_Sum((unsigned char *)&mapCommandData, 5);
	Append_CRC16_Check_Sum((unsigned char *)&mapCommandData, count);

	uart6SendBytes(&mapCommandData, count);
}

/**
 * @brief 发送多字节数据（采用DMA）
 *
 * @param ptr 地址
 * @param len 长度
 */
void Judgement::uart6SendBytes(void *ptr, u8 len)
{
	static DMA_InitTypeDef DMA_InitStructure;
	static NVIC_InitTypeDef NVIC_InitStructure;
	static u8 firstLoad = 1; //首次加载时加载配置缺省值
	if (firstLoad == 1)
	{
		firstLoad = 0;
		DMA_InitStructure.DMA_Channel = DMA_Channel_5;
		DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) & (USART6->DR); //外设基地址
		DMA_InitStructure.DMA_DIR = DMA_DIR_MemoryToPeripheral;				  //数据传输方向：内存到外设
		//dma缓存大小
		DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不变
		DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;					//内置地址寄存器递增
		DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte; //数据宽度为八位
		DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//数据宽度为八位
		DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;							//工作模式为环形
		DMA_InitStructure.DMA_Priority = DMA_Priority_High;						//dma通道拥有高优先级

		NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream7_IRQn;
		NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 3;
		NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
		NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;

		DMA_ITConfig(DMA2_Stream7, DMA_IT_TC, DISABLE);
	}
	DMA_Cmd(DMA2_Stream7, DISABLE);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)ptr; //内存基地址
	DMA_ClearFlag(DMA2_Stream7, DMA_FLAG_TCIF7);		   //|DMA_FLAG_FEIF7|DMA_FLAG_HTIF7);
	DMA_InitStructure.DMA_BufferSize = wholePackLen;	   //dma缓存大小
	DMA_Init(DMA2_Stream7, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream7, ENABLE);
	NVIC_Init(&NVIC_InitStructure);
}

/**
 * @brief 将ID枚举转换为友方机器人id
 *
 * @param robotIdDef ID枚举
 * @return uint16_t 友方机器人id
 */
uint16_t Judgement::IdToMate(RobotIdDef robotIdDef)
{
	if (gameRobotStatus.robotId > RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF;
	}
	return (uint16_t)robotIdDef;
}

/**
 * @brief 将机器人id转换为友方机器人id
 *
 * @param robotId 有效的机器人id
 * @return uint16_t 友方机器人id
 */
uint16_t Judgement::IdToMate(uint16_t robotId)
{
	return IdToMate(RobotIdDef(robotId % RED_BLUE_ID_DIFF));
}

/**
 * @brief 根据自身id返还客户端id
 *
 * @return uint16_t 客户端id
 */
uint16_t Judgement::ClientId()
{
	return gameRobotStatus.robotId + ROBOT_CLIENT_ID_DIFF;
}

/**
 * @brief 将ID枚举转换为敌方机器人id
 *
 * @param robotIdDef ID枚举
 * @return uint16_t 敌方机器人id
 */
uint16_t Judgement::IdToEnemy(RobotIdDef robotIdDef)
{
	if (gameRobotStatus.robotId < RED_BLUE_ID_DIFF)
	{
		return (uint16_t)robotIdDef + RED_BLUE_ID_DIFF;
	}
	return robotIdDef;
}

/**
 * @brief 将机器人id转换为敌方机器人id
 *
 * @param robotId 有效的机器人id
 * @return uint16_t 敌方机器人id
 */
uint16_t Judgement::IdToEnemy(uint16_t robotId)
{
	return IdToEnemy(RobotIdDef(robotId % RED_BLUE_ID_DIFF));
}

#endif