#include "flash_var.h"
#include "stm32f4xx_flash.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS使用
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/

void FlashSave_Task(void *pvParameters)
{
	while(1)
	{
		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY))
		{
			IFlash.saving = true;
			IFlash.save();
			IFlash.saving = false;
		}

	}
}

/**
 * @brief 读取指定地址的半字(16位数据)
 * @param  faddr            读取的地址
 * @return u32 				对应数据
 */
u32 STMFLASH_ReadWord(u32 faddr)
{
	return *(vu32 *)faddr;
}

/**
 * @brief 获取某个地址所在的flash扇区
 * @param  addr             flash地址
 * @return uint16_t 		0~11,即addr所在的扇区
 */
uint16_t STMFLASH_GetFlashSector(u32 addr)
{
	if (addr < ADDR_FLASH_SECTOR_1)
		return FLASH_Sector_0;
	if (addr < ADDR_FLASH_SECTOR_2)
		return FLASH_Sector_1;
	if (addr < ADDR_FLASH_SECTOR_3)
		return FLASH_Sector_2;
	if (addr < ADDR_FLASH_SECTOR_4)
		return FLASH_Sector_3;
	if (addr < ADDR_FLASH_SECTOR_5)
		return FLASH_Sector_4;
	if (addr < ADDR_FLASH_SECTOR_6)
		return FLASH_Sector_5;
	if (addr < ADDR_FLASH_SECTOR_7)
		return FLASH_Sector_6;
	if (addr < ADDR_FLASH_SECTOR_8)
		return FLASH_Sector_7;
	if (addr < ADDR_FLASH_SECTOR_9)
		return FLASH_Sector_8;
	if (addr < ADDR_FLASH_SECTOR_10)
		return FLASH_Sector_9;
	if (addr < ADDR_FLASH_SECTOR_11)
		return FLASH_Sector_10;
	return FLASH_Sector_11;
}

/**
 * @brief  从指定地址开始写入指定长度的数据
 * @param  WriteAddr        起始地址(此地址必须为4的倍数!!)
 * @param  pBuffer          数据指针
 * @param  NumToWrite       字(32位)数(就是要写入的32位数据的个数.)
 * @note   //特别注意:因为STM32F4的扇区实在太大,没办法本地保存扇区数据,所以本函数
 *         写地址如果非0XFF,那么会先擦除整个扇区且不保存扇区数据.所以
 *         写非0XFF的地址,将导致整个扇区数据丢失.建议写之前确保扇区里
 *         没有重要数据,最好是整个扇区先擦除了,然后慢慢往后写.
 */
void STMFLASH_Write(u32 WriteAddr, u32 *pBuffer, u32 NumToWrite)
{
	FLASH_Status status = FLASH_COMPLETE;
	u32 addrx = 0;
	u32 endaddr = 0;
	if (WriteAddr < STM32_FLASH_BASE || WriteAddr % 4)
		return;					 //非法地址
	FLASH_Unlock();				 //解锁
	FLASH_DataCacheCmd(DISABLE); //FLASH擦除期间,必须禁止数据缓存

	addrx = WriteAddr;					  //写入的起始地址
	endaddr = WriteAddr + NumToWrite * 4; //写入的结束地址
	if (addrx < 0X1FFF0000)				  //只有主存储区,才需要执行擦除操作!!
	{
		while (addrx < endaddr) //扫清一切障碍.(对非FFFFFFFF的地方,先擦除)
		{
			if (STMFLASH_ReadWord(addrx) != 0XFFFFFFFF) //有非0XFFFFFFFF的地方,要擦除这个扇区
			{
				status = FLASH_EraseSector(STMFLASH_GetFlashSector(addrx), VoltageRange_3); //VCC=2.7~3.6V之间!!
				if (status != FLASH_COMPLETE)
					break; //发生错误了
			}
			else
				addrx += 4;
		}
	}
	if (status == FLASH_COMPLETE)
	{
		while (WriteAddr < endaddr) //写数据
		{
			if (FLASH_ProgramWord(WriteAddr, *pBuffer) != FLASH_COMPLETE) //写入数据
			{
				break; //写入异常
			}
			WriteAddr += 4;
			pBuffer++;
		}
	}
	FLASH_DataCacheCmd(ENABLE); //FLASH擦除结束,开启数据缓存
	FLASH_Lock();				//上锁
}

/**
 * @brief 从指定地址开始读出指定长度的数据
 * @param  ReadAddr         起始地址
 * @param  pBuffer          数据指针
 * @param  NumToRead        字(4位)数
 */
void STMFLASH_Read(u32 ReadAddr, u32 *pBuffer, u32 NumToRead)
{
	u32 i;
	for (i = 0; i < NumToRead; i++)
	{
		pBuffer[i] = STMFLASH_ReadWord(ReadAddr); //读取4个字节.
		ReadAddr += 4;							  //偏移4个字节.
	}
}

/*保存至存储介质*/
void SaveToMEMORY(u32 *pBuff, u32 size)
{
	STMFLASH_Write(InnerSECTOR, pBuff, size);
}

/**
 * @brief 从存储介质读出，返回new创建的数组指针
 * @return u32* 			返回创建的数组指针
 * @note
 */
u32 *ReadFromMEMORY()
{
	u32 size = 0;
	STMFLASH_Read(InnerSECTOR, &size, 1);
	if (size == 0 || size == 0xFFFFFFFF)
		return NULL;
	u32 *pBuff = new u32[size];
	STMFLASH_Read(InnerSECTOR, pBuff, size);
	return pBuff;
}

void (*FlashVar::pSaveFunc)(u32 *saveBuffer, u32 buffsize) = SaveToMEMORY;
u32 *(*FlashVar::pReadFunc)() = ReadFromMEMORY;

FlashVar IFlash;
