/**
  ******************************************************************************
  * @file    Project/STM32F4xx_StdPeriph_Templates/stm32f4xx_it.c 
  * @author  MCD Application Team
  * @version V1.8.0
  * @date    04-November-2016
  * @brief   Main Interrupt Service Routines.
  *          This file provides template for all exceptions handler and 
  *          peripherals interrupt service routine.
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
  *
  * Licensed under MCD-ST Liberty SW License Agreement V2, (the "License");
  * You may not use this file except in compliance with the License.
  * You may obtain a copy of the License at:
  *
  *        http://www.st.com/software_license_agreement_liberty_v2
  *
  * Unless required by applicable law or agreed to in writing, software 
  * distributed under the License is distributed on an "AS IS" BASIS, 
  * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  * See the License for the specific language governing permissions and
  * limitations under the License.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_it.h"
#include "board.h"
#include "FreeRTOS.h"					//FreeRTOS使用	
#include "task.h"
/** @addtogroup Template_Project
  * @{
  */

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
/* Private function prototypes -----------------------------------------------*/
/* Private functions ---------------------------------------------------------*/

/******************************************************************************/
/*            Cortex-M4 Processor Exceptions Handlers                         */
/******************************************************************************/

/**
  * @brief  This function handles NMI exception.
  * @param  None
  * @retval None
  */
void NMI_Handler(void)
{
}

/**
  * @brief  This function handles Hard Fault exception.
  * @param  None
  * @retval None
  */
void HardFault_Handler(void)
{
  /* Go to infinite loop when Hard Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Memory Manage exception.
  * @param  None
  * @retval None
  */
void MemManage_Handler(void)
{
  /* Go to infinite loop when Memory Manage exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Bus Fault exception.
  * @param  None
  * @retval None
  */
void BusFault_Handler(void)
{
  /* Go to infinite loop when Bus Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles Usage Fault exception.
  * @param  None
  * @retval None
  */
void UsageFault_Handler(void)
{
  /* Go to infinite loop when Usage Fault exception occurs */
  while (1)
  {
  }
}

/**
  * @brief  This function handles SVCall exception.
  * @param  None
  * @retval None
  */
//void SVC_Handler(void)
//{
//}

/**
  * @brief  This function handles Debug Monitor exception.
  * @param  None
  * @retval None
  */
void DebugMon_Handler(void)
{
}

/**
  * @brief  This function handles PendSVC exception.
  * @param  None
  * @retval None
  */
//void PendSV_Handler(void)
//{
//}

/**
  * @brief  This function handles SysTick Handler.
  * @param  None
  * @retval None
  */
extern u8 Init_OK;
extern void xPortSysTickHandler(void);

void SysTick_Handler(void)
{
	 if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//FreeRTOS系统已经运行
    {
        xPortSysTickHandler();	
    }
	sysTickUptime++;
	static u8 upTimeCnt=0;
	if(++upTimeCnt==4)
	{
		sysTickUptimeUs++;
		upTimeCnt=0;
	}
	
	#if 0
	extern void testMotor(void);
	static int loop5=0;
	loop5++;
	if(loop5/MPRE >= 5)
	{
		testMotor();
		loop5 = 0;
	}
	#endif
	#ifdef _SCHEDULE_H
		
		 static schedule loop;
		
		loop.cnt_1ms++;
		loop.cnt_2ms++;
		loop.cnt_5ms++;
		loop.cnt_10ms++;
		loop.cnt_20ms++;
		loop.cnt_50ms++;
		if(0)
		{
			if(loop.cnt_1ms/MPRE >= 1)
			{
				TDT_Loop_1000Hz();	
				loop.cnt_1ms = 0;
			}  
			if(loop.cnt_2ms/MPRE >= 2)
			{
				TDT_Loop_500Hz();
				loop.cnt_2ms = 0;
			}
			if(loop.cnt_5ms/MPRE >= 5)
			{	
				TDT_Loop_200Hz();
				loop.cnt_5ms = 0;
			}
			if(loop.cnt_10ms/MPRE >= 10)
			{
				TDT_Loop_100Hz();
				loop.cnt_10ms = 0;
			}
			if(loop.cnt_20ms/MPRE >= 20)
			{
				TDT_Loop_50Hz();
				loop.cnt_20ms = 0;
			}
			if(loop.cnt_50ms/MPRE >= 50)
			{		
				TDT_Loop_20Hz();
				loop.cnt_50ms = 0;
			}
		}

	#endif

}

/******************************************************************************/
/*                 STM32F4xx Peripherals Interrupt Handlers                   */
/*  Add here the Interrupt Handler for the used peripheral(s) (PPP), for the  */
/*  available peripheral interrupt handler's name please refer to the startup */
/*  file (startup_stm32f4xx.s).                                               */
/******************************************************************************/

/**
  * @brief  This function handles PPP interrupt request.
  * @param  None
  * @retval None
  */
/*void PPP_IRQHandler(void)
{
}*/

/**
  * @}
  */ 


/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
