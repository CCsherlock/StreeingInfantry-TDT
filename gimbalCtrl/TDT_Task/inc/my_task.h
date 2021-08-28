#ifndef __MY_TASK_H__
#define __MY_TASK_H__

#include "board.h"

#ifdef __cplusplus
 extern "C" {
#endif	/*__cplusplus*/


/*自定义任务创建函数，用于创建任务，参数c：capital全大写；l：lowercase首字母大写*/
//	 	mTaskCreate(START,Start);
#define mTaskCreate(c,l)    xTaskCreate((TaskFunction_t )l##_Task,		\
										(const char*    )#l,		\
										(uint16_t       )c##_STK_SIZE,	\
										(void*          )NULL,			\
										(UBaseType_t    )c##_TASK_PRIO,	\
										(TaskHandle_t*  )&l##Task_Handler)
/*获取任务详细信息*/
//特殊用法，设置二级转换宏，用于展开参数		
#define mTaskGetInfo(x)	_mTaskGetInfo(x)
#define _mTaskGetInfo(x)	vTaskGetInfo((TaskHandle_t)x##Task_Handler,	\
							(TaskStatus_t*)&x##TaskStatus,	\
							(BaseType_t)pdTRUE,			\
							(eTaskState)eInvalid);		

//由于滴答定时器中断修改，定义如下以宏，将ms转换为节拍
//或者使用pdMS_TO_TICKS()将时间转换为节拍
#ifndef MPRE
	#error Please define MPRE at first.							
#endif
//任务延时函数
#define mTaskDelay(x) vTaskDelay(x*MPRE)
//将ms转换为节拍	
#define MS(x) x*MPRE
//定时器
#define mTimerCreate(a,b,c,d,e) xTimerCreate(a,b*MPRE,c,d,e)
	
/////////////////DBUG调试用

/*宏：用于打印信息到lcd，参数y：行数0-7，a字符串指针*/
#define LCD_P(y,a)   LCD_Print(0, (y)*8,(u8*)a,TYPE6X8,TYPE6X8);
/*宏：配合sprintf函数，打印信息。参数y：行数，...此处用法与printf相同*/
#define LCD(ID,y,...) 	OLED_CS(ID);sprintf(publicChar,__VA_ARGS__); LCD_P(y,publicChar);OLED_CS(!ID);


//LCD显示任务栈深度高水位线 参数：a任务摘要，x显示的x坐标，y显示的行数0-7						
#define Dbug_LCD_HWM(a,x,y)		extern TaskHandle_t a##Task_Handler;	\
								char a##HWM[20]="";				\
								sprintf(a##HWM,#a" HWM:%ld",uxTaskGetStackHighWaterMark(a##Task_Handler));	\
								LCD_Print(x, y*8,(u8*)a##HWM,TYPE6X8,TYPE6X8);

/*串口调试用*/
//输出任务栈深度高水位线
#define Dbug_Printf_HWM(x) 				extern TaskHandle_t x##Task_Handler;	\
										printf(#x"水位线%ld\r\n",uxTaskGetStackHighWaterMark(x##Task_Handler));
										



#ifdef __cplusplus
}
#endif	/*__cplusplus*/
#endif
