#include "dbug_task.h"
/**FreeRTOS****************/
#include "FreeRTOS.h"					//FreeRTOS使用	 
#include "task.h"
#include "queue.h"
/**TDT_Task*********************/
#include "my_task.h"
/****TDT_Device*****/
#include "oled.h"

/*
初始化函数

#include "oled.h"

	OLED_GPIO_Init();
	LCD_Init(0);	
	LCD_Init(1);	//如果只有一个OLED，此句忽略


*/

/*调试模块关键设置*/
#define DBUG			//调试模式,注释即关闭

#define TASK_NUM  20	//当前任务数量，关系到变量大小分配，可大不可小，否则HardFault，最小为3

#define TASKS_PER_PAGE 6  //一页显示的任务数

char publicChar[27]="Hello world";	/*公用字符串：长度为一行显示的最多字符串（实际多一点）：用于宏：LCD显示时字符的临时存储*/
u16 OledDbugRunCnt=0;	/*Dbug时钟，每运行一次加一*/
TaskStatus_t IdleTaskStatus;	//空闲任务数据，用于获取CPU占用率


void Dbug_Task(void *pvParameters)
{
	u8 nowPage=1;//当前页码，
	u8 allPage=1;//总页数
	
	//OLED初始化
	OLED_GPIO_Init();
	LCD_Init(1);
	LCD_Init(0);
	while(1){
		/*Dbug时钟-重要*/
		OledDbugRunCnt++;
		/*任务数量，用于顶部信息区显示和for循环，可低不可高，包括空闲任务和Tim*/
		u8 taskNum=uxTaskGetNumberOfTasks();
		if(taskNum>TASK_NUM)
		{
			LCD(0,0,"taskNum>TASK_NUM!!!  ");
			return;
		}
		//获取当前任务信息
		vTaskGetInfo(xTaskGetIdleTaskHandle(),&IdleTaskStatus,pdTRUE,eInvalid);
		//得到页数
		allPage=(taskNum/(TASKS_PER_PAGE+1)) +1;
		//表格头区---任务 状态 水位 占比
		LCD(0,1,"Task  S Hwm CPU");
		//一个大周期一次性获取所有数据，免得系统皮
		char LineChar[TASK_NUM][25];
		if(OledDbugRunCnt%(TASKS_PER_PAGE*allPage)==1)
		{
			mTaskGetLcdInfo((char*)LineChar[0]);
		}
		if(OledDbugRunCnt%TASKS_PER_PAGE==1)
		{
			nowPage=(OledDbugRunCnt/TASKS_PER_PAGE)%allPage+1;
			for(u8 i=0;i<TASKS_PER_PAGE;i++)
			{
				if(i<(taskNum-(nowPage-1)*TASKS_PER_PAGE))
				{
					LCD(0,2+i,"%s",LineChar[i+(nowPage-1)*TASKS_PER_PAGE]);
				}else
				{
					OLED_CS(0);
					LCD_CLS_y((2+i)*8);//清除旧行
				}
			}
		}
		//顶部信息栏-涉及到页码刷新所以放到最后执行
		const char signCh[]={'M','T','A','O'};//虽然理论上有现场恢复，但是还是发现部分变量被修改，所以部分变量实时定义
		LCD(0,0,"%cT:%ld C:%.0f%% P:%d/%d M ",OledDbugRunCnt%2==0?' ':'*',uxTaskGetNumberOfTasks(),100.0f-((float)IdleTaskStatus.ulRunTimeCounter/FreeRTOSRunTimeTicks)*100,nowPage,allPage); 
		
		vTaskDelay(pdMS_TO_TICKS(500));//操作系统延时还算靠谱
    }
}









