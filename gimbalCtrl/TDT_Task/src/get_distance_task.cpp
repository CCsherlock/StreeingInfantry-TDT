#include "get_distance_task.h"
#include "steeringTop_task.h"
/**FreeRTOS*START***************/
#include "FreeRTOS.h" //FreeRTOS
#include "timers.h"
#include "list.h"
#include "queue.h"
#include "task.h"
/**FreeRTOS*END***************/

autoIntoStation::autoIntoStation()
{

}
/**
 * @brief GY53 初始化
 *
 */
void autoIntoStation::senserInit()
{
	gy53Front = new gy_53(USART1,9600);	//LF
	gy53Left1 = new gy_53(USART3,9600);	//LB
	gy53Left2 = new gy_53(USART6,9600);	//FF

	gy53Front->gy_53_Init();
	gy53Left1->gy_53_Init();
	gy53Left2->gy_53_Init();
}
/**
 * @brief 获取当前距离
 *
 */
void autoIntoStation::getDistance()
{
	distance[LF] = GY_53distence[LF];
	distance[LB] = GY_53distence[LB];
	distance[FF] = GY_53distence[FF];
}
/**
 * @brief 判断当前运行状态
 *
 */
void autoIntoStation::autoGotoStation()
{
	judgeSpeed[YAW] = MAX_CHASSIS_VW_SPEED*0.5f;
	judgeSpeed[DISY] = MAX_CHASSIS_VY_SPEED*0.5f;
	judgeSpeed[DISX] = MAX_CHASSIS_VX_SPEED*0.5f;
	SideDistance = (distance[LF]+distance[LB]);
	if(judgeStep>2)
	{
		autoIntoStationFlag = 0;
		return;
	}
	switch(judgeStep)
	{
		case YAW:
			if(distance[LF] - distance[LB]>threShold[YAW])
				SteeringTop.customSpeedIn.data[2] = judgeSpeed[YAW];
			else if(distance[LF] - distance[LB]<-threShold[YAW])
				SteeringTop.customSpeedIn.data[2] = -judgeSpeed[YAW];
			else
				judgeStep++;
			break;
		case DISY:
			if(SideDistance>threShold[DISY]+20)
				SteeringTop.customSpeedIn.data[1] = -judgeSpeed[DISY];
			else if(SideDistance<threShold[DISY]-20)
				SteeringTop.customSpeedIn.data[1] = judgeSpeed[DISY];
			else
				judgeStep++;
			break;
		case DISX:
			if(distance[FF]>threShold[DISX]+20)
				SteeringTop.customSpeedIn.data[0] = -judgeSpeed[DISX];
			else if(distance[FF]<threShold[DISX]-20)
				SteeringTop.customSpeedIn.data[0] = judgeSpeed[DISX];
			else
				judgeStep++;
			break;
		default:
			break;
	}
}
autoIntoStation intoStation;
void GetGY53_Task(void *pvParameters)
{
	intoStation.senserInit();
	while(1)
	{
		if(intoStation.autoIntoStationFlag != 0)
		{
			intoStation.autoGotoStation();
		}
		vTaskDelay(pdMS_TO_TICKS(5));
	}
}