#ifndef __DBUG_TASK_H__
#define __DBUG_TASK_H__

#include "board.h"


extern char publicChar[27];

void Dbug_Img(u16 Camera_Data);
void OLED_GPIO_Init(void);

#ifdef __cplusplus
 extern "C" {
#endif	/*__cplusplus*/

void USART1_IRQHandler(void);
	 
#ifdef __cplusplus
}
#endif	/*__cplusplus*/
#endif
