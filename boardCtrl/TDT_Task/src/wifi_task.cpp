/****************************** 
File name: TDT_Bsp\src\usart.cpp 
Description: 串口 
function: 
	—————————————————————————————————————————————————————————————————————————— 
 
	—————————————————————————————————————————————————————————————————————————— 
Author: 祖传 
Version: 1.1.1.191112_alpha 
Date: 19.11.12 
History: 
	—————————————————————————————————————————————————————————————————————————— 
	19.11.12 首次完成 
	—————————————————————————————————————————————————————————————————————————— 
****************************  */ 
#include "wifi_task.h" 
/**FreeRTOS*START***************/ 
#include "FreeRTOS.h"					//FreeRTOS使用	  
#include "timers.h" 
#include "list.h" 
#include "queue.h" 
#include "task.h" 
/**FreeRTOS*END***************/ 
#include "imu.h" 
#include "mpu6050.h" 
  
extern   Mpu6050 Mpu6050_Top; 
 
void usart_monitor(void); 

ESP8266_SendStruct_t esp8266_SendStruct; 
ESP8266_RequestStruct_t ESP8266_RequestStruct,ESP8266_ChangeStruct; 
const u8 usart_SendStructLenth = sizeof(ESP8266_SendStruct_t); 
#define ESP8266_RECV_LENTH 80 
char esp8266_Recv_Buffer[ESP8266_RECV_LENTH] = "";

//Wi-fi名称
const char SSID[] = "MERCURY_E17E";
//Wi-fi 密码
const char PWD[] = ""; //
//udp 客户端IP
const char UDP_IP[] = "219.216.98.137";
//udp 客户端端口
const char UDP_Port[] = "2333";

/**
 * @brief  发送内容填充函数
 * @param  cmd              根据不同命令发送对应的值
 */
void usart_monitor_GetValue(u8 cmd) 
{ 
    if(cmd == 0)//发送接收的数据 
    { 
        esp8266_SendStruct.frameHeader = 0xa5;
        /*========================↓Replace↓========================*/
        esp8266_SendStruct.sysUptime = sysTickUptimeUs;
		esp8266_SendStruct.gimbalTopAngle_Pitch = Mpu6050_Top.Angle.pitch; 
		esp8266_SendStruct.gimbalTopAngle_Yaw = Mpu6050_Top.Angle.yaw; 
		esp8266_SendStruct.gimbalTopAngle_Roll = Mpu6050_Top.Angle.roll;
        /*========================↑Replace↑========================*/
    } 
    else if(cmd == 1)//发送当前数据供上位机参考 
    { 
        ESP8266_RequestStruct.frameHeader = 0x5a;
        /*========================↓Replace↓========================*/
        //        ESP8266_RequestStruct.pitchKp = chassis.pidAnglePara.kp;
        //        ESP8266_RequestStruct.pitchKd = chassis.pidAnglePara.kd;
        //        ESP8266_RequestStruct.yawKp = chassis.pidYawPara.kp;
        //        ESP8266_RequestStruct.yawKi = chassis.pidYawPara.ki;
        //        ESP8266_RequestStruct.yawIntegralMax = chassis.pidYawPara.integralErrorMax;
        //        ESP8266_RequestStruct.yawKd = chassis.pidYawPara.kd;
        //        ESP8266_RequestStruct.speedKp = chassis.pidSpeedPara.kp;
        //        ESP8266_RequestStruct.speedKi = chassis.pidSpeedPara.ki;
        //        ESP8266_RequestStruct.speedIntegralMax = chassis.pidSpeedPara.integralErrorMax;
        //        ESP8266_RequestStruct.speedKd = chassis.pidSpeedPara.kd;
        //        ESP8266_RequestStruct.speedSetAmp = chassis.speedSetAmp;
        //        ESP8266_RequestStruct.yawSetAmp = chassis.yawSetAmp;
        //        ESP8266_RequestStruct.gyroKp = Kp;
        //        ESP8266_RequestStruct.gyroKi = Ki;
        //        ESP8266_RequestStruct.deforceAngle = chassis.deforceAngle;
        /*========================↑Replace↑========================*/
    } 
} 

/**
 * @brief  从上位机获取更新值并更新
 */
void recv_dataChange() 
{ 
    memcpy(&ESP8266_ChangeStruct,esp8266_Recv_Buffer,sizeof(ESP8266_ChangeStruct));
    /*========================↓Replace↓========================*/
    //    chassis.pidAnglePara.kp = ESP8266_ChangeStruct.pitchKp;
    //    chassis.pidAnglePara.kd = ESP8266_ChangeStruct.pitchKd;
    //    chassis.pidYawPara.kp = ESP8266_ChangeStruct.yawKp;
    //    chassis.pidYawPara.ki = ESP8266_ChangeStruct.yawKi;
    //    chassis.pidYawPara.integralErrorMax = ESP8266_ChangeStruct.yawIntegralMax;
    //    chassis.pidYawPara.kd = ESP8266_ChangeStruct.yawKd;
    //    chassis.pidSpeedPara.kp = ESP8266_ChangeStruct.speedKp;
    //    chassis.pidSpeedPara.ki = ESP8266_ChangeStruct.speedKi;
    //    chassis.pidSpeedPara.integralErrorMax = ESP8266_ChangeStruct.speedIntegralMax;
    //    chassis.pidSpeedPara.kd = ESP8266_ChangeStruct.speedKd;
    //    chassis.speedSetAmp = ESP8266_ChangeStruct.speedSetAmp;
    //    chassis.yawSetAmp = ESP8266_ChangeStruct.yawSetAmp;
    //    Kp = ESP8266_ChangeStruct.gyroKp;
    //    Ki = ESP8266_ChangeStruct.gyroKi;
    //    chassis.deforceAngle = ESP8266_ChangeStruct.deforceAngle;
    /*========================↑Replace↑========================*/
} 
 
 
/**
 * @brief wifi交互任务
 * @param  pvParameters     任务参数
 */
void Wifi_Task(void *pvParameters) 
{ 
	TDT_usart_monitor_Init(); 
	while(1) 
	{ 
		usart_monitor(); 
		//此处为较为稳定的延迟，如果网速好可以减少此延时，若过小则受到的数据大受影响
		//正常情况数据接收周期与延时对应周期相等
		vTaskDelay(pdMS_TO_TICKS(25)); 
	} 
} 
 
//8266 连接状态
enum ESP8266_Link_Status_t 
{ 
    DISCONNECT,//未连接 
    CONNECTED,//已连接但未确认wifi名称 
    RIGHT_CONNECT,//已连接且确认wifi名称 
    CIPMODE_SUCCESS,//已经开启透传模式 
    CIPSTART_SUCCESS,//已经连接UDP IP 
    CIPSEND_SUCCESS//已经开启透传模式 
}ESP8266_Link_Status = CIPSEND_SUCCESS; 

//esp8266 在忙标志位
u8 busy_Flag; 
//esp8266 最大失败重试次数
const u8 fail_CNT_MAX = 10; 
u8 fail_CNT = 0; 

//esp8266 需要发送的命令/数据
enum ESP8266_Send_Status_t 
{ 
    SEND_QUITAP,//退出连接AP 
    SEND_JOINAP,//发送连接连接AP 
    SEND_CHECKAP,//检查连接的AP是否正确 
    SEND_CIPMODE,//透传模式 
    SEND_CIPSTART,//开启连接UDP IP 
    SEND_CIPSEND,//开启透传模式 
    SEND_DATA,//发送数据 
    SEND_REQUEST,//发送回传数据 
    SEND_CWMODE,//发送应用模式 
    SEND_RST,//发送重启 
    SEND_QUITCIPSEND//退出透传模式 
}ESP8266_Send_Status = SEND_QUITCIPSEND; 

DMA_InitTypeDef   DMA_InitStructure_Tx, DMA_InitStructure_Rx; 

//命令发送数组
char esp8266_cmd_sendBuff[100]; 

 
void TDT_usart_monitor_Init(void) 
{ 
    USART_InitTypeDef USART_InitStructure; 
    GPIO_InitTypeDef  GPIO_InitStructure; 
    NVIC_InitTypeDef NVIC_InitStructure; 
 
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC | RCC_AHB1Periph_DMA2,ENABLE); 
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6,ENABLE); 
 
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource6,GPIO_AF_USART6); 
    GPIO_PinAFConfig(GPIOC,GPIO_PinSource7,GPIO_AF_USART6); 
 
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7|GPIO_Pin_6; 
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF; 
    GPIO_InitStructure.GPIO_OType = GPIO_OType_OD; 
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_100MHz; 
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL; 
    GPIO_Init(GPIOC,&GPIO_InitStructure); 
 
 
    USART_DeInit(USART6); 
    USART_InitStructure.USART_BaudRate = 4608000; 
    USART_InitStructure.USART_WordLength = USART_WordLength_8b; 
    USART_InitStructure.USART_StopBits = USART_StopBits_1; 
    USART_InitStructure.USART_Parity = USART_Parity_No; 
    USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx; 
    USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; 
    USART_Init(USART6,&USART_InitStructure); 
    USART_ITConfig(USART6,USART_IT_IDLE,ENABLE); 
    USART_Cmd(USART6,ENABLE); 
    USART_DMACmd(USART6,USART_DMAReq_Rx|USART_DMAReq_Tx,ENABLE); 
 
    /*接收DMA配置*/ 
    DMA_InitStructure_Rx.DMA_Channel= DMA_Channel_5; 
    DMA_InitStructure_Rx.DMA_PeripheralBaseAddr =  (uint32_t)&(USART6->DR);		//外设基地址 
    DMA_InitStructure_Rx.DMA_DIR = DMA_DIR_PeripheralToMemory;		//数据传输方向：内存到外设 
    DMA_InitStructure_Rx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不变 
    DMA_InitStructure_Rx.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内置地址寄存器递增 
    DMA_InitStructure_Rx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为八位 
    DMA_InitStructure_Rx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//数据宽度为八位 
    DMA_InitStructure_Rx.DMA_Mode = DMA_Mode_Normal;					//工作模式为环形 
    DMA_InitStructure_Rx.DMA_Priority = DMA_Priority_High;			//dma通道拥有高优先级 
    DMA_InitStructure_Rx.DMA_Memory0BaseAddr =(uint32_t)&esp8266_Recv_Buffer; 		//内存基地址 
    DMA_InitStructure_Rx.DMA_BufferSize = ESP8266_RECV_LENTH;			//dma缓存大小 
    DMA_InitStructure_Rx.DMA_FIFOMode = DMA_FIFOMode_Disable; 
    DMA_InitStructure_Rx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; 
    DMA_InitStructure_Rx.DMA_MemoryBurst = DMA_Mode_Circular; 
    DMA_InitStructure_Rx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
    DMA_Init(DMA2_Stream1,&DMA_InitStructure_Rx); 
    DMA_Cmd(DMA2_Stream1,ENABLE); 
 
    /*发送DMA配置，暂不开启*/ 
    DMA_InitStructure_Tx.DMA_Channel= DMA_Channel_5; 
    DMA_InitStructure_Tx.DMA_PeripheralBaseAddr =  (uint32_t)&(USART6->DR);		//外设基地址 
    DMA_InitStructure_Tx.DMA_DIR = DMA_DIR_MemoryToPeripheral;		//数据传输方向：内存到外设 
    DMA_InitStructure_Tx.DMA_PeripheralInc = DMA_PeripheralInc_Disable;		//外设地址不变 
    DMA_InitStructure_Tx.DMA_MemoryInc = DMA_MemoryInc_Enable;		//内置地址寄存器递增 
    DMA_InitStructure_Tx.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;	//数据宽度为八位 
    DMA_InitStructure_Tx.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;			//数据宽度为八位 
    DMA_InitStructure_Tx.DMA_Mode = DMA_Mode_Normal;					//工作模式为环形 
    DMA_InitStructure_Tx.DMA_Priority = DMA_Priority_High;			//dma通道拥有高优先级 
    DMA_InitStructure_Tx.DMA_Memory0BaseAddr =(uint32_t)&esp8266_SendStruct; 		//内存基地址 
    DMA_InitStructure_Tx.DMA_BufferSize = usart_SendStructLenth;			//dma缓存大小 
    DMA_InitStructure_Tx.DMA_FIFOMode = DMA_FIFOMode_Disable; 
    DMA_InitStructure_Tx.DMA_FIFOThreshold = DMA_FIFOThreshold_1QuarterFull; 
    DMA_InitStructure_Tx.DMA_MemoryBurst = DMA_Mode_Normal; 
    DMA_InitStructure_Tx.DMA_PeripheralBurst = DMA_PeripheralBurst_Single; 
 
    NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn; 
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; 
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0; 
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; 
    NVIC_Init(&NVIC_InitStructure); 
 
} 
 
void usart_monitor(void) 
{ 
    static u8 wait_Cnt = 0;//等待 
    if(busy_Flag == 0) 
    { 
        switch(ESP8266_Send_Status) 
        { 
        case SEND_REQUEST: 
        {     
            usart_monitor_GetValue(1); 
            ESP8266_RequestStruct.frameHeader = 0x5A;          
            break; 
        } 
        case SEND_DATA: 
        { 
            usart_monitor_GetValue(0); 
            esp8266_SendStruct.frameHeader = 0xA5; 
            esp8266_SendStruct.RESERVED1 = 0x0d; 
            esp8266_SendStruct.RESERVED2 = 0x0A;             
            break; 
        } 
        case SEND_JOINAP: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CWJAP=\""); 
            strcat(esp8266_cmd_sendBuff, SSID); 
            strcat(esp8266_cmd_sendBuff, "\",\""); 
            strcat(esp8266_cmd_sendBuff, PWD); 
            strcat(esp8266_cmd_sendBuff, "\"\r\n"); 
            break; 
        } 
        case SEND_CHECKAP: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CWJAP?\r\n"); 
            break; 
        } 
        case SEND_QUITAP: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CWQAP\r\n"); 
            break; 
        } 
        case SEND_CIPMODE: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CIPMODE=1\r\n"); 
            break; 
        } 
        case SEND_CIPSEND: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CIPSEND\r\n"); 
            break; 
        } 
        case SEND_CIPSTART: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CIPSTART=\"UDP\",\""); 
            strcat(esp8266_cmd_sendBuff, UDP_IP); 
            strcat(esp8266_cmd_sendBuff, "\","); 
            strcat(esp8266_cmd_sendBuff, UDP_Port); 
            strcat(esp8266_cmd_sendBuff, "\r\n"); 
            break; 
        } 
        case SEND_CWMODE: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+CWMODE=1\r\n"); 
            break; 
        } 
        case SEND_RST: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            strcat(esp8266_cmd_sendBuff, "AT+RST\r\n"); 
            fail_CNT = 0; 
            break; 
        } 
        case SEND_QUITCIPSEND: 
        { 
            memset(esp8266_cmd_sendBuff,0,100); 
            if(wait_Cnt++ == 1) 
            { 
                strcat(esp8266_cmd_sendBuff, "+++\r\n"); 
                wait_Cnt = 0; 
            } 
            else 
            { 
                strcat(esp8266_cmd_sendBuff, "+++"); 
            } 
 
            break; 
        } 
        } 
		DMA_Cmd(DMA2_Stream6,DISABLE); 
		while(DMA_GetCmdStatus(DMA2_Stream6) == ENABLE) {} 
		DMA_DeInit(DMA2_Stream6); 
		if(ESP8266_Send_Status == SEND_DATA) 
		{ 
			DMA_InitStructure_Tx.DMA_Memory0BaseAddr = (u32)&esp8266_SendStruct; 
			DMA_InitStructure_Tx.DMA_BufferSize = usart_SendStructLenth; 
		}else if(ESP8266_Send_Status == SEND_REQUEST) 
		{ 
			DMA_InitStructure_Tx.DMA_Memory0BaseAddr = (u32)&ESP8266_RequestStruct; 
			DMA_InitStructure_Tx.DMA_BufferSize = sizeof(ESP8266_RequestStruct); 
		} 
		else 
		{ 
			DMA_InitStructure_Tx.DMA_Memory0BaseAddr = (u32)esp8266_cmd_sendBuff; 
			DMA_InitStructure_Tx.DMA_BufferSize = strlen(esp8266_cmd_sendBuff); 
		} 
		DMA_Init(DMA2_Stream6,&DMA_InitStructure_Tx); 
		DMA_Cmd(DMA2_Stream6,ENABLE); 
    } 
} 
 
 
void USART6_IRQHandler(void) 
{ 
    u8 ii; 
    u8 Recv_Cnt; 
    if(USART_GetITStatus(USART6,USART_IT_IDLE) != RESET) 
    { 
        ii = USART6->SR; 
        ii = USART6->DR; 
        ii++; 
        USART_ClearITPendingBit(USART6,USART_IT_IDLE); 
        Recv_Cnt = ESP8266_RECV_LENTH - DMA_GetCurrDataCounter(DMA2_Stream1); 
        if(Recv_Cnt > 2) 
        { 
            DMA_Cmd(DMA2_Stream1,DISABLE); 
            esp8266_Recv_Buffer[ESP8266_RECV_LENTH-1] = 0; 
            if(strstr(esp8266_Recv_Buffer,"\r\n") != NULL) 
            { 
                if(strstr(esp8266_Recv_Buffer,"OK") != NULL) 
                { 
                    switch(ESP8266_Send_Status) 
                    { 
                        case SEND_CIPMODE: 
                        { 
                            busy_Flag = 0; 
                            ESP8266_Link_Status = CIPMODE_SUCCESS; 
                            ESP8266_Send_Status = SEND_CIPSTART; 
                            break; 
                        } 
                        case SEND_CIPSTART: 
                        { 
                            busy_Flag = 0; 
                            ESP8266_Link_Status = CIPSTART_SUCCESS; 
                            ESP8266_Send_Status = SEND_CIPSEND; 
                            break; 
                        } 
                        case SEND_QUITAP: 
                        { 
                            busy_Flag = 0; 
                            ESP8266_Link_Status = DISCONNECT; 
                            ESP8266_Send_Status = SEND_JOINAP; 
                            break; 
                        } 
                        case SEND_CWMODE: 
                        { 
                            busy_Flag = 0; 
                            ESP8266_Link_Status = DISCONNECT; 
                            ESP8266_Send_Status = SEND_RST; 
                            break; 
                        } 
                        case SEND_JOINAP: 
                        { 
                            busy_Flag = 0; 
                            ESP8266_Link_Status = CONNECTED; 
                            ESP8266_Send_Status = SEND_CHECKAP; 
                            break; 
                        } 
                        case SEND_CHECKAP: 
                        { 
                            if(strstr(esp8266_Recv_Buffer,SSID) == NULL)//WIFI名字不对 
                            { 
                                busy_Flag = 0; 
                                ESP8266_Link_Status = DISCONNECT; 
                                ESP8266_Send_Status = SEND_QUITAP; 
                            } 
                            else 
                            { 
                                ESP8266_Link_Status = RIGHT_CONNECT; 
                                ESP8266_Send_Status = SEND_CIPMODE;//等待返回OK，并且认为正在busy 
                            } 
                            break; 
                        } 
                        case SEND_CIPSEND: 
                        { 
                            busy_Flag = 0; 
                            ESP8266_Link_Status = CIPSEND_SUCCESS; 
                            ESP8266_Send_Status = SEND_DATA; 
                            break; 
                        } 
						default: 
						break; 
                    } 
                    memset(esp8266_Recv_Buffer,0,ESP8266_RECV_LENTH); 
                } 
                else if(strstr(esp8266_Recv_Buffer,"busy") != NULL) 
                { 
                    busy_Flag = 1; 
                } 
                else if(ESP8266_Send_Status == SEND_CIPSTART && strstr(esp8266_Recv_Buffer,"ALREADY CONNECTED") != NULL)//已连接 
                { 
                    busy_Flag = 0; 
                    ESP8266_Link_Status = CIPSTART_SUCCESS; 
                    ESP8266_Send_Status = SEND_CIPSEND; 
                } 
                else if(strstr(esp8266_Recv_Buffer,"ERROR") != NULL || strstr(esp8266_Recv_Buffer,"FAIL") != NULL)//连接不上或者失败，失败次数超过fail_CNT_MAX重启 
                { 
                    busy_Flag = 0; 
                    fail_CNT++; 
                    if(fail_CNT >= fail_CNT_MAX) 
                    { 
                        ESP8266_Link_Status = DISCONNECT; 
                        ESP8266_Send_Status = SEND_CWMODE; 
                    } 
                    else 
                    { 
                        ESP8266_Link_Status = DISCONNECT; 
                        ESP8266_Send_Status = SEND_QUITAP; 
                    } 
                    memset(esp8266_Recv_Buffer,0,ESP8266_RECV_LENTH); 
                } 
				else   
                {  
                    if(ESP8266_Send_Status == SEND_CHECKAP && strstr(esp8266_Recv_Buffer,"AT+CWJAP?") != NULL)  
                    {  
                        if(strstr(esp8266_Recv_Buffer,SSID) == NULL)//WIFI名字不对  
                        {  
                            busy_Flag = 0;  
                            ESP8266_Link_Status = DISCONNECT;  
                            ESP8266_Send_Status = SEND_QUITAP;  
                        }  
                        else  
                        {  
                            ESP8266_Link_Status = RIGHT_CONNECT;  
                            ESP8266_Send_Status = SEND_CIPMODE;//等待返回OK，并且认为正在busy  
                        }  
                    }  
                }  
            
            }  
            else  
            {  
                if(esp8266_Recv_Buffer[0] == 0x5A)  
                { 
                    if(strstr(esp8266_Recv_Buffer+1,"get origin data")) 
                    { 
                        ESP8266_Send_Status = SEND_REQUEST; 
                    }else if(strstr(esp8266_Recv_Buffer+1,"dataGet")) 
                    { 
                        ESP8266_Send_Status = SEND_DATA; 
                    } 
                    else if(Recv_Cnt == sizeof(ESP8266_ChangeStruct)) 
                    { 
                        recv_dataChange(); 
                    } 
                } 
            } 
            while(DMA_GetCmdStatus(DMA2_Stream1) == ENABLE) {} 
            DMA_DeInit(DMA2_Stream1); 
            DMA_InitStructure_Rx.DMA_Memory0BaseAddr = (u32)esp8266_Recv_Buffer; 
            DMA_InitStructure_Rx.DMA_BufferSize = ESP8266_RECV_LENTH; 
            DMA_Init(DMA2_Stream1,&DMA_InitStructure_Rx); 
            DMA_Cmd(DMA2_Stream1,ENABLE);   
        } 
    } 
} 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
 
