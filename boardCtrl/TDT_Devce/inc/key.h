/**
 * @file key.h
 * @author 彭阳 
 * @brief 
 * @version 2.3.0
 * @date 2020-12-22
 * 
 */

#ifndef _KEY__H
#define _KEY__H

#include "board.h"
#include "cycle.h"
#include "FreeRTOS.h"
#include "timers.h"

/**
 * @brief 按键类
 * @note 其他通过高低电平反馈的外设也可以用
 *       默认为开漏上拉 低电平为按下
 * 
 */
class Key
{
private:
    GPIO_TypeDef *GPIOx; /*LED_PORT*/
    uint16_t GPIO_Pin_x; /*LED_Pin*/
    uint8_t realStatus;  //0为低电平，1为高电平
    Cycle *time;         //计时

    //时间单位为ms
    uint32_t timeThreshold; //长按时间阈值
    uint8_t longPressSpeed; //长按增长频率
    uint32_t pressTime;     //长按的时间

    static TimerHandle_t timer; //按键扫描定时器

    Key *lastkey;           //下一个按键
    static Key *finalKey;   //最后一个按键
    static Key *currentKey; //当前按键

    void init(void); //单个按键初始化
    void scan(void); //单个按键扫描

public:
    uint8_t status; //0-悬空 1-按了一下 （此变量可以将长按按键视为快速按动）

    Key(GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin_x, uint32_t timeThreshold = 1000, uint8_t longPressSpeed = 5);

    uint8_t getRealStatus(); //获取真实按键状态
    static void keyInit();   //初始化全部按键
    static void keyScan();   //扫描全部按键
};

#endif
