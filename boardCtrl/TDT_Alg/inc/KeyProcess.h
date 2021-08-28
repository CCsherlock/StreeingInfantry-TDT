/**
 * @file KeyProcess.h
 * @author 梁文生
 * @brief DBUS按键处理
 * @version 0.1
 * @date 2021-03-22
 * 
 * @copyright Copyright (c) 2021
 * 
 */
#ifndef __KEY_PROCESS_H
#define __KEY_PROCESS_H

#include "board.h"
#include "dbus.h"
#include <stddef.h>
#include "cycle.h"
#include <vector>

class KeyProcess
{
public:
	/**
	 * @brief Key Process构造器
	 * 
	 * @param keyValue 	键值，不允许重复
	 * @param press 	按下回调函数
	 * @param release 	释放回调函数（按下了多长时间(毫秒））
	 * @param hold 		按住回调函数（按下了多长时间(毫秒））
	 */
	KeyProcess(uint16_t keyValue, void (*press)(uint32_t *interval) = 0, void (*release)(uint32_t *interval) = 0, void (*hold)(uint32_t *interval) = 0);

	/**
	 * @brief 根据键值获取对象
	 * 
	 * @param keyValue 键值
	 * @return KeyProcess* 
	 */
	KeyProcess *getKeyProcess(uint16_t keyValue);

	/**
	 * @brief 键值循环处理函数
	 * 
	 * @param keyValue 键值
	 */
	static void keyHandle(uint16_t keyValue);

	inline u8 getEnable() { return _enable; };
	void setEnAble(u8 enable);
	inline void enable() { this->_enable = 1; };
	void disable();

	inline void setPressCallback(void (*press)(uint32_t *interval)) { this->press = press; }

	inline void setReleaseCallback(void (*release)(uint32_t *interval)) { this->release = release; }

	inline void setHoldCallback(void (*hold)(uint32_t *interval)) { this->hold = hold; }

private:
	static Cycle keyCycle;
	static std::vector<KeyProcess *> *keyList;
	static std::vector<uint16_t> *keyValueList;
	
	static void keyTravel(uint16_t keyValue, uint16_t indexFrom, uint16_t indexTo, uint32_t interval);

	KeyProcess();

	u8 _enable : 1;
	u8 hasPress : 1;
	uint32_t interval;
	inline void preHold()
	{
		if (hold)
			hold(&interval);
	};
	inline void preRelease()
	{
		if (release)
			release(&interval);
		interval = 0;
	};

	inline void prePress()
	{
		if (press)
			press(&interval);
		interval = 0;
	};

	void (*hold)(uint32_t *interval);

	void (*release)(uint32_t *interval);

	void (*press)(uint32_t *interval);
};

#endif
