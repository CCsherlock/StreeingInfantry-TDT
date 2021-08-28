#include "KeyProcess.h"

std::vector<KeyProcess *> *KeyProcess::keyList = 0;
std::vector<uint16_t> *KeyProcess::keyValueList = 0;
Cycle KeyProcess::keyCycle = Cycle();

KeyProcess::KeyProcess(uint16_t keyValue, void (*press)(uint32_t *interval), void (*release)(uint32_t *interval),
					   void (*hold)(uint32_t *interval)) : press(press), release(release), hold(hold), interval(0), _enable(1),
														   hasPress(0)
{
	if (!keyList)
	{
		keyList = new std::vector<KeyProcess *>;
		keyValueList = new std::vector<uint16_t>;
		for (int i = 0; i < 16; i++)
		{
			keyValueList->push_back(1 << i);
			keyList->push_back(new KeyProcess());
		}
	}

	auto it = std::find(keyValueList->begin(), keyValueList->end(), keyValue);

	if (it != keyValueList->end())
	{
		int index = it - keyValueList->begin();
		delete (*keyList)[index];
		(*keyList)[index] = this;
		return;
	}
	keyValueList->push_back(keyValue);
	keyList->push_back(this);
}

KeyProcess::KeyProcess() : press(0), release(0), hold(0), interval(0), _enable(1), hasPress(0)
{
}

void KeyProcess::keyHandle(uint16_t keyValue)
{
	if(keyValueList  == 0)
		return;
	uint16_t count = keyValueList->size();
	if (count == 0)
		return;

	uint32_t keyInterval = keyCycle.getCycleT() * 1000;
	if (keyInterval > 500)
		keyInterval = 0;

	uint16_t i = 0;
	u8 skipSingleKey; //跳过除wasd的单键
	KeyProcess *item;

	u8 ctrl_shift_lastHasPress = (*keyList)[4]->hasPress || (*keyList)[5]->hasPress;
	if (keyValue & (KEY_CTRL | KEY_SHIFT)) //按下了ctrl或shift
	{
		skipSingleKey = 1;
		if (!ctrl_shift_lastHasPress) //上次未按下
		{
			for (i = 6; i < 16; i++)
				(*keyList)[i]->disable(); //将单击键失能
		}
	}
	else
	{
		skipSingleKey = 0;
		if (ctrl_shift_lastHasPress) //上次已经按下按下
		{
			for (i = 6; i < 16; i++)
				(*keyList)[i]->enable(); //将单击键使能
		}
	}

	keyTravel(keyValue, 0, 6, keyInterval);

	i = skipSingleKey ? 16 : 6;
	keyTravel(keyValue, i, count, keyInterval);
}

void KeyProcess::setEnAble(u8 enable)
{
	if (enable)
		this->enable();
	else
		disable();
}

void KeyProcess::disable()
{
	this->_enable = 0;
	if (hasPress)
	{
		preRelease();
		hasPress = 0;
	}
}

void KeyProcess::keyTravel(uint16_t keyValue, uint16_t indexFrom, uint16_t indexTo, std::uint32_t interval)
{
	KeyProcess *item;
	uint16_t itemKeyValue;
	for (uint16_t i = indexFrom; i < indexTo; i++)
	{
		item = (*keyList)[i];
		itemKeyValue = (*keyValueList)[i];
		if (itemKeyValue == (keyValue & itemKeyValue)) //按下
		{
			if (item->hasPress) //之前已经按下
			{
				item->interval += interval;
				item->preHold();
				continue;
			}
			item->hasPress = 1; //之前没按下
			item->prePress();
			continue;
		}
		if (item->hasPress)
		{
			item->hasPress = 0;
			item->preRelease();
		}
	}
}

KeyProcess *KeyProcess::getKeyProcess(uint16_t keyValue)
{
	int count = keyValueList->size();
	for(int i = 0; i < count; i++)
	{
		if(keyValue == (*keyValueList)[i])
			return (*keyList)[i];
	}
	return 0;
}
