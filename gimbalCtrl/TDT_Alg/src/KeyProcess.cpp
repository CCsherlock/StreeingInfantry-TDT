#include "KeyProcess.h"
#include <vector>

static std::vector<KeyProcess *> *keyList = 0;
static std::vector<uint16_t> *keyValueList = 0;

Cycle KeyProcess::keyCycle = Cycle();

KeyProcess::KeyProcess(uint16_t keyValue, void (*press)(uint32_t *interval), void (*release)(uint32_t *interval),
					   void (*hold)(uint32_t *interval), u8 keyProcessCtrlToDisable, u8 keyProcessShiftToDisable) :
						   press(press), release(release), hold(hold), interval(0), _enable(1), hasPress(0),
					   keyProcessCtrlToDisable(keyProcessCtrlToDisable), keyProcessShiftToDisable(keyProcessShiftToDisable)
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

	if (keyValue & KEY_SHIFT) //如果有shift的按键
	{
		this->keyProcessShiftToDisable = 0;
	}

	if (keyValue & KEY_CTRL) //如果有ctrl的按键
	{
		this->keyProcessCtrlToDisable = 0;
	}

	keyValueList->push_back(keyValue);
	keyList->push_back(this);
}

KeyProcess::KeyProcess() : press(0), release(0), hold(0), interval(0), _enable(1), hasPress(0)
{
}

void KeyProcess::keyHandle(uint16_t keyValue)
{
	if (keyValueList == 0)
		return;
	uint16_t count = keyValueList->size();
	if (count == 0)
		return;

	uint32_t keyInterval = keyCycle.getCycleT() * 1000;
	if (keyInterval > 500)
		keyInterval = 0;

	keyTravel(keyValue, 0, count, keyInterval);
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
	u8 ctrlPressed = ((keyValue & KEY_CTRL) == KEY_CTRL);
	u8 shiftPressed = ((keyValue & KEY_SHIFT) == KEY_SHIFT);
	for (uint16_t i = indexFrom; i < indexTo; i++)
	{
		item = (*keyList)[i];
		itemKeyValue = (*keyValueList)[i];
		if (shiftPressed && item->keyProcessShiftToDisable)
		{
			item->disable();
			continue;
		}
		else
		{
			item->enable();
		}

		if (ctrlPressed && item->keyProcessCtrlToDisable)
		{
			item->disable();
			continue;
		}
		else
		{
			item->enable();
		}

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
	for (int i = 0; i < count; i++)
	{
		if (keyValue == (*keyValueList)[i])
			return (*keyList)[i];
	}
	return 0;
}
