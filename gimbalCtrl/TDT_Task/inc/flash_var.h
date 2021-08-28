#ifndef __FLASH_VAR_H__
#define __FLASH_VAR_H__
#include "board.h"

#ifndef NULL
#define NULL 0
#endif

#define FLASH_VAR_READ_ERROR_SAVING 1
#define FLASH_VAR_READ_ERROR_EMPTY_FLASH 2


template <typename T>
class Node
{
public:
	T m_value;
	Node<T>* m_next;
	Node(T value) { m_value = value; m_next = NULL; }
	void append(Node<T>* new_node) { m_next = new_node; }
};

template <typename T>
class Line
{
	Node<T>* m_start;
	Node<T>* m_end;
public:
	Node<T>* m_now;
	u32 length;
	Line()
	{
		length = 0;
		m_start = NULL;
		m_end = NULL;
	}
	~Line()
	{
		if (m_start == NULL) return;
		Node<T>* ptr = m_start;
		while (ptr->m_next != NULL)
		{
			Node<T>* del_ptr = ptr;
			ptr = ptr->m_next;
			delete del_ptr;
		}
		delete ptr;
	}
	void append(T value)
	{
		Node<T>* new_node = new Node<T>(value);
		if (m_end == NULL)
		{
			m_start = new_node;
			m_end = new_node;
		}
		else
		{
			m_end->append(new_node);
			m_end = new_node;
		}
		length++;
	}
	void insert(u32 index, T value)
	{
		Node<T>* new_node = new Node<T>(value);
		Node<T>* ptr = m_start;
		if (index == 0)
		{
			new_node->append(m_start);
			m_start = new_node;
			length++;
		}
		else if (index < length)
		{
			for (u32 a = 0; a < index - 1; a++)
			{
				if (ptr->m_next != NULL) ptr = ptr->m_next;
				else return;
			}
			new_node->append(ptr->m_next);
			ptr->append(new_node);
			length++;
		}
		else append(value);
	}
	T* reset() { m_now = m_start; return &m_now->m_value; };
	T* next()
	{
		if (m_now != NULL) m_now = m_now->m_next;
		return &m_now->m_value;
	}
	T& operator[] (u32 index)
	{
		Node<T>* ptr = m_start;
		if (index < length)
		{
			for (u32 a = 0; a < index; a++)
			{
				if (ptr->m_next != NULL) ptr = ptr->m_next;
				else break;
			}
		}
		return ptr->m_value;
	}
};

class FlashVar
{
	Line<u16> line_id;
	Line<u16> line_size;
	Line<u8*> line_pos;
	const u32 exbuff_len = 2;
	void(*pSaveFunc)(u32* saveBuffer, u32 buffsize);
	u32*(*pReadFunc)();
public:
	FlashVar(void(*pSaveFunc)(u32* saveBuffer, u32 buffsize),u32*(*pReadFunc)()):
		pSaveFunc(pSaveFunc),pReadFunc(pReadFunc){};

	template <typename T>
	void link(T& value, u16 ID)
	{
		link((u8*)(&value),sizeof(value),ID);
	}
	
	void link(u8* valuePtr, u8 size, u16 ID)
	{
		u32 index = 0;
		u16* ptr_size = line_size.reset();
		while (ptr_size != NULL)
		{
			if (size <= *ptr_size)
			{
				line_id.insert(index, ID);
				line_size.insert(index, size);
				line_pos.insert(index, valuePtr);
				return;
			}
			index++;
			ptr_size = line_size.next();
		}
		line_id.append(ID);
		line_size.append(size);
		line_pos.append(valuePtr);
	}

	//template <typename T>
	//T Init(T& value, u16 ID) { Link(value, ID); return value; }

	void save()
	{
		if (pSaveFunc == NULL) return;
		Line<u16>size;
		Line<u16>number;
		u32 datasize = 0;
		u16* ptr_size = line_size.reset();
		/*获取总长度*/
		while (ptr_size != NULL)
		{
			if (size.length == 0 || size[size.length - 1] != *ptr_size)
			{
				size.append(*ptr_size);
				number.append(1);
			}
			else number[number.length - 1]++;
			datasize += *ptr_size;
			ptr_size = line_size.next();
		}
		u32 sizebuff_len = size.length;
		u32 idbuff_len = line_id.length / 2 + line_id.length % 2;
		u32 databuff_len = datasize % 4 == 0 ? datasize / 4 : datasize / 4 + 1;

		u32 buffsize = exbuff_len + sizebuff_len + idbuff_len + databuff_len;
		u32* buff = new u32[buffsize+1];
		if (buff == NULL) return;

		u32 id_addr = exbuff_len + sizebuff_len;
		u32 data_addr = id_addr + idbuff_len;

		u8* buff_data = (u8*)(&buff[data_addr]);
		u16* buff_id = (u16*)(&buff[id_addr]);
		u32* buff_size = &buff[exbuff_len];

		buff[0] = buffsize;
		buff[1] = id_addr << 16 | data_addr;

		auto p_size = size.reset();
		auto p_number = number.reset();
		while (p_size != NULL)
		{
			*buff_size = ((u32)(*p_size) << 16) | (*p_number); buff_size++;
			p_size = size.next();
			p_number = number.next();
		}

		ptr_size = line_size.reset();
		u16* ptr_id = line_id.reset();
		u8** ptr_pos = line_pos.reset();
		while (ptr_size != NULL)
		{
			*buff_id = *ptr_id; buff_id++;
			u8* p_data = *ptr_pos;
			for (u32 a = 0; a < *ptr_size; a++)
			{
				*buff_data = *p_data;
				buff_data++;
				p_data++;
			}
			ptr_size = line_size.next();
			ptr_id = line_id.next();
			ptr_pos = line_pos.next();
		}
		pSaveFunc(buff, buffsize);
		delete[] buff;
	}
	u8 read()
	{
		if (saving) return FLASH_VAR_READ_ERROR_SAVING;//正在读，退出
		u32* readBuff = pReadFunc();
		if (readBuff == NULL) return FLASH_VAR_READ_ERROR_EMPTY_FLASH;
		read(readBuff);
		delete[] readBuff;
		return 0;
	}
	void read(u32* buff)
	{
		u32 id_addr = buff[1] >> 16;
		u32 data_addr = buff[1] & 0x0000FFFF;
		Line<u16> save_size;
		u16* pID = (u16*)(&buff[id_addr]);
		u8* pData = (u8*)(&buff[data_addr]);
		for (u32 a = exbuff_len; a < id_addr; a++)
		{
			u16 size = (u16)(buff[a] >> 16);
			u16 num = buff[a] & 0x0000FFFF;
			for (u32 i = 0; i < num; i++) save_size.append(size);
		}
		u16* pSize = save_size.reset();
		while (pSize != NULL)
		{
			int index = indexOfID(*pID);
			if (index != -1 && line_size[index] == *pSize)
			{
				u8* pVar = line_pos[index];
				for (u32 a = 0; a < *pSize; a++)
				{
					*pVar = *pData;
					pVar++;
					pData++;
				}
			}
			else pData += *pSize;
			pID++;
			pSize = save_size.next();
		}
	}
	bool saving;
private:
	int indexOfID(u16 ID)
	{
		int index = 0;
		u16* pID = line_id.reset();
		while (pID != NULL)
		{
			if (*pID == ID) return index;
			pID = line_id.next();
			index++;
		}
		return -1;
	}
};

extern FlashVar IFlash;


#endif
