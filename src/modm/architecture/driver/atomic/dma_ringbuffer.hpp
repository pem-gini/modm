#pragma once

#include <array>
#include <cstddef>

namespace modm
{
namespace atomic
{

class DmaRingBuffer
{
public:
	virtual size_t
	maxSize() = 0;
	virtual uint8_t
	at(size_t index) const = 0;
	virtual size_t
	getIndex() const = 0;
	virtual const uint8_t*
	data() const = 0;
	virtual bool
	read(uint8_t* target, size_t length) = 0;
	virtual size_t
	getBytesWritten(size_t writeIndex) = 0;
};

template<class T, size_t SIZE>
class DmaRxRingBuffer : public DmaRingBuffer
{
public:
	size_t
	maxSize()
	{
		return SIZE;
	}
	uint8_t
	at(size_t index) const
	{
		return buf.at(index);
	}
	size_t
	getIndex() const
	{
		return tail;
	}
	const uint8_t*
	data() const
	{
		return reinterpret_cast<const uint8_t*>(buf.data());
	}
	bool
	read(uint8_t* target, size_t length)
	{
		if (length > SIZE) { return false; }
		if ((tail + length) > SIZE)
		{
			size_t s1 = SIZE - tail;
			size_t s2 = length - s1;
			T* target_mid = target + s1;
			std::memcpy(target, &buf.at(tail), s1);
			std::memcpy(target_mid, &buf.at(0), s2);
		} else
		{
			std::memcpy(target, &buf.at(tail), length);
		}
		auto tmptail = tail + length;
		if (tmptail >= (SIZE + 1)) { tmptail = tmptail - SIZE; }
		tail = tmptail;
		return true;
	}
	size_t
	getBytesWritten(size_t writeIndex)
	{
		size_t bytesWritten = 0;
		size_t r = getIndex();
		if (writeIndex < r)
		{
			bytesWritten = maxSize() - r + writeIndex;
		} else
		{
			bytesWritten = writeIndex - r;
		}
		return bytesWritten;
	}

private:
	std::array<uint8_t, SIZE> buf;
	T tail = 0;
};

}  // namespace atomic
}  // namespace modm