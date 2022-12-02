#pragma once

#include <array>
#include <cstddef>

namespace modm
{
namespace atomic
{

template<class T, size_t SIZE>
class DmaRxRingBuffer
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
		if (tmptail >= (SIZE)) { tmptail = tmptail - SIZE; }
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
	size_t tail = 0;
};

}  // namespace atomic
}  // namespace modm