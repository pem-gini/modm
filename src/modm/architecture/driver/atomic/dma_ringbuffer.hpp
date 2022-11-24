#pragma once

#include <array>
#include <cstddef>

namespace modm
{
namespace atomic
{

template <class T, size_t SIZE> class DmaRxRingBuffer {
public:
  static constexpr size_t maxSize(){ return SIZE; }
  T &front() { return buf.at(tail); }
  void pop() {
    if (tail == SIZE - 1) {
      tail = 0;
    } else {
      tail++;
    }
  }
  T* data(){ return buf.begin(); }
  bool read(T *target, size_t length) {
    if (length > SIZE) {
      return false;
    }
    if ((tail + length) > SIZE) {
      size_t s1 = SIZE - tail;
      size_t s2 = length - s1;
      T *target_mid = target + s1;
      std::memcpy(target, &buf.at(tail), s1);
      std::memcpy(target_mid, &buf.at(0), s2);
    } else {
      std::memcpy(target, &buf.at(tail), length);
    }
    auto tmptail = tail + length;
    if (tmptail >= (SIZE + 1)) {
      tmptail = tmptail - SIZE;
    }
    tail = tmptail;
    return true;
  }
  std::array<T, SIZE> buf;
  T tail = 0;
};

}  // namespace atomic
}  // namespace modm