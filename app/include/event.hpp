#ifndef EVENT_HPP
#define EVENT_HPP

#include <cstdint>

struct Time {
  uint64_t timestamp;
/*
  Något här ...
*/
};

struct Event {
  uint64_t timestamp;
  uint16_t x;
  uint16_t y;
  bool polarity;
};

#endif