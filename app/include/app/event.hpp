#ifndef EVENT_HPP
#define EVENT_HPP

#include <cstdint>
#include "tf2/time.hpp"

struct Event
{
  // uint64_t timestamp;
  tf2::TimePoint timestamp;
  uint16_t x;
  uint16_t y;
  bool polarity;
};

#endif