#ifndef EVENT_HPP
#define EVENT_HPP

#include <cstdint>
#include <vector>
#include "tf2/time.hpp"

struct Event
{
	// If SyncedEvents keeps track of the timestamp then we could ditch it here?
	tf2::TimePoint timestamp;
	uint16_t x;
	uint16_t y;
	bool polarity;
};

struct SyncedEvents{
	std::vector<Event> events;
	tf2::TimePoint timestamp;
	// ssize_t size;
};

#endif