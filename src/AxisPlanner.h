#pragma once

#include <Arduino.h>
#include "Planner.h"

// Arduino convenience wrapper. Planner<...> itself has no Arduino dependency.
template <uint8_t Axes, uint8_t Buffer = 8>
class AxisPlanner : public planner::Planner<Axes, Buffer> {
 public:
    axis::Result start() { return this->startAt(static_cast<uint32_t>(micros())); }
    axis::Result resume() { return this->resumeAt(static_cast<uint32_t>(micros())); }
    bool tick() { return this->tickAt(static_cast<uint32_t>(micros())); }
    axis::Result clearFault() { return this->clearFaultAt(static_cast<uint32_t>(micros())); }
};
