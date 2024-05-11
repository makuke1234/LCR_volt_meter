#pragma once

#include "clocks.hpp"

#include <cstdint>
#include <Arduino.h>

namespace dac
{
  void init(clk::tmr timer);
  // sets new buffer
  void setbuf(const std::uint16_t * samples, std::uint32_t size);
  // enable/disable DAC output, frequency is calculated per buffer, real frequency returned
  float enable(std::uint32_t frequency, bool flag = true);
}
