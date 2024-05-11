#pragma once

#include "portAccess.hpp"
#include "clocks.hpp"

#include <cstdint>

namespace pwm
{
	constexpr std::uint8_t dutyCycleArrSize_0 = 4, dutyCycleArrSize_3 = 2;

	extern std::uint8_t portPinNum0[dutyCycleArrSize_0], portPinNum3[dutyCycleArrSize_3];

	void init0(std::uint32_t frequency);
  void init3(std::uint32_t frequency);

	void add0(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty = 0, bool isinvert = false);
  void add3(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty = 0);
	void duty0(std::uint8_t idx, std::uint8_t duty);
  void duty0(std::uint8_t idx, float duty);
  std::uint8_t getduty0(std::uint8_t idx);
  float getduty0f(std::uint8_t idx);
  void duty3(std::uint8_t idx, std::uint8_t duty);
  void duty3(std::uint8_t idx, float duty);
  std::uint8_t getduty3(std::uint8_t idx);
  float getduty3f(std::uint8_t idx);
	void remove0(std::uint8_t idx);
  void remove3(std::uint8_t idx);
}
