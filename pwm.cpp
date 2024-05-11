#include "clocks.hpp"
#include "pwm.hpp"

std::uint8_t pwm::portPinNum0[pwm::dutyCycleArrSize_0], pwm::portPinNum3[pwm::dutyCycleArrSize_3];

void pwm::init0(std::uint32_t frequency)
{
	// Initialize TCC0 clocks
	clk::initTmr(clk::tmr::tTCC0, GCLK_CLKCTRL_GEN_GCLK4_Val, 96000000U);

	TCC0->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
	while (TCC0->SYNCBUSY.bit.WAVE);

	// Set frequency
	const auto ticks = 96000000U / frequency;
	TCC0->PER.reg = ticks;
	while (TCC0->SYNCBUSY.bit.PER);

	// Start timer
	TCC0->CTRLA.bit.ENABLE = 1;
	while (TCC0->SYNCBUSY.bit.ENABLE);
}
void pwm::init3(std::uint32_t frequency)
{
	// Initialize TC3 clocks
	clk::initTmr(clk::tmr::tTC3, GCLK_CLKCTRL_GEN_GCLK4_Val, 96000000U);

	// Set frequency
	auto prescaler = 96000000U / (frequency * 256U);

	// set up prescaler to get close to desired frequency
	auto prescaler_reg = 0;
	{
		auto temppre = prescaler;
		while (temppre >>= 1)
		{
			++prescaler_reg;
		}
	}
	prescaler_reg = (prescaler_reg > 4U) ? ((prescaler_reg - 4U) / 2U + 4U) : prescaler_reg;
	prescaler_reg = (prescaler_reg > 7U) ? 7U : prescaler_reg;
	prescaler = 1U << ((prescaler_reg > 4U) ? (prescaler_reg - 4U) * 2 + 4U : prescaler_reg);

	TC3->COUNT8.CTRLA.reg |=
		TC_CTRLA_PRESCALER(prescaler_reg) |
		TC_CTRLA_MODE_COUNT8;
	while (TC3->COUNT8.STATUS.bit.SYNCBUSY);

	// Start timer
	TC3->COUNT8.CTRLA.bit.ENABLE = 1;
	while (TC3->COUNT8.STATUS.bit.SYNCBUSY);
}

void pwm::add0(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty, bool isinvert)
{
	pin = ARDUINO_PIN_TO_PORT_PIN(pin);
	const auto apin = pin;
	const auto grp = PGrp(pin);
	pin &= 0x1F;
	// Enable peripheral multiplexer
	PORT->Group[grp].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
	
	const auto pmuxIdx = pin / 2;

	/* Function E:
	 * PA04, PA05, PA08, PA09
	 */
	/* Function F:
	 * PA10, PA11, PB10, PB11, PA12, PA13, PA14, PA15, PA16, PA17, PA18, PA19, PA20, PA21, PA22, PA23
	 */

	// Select function
	auto function = (!grp && (pin <= 9) && (bool[10]){ 0, 0, 0, 0, 1, 1, 0, 0, 1, 1 }[pin]) ? PORT_PMUX_PMUXO_E_Val : PORT_PMUX_PMUXO_F_Val;

	// Even/odd function selection shifting
	function = (pin % 2) ? (function << PORT_PMUX_PMUXO_Pos) : (function << PORT_PMUX_PMUXE_Pos);

	// Set pin function
	PORT->Group[grp].PMUX[pmuxIdx].reg |= function;

	// CC0 -> WO[0], WO[4]
	// CC1 -> WO[1], WO[5]
	// CC2 -> WO[2], WO[6]
	// CC3 -> WO[3], WO[7]
	pwm::portPinNum0[idx] = apin;
  // invert PWM
  if (isinvert)
  {
    TCC0->WAVE.reg |= TCC_WAVE_POL(1 << idx);
    while (TCC0->SYNCBUSY.bit.WAVE);
  }
	pwm::duty0(idx, defDuty);
}
void pwm::add3(std::uint8_t idx, std::uint8_t pin, std::uint8_t defDuty)
{
	pin = ARDUINO_PIN_TO_PORT_PIN(pin);
	const auto apin = pin;
	const auto grp = PGrp(pin);
	pin &= 0x1F;
	// Enable peripheral multiplexer
	PORT->Group[grp].PINCFG[pin].reg |= PORT_PINCFG_PMUXEN;
	
	const auto pmuxIdx = pin / 2;

	/* Function E:
	 * PA14, PA15, PA18, PA19
	 */

	// Even/odd function selection shifting
	std::uint32_t function = (pin % 2) ? (PORT_PMUX_PMUXO_E_Val << PORT_PMUX_PMUXO_Pos) : (PORT_PMUX_PMUXO_E_Val << PORT_PMUX_PMUXE_Pos);

	// Set pin function
	PORT->Group[grp].PMUX[pmuxIdx].reg |= function;

	// CC0 -> WO[0], WO[4]
	// CC1 -> WO[1], WO[5]
	pwm::portPinNum3[idx] = apin;
	pwm::duty3(idx, defDuty);
}
void pwm::duty0(std::uint8_t idx, std::uint8_t duty)
{
	TCC0->CCB[idx].reg = (TCC0->PER.reg * std::uint32_t(duty)) / 255U;
	while (TCC0->SYNCBUSY.vec.CCB);
}
void pwm::duty0(std::uint8_t idx, float duty)
{
  duty = (duty > 1.0f) ? 1.0f : duty;
	TCC0->CCB[idx].reg = std::uint32_t((float)TCC0->PER.reg * duty + 0.5f);
	while (TCC0->SYNCBUSY.vec.CCB);
}
std::uint8_t pwm::getduty0(std::uint8_t idx)
{
  return std::uint8_t((TCC0->CCB[idx].reg * 255U) / TCC0->PER.reg);
}
float pwm::getduty0f(std::uint8_t idx)
{
  return float(TCC0->CCB[idx].reg) / float(TCC0->PER.reg);
}
void pwm::duty3(std::uint8_t idx, std::uint8_t duty)
{
	TC3->COUNT8.CC[idx].reg = duty;
	while (TC3->COUNT8.STATUS.bit.SYNCBUSY);
}
void pwm::duty3(std::uint8_t idx, float duty)
{
  duty = (duty > 1.0f) ? 1.0f : duty;
	TC3->COUNT8.CC[idx].reg = std::uint8_t(255.f * duty + 0.5f);
	while (TC3->COUNT8.STATUS.bit.SYNCBUSY);
}
std::uint8_t pwm::getduty3(std::uint8_t idx)
{
  return std::uint8_t(TC3->COUNT8.CC[idx].reg);
}
float pwm::getduty3f(std::uint8_t idx)
{
  return float(TC3->COUNT8.CC[idx].reg) / 255.f;
}
void pwm::remove0(std::uint8_t idx)
{
	const auto pin = pwm::portPinNum0[idx];
	// Disable wave output on pin
	PORT->Group[PGrp(pin)].PINCFG[pin & 0x1F].reg &= ~PORT_PINCFG_PMUXEN;
}
void pwm::remove3(std::uint8_t idx)
{
	const auto pin = pwm::portPinNum3[idx];
	// Disable wave output on pin
	PORT->Group[PGrp(pin)].PINCFG[pin & 0x1F].reg &= ~PORT_PINCFG_PMUXEN;
}
