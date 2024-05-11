#include "usbheartbeat.hpp"

static bool s_isConnected = false;

void TC4_Handler()
{
	static std::uint32_t oldCount = 0;

	const auto count = USB->DEVICE.FNUM.bit.FNUM;
	s_isConnected = (count != oldCount);
	oldCount = count;

	// Clear overflow interrupt flag
	TC4->COUNT16.INTFLAG.bit.MC0 = 1;
}

void heartbeat::init()
{
	// Initialize timer TC4
	std::uint32_t tmrFreq = 96000000U;
	if (!clk::isInit(clk::tmr::tTC4))
	{
		clk::initTmr(clk::tmr::tTC4, GCLK_CLKCTRL_GEN_GCLK4_Val, tmrFreq);
	}
	else
	{
		tmrFreq = clk::tmrSpeed(clk::tmr::tTC4);
	}

	// Set up TC4 prescaler according to frequency, try to get frequency as close to 10000 Hz as possible
	// Also set TC4 into 16 bit mode
	const auto frequency = 200U;
	std::uint32_t prescaler = (tmrFreq / frequency) / 65535U + 1U;
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

	TC4->COUNT16.CTRLA.bit.PRESCALER = prescaler_reg;
	TC4->COUNT16.CTRLA.reg |=
		TC_CTRLA_MODE_COUNT16 |
		TC_CTRLA_WAVEGEN_MFRQ;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

	// Set up TC4 period register according to frequency, get it to fire every ~5 milliseconds
	std::uint32_t period = tmrFreq / (prescaler * frequency) - 1;
	period = (period > 65535U) ? 65535U : period;

	TC4->COUNT16.CC[0].reg = std::uint16_t(period);
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable TC4 interrupts
	NVIC_SetPriority(TC4_IRQn, 3);
	NVIC_EnableIRQ(TC4_IRQn);
	TC4->COUNT16.INTENSET.bit.MC0 = 1;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

	// Enable TC4
	TC4->COUNT16.CTRLA.bit.ENABLE = 1;
	while (TC4->COUNT16.STATUS.bit.SYNCBUSY);

	// Wait to get at least some pulses
	delay(7);
}
bool heartbeat::isConnected()
{
	return s_isConnected != 0;
}
