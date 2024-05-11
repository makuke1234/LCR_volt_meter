#include "ADC.hpp"

adc::AdcCalData adc::calData;
volatile bool s_freerun = false, s_sampleBusy = false;

std::uint16_t adc::adcResArr[std::uint8_t(adc::Channel::size)] = { 0 };
volatile std::uint8_t adc::adcReady = UINT8_MAX;

static void syncgclk()
{
	while (GCLK->STATUS.bit.SYNCBUSY);
}
#define syncadc() while (ADC->STATUS.bit.SYNCBUSY)


void ADC_Handler()
{
  const std::uint8_t ch = std::uint8_t(ADC->INPUTCTRL.bit.MUXPOS);
  std::uint8_t idx = 0;
  switch (ch)
  {
  case adc::pinToMux(OUT_MEAS_ADC):
    idx = std::uint8_t(adc::Channel::OutMeas);
    break;
  case adc::pinToMux(BTN_ADC):
    idx = std::uint8_t(adc::Channel::Btn);
    break;
  case 0x18:
    idx = std::uint8_t(adc::Channel::IntTemp);
    break;
  case 0x19:
    idx = std::uint8_t(adc::Channel::CalRef);
    break;
  case 0x1B:
    idx = std::uint8_t(adc::Channel::IOSupply_1_4);
    break;
  case 0x1A:
    idx = std::uint8_t(adc::Channel::CoreSupply_1_4);
    break;
  default:
    assert(!"Invalid adc channel!");
  }
  adc::adcResArr[idx] = ADC->RESULT.reg;
  adc::adcReady = idx;

  ADC->INTFLAG.reg = ADC_INTENSET_RESRDY;
}

static std::uint16_t s_sample(bool async = false)
{
  if (async && s_freerun)
  {
    if (adc::adcReady == UINT8_MAX)
    {
      return 0;
    }
    return adc::adcResArr[adc::adcReady];
  }

  if (s_freerun)
  {
    adc::adcReady = UINT8_MAX;
    while (adc::adcReady == UINT8_MAX);
    return adc::adcResArr[adc::adcReady];
  }
  else
  {
    ADC->SWTRIG.bit.START = 1;			// Initiate software trigger to start ADC conversion
    syncadc();
    while (!ADC->INTFLAG.bit.RESRDY);	// Wait for conversion
    ADC->INTFLAG.bit.RESRDY = 1;		// Clear interrupt flag
    syncadc();

    return ADC->RESULT.reg;
  }
}

/*std::uint16_t adc::toCounts(std::uint16_t val, std::uint8_t resolution)
{
	const auto resDelta = std::int8_t(resolution) - ADC_RESOLUTION_BITS;
	if (!resDelta)
	{
		return val;
	}
	else if (resDelta < 0)
	{
		return (val >> (-resDelta));
	}
	else
	{
		return (val << resDelta);
	}
}
std::uint16_t adc::fromCounts(std::uint16_t val, std::uint8_t resolution)
{
	return adc::toCounts(val, 2 * ADC_RESOLUTION_BITS - resolution);
}*/

adc::LogRow::LogRow()
{
	const auto regVal1 = ADC_CAL_LOG_ROW_READ(0), regVal2 = ADC_CAL_LOG_ROW_READ(1);

	this->roomTempValInt = ADC_CAL_EXTRACT(regVal1, ADC_CAL_ROOM_TEMP_VAL_INT);
	this->roomTempValDec = ADC_CAL_EXTRACT(regVal1, ADC_CAL_ROOM_TEMP_VAL_DEC);
	this->hotTempValInt  = ADC_CAL_EXTRACT(regVal1, ADC_CAL_HOT_TEMP_VAL_INT);
	this->hotTempValDec  = ADC_CAL_EXTRACT(regVal1, ADC_CAL_HOT_TEMP_VAL_DEC);
	this->roomInt1VVal   = ADC_CAL_EXTRACT(regVal1, ADC_CAL_ROOM_INT1V_VAL);
	this->hotInt1VVal    = ADC_CAL_EXTRACT(regVal2, ADC_CAL_HOT_INT1V_VAL);
	this->roomAdcVal     = ADC_CAL_EXTRACT(regVal2, ADC_CAL_ROOM_ADC_VAL);
	this->hotAdcVal      = ADC_CAL_EXTRACT(regVal2, ADC_CAL_HOT_ADC_VAL);

	const auto calcref = [](std::int8_t val) -> float
	{
		return 1.0f - float(val) / 1000.f;
	};

	// Calculate constants
	this->INT1VH = calcref(this->hotInt1VVal);
	this->INT1VR = calcref(this->roomInt1VVal);
	this->tempH  = float(this->hotTempValInt)  + float(this->hotTempValDec)  / 10.0f;
	this->tempR  = float(this->roomTempValInt) + float(this->roomTempValDec) / 10.0f;
	this->VADCH  = float(this->hotAdcVal)  / (this->INT1VH * this->maxCountsFloat);
	this->VADCR  = float(this->roomAdcVal) / (this->INT1VR * this->maxCountsFloat);
}

adc::AdcCalData::AdcCalData()
{
  this->tempHR = this->lr.tempH - this->lr.tempR;
  this->VADCHR = this->lr.VADCH - this->lr.VADCR;
}

std::uint8_t adc::init(std::uint8_t adcResolution, bool freerun, std::uint16_t overSamplingSamples)
{
	std::uint8_t ret = adcResolution;

	// Set up ADC clock source

	GCLK->GENCTRL.reg |=
		GCLK_GENCTRL_ID(3) |	// Use GLK3, as it is already set up as 8 MHz internal OSC
		GCLK_GENCTRL_GENEN;

	syncgclk();

	// Set ADC to use GCLK3 as 8 Mhz
	GCLK->CLKCTRL.reg =
		GCLK_CLKCTRL_ID_ADC |
		GCLK_CLKCTRL_GEN_GCLK3 |
		GCLK_CLKCTRL_CLKEN;

	syncgclk();

	// Select ADC reference
	analogReference(AR_INTERNAL1V0);

	uint32_t bias      = (*((uint32_t *) ADC_FUSES_BIASCAL_ADDR) & ADC_FUSES_BIASCAL_Msk) >> ADC_FUSES_BIASCAL_Pos;
	uint32_t linearity = (*((uint32_t *) ADC_FUSES_LINEARITY_0_ADDR) & ADC_FUSES_LINEARITY_0_Msk) >> ADC_FUSES_LINEARITY_0_Pos;
	linearity |= ((*((uint32_t *) ADC_FUSES_LINEARITY_1_ADDR) & ADC_FUSES_LINEARITY_1_Msk) >> ADC_FUSES_LINEARITY_1_Pos) << 5;

	/* Wait for bus synchronization. */
	syncadc();

	/* Write the calibration data. */
	ADC->CALIB.reg = ADC_CALIB_BIAS_CAL(bias) | ADC_CALIB_LINEARITY_CAL(linearity);
	syncadc();

	// Configure ADC resolution
	std::uint32_t initialRes;

	switch (adcResolution)
	{
	case 8:
		initialRes = ADC_CTRLB_RESSEL_8BIT;
		break;
	case 10:
		initialRes = ADC_CTRLB_RESSEL_10BIT;
		break;
	case 12:
	case 13:
	case 14:
	case 15:
	case 16:
		initialRes = ADC_CTRLB_RESSEL_12BIT;
		break;
	default:
		assert(!"Invalid ADC resolution!");
	}

	ADC->CTRLB.reg = ADC_CLK_DIV | initialRes;	// Select initial ADC resolution & peripheral clock divider
	syncadc();

	if ((adcResolution > 12) || ((adcResolution == 12) && overSamplingSamples))
	{
		// Enable averaging mode
		std::uint32_t sNum;

		if (overSamplingSamples)
		{
			switch (overSamplingSamples)
			{
			case 1:
				sNum = ADC_AVGCTRL_SAMPLENUM_1;
				overSamplingSamples = 0;
				break;
			case 2:
				sNum = ADC_AVGCTRL_SAMPLENUM_2;
				overSamplingSamples = 1;
				break;
			case 4:
				sNum = ADC_AVGCTRL_SAMPLENUM_4;
				overSamplingSamples = 2;
				break;
			case 8:
				sNum = ADC_AVGCTRL_SAMPLENUM_8;
				overSamplingSamples = 3;
				break;
			case 16:
				sNum = ADC_AVGCTRL_SAMPLENUM_16;
				overSamplingSamples = 4;
				break;
			case 32:
				sNum = ADC_AVGCTRL_SAMPLENUM_32;
				overSamplingSamples = 5;
				break;
			case 64:
				sNum = ADC_AVGCTRL_SAMPLENUM_64;
				overSamplingSamples = 6;
				break;
			case 128:
				sNum = ADC_AVGCTRL_SAMPLENUM_128;
				overSamplingSamples = 7;
				break;
			case 256:
				sNum = ADC_AVGCTRL_SAMPLENUM_256;
				overSamplingSamples = 8;
				break;
			case 512:
				sNum = ADC_AVGCTRL_SAMPLENUM_512;
				overSamplingSamples = 9;
				break;
			case 1024:
				sNum = ADC_AVGCTRL_SAMPLENUM_1024;
				overSamplingSamples = 10;
				break;
			default:
				assert(!"Invalid number of oversampling samples!");
			}
		}
		else
		{
			switch (adcResolution)
			{
			case 12:
				sNum = ADC_AVGCTRL_SAMPLENUM_2;
				overSamplingSamples = 1;
				break;
			case 13:
				sNum = ADC_AVGCTRL_SAMPLENUM_4;
				overSamplingSamples = 2;
				break;
			case 14:
				sNum = ADC_AVGCTRL_SAMPLENUM_16;
				overSamplingSamples = 4;
				break;
			case 15:
				sNum = ADC_AVGCTRL_SAMPLENUM_64;
				overSamplingSamples = 6;
				break;
			case 16:
				sNum = ADC_AVGCTRL_SAMPLENUM_256;
				overSamplingSamples = 8;
				break;
			default:
				assert(!"This error is impossible to produce!");
			}
		}

		/*
			Oversampling ADJRES config table for 12 ... 16 bits output

			12 bits:
			0 -> 1 sample   -> 0
			1 -> 2 samples  -> 1
			2 -> 4 samples  -> 2
			3 -> 8 samples  -> 3
			4 -> 16 samples -> 4
			...

			13 bits:
			1 -> (2 samples)-> 0
			2 -> 4 samples  -> 1
			3 -> 8 samples  -> 2
			4 -> 16 samples -> 3
			5 -> 32 samples -> 3
			6 -> 64 samples -> 3
			...
			
			14 bits:
			2 -> (4 samples) -> 0
			3 -> (8 samples) -> 1
			4 -> 16 samples  -> 2
			5 -> 32 samples  -> 2
			6 -> 64 samples  -> 2
			7 -> 128 samples -> 2
			8 -> 256 samples -> 2
			...

			15 bits:
			5  -> (32 samples) -> 0
			6  -> 64  samples  -> 1
			7  -> 128 samples  -> 1
			8  -> 256 samples  -> 1
			9  -> 512 samples  -> 1
			10 -> 1024 samples -> 1

			16 bits:
			8  -> 256 samples  -> 0
			9  -> 512 samples  -> 0
			10 -> 1024 samples -> 0

		*/

		adcResolution -= 12;

		std::int8_t adjRes = algo::clamp<std::int8_t>(overSamplingSamples, 0, 4) - adcResolution;
		adjRes = algo::clamp<std::int8_t>(adjRes, 0, 4);
		// Calculate real achievable resolution
		const auto minOverSample = 2 * adcResolution;
		ret -= algo::clamp<std::int8_t>((minOverSample - overSamplingSamples) / 2, 0, 4);

		adcResolution += 12;


		ADC->AVGCTRL.reg = sNum | ADC_AVGCTRL_ADJRES(adjRes);	// Select averaging amount
		syncadc();
		ADC->CTRLB.reg |= ADC_CTRLB_RESSEL_16BIT;	// Activate averaging mode by selecting 16 bit resolution
		syncadc();
	}

	// Calibrate ADC only if setting up was successful
	if (ret == adcResolution)
	{
    SERIAL.println("Starting ADC...");
		adc::startAdc(freerun);
		adc::setSamplingTime(63);
    SERIAL.println("Sampling...");
		const auto tempSample = adc::sample(Channel::IntTemp, true);
		adc::setSamplingTime(ADC_SAMPLETIME);
		adc::calibrate(tempSample, true);
		adc::getSupply(true);
	}

	return ret;
}

void adc::startAdc(bool freerun)
{
	// Enable ADC peripheral
	PM->APBCMASK.reg |= PM_APBCMASK_ADC;

	// Enable temperature sensor
	SYSCTRL->VREF.bit.TSEN = 1;
	
  s_freerun = freerun;
  if (ADC->CTRLB.bit.FREERUN != freerun)
  {
    ADC->CTRLB.bit.FREERUN = freerun;
    syncadc();
    
    if (freerun)
    {
      SERIAL.println("Enabling ADC IRQ...");
      ADC->INTENSET.reg |= ADC_INTENSET_RESRDY;
      syncadc();

      NVIC_EnableIRQ(ADC_IRQn);
      NVIC_SetPriority(ADC_IRQn, 0);
    }
    else
    {
      NVIC_DisableIRQ(ADC_IRQn);
    }
  }

	// Enable ADC
  SERIAL.println("ADC enable");
	ADC->CTRLA.bit.ENABLE = 0x01;
	syncadc();

  if (freerun)
  {
    SERIAL.println("SWTRIG");
    ADC->SWTRIG.bit.START = 1;
  }

	// The first sample is rubbish anyway
	//s_sample();
}
void adc::stopAdc()
{
	// Disable ADC
	ADC->CTRLA.bit.ENABLE = 0x00;
	syncadc();
	
	// Disable temp sensor
	SYSCTRL->VREF.bit.TSEN = 0;

	// Disable ADC peripheral clock to save power
	PM->APBCMASK.reg &= ~PM_APBCMASK_ADC;
}

void adc::setGain(Gain gainIdx)
{
	std::uint8_t gain;
	switch (gainIdx)
	{
	case Gain::g0x5:
		gain = 0xF;
		break;
	case Gain::g1x:
		gain = 0x0;
		break;
	case Gain::g2x:
		gain = 0x1;
		break;
	case Gain::g4x:
		gain = 0x2;
		break;
	case Gain::g8x:
		gain = 0x3;
		break;
	case Gain::g16x:
		gain = 0x4;
		break;
	default:
		assert(!"Invalid gain setting!");
	}

	adc::calData.gainIdx     = std::uint8_t(gainIdx);
	adc::calData.gainSetting = gain;
}
void adc::setSamplingTime(std::uint8_t time)
{
	assert(time < 64);

	ADC->SAMPCTRL.reg = time;
	syncadc();
}
std::uint16_t adc::sample(Channel channel, bool preciseTemp, bool diffMode)
{
  s_sampleBusy = true;

	preciseTemp &= (ADC_CLK_DIV != ADC_SLOW_CLK_DIV);
	const auto prescaler = ADC->CTRLB.bit.PRESCALER;

	if (preciseTemp)
	{
		ADC->CTRLB.bit.PRESCALER = ADC_SLOW_CLK_DIV >> ADC_CTRLB_PRESCALER_Pos;
		syncadc();
	}
	if (ADC->CTRLB.bit.DIFFMODE != diffMode)
	{
		ADC->CTRLB.bit.DIFFMODE = diffMode;
		syncadc();
	}

	std::uint8_t chPos, chNeg = 0x19;
	switch (channel)
	{
	case Channel::OutMeas:
		chPos = adc::pinToMux(OUT_MEAS_ADC);
		break;
	case Channel::Btn:
		chPos = adc::pinToMux(BTN_ADC);
		break;
	case Channel::IntTemp:
		chPos = 0x18;
		break;
	case Channel::Cal0:
		chPos = 0x00;
		chNeg = 0x00;
		break;
	case Channel::CalRef:
		chPos = 0x19;
		break;
	case Channel::IOSupply_1_4:
		chPos = 0x1B;
		break;
	case Channel::CoreSupply_1_4:
		chPos = 0x1A;
		chNeg = 0x18;
		break;
	default:
		assert(!"Invalid adc channel!");
	}

	ADC->INPUTCTRL.bit.MUXPOS = chPos;	// Select MUX channel
	syncadc();
	if (diffMode)
	{
		ADC->INPUTCTRL.bit.MUXNEG = (channel == Channel::IntTemp) ? 0x18 : chNeg;	// Select I/O ground as negative input, internal ground for temperature sensing
		syncadc();
	}
	
	// Select proper gain, temperature gain must be 1x
	ADC->INPUTCTRL.bit.GAIN = (channel == Channel::IntTemp) ? 0x00 : adc::calData.gainSetting;
	syncadc();

	if (preciseTemp)
	{
		s_sample();
	}

	const auto result = s_sample();
	if (preciseTemp)
	{
		ADC->CTRLB.bit.PRESCALER = prescaler;
		syncadc();
	}
  s_sampleBusy = false;

	return result;
}
void adc::sampleAsync(Channel channel, bool diffMode)
{
  if (s_sampleBusy)
  {
    return;
  }
	if (ADC->CTRLB.bit.DIFFMODE != diffMode)
	{
		ADC->CTRLB.bit.DIFFMODE = diffMode;
		syncadc();
	}

	std::uint8_t chPos, chNeg = 0x19;
	switch (channel)
	{
	case Channel::OutMeas:
		chPos = adc::pinToMux(OUT_MEAS_ADC);
		break;
	case Channel::Btn:
		chPos = adc::pinToMux(BTN_ADC);
		break;
	case Channel::IntTemp:
		chPos = 0x18;
		break;
	case Channel::CalRef:
		chPos = 0x19;
		break;
	case Channel::IOSupply_1_4:
		chPos = 0x1B;
		break;
	case Channel::CoreSupply_1_4:
		chPos = 0x1A;
		chNeg = 0x18;
		break;
	default:
		assert(!"Invalid adc channel!");
	}

  if (ADC->INPUTCTRL.bit.MUXPOS != chPos)
  {
    ADC->INPUTCTRL.bit.MUXPOS = chPos;	// Select MUX channel
    syncadc();
  }
	if (diffMode)
	{
		ADC->INPUTCTRL.bit.MUXNEG = (channel == Channel::IntTemp) ? 0x18 : chNeg;	// Select I/O ground as negative input, internal ground for temperature sensing
		syncadc();
	}
	
	// Select proper gain, temperature gain must be 1x
  const auto gain = (channel == Channel::IntTemp) ? 0x00 : adc::calData.gainSetting;
	if (ADC->INPUTCTRL.bit.GAIN != gain)
  {
    ADC->INPUTCTRL.bit.GAIN = gain;
    syncadc();
  }
}
float adc::getVolts(std::uint16_t sample)
{
  const float maxCounts = float(ADC_MAX_COUNTS) * adc::calData.gainCal[adc::calData.gainIdx];
	/*SERIAL.print("Sample: ");
	SERIAL.print(sample);
	SERIAL.print("; Max counts: ");
	SERIAL.println(maxCounts >> 11);*/

	float volts = adc::calData.ref1VReal * float(sample) / maxCounts;

	return volts;
}
void adc::calibrate(std::uint16_t tempSample, bool fullCal)
{
	if (fullCal)
	{
		// Calibrate zero
		const auto oldGain = adc::calData.gainIdx;

    adc::setGain(Gain::g1x);
		std::uint32_t refMax    = adc::sample(Channel::IOSupply_1_4, true);
		adc::setGain(Gain::g0x5);
		std::uint32_t refCounts = adc::sample(Channel::IOSupply_1_4, true);

		adc::calData.gainCal[std::uint8_t(Gain::g0x5)] = (float)refCounts / (float)refMax;

		adc::setGain(Gain::g1x);
		refMax    = adc::sample(Channel::CoreSupply_1_4, true);
		adc::setGain(Gain::g2x);
		refCounts = adc::sample(Channel::CoreSupply_1_4, true);
		
		adc::calData.gainCal[std::uint8_t(Gain::g2x)] = (float)refCounts / (float)refMax;

		adc::setGain(Gain(oldGain));
	}
	
	// Slightly modified code from Atmel application note AT11481: ADC Configurations with Examples

	float INT1VM;	/* Voltage calculation for reality INT1V value during the ADC conversion */

	const auto & lr = adc::calData.lr;
	const auto coarse_temp = adc::getTemp(tempSample);

	INT1VM = lr.INT1VR + (((lr.INT1VH - lr.INT1VR) * (coarse_temp - lr.tempR)) / (adc::calData.tempHR));
	// Set new reference calibration value
	adc::calData.ref1VReal = INT1VM;
}
float adc::getTemp(std::uint16_t tempSample)
{
	const auto & lr = adc::calData.lr;
	const auto volts = adc::getVolts(tempSample);
	return lr.tempR + ((adc::calData.tempHR) * (volts - lr.VADCR)/(adc::calData.VADCHR));
}

float adc::getSupply(bool preciseMeas)
{
	const auto sample = adc::sample(Channel::IOSupply_1_4, preciseMeas);
  adc::calData.supplyVoltage = 4.0f * adc::getVolts(sample);

	return adc::calData.supplyVoltage;
}
