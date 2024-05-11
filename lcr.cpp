#include "lcr.hpp"
#include "pwm.hpp"

lcr::volts::MeterCalData lcr::volts::calData;
lcr::ohms::MeterCalData lcr::ohms::calData;
lcr::imp::MeterCalData lcr::imp::calData;

lcr::imp::MeterCalData::MeterCalData()
{
  this->calcParasiticImp(50);
}

std::int32_t lcr::autoScaleGetSample(
  void * caldata,
  lcr::sampleFunc_t sampleFunc,
  lcr::gainFunc_t gainFunc,
  float & oldgain, float maxgain, float mingain,
  std::int32_t minSample, std::int32_t maxSample,
  std::int32_t minSampleMinGain, std::int32_t maxSampleMinGain,
  bool precisemode
)
{
  auto gain = gainFunc(caldata, oldgain);
  std::int32_t sample, avgSample;
  avgSample = sampleFunc(caldata, precisemode, sample);

  const auto getMaxSample = [&]()
  {
    return (gain < mingain) ? maxSampleMinGain : maxSample;
  };
  const auto getMinSample = [&]()
  {
    return (gain < mingain) ? minSampleMinGain : minSample;
  };

  // calculate ideal gain
  if (((sample > 0) && (sample <= getMaxSample())) ||
      ((sample < 0) && (sample >= getMinSample())))
  {
    float insample = float(abs(sample)) / gain;
    gain = float((sample < 0) ? -getMinSample() : getMaxSample()) / insample;
    const auto tempgain = gain;
    gain = gainFunc(caldata, gain);
    if ((tempgain != gain) || (gain <= mingain))
    {
      avgSample = sampleFunc(caldata, precisemode, sample);
    }
  }
  
  // autogain algorithm, if input sample too large
  while ((sample > getMaxSample()) || (sample < getMinSample()))
  {
    auto prevgain = gain;
    gain = gainFunc(caldata, gain / 1.5f);
    if (prevgain == gain)
    {
      break;
    }
    avgSample = sampleFunc(caldata, precisemode, sample);
  }

  if (gain != oldgain)
  {
    oldgain = gain;
  }

  //SERIAL.print(sample);
  //SERIAL.print(", ");
  //SERIAL.print(gain);
  //SERIAL.print(", ");
  //SERIAL.println(digitalRead(OHMS_INVERT));

  return avgSample;
}



std::int32_t lcr::volts::sampleVolts(void * caldata, bool precisemode, std::int32_t & instantSample)
{
  instantSample = lcr::volts::sampleAdc(caldata, ADC_OHMS_CH, ADC_VB0V3_CH);
  return instantSample;
}
std::int32_t lcr::volts::sampleAdc(void * caldata, sdadc::Channel chpos, sdadc::Channel chneg)
{
  // do measurement
  bool suc = false;
  int32_t sample = sdadc::getSample(chpos, chneg, suc);
  
  if (sample <= -NEGATIVE_SWITCH_THRES)
  {
    // swap leads
    digitalWrite(OHMS_INVERT, !digitalRead(OHMS_INVERT));
    // do another measurement
    suc = false;
    sample = sdadc::getSample(chpos, chneg, suc);
  }

  return sample;
}

float lcr::volts::gainVolts(void * caldata, float gain)
{
  // like gainAdc, but also switches instrumentation amp gains

  // tries to get max gain first with inst. amp
  constexpr auto sz = std::uint8_t(lcr::volts::Gain::size);

  bool done = false;
  float g;
  std::uint8_t idx;
  if (g > lcr::volts::calData.igains[0])
  {
    for (std::uint8_t i = 0; i < sz; ++i)
    {
      float g = lcr::volts::calData.igains[sz-i-1];
      if (g <= gain)
      {
        done = true;
        idx = sz-i-1;
        break;
      }
    }
  }
  if (!done)
  {
    g = lcr::volts::calData.igains[0];
    idx = 0;
  }

  lcr::volts::calData.setGain(lcr::volts::Gain(idx));

  // solves residual gain with ADC
  return g * lcr::volts::gainAdc(caldata, gain / g);
}
float lcr::volts::gainAdc_(void * caldata, float gain, bool daisychained)
{
  if ((gain < ((2.4f / 3.3f) * sdadc::calData.gain[1])) && !daisychained)
  {
    digitalWrite(VOLTS_RANGE2, 1);
    return lcr::volts::calData.hvatten_gain * lcr::volts::gainAdc_(caldata, gain / lcr::volts::calData.hvatten_gain, true);
  }
  else
  {
    if (!daisychained)
    {
      digitalWrite(VOLTS_RANGE2, 0);
    }
    constexpr auto sz = std::uint8_t(sdadc::Gain::size);
    for (std::uint8_t i = 0; i < (sz - 0); ++i)
    {
      float g = sdadc::calData.gain[sz-i-1];
      if (g <= gain)
      {
        sdadc::setGain(sdadc::Gain(sz-i-1));
        return g;
      }
    }
  }

  sdadc::setGain(sdadc::Gain::g0x33);
  return sdadc::calData.gain[0];
}
float lcr::volts::gainAdc(void * caldata, float gain)
{
  return lcr::volts::gainAdc_(caldata, gain, false);
}

void lcr::volts::MeterCalData::setGain(lcr::volts::Gain gain)
{
  std::uint8_t idx = std::uint8_t(gain);

  digitalWrite(OHMS_22X,  (idx == 1) || (idx == 3));
  digitalWrite(OHMS_202X, (idx == 2) || (idx == 3));

  this->gain = gain;
}

void lcr::volts::MeterCalData::loadCal(const algo::array<float, VOLTS_CAL_SIZE> & data)
{
  this->igains[std::uint8_t(volts::Gain::g2x)]     = data.at(CAL_VOLTS_GAIN2X_ID);
  this->igains[std::uint8_t(volts::Gain::g22x)]    = data.at(CAL_VOLTS_GAIN22X_ID);
  this->igains[std::uint8_t(volts::Gain::g202x)]   = data.at(CAL_VOLTS_GAIN202X_ID);
  this->igains[std::uint8_t(volts::Gain::g222x)]   = data.at(CAL_VOLTS_GAIN222X_ID);
  this->ioffsets[std::uint8_t(volts::Gain::g2x)]   = data.at(CAL_VOLTS_OFF2X_ID);
  this->ioffsets[std::uint8_t(volts::Gain::g22x)]  = data.at(CAL_VOLTS_OFF22X_ID);
  this->ioffsets[std::uint8_t(volts::Gain::g202x)] = data.at(CAL_VOLTS_OFF202X_ID);
  this->ioffsets[std::uint8_t(volts::Gain::g222x)] = data.at(CAL_VOLTS_OFF222X_ID);
  
  this->hvatten_gain   = data.at(CAL_VOLTS_HVATTEN_GAIN_ID);
  this->hvatten_offset = data.at(CAL_VOLTS_HVATTEN_OFF_ID);

  this->supplyDiv_gain   = data.at(CAL_VOLTS_SDIV_GAIN_ID);
  this->rsupplyDiv_gain  = 1.0f / this->supplyDiv_gain;
  this->supplyDiv_offset = data.at(CAL_VOLTS_SDIV_OFF_ID);
}
void lcr::volts::MeterCalData::saveCal(algo::array<float, VOLTS_CAL_SIZE> & data) const
{
  data[CAL_VOLTS_GAIN2X_ID]   = this->igains[std::uint8_t(volts::Gain::g2x)];
  data[CAL_VOLTS_GAIN22X_ID]  = this->igains[std::uint8_t(volts::Gain::g22x)];
  data[CAL_VOLTS_GAIN202X_ID] = this->igains[std::uint8_t(volts::Gain::g202x)];
  data[CAL_VOLTS_GAIN222X_ID] = this->igains[std::uint8_t(volts::Gain::g222x)];
  data[CAL_VOLTS_OFF2X_ID]    = this->ioffsets[std::uint8_t(volts::Gain::g2x)];
  data[CAL_VOLTS_OFF22X_ID]   = this->ioffsets[std::uint8_t(volts::Gain::g22x)];
  data[CAL_VOLTS_OFF202X_ID]  = this->ioffsets[std::uint8_t(volts::Gain::g202x)];
  data[CAL_VOLTS_OFF222X_ID]  = this->ioffsets[std::uint8_t(volts::Gain::g222x)];

  data[CAL_VOLTS_HVATTEN_GAIN_ID] = this->hvatten_gain;
  data[CAL_VOLTS_HVATTEN_OFF_ID]  = this->hvatten_offset;

  data[CAL_VOLTS_SDIV_GAIN_ID] = this->supplyDiv_gain;
  data[CAL_VOLTS_SDIV_OFF_ID]  = this->supplyDiv_offset;
}

void lcr::volts::MeterCalData::selfCalOffset()
{
  const auto prevosr = sdadc::calData.osrRatio;
  sdadc::config(sdadc::OSR::osr8192, sdadc::calData.format);

  // enable 10 meg input
  digitalWrite(VOLTS_RANGE1, 1);

  // test without voltage divider

  this->setGain(volts::Gain::g2x);
  sdadc::setGain(sdadc::Gain::g16x);
  bool suc = false;
  auto sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts = sdadc::toVolts(sample);
  this->ioffsets[std::uint8_t(volts::Gain::g2x)] = volts;

  this->setGain(volts::Gain::g22x);
  //sdadc::setGain(adc::Gain::g16x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = sdadc::toVolts(sample);
  this->ioffsets[std::uint8_t(volts::Gain::g22x)] = volts;

  this->setGain(volts::Gain::g202x);
  sdadc::setGain(sdadc::Gain::g4x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = sdadc::toVolts(sample);
  this->ioffsets[std::uint8_t(volts::Gain::g202x)] = volts;

  this->setGain(volts::Gain::g222x);
  //sdadc::setGain(sdadc::Gain::g4x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = sdadc::toVolts(sample);
  this->ioffsets[std::uint8_t(volts::Gain::g222x)] = volts;

  // test with voltage divier
  digitalWrite(VOLTS_RANGE2, 1);

  this->setGain(volts::Gain::g22x);
  sdadc::setGain(sdadc::Gain::g2x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = sdadc::toVolts(sample) - this->ioffsets[std::uint8_t(volts::Gain::g22x)];
  auto offset = volts;

  this->setGain(volts::Gain::g2x);
  sdadc::setGain(sdadc::Gain::g16x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = sdadc::toVolts(sample) - this->ioffsets[std::uint8_t(volts::Gain::g2x)];
  this->igains[std::uint8_t(volts::Gain::g22x)] = this->igains[std::uint8_t(volts::Gain::g2x)] * offset / volts;
  this->hvatten_offset = volts;
  this->hvatten_offset /= this->igains[std::uint8_t(volts::Gain::g2x)];

  this->igains[std::uint8_t(volts::Gain::g222x)] = this->igains[std::uint8_t(volts::Gain::g202x)] + this->igains[std::uint8_t(volts::Gain::g22x)];

  //this->setGain(volts::Gain::g2x);
  sdadc::setGain(sdadc::Gain::g1x);
  sdadc::config(prevosr, sdadc::calData.format);

  digitalWrite(VOLTS_RANGE1, 0);
  digitalWrite(VOLTS_RANGE2, 0);
}
void lcr::volts::MeterCalData::selfCal1Volt()
{
  const auto prevosr = sdadc::calData.osrRatio;
  sdadc::config(sdadc::OSR::osr8192, sdadc::calData.format);

  // enable 10 meg input
  digitalWrite(VOLTS_RANGE1, 1);

  // test without voltage divider
  digitalWrite(VOLTS_RANGE2, 0);

  this->setGain(volts::Gain::g2x);
  sdadc::setGain(sdadc::Gain::g1x);
  bool suc = false;
  auto sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts1 = sdadc::toVolts(sample) - this->ioffsets[std::uint8_t(volts::Gain::g2x)];
  auto multiplier = volts1 / this->igains[std::uint8_t(volts::Gain::g2x)];
  this->igains[std::uint8_t(volts::Gain::g2x)] = volts1;

  for (std::uint8_t i = 1, sz = std::uint8_t(volts::Gain::size); i < sz; ++i)
  {
    this->igains[i] *= multiplier;
  }
  this->hvatten_offset /= multiplier;

  // test with voltage divier
  digitalWrite(VOLTS_RANGE2, 1);


  //this->setGain(volts::Gain::g2x);
  sdadc::setGain(sdadc::Gain::g16x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts1 = (sdadc::toVolts(sample) - this->ioffsets[std::uint8_t(volts::Gain::g2x)]) / this->igains[std::uint8_t(volts::Gain::g2x)];
  this->hvatten_gain = volts1 - this->hvatten_offset;

  this->setGain(volts::Gain::g22x);
  sdadc::setGain(sdadc::Gain::g1x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts1_22x = (sdadc::toVolts(sample) - this->ioffsets[std::uint8_t(volts::Gain::g22x)]);
  this->igains[std::uint8_t(volts::Gain::g22x)] = volts1_22x / volts1;

  //calculate 222x gain
  this->igains[std::uint8_t(volts::Gain::g222x)] = this->igains[std::uint8_t(volts::Gain::g202x)] + this->igains[std::uint8_t(volts::Gain::g22x)];

  this->setGain(volts::Gain::g2x);
  //sdadc::setGain(sdadc::Gain::g1x);
  sdadc::config(prevosr, sdadc::calData.format);

  digitalWrite(VOLTS_RANGE1, 0);
  digitalWrite(VOLTS_RANGE2, 0);
}



void lcr::volts::init()
{

}

float lcr::volts::zeroReading(float reading, bool enable)
{
  if (!enable)
  {
    lcr::volts::calData.zeroOffset = 0.0f;
    return 0.0f;
  }
  lcr::volts::calData.zeroOffset += reading;
  return lcr::volts::calData.zeroOffset;
}

bool lcr::volts::mode()
{
  if (!lcr::canChangeMode())
  {
    return false;
  }
 
  digitalWrite(VOLTS_RANGE1, 1);

  return true;
}
bool lcr::volts::modeThermo()
{
  if (!lcr::canChangeMode())
  {
    return false;
  }

  // leave floating

  return true;
}
float lcr::volts::measure(float * adcgain)
{
  auto sample = lcr::autoScaleGetSample(
    const_cast<lcr::volts::MeterCalData *>(&lcr::volts::calData),
    lcr::volts::calData.getSample,
    lcr::volts::calData.setGain_,
    const_cast<float &>(lcr::volts::calData.gainf),
    (lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::size) - 1]) *
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g64x)] + 0.5f,
    (lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::g2x)]) *
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] *
    lcr::volts::calData.hvatten_gain + 0.1f,
    -(MCP3461_MAX_COUNT_17BIT - 1), (MCP3461_MAX_COUNT_17BIT - 1),
    -(sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] * (MCP3461_MAX_COUNT_17BIT - 1)), (2.3 / 3.3f * (MCP3461_MAX_COUNT_17BIT - 1)),
    false
  );
  float volts = sdadc::toVolts(sample);
  if (adcgain != nullptr)
  {
    *adcgain = sdadc::calData.gain[std::uint8_t(sdadc::calData.gainSel)];
  }

  volts = (volts - lcr::volts::calData.ioffsets[std::uint8_t(lcr::volts::calData.gain)]) / lcr::volts::calData.igains[std::uint8_t(lcr::volts::calData.gain)];
  if (digitalRead(OHMS_INVERT))
  {
    volts = -volts;
  }
  if (digitalRead(VOLTS_RANGE2))
  {
    volts = (volts - lcr::volts::calData.hvatten_offset) / lcr::volts::calData.hvatten_gain;
  }
  return volts - lcr::volts::calData.zeroOffset;
}
float lcr::volts::measureFast(float * adcgain)
{
  // set instrumentation amp to lowest gain
  digitalWrite(OHMS_22X,  0);
  digitalWrite(OHMS_202X, 0);

  // measure
  sdadc::setGain(sdadc::Gain::g1x);
  int32_t sample2;
  auto sample = lcr::volts::calData.getSample(const_cast<lcr::volts::MeterCalData *>(&lcr::volts::calData), false, sample2);

  float volts = sdadc::toVolts(sample);
  if (adcgain != nullptr)
  {
    *adcgain = sdadc::calData.gain[std::uint8_t(sdadc::calData.gainSel)];
  }
  volts = volts / lcr::volts::calData.igains[0] - lcr::volts::calData.ioffsets[0];
  if (digitalRead(OHMS_INVERT))
  {
    volts = -volts;
  }
  if (digitalRead(VOLTS_RANGE2))
  {
    volts = (volts - lcr::volts::calData.hvatten_offset) / lcr::volts::calData.hvatten_gain;
  }
  return volts;
}
float lcr::volts::measureThermo(float & diff, bool & suc, float * adcGain)
{
  float tempvolts;
  return lcr::volts::measureThermo(tempvolts, diff, suc, adcGain);
}
float lcr::volts::measureThermo(float & tempvolts, float & diff, bool & suc, float * adcGain)
{
  tempvolts = lcr::volts::measure(adcGain);
  auto coldjunc = lcr::volts::mesaureColdJunction(suc);
  if (!suc)
  {
    return 0.0f;
  }

  // for type K thermocouple, S is about 4.1 uV/deg. C
  // https://www.omega.com/en-us/resources/thermocouple-junction-principles
  float S = 4.1e-6;
  auto tempthermo = tempvolts / S;
  diff = tempthermo;

  return tempthermo + coldjunc;
}
float lcr::volts::measureSupply(bool & suc, float * adcgain)
{
  static std::uint16_t measCount = 0;
  static float prevMeas = 0.0f;
  
  measCount = (measCount + 1) % 10;
  if (measCount != 1)
  {
    return prevMeas;
  }

  sdadc::setGain(sdadc::Gain::g4x);
  if (adcgain != nullptr)
  {
    *adcgain = sdadc::calData.gain[std::uint8_t(sdadc::calData.gainSel)];
  }
  auto sample = sdadc::getSample(ADC_VB0V3_CH, sdadc::Channel::agnd, suc);
  if (!suc)
  {
    return 0.0f;
  }

  float volts = sdadc::toVolts(sample);
  volts = volts * lcr::volts::calData.rsupplyDiv_gain - lcr::volts::calData.supplyDiv_offset;
  
  /*sdadc::setGain(sdadc::Gain::avdd);
  auto sample = sdadc::getSample(sdadc::Channel::avdd, sdadc::Channel::agnd, suc);
  if (!suc)
  {
    return 0.0f;
  }

  float volts = sdadc::toVolts(sample);*/
  prevMeas = volts;
  return volts;
}
float lcr::volts::mesaureColdJunction(bool & suc, float * adcgain)
{
  sdadc::setGain(sdadc::Gain::temp);
  if (adcgain != nullptr)
  {
    *adcgain = sdadc::calData.gain[std::uint8_t(sdadc::calData.gainSel)];
  }
  auto sample = sdadc::getSample(sdadc::Channel::temp_diode_p, sdadc::Channel::temp_diode_m, suc);
  if (!suc)
  {
    return 0.0f;
  }

  float temp = sdadc::toTemp(sample);
  return temp;
}



float lcr::ohms::gainOhms(void * caldata, float gain)
{
  return lcr::volts::gainVolts(caldata, gain);
}

void lcr::ohms::MeterCalData::loadCal(const algo::array<float, OHMS_CAL_SIZE> & data)
{
  this->gains[std::uint8_t(ohms::Gain::g3900r)]   = data.at(CAL_OHMS_GAIN3900R_ID);
  this->gains[std::uint8_t(ohms::Gain::g47r)]     = data.at(CAL_OHMS_GAIN47R_ID);
  this->offsets[std::uint8_t(ohms::Gain::g3900r)] = data.at(CAL_OHMS_OFF3900R_ID);
  this->offsets[std::uint8_t(ohms::Gain::g47r)]   = data.at(CAL_OHMS_OFF47R_ID);
}
void lcr::ohms::MeterCalData::saveCal(algo::array<float, OHMS_CAL_SIZE> & data) const
{
  data[CAL_OHMS_GAIN3900R_ID] = this->gains[std::uint8_t(ohms::Gain::g3900r)];
  data[CAL_OHMS_GAIN47R_ID]   = this->gains[std::uint8_t(ohms::Gain::g47r)];
  data[CAL_OHMS_OFF3900R_ID]  = this->offsets[std::uint8_t(ohms::Gain::g3900r)];
  data[CAL_OHMS_OFF47R_ID]    = this->offsets[std::uint8_t(ohms::Gain::g47r)];
}

void lcr::ohms::MeterCalData::selfCal()
{
  const auto rtest = 100.0f;
  const auto prevosr = sdadc::calData.osrRatio;
  sdadc::config(sdadc::OSR::osr8192, sdadc::calData.format);

  // set 3900r, known current, 1 mV over 100R -> set current to 10 uA -> 39 mV over 3900 R
  // set current
  digitalWrite(OHMS_CUR_RANGE, 1);
  pwm::duty0(OHMS_CC, algo::clamp((0.001f * this->gains[std::uint8_t(ohms::Gain::g3900r)] / rtest) / (float(DEF_OHMS_CUR_IN_DIV) * float(DEF_LCR_SUPPLY)), float(DEF_OHMS_DUTY_MIN), float(DEF_OHMS_DUTY_MAX)));
  lcr::ohms::calData.testdutyf = pwm::getduty0f(OHMS_CC);

  bool suc = false;
  const auto supply = lcr::volts::measureSupply(suc);
  // wait for the current to settle
  delay(50);


  // measure voltage @22x + 16x, 202x + 2x & 222x + 2x, calculate 202x and 222x voltage gains

  volts::calData.setGain(volts::Gain::g22x);
  sdadc::setGain(sdadc::Gain::g16x);
  suc = false;
  auto sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts22 = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g22x)]) / volts::calData.igains[std::uint8_t(volts::Gain::g22x)];

  volts::calData.setGain(volts::Gain::g202x);
  sdadc::setGain(sdadc::Gain::g2x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts202 = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g202x)]);
  volts::calData.igains[std::uint8_t(volts::Gain::g202x)] = volts202 / volts22;

  volts::calData.setGain(volts::Gain::g222x);
  //sdadc::setGain(sdadc::Gain::g2x);
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts222 = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g222x)]);
  volts::calData.igains[std::uint8_t(volts::Gain::g222x)] = volts222 / volts22;





  // measure offsets, voltage gain fixed to 22x, adc gain 16x
  // set current @ 10 mV over 3900r
  pwm::duty0(OHMS_CC, algo::clamp(0.01f / (float(DEF_OHMS_CUR_IN_DIV) * float(DEF_LCR_SUPPLY)), float(DEF_OHMS_DUTY_MIN), float(DEF_OHMS_DUTY_MAX)));
  lcr::ohms::calData.testdutyf = pwm::getduty0f(OHMS_CC);
  auto voltstest = lcr::ohms::calData.testdutyf * supply * float(DEF_OHMS_CUR_IN_DIV);
  delay(50);
  
  volts::calData.setGain(volts::Gain::g22x);
  sdadc::setGain(sdadc::Gain::g16x);

  // set 3900r, (almost) 0 current
  digitalWrite(OHMS_CUR_RANGE, 1);
  // measure offset
  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  auto volts = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g22x)]) / volts::calData.igains[std::uint8_t(volts::Gain::g22x)];

  this->offsets[std::uint8_t(ohms::Gain::g3900r)] = voltstest - volts * this->gains[std::uint8_t(ohms::Gain::g3900r)] / rtest;

  // set 47r, 0 current
  digitalWrite(OHMS_CUR_RANGE, 0);
  // measure offset

  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g22x)]) / volts::calData.igains[std::uint8_t(volts::Gain::g22x)];

  this->offsets[std::uint8_t(ohms::Gain::g47r)] = voltstest - volts * this->gains[std::uint8_t(ohms::Gain::g47r)] / rtest;


  // set current @120 uA -> 0.47 volts over 3900r -> 0.012 volts over 100R
  pwm::duty0(OHMS_CC, algo::clamp((1.0f * this->gains[std::uint8_t(ohms::Gain::g47r)] / rtest) / (float(DEF_OHMS_CUR_IN_DIV) * float(DEF_LCR_SUPPLY)), float(DEF_OHMS_DUTY_MIN), float(DEF_OHMS_DUTY_MAX)));
  lcr::ohms::calData.testdutyf = pwm::getduty0f(OHMS_CC);
  delay(50);

  // set 3900, known current
  digitalWrite(OHMS_CUR_RANGE, 1);
  voltstest = lcr::ohms::calData.testdutyf * supply * float(DEF_OHMS_CUR_IN_DIV) - this->offsets[std::uint8_t(ohms::Gain::g3900r)];
  // measure real current -> calc gain

  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g22x)]) / volts::calData.igains[std::uint8_t(volts::Gain::g22x)];

  this->gains[std::uint8_t(lcr::ohms::Gain::g3900r)] = rtest * voltstest / volts;

  // set 47r, known current, 0.47 volts over 47r -> ~10 mA -> 1 V over 100R
  digitalWrite(OHMS_CUR_RANGE, 0);
  voltstest = lcr::ohms::calData.testdutyf * supply * float(DEF_OHMS_CUR_IN_DIV) - this->offsets[std::uint8_t(ohms::Gain::g47r)];
  // measure real current -> calc gain

  suc = false;
  sample = sdadc::getSample(ADC_OHMS_CH, ADC_VB0V3_CH, suc);
  volts = (sdadc::toVolts(sample) - volts::calData.ioffsets[std::uint8_t(volts::Gain::g22x)]) / volts::calData.igains[std::uint8_t(volts::Gain::g22x)];

  this->gains[std::uint8_t(lcr::ohms::Gain::g47r)] = rtest * voltstest / volts;

  volts::calData.setGain(volts::Gain::g2x);
  sdadc::setGain(sdadc::Gain::g1x);
  sdadc::config(prevosr, sdadc::calData.format);

  digitalWrite(OHMS_CUR_RANGE, 1);
}


void lcr::ohms::init()
{

}

float lcr::ohms::zeroReading(float reading, bool enable)
{
  if (!enable)
  {
    lcr::ohms::calData.zeroOffset = 0.0f;
    return 0.0f;
  }
  lcr::ohms::calData.zeroOffset += reading;
  return lcr::ohms::calData.zeroOffset;
}

bool lcr::ohms::mode()
{
  if (!lcr::canChangeMode())
  {
    return false;
  }

  // leave floating

  return true;
}
float lcr::ohms::measure()
{
  auto sample = lcr::autoScaleGetSample(
    const_cast<lcr::ohms::MeterCalData *>(&lcr::ohms::calData),
    lcr::ohms::calData.getSample,
    lcr::ohms::calData.setGain,
    const_cast<float &>(lcr::ohms::calData.gainf),
    (lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::size) - 1]) *
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g64x)] + 0.5f,
    (lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::g2x)]) *
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] + 0.1f,
    -(MCP3461_MAX_COUNT_17BIT - 1), (MCP3461_MAX_COUNT_17BIT - 1),
    -(sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] * (MCP3461_MAX_COUNT_17BIT - 1)), (2.3 / 3.3f * (MCP3461_MAX_COUNT_17BIT - 1)),
    false
  );
  float voltsres = sdadc::toVolts(sample);
  //SERIAL.println(voltsres);
  
  voltsres = voltsres / lcr::volts::calData.igains[std::uint8_t(lcr::volts::calData.gain)] - lcr::volts::calData.ioffsets[std::uint8_t(lcr::volts::calData.gain)];
  // get duty cycle
  bool suc = false;
  const auto testcurrent = lcr::ohms::getcurrent(suc);

  // resistance is R = U / I
  return voltsres / testcurrent - lcr::ohms::calData.zeroOffset;
}
float lcr::ohms::getcurrent(bool & suc)
{
  const auto supply = lcr::volts::measureSupply(suc);

  auto voltstest = lcr::ohms::calData.testdutyf * supply * float(DEF_OHMS_CUR_IN_DIV);
  const auto range = !digitalRead(OHMS_CUR_RANGE);
  auto testcurrent = (voltstest - lcr::ohms::calData.offsets[range]) / lcr::ohms::calData.gains[range];

  return testcurrent;
}
float lcr::ohms::setcurrent(float cur)
{
  bool suc = false;
  const auto supply = lcr::volts::measureSupply(suc);

  // calculate first range max test current
  const float maxcur = ( ( float(DEF_OHMS_DUTY_MAX) * supply * float(DEF_OHMS_CUR_IN_DIV) ) -
    lcr::ohms::calData.offsets[std::uint8_t(lcr::ohms::Gain::g3900r)] ) /
    lcr::ohms::calData.gains[std::uint8_t(lcr::ohms::Gain::g3900r)];

  const bool range = (cur > maxcur);
  digitalWrite(OHMS_CUR_RANGE, !range);

  const auto gain = lcr::ohms::calData.gains[range];
  const auto off  = lcr::ohms::calData.offsets[range];
  // calculate correct dutycycle for desired current "cur"
  float duty = (cur * gain + off) / ( supply * float(DEF_OHMS_CUR_IN_DIV) );
  pwm::duty0(OHMS_CC, algo::clamp(duty, float(DEF_OHMS_DUTY_MIN), float(DEF_OHMS_DUTY_MAX)));


  // also stores test current duty cycle internally
  lcr::ohms::calData.testdutyf = pwm::getduty0f(OHMS_CC);
  auto voltstest = lcr::ohms::calData.testdutyf * supply * float(DEF_OHMS_CUR_IN_DIV);
  auto testcurrent = (voltstest - off) / gain;

  //return testcurrent; 
}



std::int32_t lcr::imp::sampleCap(void * caldata, bool precisemode, std::int32_t & instantSample)
{
  instantSample = lcr::volts::sampleAdc(caldata, ADC_CAP_CH, ADC_VB0V3_CH);
  return instantSample;
}
std::int32_t lcr::imp::sampleCurrent(void * caldata, bool precisemode, std::int32_t & instantSample)
{
  instantSample = lcr::volts::sampleAdc(caldata, ADC_CURRENT_CH, ADC_VB0V3_CH);
  return instantSample;
}
std::int32_t lcr::imp::sampleOut(void * caldata, bool precisemode, std::int32_t & instantSample)
{
  instantSample = adc::sample(adc::Channel::OutMeas, precisemode);
  return instantSample;
}

float lcr::imp::gainCap(void * caldata, float gain)
{
  return lcr::imp::calData.gainVolt * lcr::volts::gainAdc(caldata, gain * lcr::imp::calData.rgainVolt);
}
float lcr::imp::gainCurrent(void * caldata, float gain)
{
  return lcr::imp::calData.gainCur * lcr::volts::gainAdc(caldata, gain * lcr::imp::calData.rgainCur);
}
float lcr::imp::gainOut(void * caldata, float gain)
{
  for (std::uint8_t i = 0; i < 6; ++i)
  {
    float g = adc::calData.gainCal[6-i-1];
    if (g <= gain)
    {
      adc::setGain(adc::Gain(6-i-1));
      return g;
    }
  }

  adc::setGain(adc::Gain::g0x5);
  return adc::calData.gainCal[0];
}

void lcr::imp::MeterCalData::loadCal(const algo::array<float, IMP_CAL_SIZE> & data)
{
  this->gainCur     = data.at(CAL_IMP_GAINCUR_ID);
  this->rgainCur    = 1.0f / this->gainCur;
  this->gainVolt    = data.at(CAL_IMP_GAINVOLT_ID);
  this->rgainVolt   = 1.0f / this->gainVolt;
  this->offCur      = data.at(CAL_IMP_OFFCUR_ID);
  this->offVolt     = data.at(CAL_IMP_OFFVOLT_ID);
  this->shuntRes[0] = data.at(CAL_IMP_SHUNT1_ID);
  this->shuntRes[1] = data.at(CAL_IMP_SHUNT2_ID);
  this->shuntRes[2] = data.at(CAL_IMP_SHUNT3_ID);
  this->c1          = data.at(CAL_IMP_C1_ID);
  this->c2          = data.at(CAL_IMP_C2_ID);
  this->offRes      = data.at(CAL_IMP_OFFRES_ID);
  this->offCap      = data.at(CAL_IMP_OFFCAP_ID);
  this->offInd      = data.at(CAL_IMP_OFFIND_ID);
}
void lcr::imp::MeterCalData::saveCal(algo::array<float, IMP_CAL_SIZE> & data) const
{
  data[CAL_IMP_GAINCUR_ID]  = this->gainCur;
  data[CAL_IMP_GAINVOLT_ID] = this->gainVolt;
  data[CAL_IMP_OFFCUR_ID]   = this->offCur;
  data[CAL_IMP_OFFVOLT_ID]  = this->offVolt;
  data[CAL_IMP_SHUNT1_ID]   = this->shuntRes[0];
  data[CAL_IMP_SHUNT2_ID]   = this->shuntRes[1];
  data[CAL_IMP_SHUNT3_ID]   = this->shuntRes[2];
  data[CAL_IMP_C1_ID]       = this->c1;
  data[CAL_IMP_C2_ID]       = this->c2;
  data[CAL_IMP_OFFRES_ID]   = this->offRes;
  data[CAL_IMP_OFFCAP_ID]   = this->offCap;
  data[CAL_IMP_OFFIND_ID]   = this->offInd;
}

void lcr::imp::MeterCalData::selfCal()
{
  // set generator to output 1 kHz
  lcrlogic::state.genOut(1000U);

  const auto rtest = 100.0f;
  const auto prevosr = sdadc::calData.osrRatio;
  sdadc::config(sdadc::OSR::osr8192, sdadc::calData.format);

  // enable LCR output
  digitalWrite(LCR_ENABLE, 1);


  // test 100k (default) @ 1 kHz
  //lcr::imp::calData.selRange = 0;
  lcr::imp::setrange(0);

  bool suc = false;
  auto supply = lcr::volts::measureSupply(suc);

  // wait for generator to settle
  delay(1000);

  // measure
  auto vdut = lcr::imp::measureVolts();
  auto vshunt = lcr::imp::measureCur();

  float resistance, reactance;
  lcr::imp::calcMeas1(vdut, vshunt, supply, resistance, reactance);
  lcr::imp::calData.shuntRes[lcr::imp::calData.selRange] *= rtest / resistance;

  // test 1k @ 1 kHz
  lcr::imp::setrange(1);
  delay(1000);

  // measure
  vdut = lcr::imp::measureVolts();
  vshunt = lcr::imp::measureCur();
  
  lcr::imp::calcMeas1(vdut, vshunt, supply, resistance, reactance);
  lcr::imp::calData.shuntRes[lcr::imp::calData.selRange] *= rtest / resistance;

  // test 10 ohms @ 1 kHz
  lcr::imp::setrange(2);
  delay(1000);

  // measure
  vdut = lcr::imp::measureVolts();
  vshunt = lcr::imp::measureCur();

  lcr::imp::calcMeas1(vdut, vshunt, supply, resistance, reactance);
  lcr::imp::calData.shuntRes[lcr::imp::calData.selRange] *= rtest / resistance;

  // restore adc conf
  sdadc::setGain(sdadc::Gain::g1x);
  sdadc::config(prevosr, sdadc::calData.format);

  // disable LCR output
  digitalWrite(LCR_ENABLE, 0);
  lcr::imp::setrange(0);
}

float lcr::imp::MeterCalData::getShuntRes() const
{
  switch (this->selRange)
  {
  case 0:
    return shuntRes[0];
  case 1:
  case 2:
    return 1.0f / (1.0f / shuntRes[0] + 1.0f / shuntRes[this->selRange]);
  }
}

void lcr::imp::MeterCalData::calcParasiticImp(float frequency)
{
  // (1/c2) / (1/c1 + 1/c2) = c1 / (c1+c2)
  this->parFactor = (this->c1 == 0.0f && this->c2 == 0.0) ? 1.0f : this->c1 / (this->c1 + this->c2);

  // zc1, zc2, zcpar;
  this->zc1 = (this->c1 == 0.0f) ? 1e9 : 1.0f / (2.0f * float(M_PI) * frequency * this->c1);
  this->zc2 = (this->c2 == 0.0f) ? 1e9 : 1.0f / (2.0f * float(M_PI) * frequency * this->c2);

  this->zcpar = 1.0f / (1.0f / this->zc1 + 1.0f / this->zc2);
}

void lcr::imp::init()
{

}

float lcr::imp::zeroReadingRes(float reading, bool enable)
{
  if (!enable)
  {
    lcr::imp::calData.offRes = 0.0f;
    return 0.0f;
  }
  lcr::imp::calData.offRes += reading;
  return lcr::imp::calData.offRes;
}
float lcr::imp::zeroReadingCap(float reading, bool enable)
{
  if (!enable)
  {
    lcr::imp::calData.offCap = 0.0f;
    return 0.0f;
  }
  lcr::imp::calData.offCap += reading;
  return lcr::imp::calData.offCap;
}
float lcr::imp::zeroReadingInd(float reading, bool enable)
{
  if (!enable)
  {
    lcr::imp::calData.offInd = 0.0;
    return 0.0f;
  }
  lcr::imp::calData.offInd += reading;
  return lcr::imp::calData.offInd;
}

bool lcr::imp::mode()
{
  if (!lcr::canChangeMode())
  {
    return false;
  }

  // enable LCR output
  digitalWrite(LCR_ENABLE, 1);

  // resort back to range2
  lcr::imp::setrange(1);

  return true;
}
float lcr::imp::measureVolts()
{
  /*auto samplecap = lcr::autoScaleGetSample(
    const_cast<lcr::imp::MeterCalData *>(&lcr::imp::calData),
    lcr::imp::calData.getSample1,
    lcr::imp::calData.setGain1,
    const_cast<float &>(lcr::imp::calData.gainCapf),
    lcr::imp::calData.gainCap *
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g64x)] + 0.5f,
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] + 0.1f,
    -(MCP3461_MAX_COUNT_17BIT - 1), (MCP3461_MAX_COUNT_17BIT - 1),
    -(sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] * (MCP3461_MAX_COUNT_17BIT - 1)), (2.3 / 3.3f * (MCP3461_MAX_COUNT_17BIT - 1)),
    false
  );*/

  lcr::imp::calData.setGain1(&lcr::imp::calData, 1.0f);
  std::int32_t samplecap;
  lcr::imp::calData.getSample1(&lcr::imp::calData, false, samplecap);

  auto voltscap = sdadc::toVolts(samplecap);
  return voltscap;
}
float lcr::imp::measureCur()
{
  /*auto samplecur = lcr::autoScaleGetSample(
    const_cast<lcr::imp::MeterCalData *>(&lcr::imp::calData),
    lcr::imp::calData.getSample2,
    lcr::imp::calData.setGain2,
    const_cast<float &>(lcr::imp::calData.gainCurrentf),
    lcr::imp::calData.gainCur *
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g64x)] + 0.5f,
    sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] + 0.1f,
    -(MCP3461_MAX_COUNT_17BIT - 1), (MCP3461_MAX_COUNT_17BIT - 1),
    -(sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)] * (MCP3461_MAX_COUNT_17BIT - 1)), (2.3 / 3.3f * (MCP3461_MAX_COUNT_17BIT - 1)),
    false
  );*/

  lcr::imp::calData.setGain2(&lcr::imp::calData, 1.0f);
  std::int32_t samplecur;
  lcr::imp::calData.getSample2(&lcr::imp::calData, false, samplecur);

  auto voltscur = sdadc::toVolts(samplecur);
  return voltscur;
}
void lcr::imp::measure(float & voltage, float & current, float & supply, bool peaking)
{
  // mesaure voltage on capacitor
  auto volt = lcr::imp::measureVolts();
  // mesaure voltage on shunt
  auto cur = lcr::imp::measureCur();

  // mesaure supply for reference
  bool suc = false;
  supply = lcr::volts::measureSupply(suc);


  volt = volt * lcr::imp::calData.rgainVolt - lcr::imp::calData.offVolt;
  cur  = cur * lcr::imp::calData.rgainCur   - lcr::imp::calData.offCur;

  static float voltsamples[SLIDING_MAX_SAMPLES] = { 0.0f }, cursamples[SLIDING_MAX_SAMPLES] = { 0.0f }, voltmax = 0.0f, curmax = 0.0f;
  static std::uint8_t nums = 0, idx = 0;
  
  if (peaking)
  {
    bool reeval1 = false, reeval2 = false;
    if (nums < SLIDING_MAX_SAMPLES)
    {
      ++nums;
      voltmax = std::fmax(voltmax, volt);
      curmax = std::fmax(curmax, cur);
    }
    else
    {
      if (volt >= voltmax)
      {
        voltmax = volt;
      }
      else if (voltsamples[idx] == voltmax)
      {
        // re-evaluate maximum
        reeval1 = true;
      }

      if (cur >= curmax)
      {
        curmax = cur;
      }
      else if (cursamples[idx] == curmax)
      {
        reeval2 = true;
      }
    }
    voltsamples[idx] = volt;
    cursamples[idx] = cur;
    idx = (idx + 1) % SLIDING_MAX_SAMPLES;

    if (reeval1)
    {
      voltmax = voltsamples[0];
      for (std::uint8_t i = 1; i < SLIDING_MAX_SAMPLES; ++i)
      {
        voltmax = std::fmax(voltmax, voltsamples[i]);
      }
    }
    if (reeval2)
    {
      curmax = cursamples[0];
      for (std::uint8_t i = 1; i < SLIDING_MAX_SAMPLES; ++i)
      {
        curmax = std::fmax(curmax, cursamples[i]);
      }
    }
    
    // apply moving maximum over the last 21 samples
    volt = voltmax;
    cur  = curmax;
  }

  voltage = volt;
  current = cur;

  /*SERIAL.print("LCR vc: ");
  SERIAL.print(voltscap, 4);
  SERIAL.print("; vi: ");
  SERIAL.print(voltscur, 4);
  SERIAL.print("; supply: ");
  SERIAL.print(supply, 3);
  SERIAL.println();*/
}
static void s_calcMeas_impl(float volts, float cur, float supply, bool isAlt2, float & resistance, float & reactance)
{
  const auto U = LCR_AMPLITUDE(supply);

  bool setRes = true;

  const auto U2 = U * U, volts2 = volts * volts, cur2 = cur * cur;

  float alpha = (!volts) ? 1.0f : (U2 + volts2 - cur2) / (2.0f * U * volts);
  alpha = (fabs(alpha) > 1.0f) ? 0.0f : acosf(alpha);

  float beta = (!cur) ? 1.0f : (U2 + cur2 - volts2) / (2.0f * U * cur);
  beta = (fabs(beta) > 1.0f) ? 0.0f : acosf(beta);

  const auto r = lcr::imp::calData.getShuntRes();


  // impedants r'ga paralleelselt
  float Zrpar_r, Zrpar_i;
  
  // parasiitsuste faktor
  float kpar_r = 0.0f, kpar_i = 0.0f;

  Zrpar_r = 1 / r;
  Zrpar_i = 0.0f;
  switch (lcr::imp::calData.selRange)
  {
  case 0:
  case 2:
    {
      float tr, ti;
      algo::cdiv(1.0f, 0.0f, lcr::imp::calData.shuntRes[1], -lcr::imp::calData.zc1, tr, ti);
      Zrpar_r += tr;
      Zrpar_i += ti;

      algo::cdiv(1.0f, 0.0f, lcr::imp::calData.shuntRes[1], -lcr::imp::calData.zcpar, tr, ti);
      kpar_r += tr;
      kpar_i += ti;
    }
    break;
  }
  switch (lcr::imp::calData.selRange)
  {
  case 0:
  case 1:
    {
      float tempr, tempi;
      algo::cdiv(1.0f, 0.0f, lcr::imp::calData.shuntRes[2], -lcr::imp::calData.zc1, tempr, tempi);
      Zrpar_r += tempr;
      Zrpar_i += tempi;

      algo::cdiv(1.0f, 0.0f, lcr::imp::calData.shuntRes[2], -lcr::imp::calData.zcpar, tempr, tempi);
      kpar_r += tempr;
      kpar_i += tempi;
    }
    break;
  }
  algo::cdiv(1.0f, 0.0f, Zrpar_r, Zrpar_i, Zrpar_r, Zrpar_i);

  algo::cdiv(1.0f, 0.0f, kpar_r, kpar_i, kpar_r, kpar_i);
  kpar_r += r;
  algo::cdiv(r, 0.0f, kpar_r, kpar_i, kpar_r, kpar_i);

  // Thevenini väljundpinge
  float thev_r, thev_i;
  algo::cmul((lcr::imp::calData.parFactor - 1.0f) * U, 0.0f, kpar_r, kpar_i, thev_r, thev_i);
  thev_r += U;

  const auto thev2 = thev_r * thev_r + thev_i * thev_i;

  if ((thev2 * (0.95f * 0.95f)) < volts2)
  {
    resistance = nanf("");
    setRes = false;
  }

  // pinge r peal tühises olekus
  float Ur0_r, Ur0_i;
  Ur0_r = U - thev_r;
  Ur0_i = -thev_i;

  float Uz_r, Uz_i, Ur_r, Ur_i;
  Uz_r = volts * cosf(alpha);
  Ur_r = cur   * cosf(beta);
  if (volts2 < (Ur0_r * Ur0_r + Ur0_i * Ur0_i))
  {
    Uz_i = -volts * sinf(alpha);
    Ur_i =  cur   * sinf(beta);
  }
  else
  {
    Uz_i =  volts * sinf(alpha);
    Ur_i = -cur   * sinf(beta);
  }

  // vool läbi r
  float Ir_r, Ir_i;
  algo::cdiv(Ur_r, Ur_i, r, 0.0f, Ir_r, Ir_i);

  // koguvool generaatori väljundis
  float Ik_r, Ik_i;
  algo::cdiv(Ur_r, Ur_i, Zrpar_r, Zrpar_i, Ik_r, Ik_i);

  // vool läbi ülemiste mahtuvuste
  float Iup_r = Ik_r - Ir_r, Iup_i = Ik_i - Ir_i;

  // pinge alumisel mahtuvusel
  float Udn_r, Udn_i;
  algo::cmul(0.5f * Iup_r, 0.5f * Iup_i, 0.0f, -lcr::imp::calData.zc1, Udn_r, Udn_i);
  Udn_r = U - Udn_r;
  Udn_i = -Udn_i;

  // vool läbi alumiste mahtuvuste
  float Idn_r, Idn_i;
  algo::cdiv(Udn_r, Udn_i, 0.0f, -lcr::imp::calData.zc2, Idn_r, Idn_i);
  Idn_r *= 2.0f;
  Idn_i *= 2.0f;

  // vool läbi teiste mõõtepiirkondade takistite
  float Imeas_r, Imeas_i;
  Imeas_r = Iup_r - Idn_r;
  Imeas_i = Iup_i - Idn_i;

  float Iz_r, Iz_i;

  if (isAlt2)
  {
    // induktiivsuse arvutamisel vool läbi koormuse
    Iz_r = Ir_r + Imeas_r;
    Iz_i = Ir_i + Imeas_i;
  }
  else
  {
    // mahtuvuse arvutamisel vool läbi koormuse
    Iz_r = Ir_r - Imeas_r;
    Iz_i = Ir_i - Imeas_i;
  }

  // impedants
  float tempr;
  algo::cdiv(Uz_r, Uz_i, Iz_r, Iz_i, tempr, reactance);
  reactance = fabs(reactance);
  if (setRes && (reactance > r))
  {
    tempr = nanf("");
  }
  
  if (setRes)
  {
    resistance = tempr;
  }
}
void lcr::imp::calcMeas1(float volts, float cur, float supply, float & resistance, float & reactance)
{
  // calculate normalized capacitor voltage


  // calculate shunt current
  //auto shuntcurrent = voltscur / lcr::imp::calData.shuntRes[lcr::imp::calData.selRange];

  // esr and reactance in series -> Z = sqrt(Zr^2 + Zi^2) -> Z^2 = Zr^2 + Zi^2
  // R (shunt) in series with that -> forms voltage divider
  // whole impedance real : R + r
  // whole impedance imaginary : sqrt(Z^2 - r^2)
  // voltage on capacitor squared -> V_DAC^2 * Z^2 / ((r+R)^2 + (Z^2-r^2))

  // real resistance

  /*const auto vc2 = volts * volts;
  const auto ric2 = 1.0f / (cur * cur);
  const auto R = lcr::imp::calData.shuntRes[lcr::imp::calData.selRange];
  auto amp2 = LCR_AMPLITUDE(supply);
  amp2 *= amp2;

  resistance = 0.5f * R * ((amp2 - vc2) * ric2 - 1.0f);
  // reactance
  auto reac2 = R * R * vc2 * ric2 - resistance * resistance;
  if (reac2 < 0.0f)
  {
    reactance = -sqrtf(-reac2);
  }
  else
  {
    reactance = sqrt(reac2);
  }*/

  s_calcMeas_impl(volts, cur, supply, false, resistance, reactance);

  resistance -= lcr::imp::calData.offRes;
}
void lcr::imp::calcMeas2(float volts, float cur, float supply, float & resistance, float & reactance)
{
  s_calcMeas_impl(volts, cur, supply, true, resistance, reactance);

  resistance -= lcr::imp::calData.offRes;
}
void lcr::imp::setrange(std::uint8_t rangeidx)
{
  if (lcr::imp::calData.selRange == rangeidx)
  {
    return;
  }
  digitalWrite(LCR_RANGE3, 1);
  digitalWrite(LCR_RANGE2, 1);
  digitalWrite(LCR_RANGE1, !(rangeidx == 0));
  digitalWrite(LCR_RANGE2, !(rangeidx == 1));
  digitalWrite(LCR_RANGE3, !(rangeidx == 2));

  lcr::imp::calData.selRange = rangeidx;
}

float lcr::imp::calcCap(float reactance, float frequency)
{
  if (!reactance)
  {
    return nan("");
  }
  return (1.0f / (reactance * 2.0f * float(M_PI) * frequency)) - lcr::imp::calData.offCap;
}
float lcr::imp::calcInd(float reactance, float frequency)
{
  return (reactance / (2.0f * float(M_PI) * frequency)) - lcr::imp::calData.offInd;
}
float lcr::imp::calcPhase(bool inductor, float resistance, float reactance)
{
  if (inductor)
  {
    return atanf(reactance / resistance);
  }
  else
  {
    return atanf(-reactance / resistance);
  }
}
float lcr::imp::calcDissipation(float resistance, float reactance)
{
  return resistance / reactance;
}
float lcr::imp::calcQ(float resistance, float reactance)
{
  return reactance / resistance;
}



bool lcr::canChangeMode()
{
  digitalWrite(VOLTS_RANGE1, 1);
  // measure voltage on OHMS_ADC to determine, if it's safe
  float volts = lcr::volts::measureFast();

  digitalWrite(VOLTS_RANGE1, 0);

  return (volts < SAFE_RANGE_SWITCH_THRESHOLD);
}
