// got inspiration from https://github.com/nerdyscout/Arduino_MCP3x6x_Library/blob/main/lib/MCP3x6x/MCP3x6x.cpp

#include "mcp3461.hpp"
#include "portAccess.hpp"

constexpr std::uint8_t sdadc::Configs::c_chids[MCP3x6x_SCAN_NUM_IDS];

sdadc::AdcCalData sdadc::calData;
sdadc::Configs sdadc::configs = {
  { MCP3461_DEFAULT_CONFIG0 },
  { MCP3461_DEFAULT_CONFIG1 },
  { MCP3461_DEFAULT_CONFIG2 },
  { MCP3461_DEFAULT_CONFIG3 },
  MCP3461_DEFAULT_MUX
};

static SPIClassSAMD * s_padcSPI = nullptr;
static std::int8_t s_adcCS = -1;
static std::uint32_t s_spiFreq = 1000000U;

static float s_gainref = 0.0f;

//#define SEL_CS()   do { if (s_adcCS != -1) { OutClr(s_adcCS); } } while (0)
//#define STOP_CS()  do { if (s_adcCS != -1) { OutSet(s_adcCS); } } while (0)
#define SEL_CS()   do { digitalWrite(s_adcCS, 0); } while (0)
#define STOP_CS()  do { digitalWrite(s_adcCS, 1); } while (0)


void sdadc::AdcCalData::loadCal(const algo::array<float, MCP3461_CAL_SIZE> & data)
{
  this->gain[std::uint8_t(sdadc::Gain::g0x33)] = data.at(CAL_ADC_GAIN_0_33X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g1x)]   = data.at(CAL_ADC_GAIN_1X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g2x)]   = data.at(CAL_ADC_GAIN_2X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g4x)]   = data.at(CAL_ADC_GAIN_4X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g8x)]   = data.at(CAL_ADC_GAIN_8X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g16x)]  = data.at(CAL_ADC_GAIN_16X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g32x)]  = data.at(CAL_ADC_GAIN_32X_ID);
  this->gain[std::uint8_t(sdadc::Gain::g64x)]  = data.at(CAL_ADC_GAIN_64X_ID);

  this->vref = data.at(CAL_ADC_VREF_ID);
}
void sdadc::AdcCalData::saveCal(algo::array<float, MCP3461_CAL_SIZE> & data) const
{
  data[CAL_ADC_GAIN_0_33X_ID] = this->gain[std::uint8_t(sdadc::Gain::g0x33)];
  data[CAL_ADC_GAIN_1X_ID]    = this->gain[std::uint8_t(sdadc::Gain::g1x)];
  data[CAL_ADC_GAIN_2X_ID]    = this->gain[std::uint8_t(sdadc::Gain::g2x)];
  data[CAL_ADC_GAIN_4X_ID]    = this->gain[std::uint8_t(sdadc::Gain::g4x)];
  data[CAL_ADC_GAIN_8X_ID]    = this->gain[std::uint8_t(sdadc::Gain::g8x)];
  data[CAL_ADC_GAIN_16X_ID]   = this->gain[std::uint8_t(sdadc::Gain::g16x)];
  data[CAL_ADC_GAIN_32X_ID]   = this->gain[std::uint8_t(sdadc::Gain::g32x)];
  data[CAL_ADC_GAIN_64X_ID]   = this->gain[std::uint8_t(sdadc::Gain::g64x)];

  data[CAL_ADC_VREF_ID] = this->vref;
}

void sdadc::AdcCalData::selfCal(sdadc::Channel mpos, sdadc::Channel mneg)
{
  const auto prevosr = this->osrRatio;
  sdadc::config(sdadc::OSR::osr8192, this->format);

  std::int32_t sample033, sample1, sample2, sample4, sample8, sample16;

  const auto maxCounts = ((this->format == sdadc::Dataformat::f32bit_sgn) || (this->format == sdadc::Dataformat::f32bit_chid_sgn)) ? MCP3461_MAX_COUNT_17BIT : MCP3461_MAX_COUNT_16BIT;

  auto measall = [&]() -> void
  {
    sample033 = maxCounts;
    sample2   = maxCounts;
    sample4   = maxCounts;
    sample8   = maxCounts;
    sample16  = maxCounts;
  
    sdadc::setGain(sdadc::Gain::g1x);
    bool suc = false;
    sample1 = sdadc::getSample(mpos, sdadc::Channel::agnd, suc);
    
    if (sample1 > (maxCounts - 1))
    {
      return;
    }
    sdadc::setGain(sdadc::Gain::g0x33);
    suc = false;
    sample033 = sdadc::getSample(mpos, sdadc::Channel::agnd, suc);

    if (sample1 > std::uint16_t((float)maxCounts / (float)1.87f))
    {
      return;
    }
    sdadc::setGain(sdadc::Gain::g2x);
    suc = false;
    sample2 = sdadc::getSample(mpos, sdadc::Channel::agnd, suc);

    if (sample2 > std::uint16_t((float)maxCounts / (float)1.87f))
    {
      return;
    }
    sdadc::setGain(sdadc::Gain::g4x);
    suc = false;
    sample4 = sdadc::getSample(mpos, sdadc::Channel::agnd, suc);

    if (sample4 > std::uint16_t((float)maxCounts / (float)1.87f))
    {
      return;
    }
    sdadc::setGain(sdadc::Gain::g8x);
    suc = false;
    sample8 = sdadc::getSample(mpos, sdadc::Channel::agnd, suc);

    if (sample8 > std::uint16_t((float)maxCounts / (float)1.87f))
    {
      return;
    }
    sdadc::setGain(sdadc::Gain::g16x);
    suc = false;
    sample16 = sdadc::getSample(mpos, sdadc::Channel::agnd, suc);

    // not worth measuring gains 32x and 64x, since they are digital gains of 16x gain
  };

  // gain calibration procedure

  measall();
  // can't calibrate anything if the adc is already saturated, select internal vcm, so at least some ranges can be calibrated
  if (sample1 > (maxCounts - 1))
  {
    mpos = sdadc::Channel::vcm_int;
    mneg = sdadc::Channel::agnd;
    measall();
  }
  assert(sample1 < maxCounts);

  // set gain back to 1x to not saturate the input amplifier
  sdadc::setGain(sdadc::Gain::g1x);

  // restore previous OSR setting
  sdadc::config(prevosr, this->format);


  // calculate gains from measured samples
  this->gain[std::uint8_t(sdadc::Gain::g0x33)] = float(sample033) / float(sample1);
  this->gain[std::uint8_t(sdadc::Gain::g1x)]   = 1.0f; // Gain 1x is always 1, vref is adjusted to account for that error
  if (sample2 > (maxCounts - 1))
  {
    return;
  }
  this->gain[std::uint8_t(sdadc::Gain::g2x)]   = float(sample2) / float(sample1);
  if (sample4 > (maxCounts - 1))
  {
    return;
  }
  this->gain[std::uint8_t(sdadc::Gain::g4x)]   = float(sample4) / float(sample1);
  if (sample8 > (maxCounts - 1))
  {
    return;
  }
  this->gain[std::uint8_t(sdadc::Gain::g8x)]   = float(sample8) / float(sample1);
  if (sample16 > (maxCounts - 1))
  {
    return;
  }
  this->gain[std::uint8_t(sdadc::Gain::g16x)]  = float(sample16) / float(sample1);
  // gains 32x & 64x are just digital
  this->gain[std::uint8_t(sdadc::Gain::g32x)]  = this->gain[std::uint8_t(sdadc::Gain::g16x)] * 2.0f;
  this->gain[std::uint8_t(sdadc::Gain::g64x)]  = this->gain[std::uint8_t(sdadc::Gain::g16x)] * 4.0f;
}

void sdadc::printRegisters(int (*pfunc)(const char * fmt, ...))
{
  std::uint8_t cmd[5] = { 0 }, val[5] = { 0 };

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_CONFIG0_ADDR);
  val[0] = sdadc::spiRead(cmd[0]);
  pfunc("CONF0: %02x\n", val[0]);

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_CONFIG1_ADDR);
  val[0] = sdadc::spiRead(cmd[0]);
  pfunc("CONF1: %02x\n", val[0]);

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_CONFIG2_ADDR);
  val[0] = sdadc::spiRead(cmd[0]);
  pfunc("CONF2: %02x\n", val[0]);

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_CONFIG3_ADDR);
  val[0] = sdadc::spiRead(cmd[0]);
  pfunc("CONF3: %02x\n", val[0]);

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_IRQ_ADDR);
  val[0] = sdadc::spiRead(cmd[0]);
  pfunc("IRQ  : %02x\n", val[0]);

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_MUX_ADDR);
  val[0] = sdadc::spiRead(cmd[0]);
  pfunc("MUX  : %02x\n", val[0]);


  cmd[0] = MCP3x6x_CMD_READ_INC(MCP3x6x_RESERVED1_ADDR);
  sdadc::spiTransfer(cmd, val, 4);
  pfunc("0x%x: %02x%02x%02x\n", MCP3x6x_RESERVED1_ADDR, val[1], val[2], val[3]);

  cmd[0] = MCP3x6x_CMD_READ(MCP3x6x_RESERVED2_ADDR);
  val[1] = sdadc::spiRead(cmd[0]);
  pfunc("0x%x: %02x\n", MCP3x6x_RESERVED2_ADDR, val[1]);

  cmd[0] = MCP3x6x_CMD_READ_INC(MCP3x6x_RESERVED3_ADDR);
  sdadc::spiTransfer(cmd, val, 3);
  pfunc("0x%x: %02x%02x\n", MCP3x6x_RESERVED3_ADDR, val[1], val[2]);
}

void sdadc::init(SERCOM * module, std::uint32_t frequency, std::int8_t mosi, std::int8_t miso, std::int8_t sck, std::int8_t cs)
{
  if (s_padcSPI != nullptr)
  {
    return;
  }

  s_padcSPI = new SPIClassSAMD(
    module,
    std::uint8_t(miso), std::uint8_t(sck), std::uint8_t(mosi),
    SDADC_TX_PAD, SDADC_RX_PAD
  );
  assert(s_padcSPI != nullptr);

  if (cs != -1)
  {
    //s_adcCS = ARDUINO_PIN_TO_PORT_PIN(cs);
    s_adcCS = cs;
    //OutMode(s_adcCS);
    pinMode(s_adcCS, OUTPUT);
    STOP_CS();
  }

  s_padcSPI->begin();

  pinPeripheral(miso, SDADC_MISO_SERCOM);
  pinPeripheral(mosi, SDADC_MOSI_SERCOM);
  pinPeripheral(sck,  SDADC_SCK_SERCOM);

  sdadc::setFreq(frequency);

  sdadc::reset();
  delay(10);

  // set config registers
  configs.config0.reg  = MCP3x6x_CONFIG0_CLK_SEL_INT;
  configs.config0.reg |= MCP3x6x_CONFIG0_SHDN_OFF;
  configs.config0.reg |= MCP3x6x_CONFIG0_ADC_MODE_CONV;
  configs.config0.reg |= MCP3x6x_CONFIG0_CS_SEL_NONE;
  configs.config0.reg |= MCP3x6xR_CONFIG0_REF_INT;
  sdadc::writeReg(configs.config0.reg, MCP3x6x_CONFIG0_ADDR);

  configs.config1.reg  = std::uint8_t(calData.osrRatio) << MCP3x6x_CONFIG1_OSR_POS;
  configs.config1.reg |= MCP3x6x_CONFIG1_AMCLK_DIV0;
  sdadc::writeReg(configs.config1.reg, MCP3x6x_CONFIG1_ADDR);

  configs.config2.reg  = MCP3x6x_CONFIG2_BOOST_x1;
  configs.config2.reg |= std::uint8_t(calData.gainSel) << MCP3x6x_CONFIG2_GAIN_POS;
  configs.config2.reg |= MCP3x6x_CONFIG2_AZ_MUX_OFF;
  configs.config2.reg |= MCP3x6xR_CONFIG2_AZ_REF_OFF;
  configs.config2.bit.reserved = 1;
  sdadc::writeReg(configs.config2.reg, MCP3x6x_CONFIG2_ADDR);

  configs.config3.reg  = MCP3x6x_CONFIG3_CONV_MODE_CONTINUOUS;
  configs.config3.reg |= std::uint8_t(calData.format) << MCP3x6x_CONFIG3_DATA_FORMAT_POS;
  configs.config3.reg |= MCP3x6x_CONFIG3_CRCCOM_OFF;
  configs.config3.reg |= MCP3x6x_CONFIG3_GAINCAL_OFF;
  configs.config3.reg |= MCP3x6x_CONFIG3_OFFCAL_OFF;
  sdadc::writeReg(configs.config3.reg, MCP3x6x_CONFIG3_ADDR);

  std::uint8_t reg;
  reg  = MCP3x6x_IRQ_MODE_IRQ_HIGHZ;
  reg |= MCP3x6x_IRQ_FASTCMD_ON;
  //reg |= MCP3x6x_IRQ_STP_ON;
  sdadc::writeReg(reg, MCP3x6x_IRQ_ADDR);
}
void sdadc::setFreq(std::uint32_t frequency)
{
  assert(s_padcSPI != nullptr);

  s_spiFreq = frequency;
}
void sdadc::config(sdadc::OSR oversampling, sdadc::Dataformat format)
{
  // send dataformat
  configs.config3.bit.dataFormat = std::uint8_t(format);
  sdadc::writeReg(configs.config3.reg, MCP3x6x_CONFIG3_ADDR);

  // send oversampling ratio
  configs.config1.bit.osr = std::uint8_t(oversampling);
  sdadc::writeReg(configs.config1.reg, MCP3x6x_CONFIG1_ADDR);

  sdadc::calData.osrRatio = oversampling;
  sdadc::calData.format = format;
}
void sdadc::reset()
{
  sdadc::fastcmd(MCP3x6x_CMD_RESET_REG);
}
void sdadc::standby()
{
  sdadc::fastcmd(MCP3x6x_CMD_ADC_STANDBY);
}
void sdadc::setClock()
{
  // set internal clock
  configs.config0.bit.clk = std::uint8_t(sdadc::Clock::internal);
  sdadc::writeReg(configs.config0.reg, MCP3x6x_CONFIG0_ADDR);
}

void sdadc::spiTransfer(std::uint8_t * tx, std::uint8_t * rx, std::uint16_t size)
{
  assert(s_padcSPI != nullptr);

  s_padcSPI->beginTransaction(SPISettings(s_spiFreq, MCP3x6x_SPI_ORDER, MCP3x6x_SPI_MODE));
  SEL_CS();

  for (std::uint16_t i = 0; i < size; ++i)
  {
    rx[i] = s_padcSPI->transfer(tx[i]);
  }

  STOP_CS();
  s_padcSPI->endTransaction();
}
void sdadc::spiWrite(std::uint8_t * data, std::uint16_t sz)
{
  assert(s_padcSPI != nullptr);

  s_padcSPI->beginTransaction(SPISettings(s_spiFreq, MCP3x6x_SPI_ORDER, MCP3x6x_SPI_MODE));
  SEL_CS();

  s_padcSPI->transfer(data, sz);

  STOP_CS();
  s_padcSPI->endTransaction();
}
std::uint8_t sdadc::spiRead(std::uint8_t cmd)
{
  std::uint8_t tx[2] = { cmd, 0 }, rx[2] = { 0 };
  sdadc::spiTransfer(tx, rx, 2);

  return rx[1];
}
std::uint8_t sdadc::status()
{
  std::uint8_t tx = MCP3x6x_CMD_READ_INC(0x00), rx = 0;
  sdadc::spiTransfer(&tx, &rx, 1);

  return rx;
}

void sdadc::fastcmd(std::uint8_t cmd)
{
  sdadc::spiWrite(&cmd, 1);
}
void sdadc::read(std::int32_t * data, std::uint8_t * chid)
{
  std::uint32_t bytes = 4;
#if defined(MCP346x) || !defined(MCP356x)
  if (sdadc::calData.format == sdadc::Dataformat::f16bit)
  {
    bytes = 2;
  }
#else
  if (sdadc::calData.format == sdadc::Dataformat::f24bit)
  {
    bytes = 3;
  }
#endif

  std::uint8_t cmd[5] = { MCP3x6x_CMD_READ(MCP3x6x_ADCDATA_ADDR), 0, 0, 0, 0 }, buf[5] = { 0 };
  sdadc::spiTransfer(cmd, buf, bytes + 1);

  // reverse array
  if (bytes == 2)
  {
    // 0 = 2
    // 1 = 1

    buf[0] = buf[2];
    //buf[1] = buf[1];
    //buf[2] = 0;
    //buf[3] = 0;    
  }
  else
  {
    const auto swap = [](std::uint8_t & byte1, std::uint8_t & byte2) -> void
    {
      std::uint8_t temp = byte1;
      byte1 = byte2;
      byte2 = temp;
    };

    // 0 = 4
    // 1 = 3
    // 2 = 2
    // 3 = 1
    buf[0] = buf[4];
    swap(buf[1], buf[3]);    
    //buf[2] = buf[2];
  }

  std::uint32_t temp = *((std::uint32_t *)buf);
  if (chid != nullptr)
  {
    if (sdadc::calData.format == sdadc::Dataformat::f32bit_chid_sgn)
    {
      *chid = (temp >> 28) & 0x0F;
    }
    else
    {
      for (std::uint8_t i = 0; i < MCP3x6x_SCAN_NUM_IDS; ++i)
      {
        if (sdadc::configs.c_chids[i] == sdadc::configs.mux)
        {
          *chid = i;
          break;
        }
      }
    }
  }

#if defined(MCP346x) || !defined(MCP356x)

  struct
  {
    std::int32_t value : 17;
  } raw;

  switch (sdadc::calData.format)
  {
  case sdadc::Dataformat::f16bit:
    //bitWrite(temp, 31, bitRead(temp, 16));
    //bitClear(temp, 16);
    raw.value = temp;
    bitWrite(raw.value, 16, bitRead(raw.value, 15));
    break;
  case sdadc::Dataformat::f32bit:
    raw.value = temp >> 16;
    bitWrite(raw.value, 16, bitRead(raw.value, 15));
    break;
  case sdadc::Dataformat::f32bit_sgn:
  case sdadc::Dataformat::f32bit_chid_sgn:
    //bitWrite(temp, 31, bitRead(temp, 17));
    //temp &= 0x8000FFFF;
    raw.value = temp & 0x0001FFFF;
    break;
  }

#else

  struct
  {
    std::int32_t value : 25;
  } raw;

  switch (sdadc::calData.format)
  {
  case sdadc::Dataformat::f24bit:
    raw.value = temp;
    bitWrite(raw.value, 24, bitRead(raw.value, 23));
    break;
  case sdadc::Dataformat::f32bit:
    raw.value = temp >> 8;
    bitWrite(raw.value, 24, bitRead(raw.value, 23));
    break;
  case sdadc::Dataformat::f32bit_sgn:
  case sdadc::Dataformat::f32bit_chid_sgn:
    raw.value = temp & 0x01FFFFFF;
    break;
  }

#endif

  *data = raw.value;
}
void sdadc::writeReg(std::uint8_t byte, std::uint8_t addr)
{
  std::uint8_t cmd[2] = { MCP3x6x_CMD_WRITE_INC(addr), byte };
  sdadc::spiWrite(cmd, 2);
}
void sdadc::startConversion()
{
  sdadc::fastcmd(MCP3x6x_CMD_ADC_START);
}
bool sdadc::isContReady()
{
  return (sdadc::status() & (1 << MCP3x6x_STATUS_DR_POS)) == 0;
}

void sdadc::autozero(bool enable)
{
  assert(s_padcSPI != nullptr);

  // send auto-zero enable/disable command
  configs.config2.bit.azMu  = enable;
  configs.config2.bit.azRef = enable;
  sdadc::writeReg(configs.config2.reg, MCP3x6x_CONFIG2_ADDR);
}
void sdadc::setGain(sdadc::Gain gain)
{
  assert(s_padcSPI != nullptr);
  assert(std::uint8_t(gain) < std::uint8_t(sdadc::Gain::size));

  // send ADC set gain command
  if (configs.config2.bit.gain != std::uint8_t(gain))
  {
    configs.config2.bit.gain = std::uint8_t(gain);
    sdadc::writeReg(configs.config2.reg, MCP3x6x_CONFIG2_ADDR);
  }

  sdadc::calData.gainSel = gain;
  s_gainref = sdadc::calData.vref / sdadc::calData.gain[std::uint8_t(sdadc::calData.gainSel)];
}

std::int32_t sdadc::getSample(sdadc::Channel chpos, sdadc::Channel chneg, bool & suc)
{
  assert(s_padcSPI != nullptr);

  static bool isBusy = false;

  if (isBusy)
  {
    suc = false;
    return 0;
  }

  isBusy = true;

  std::uint8_t mux = (std::uint8_t(chpos) << 4) | std::uint8_t(chneg);

  std::int32_t adcdata;
  std::uint8_t ch;

  if (mux != configs.mux)
  {
    configs.mux = mux;
    sdadc::writeReg(configs.mux, MCP3x6x_MUX_ADDR);
    sdadc::read(&adcdata, nullptr);
  }

  if (!suc)
  {
    while (!sdadc::isContReady());
  }
  else
  {
    if (!sdadc::isContReady())
    {
      suc = false;
      isBusy = false;
      return 0;
    }
  }
  sdadc::read(&adcdata, &ch);
  suc = true;
  isBusy = false;

  return adcdata;
}
float sdadc::toVolts(std::int32_t sample)
{
  return float(sample) * s_gainref * (1.0f / 32768.0f);
}
float sdadc::toTemp(std::int32_t sample)
{
  return 0.102646f * float(sample) * s_gainref - 269.13f;
}
float sdadc::toTemp(float volts)
{
  return volts * (32768.0f * 0.102646f) - 269.13f;
}
