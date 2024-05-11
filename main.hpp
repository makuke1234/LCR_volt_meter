#pragma once

// C++ includes
#include <cstdio>
#include <cstdarg>
#include <cstdint>
#include <cstring>
#include <cstdlib>
#include <cerrno>
#include <cmath>
#include <cassert>

#include <type_traits>

// MCU includes
#include <Arduino.h>
#include <variant.h>
#include "ADC.hpp"
#include "mcp3461.hpp"

// Constants

// Info

/*
 *  clock & timer configuration:
 *
 *  GCLK0 @48 MHz PLL from 32.768 kHz crystal
 *  GCLK1 @32.768 kHz quartz crystal
 *  GCLK3 @8 MHz divided from 48 MHz PLL
 *  GCLK4 @96 MHz PLL from 32.768 kHz crystal
 *
 *
 *  TCC0 @96 MHz -> RGB LED + ohms PWM
 *  TCC1 @96 Mhz -> DAC timing
 *  TC3 @96 MHz -> LCD PWM + 1 extra unused PWM
 *  TC4 @96 Mhz -> USB heartbeat
 *  TC5 @8 Mhz -> interrupts @150 kHz; periodic data acquisition, display updates 
 *
 */

// Pin configuration

#define DAC_OUT       A0  /* PA02 */

#define BTN_ADC       A4  /* PA05 */
#define OUT_MEAS_ADC  A6  /* PA06 */

#define VOLTS_RANGE1     9    /* PA07 */
#define VOLTS_RANGE2     4    /* PA08 */
#define VOLTS_MODE_IN   27  /* PA28 */

// TCC0 24-bit
#define LED_R_PWM   3  /* PA09 */
#define LED_G_PWM   1  /* PA10 */
#define LED_B_PWM   0  /* PA11 */
#define OHMS_PWM    2  /* PA14 */
// TC3 16-bit
#define LCD_PWM     5  /* PA15 */

#define OHMS_22X      19  /* PB02 */
#define OHMS_202X     25  /* PB03 */
#define OHMS_INVERT   15  /* PB08 */

#define OHMS_CUR_RANGE  6  /* PA20 */

#define LCR_RANGE1  26  /* PA27 */
#define LCR_RANGE2  31  /* PB23 */
#define LCR_RANGE3  30  /* PB22 */
#define LCR_ENABLE  42  /* PA03 */

#define SDADC_MOSI         20  /* PA22 */
#define SDADC_MISO         12  /* PA19 */
#define SDADC_SCK          13  /* PA17 */
#define SDADC_CS           10  /* PA18 */
#define SDADC_TX_PAD       SercomSpiTXPad::SPI_PAD_0_SCK_1
#define SDADC_RX_PAD       SercomRXPad::SERCOM_RX_PAD_3
#define SDADC_MOSI_SERCOM  PIO_SERCOM
#define SDADC_MISO_SERCOM  PIO_SERCOM_ALT
#define SDADC_SCK_SERCOM   PIO_SERCOM_ALT

// external ADC channels
#define ADC_VB0V3_CH    sdadc::Channel::ch0
#define ADC_CURRENT_CH  sdadc::Channel::ch1
#define ADC_CAP_CH      sdadc::Channel::ch2
#define ADC_OHMS_CH     sdadc::Channel::ch3

#define LCD_RS  11  /* PA16 */
#define LCD_RW  38  /* PA13 */
#define LCD_EN  22  /* PA12 */
#define LCD_D4  24  /* PB11 */
#define LCD_D5  23  /* PB10 */
#define LCD_D6  21  /* PA23 */
#define LCD_D7  16  /* PB09 */

// Software defitions for hardware modules

//#define PA_OUTPUT_READ

#define LED_R_CC  1
#define LED_G_CC  2
#define LED_B_CC  3
#define OHMS_CC   0

#define LCD_CC    1

#define LED_PWM_FREQUENCY        10000    /* PWM frequency for LED & current set, 10 kHz, 96 Mhz clock -> 9600 steps, 13.2 bits */
#define LCD_PWM_FREQUENCY        10000    /* LCD PWM frequency, 10 kHz, 96 Mhz clock -> 256 steps, 8 bits */
#define LCR_UPDATE_FREQUENCY    150000    /* firmware internal "game loop" update frequency */
#define SDADC_SPI_FREQUENCY     12000000  /* 12 MHz, th. max. 300 ksps with 32 bit samples & 8 bit commands, set this to 8 MHz on SAMD20 */
#define SDADC_OSR_RATIO         sdadc::OSR::osr32

#define LCR_DEF_UPDATE_LOGIC_MS    1U
#define LCR_UPDATE_BTN_MS         10U
#define LCR_UPDATE_DISPLAY_MS    200U
#define LCR_DEF_IIR_ALPHA        (1.0 / 100.0)
#define LCR_DEF_TEMP_IIR_ALPHA   (1.0 / 1000.0)
#define LCR_UPDATE_MS_TICKS      (LCR_UPDATE_FREQUENCY / 1000U) /* Number of ticks to reach 1 millisecond */

#define BTN_DEBOUNCE_THRESHOLD_MS   30U

#define LCD_WIDTH   20
#define LCD_HEIGHT   4

#define LCD_BOOTLOGO_TIMEOUT_MS   1000

// Default LCD backlight brightness duty cycle, range 0 - 255
#define DEF_LCD_BRIGHT_ENABLED    1
#define DEF_LCD_BRIGHT          127
#define DEF_LED_BRIGHT           20

// Serial baudrate
#define DEFAULT_BAUDRATE  115200

// USE SerialUSB as default serial peripheral
#define SERIAL                  SerialUSB
// USB serial printf binding buffer length
#define SERIAL_PRINTF_BUFSIZE   128
// Serial data receive buffer length
#define SERIAL_READ_BUFSIZE      64

// ADC config, ADC uses internal 8MHz OSC as clock source
#define ADC_RESOLUTION_BITS         16
#define ADC_FREERUN               true
#define ADC_OVERSAMPLING_SAMPLES   512
#define ADC_SAMPLETIME               3
#define ADC_SLOW_CLK_DIV          ADC_CTRLB_PRESCALER_DIV64  /* 125 kHz slow conversion ADC clock */
#define ADC_CLK_DIV               ADC_CTRLB_PRESCALER_DIV8   /* 1 Mhz normal ADC clock */

// default resistor sizes in ohms and voltages, the ones with DEF_ in front of them
// have to be calibrated

#define ADC_BTN_PU_R    10000.0
#define ADC_BTN_PD_R     4700.0
#define ADC_BTN_IMP_R   (1.0 / (1.0 / ADC_BTN_PU_R + 1.0 / ADC_BTN_PD_R))
#define ADC_BTN_VREST   ((ADC_BTN_PD_R / (ADC_BTN_PD_R + ADC_BTN_PU_R)) * DEF_LCR_SUPPLY)
#define ADC_BTN_VREST_F(supply)  ((ADC_BTN_PD_R / (ADC_BTN_PD_R + ADC_BTN_PU_R)) * supply)
#define ADC_BTN1_R        470.0
#define ADC_BTN2_R       1000.0
#define ADC_BTN3_R       2200.0
#define ADC_BTN4_R       4700.0
#define ADC_BTN5_R      10000.0
#define ADC_BTN_TOL         0.15


#define DEF_LCR_SUPPLY                     3.3
#define DAC_FACTOR                         0.9  /* max sample 972 with 10 bits, min sample 51 -> 0.165 V to 3.135 V */
#define DAC_DIVIDER_HIGH_R             10000.0
#define DAC_DIVIDER_LOW_R               1200.0
#define VB0V3_HIGH_R                  100000.0
#define VB0V3_LOW_R                    10000.0
#define DEF_LCR_GAIN                  (VB0V3_LOW_R / (VB0V3_LOW_R + VB0V3_HIGH_R))
#define LCR_AMPLITUDE(supply)         ((supply * DAC_FACTOR * DAC_DIVIDER_LOW_R) / (2.0 * (DAC_DIVIDER_LOW_R + DAC_DIVIDER_HIGH_R)))
#define DEF_LCR_AMPLITUDE             LCR_AMPLITUDE(DEF_LCR_SUPPLY)
#define SAFE_RANGE_SWITCH_THRESHOLD   (DEF_LCR_SUPPLY * 0.9)
// This is a stock value that has to be measured with the external ADC
#define DEF_LCR_OFFSET                (DEF_LCR_SUPPLY * DEF_LCR_GAIN)

#define ADC_CURRENT_HIGH_R  7500.0
#define ADC_CURRENT_LOW_R   1000.0
#define ADC_VOLTAGE_HIGH_R  7500.0
#define ADC_VOLTAGE_LOW_R   1000.0
#define DEF_CURRENT_GAIN    (1.0 + ADC_CURRENT_HIGH_R / ADC_CURRENT_LOW_R)
#define DEF_VOLTAGE_GAIN    (1.0 + ADC_VOLTAGE_HIGH_R / ADC_VOLTAGE_LOW_R)

#define VOLTAGE_HIGH_R            1000000.0
#define VOLTAGE_LOW_R               47000.0
#define DEF_VOLTAGE_ATTEN         (VOLTAGE_LOW_R / (VOLTAGE_LOW_R + VOLTAGE_HIGH_R))
#define DEF_VOLTAGE_ATTEN_OFFSET        0.005
#define NEGATIVE_SWITCH_THRES         300 /* lead swap threshold in samples */

#define OHMS_NORMAL_R            10000.0
#define OHMS_RG1_R                1000.0
#define OHMS_RG2_R                 100.0
#define DEF_OHMS_GAIN_NORM           2.0
#define DEF_OHMS_GAIN_RG1        (2.0 + 2.0 * OHMS_NORMAL_R / OHMS_RG1_R)
#define DEF_OHMS_GAIN_RG2        (2.0 + 2.0 * OHMS_NORMAL_R / OHMS_RG2_R)
#define DEF_OHMS_GAIN_RG1_2      (2.0 + 2.0 * OHMS_NORMAL_R / (1.0 / (1.0 / OHMS_RG1_R + 1.0 / OHMS_RG2_R)))
#define DEF_OHMS_ADC_MAX_NORMAL      3.2
#define DEF_OHMS_ADC_MAX_RG1         3.2
#define DEF_OHMS_ADC_MAX_RG2         3.2

#define DEF_OHMS_CUR_SHUNT1_R   (1.0 / (1.0 / 47.0 + 1.0 / 3900.0))
#define DEF_OHMS_CUR_SHUNT2_R   3900.0
#define DEF_OHMS_CUR_IN_DIV        0.5
#define DEF_OHMS_DUTY_PER_MA1   (DEF_OHMS_CUR_SHUNT1_R / (1000.0 * DEF_LCR_SUPPLY * DEF_OHMS_CUR_IN_DIV))
#define DEF_OHMS_DUTY_PER_MA2   (DEF_OHMS_CUR_SHUNT2_R / (1000.0 * DEF_LCR_SUPPLY * DEF_OHMS_CUR_IN_DIV))
#define DEF_OHMS_MAX_V             1.0
#define DEF_OHMS_DUTY_MAX       (DEF_OHMS_MAX_V / (DEF_LCR_SUPPLY * DEF_OHMS_CUR_IN_DIV))
#define DEF_OHMS_DUTY_MIN          1.0 / 9600.0
#define OHMS_MAX_R_VOLTS           2.7
#define DEF_OHMS_OPAMP_OFFSET      0.0045
// Current mA per duty -> 1 / DEF_OHMS_DUTY_PER_MA1
// A per duty -> 1 / (1000.0f * DEF_OHMS_DUTY_PER_MA1)
//#define OHMS_SHUNT1_MAX_R       OHMS_MAX_R_VOLTS / (1 / (1000.0 * DEF_OHMS_DUTY_PER_MA1))
// minimum volts over lower shunt
#define OHMS_MINVOLTS_OVER_SHUNT(minduty, ohms)      (minduty * DEF_LCR_SUPPLY * DEF_OHMS_CUR_IN_DIV + DEF_OHMS_OPAMP_OFFSET)
#define OHMS_MINVOLTS_OVER_HIGHSHUNT(volts, ohms)    (volts * ohms / 1000.0 + DEF_OHMS_OPAMP_OFFSET)
#define OHMS_MINCURRENT_OVER_HIGHSHUNT(volts, ohms)  (OHMS_MINVOLTS_OVER_HIGHSHUNT(volts, ohms) / ohms)
#define OHMS_MAX_RESISTANCE(ohms)                    (OHMS_MAX_R_VOLTS / OHMS_MINCURRENT_OVER_HIGHSHUNT(OHMS_MINVOLTS_OVER_SHUNT(DEF_OHMS_DUTY_MIN, ohms), ohms))
#define OHMS_MAX_RESISTANCE_SHUNT1                   OHMS_MAX_RESISTANCE(DEF_OHMS_CUR_SHUNT1_R)
#define OHMS_MAX_RESISTANCE_SHUNT2                   OHMS_MAX_RESISTANCE(DEF_OHMS_CUR_SHUNT2_R)
// lower = (DEF_OHMS_DUTY_MIN * DEF_LCR_SUPPLY * DEF_OHMS_CUR_IN_DIV + DEF_OHMS_OPAMP_OFFSET)
// minimum volts over higher shunt
// higher = (lower * DEF_OHMS_CUR_SHUNT1_R / 1000.0 + DEF_OHMS_OPAMP_OFFSET)
// current = higher / DEF_OHMS_CUR_SHUNT1_R
// max resistance = OHMS_MAX_R_VOLTS / current

#define OHMS_SHUNT1_MAX_R       (OHMS_MAX_R_VOLTS * 1000.0 * DEF_OHMS_DUTY_PER_MA1 * DEF_OHMS_DUTY_MIN)

#define LCR_DEF_SHUNT1_R  100000.0
#define LCR_DEF_SHUNT2_R    1000.0
#define LCR_DEF_SHUNT3_R      10.0
#define LCR_DEF_RANGE1_R  LCR_DEF_SHUNT1_R
#define LCR_DEF_RANGE2_R  LCR_DEF_SHUNT2_R
#define LCR_DEF_RANGE3_R  LCR_DEF_SHUNT3_R

#define LCR_DEFAULT_FREQUENCY      10000  /* 10 kHz */
#define LCR_DEFAULT_FREQUENCY_PRE  LCRState::DacFreqPre::f10k
#define LCR_MAX_INDUCTANCE         10000.0
#define LCR_MAX_CAPACITANCE            0.01

#define ADC_MEASOUT_MAX_AVG_SAMPLES  256  /* Max number of ADC samples to take when averaging internal ADC reading */

#define DAC_MAX_RAM_SAMPLES         205
#define DAC_MAX_FLASH_SAMPLES      1447  /* smallest buffer size to guarantee no missing codes, roughly (max sample - min sample) / (1 - 1/e) */
#define DAC_MAX_PRESETS               7
#define DAC_MAX_SAMPLERATE      2048000UL

// All settings indexes located in settings.hpp

enum class Mode : std::uint8_t
{
  LCR,      // measure inductance, capacitance & resistance with AC
  Shunt,    // measure shunts with DC
  Volts,    // measure DC voltage
  Thermo,   // measure thermocouples
  Failure,  // Failure mode

  size
};

// Printing function enum
enum class Info : std::uint8_t
{
  Temp,
  TempAcc,
  Ref,
  Supply,
  Gain_0x5,
  Gain_2x,

  size
};

enum class CalCmd : std::uint8_t
{
  VoltsGain2,
  VoltsGain22,
  VoltsGain202,
  VoltsGain222,
  VoltsOff2,
  VoltsOff22,
  VoltsOff202,
  VoltsOff222,
  VoltsAttenGain,
  VoltsAttenOff,
  VoltsSupplyGain,
  VoltsSupplyOff,

  OhmsGain3900,
  OhmsGain47,
  OhmsOff3900,
  OhmsOff47,

  ImpGainCurrent,
  ImpGainVolt,
  ImpOffCurrent,
  ImpOffVolt,
  ImpShunt1,
  ImpShunt2,
  ImpShunt3,
  ImpParC1,
  ImpParC2,
  ImpOffResistance,
  ImpOffCapacitance,
  ImpOffInductance,

  AdcGain0x33,
  AdcGain1,
  AdcGain2,
  AdcGain4,
  AdcGain8,
  AdcGain16,
  AdcGain32,
  AdcGain64,
  AdcVref,

  size
};

constexpr const char * calCommands[] = {
  "vgain2",
  "vgain22",
  "vgain202",
  "vgain222",

  "voff2",
  "voff22",
  "voff202",
  "voff222",

  "hvattengain",
  "hvattenoff",

  "vddgain",
  "vddoff",

  "ohmsgain3900",
  "ohmsgain47",
  "ohmsoff3900",
  "ohmsoff47",

  "impgaincur",
  "impgainvolt",
  "impoffcur",
  "impoffvolt",
  "impshunt1",
  "impshunt2",
  "impshunt3",
  "impc1",
  "impc2",
  "impoffres",
  "impoffcap",
  "impoffind",

  "adcgain0x33",
  "adcgain1x",
  "adcgain2x",
  "adcgain4x",
  "adcgain8x",
  "adcgain16x",
  "adcgain32x",
  "adcgain64x",
  "adcvref",
};

struct packet
{
  float res, gain;
  constexpr packet(float volts)
    : res(volts), gain(1.0f)
  {}
};

// Data structures
struct LCRState
{
  volatile bool debug : 1;
  bool btn1State    : 1;
  bool btn2State    : 1;
  bool btn3State    : 1;
  bool btn4State    : 1;
  bool btn5State    : 1;
  float btnOhms;
  // true if shunt measurement mode button is engaged
  bool btnModeState : 1, btnModeIntOccur : 1;
  bool isLcdBright  : 1;
  std::uint8_t lcdBright, ledBright;
  Mode mode;

  std::uint32_t updateLogicMs = LCR_DEF_UPDATE_LOGIC_MS, updateLogicTicks = LCR_UPDATE_MS_TICKS * LCR_DEF_UPDATE_LOGIC_MS;
  std::uint32_t logicUpdated = 0;

  volatile bool update : 1, updateLogic : 1, updateDisp : 1, updateBtns : 1;
  bool resIsOL : 1, firstMeas : 1, lcrIsCap : 1, lcrIsInd : 1;
  // temperature in celsius
  float adcTemp, mcuTemp;
  
  packet volts;
  float tctemp;
  float impCap, impInd;
  float reacCapValue, esrCapValue, reacIndValue, esrIndValue;

  float lcrVolts, lcrCur, lcrSupply, lcrSupplyComp;

  float iirAlpha     = LCR_DEF_IIR_ALPHA;
  float iirAlphaTemp = LCR_DEF_TEMP_IIR_ALPHA;

  std::uint16_t ramDacSamples[DAC_MAX_RAM_SAMPLES];
  static const std::uint16_t flashDacSamples[DAC_MAX_FLASH_SAMPLES];
  float dacFrequency = 0.0f;

  enum class DacFreqPre : std::int8_t
  {
    f50,
    f60,
    f100,
    f120,
    f1k,
    f10k,
    f100k,

    size,
  };
  static constexpr std::uint32_t dacFrequencyPresets[DAC_MAX_PRESETS] = {
    50U, 60U, 100U, 120U, 1000U, 10000U, 100000U
  };
  /*static constexpr float dacPeakCoefficients[DAC_MAX_PRESETS] = {
    0.901f, 0.927f, 0.969f, 0.977f, 0.998f, 0.994f, 0.954f
  };*/
  static constexpr float dacPeakCoefficients[DAC_MAX_PRESETS] = {
    0.900f, 0.927f, 0.969f, 0.976f, 0.997f, 0.992f, 0.951f
  };
  // value -1 -> user frequency
  std::int8_t dacSelPreset;
  float peakCoefficient;

  // internal adc averaging array
  /*std::uint16_t adcSampleAvgArr[ADC_MEASOUT_MAX_AVG_SAMPLES];
  std::uint32_t adcSampleAvg;
  std::uint16_t adcSampleIdx;
  std::uint16_t adcNumSamples;*/

  constexpr LCRState()
    : debug(false),
    btn1State(false), btn2State(false), btn3State(false),
    btn4State(false), btn5State(false), btnOhms(INFINITY),
    btnModeState(false), btnModeIntOccur(false),
    isLcdBright(DEF_LCD_BRIGHT_ENABLED),
    lcdBright(DEF_LCD_BRIGHT), ledBright(DEF_LED_BRIGHT), mode(Mode::Volts),

    update(true), updateLogic(true), updateDisp(true), updateBtns(true),

    resIsOL(true), firstMeas(true), lcrIsCap(false), lcrIsInd(false),
    
    adcTemp(0.0f), mcuTemp(0.0f),

    volts(0.0f),
    tctemp(0.0f),
    impCap(0.0f), impInd(0.0f),
    reacCapValue(0.0f), esrCapValue(0.0f), reacIndValue(0.0f), esrIndValue(0.0f),

    lcrVolts(0.0f), lcrCur(0.0f), lcrSupply(0.16f), lcrSupplyComp(0.16f),

    ramDacSamples{},
    dacFrequency(float(dacFrequencyPresets[std::int8_t(LCR_DEFAULT_FREQUENCY_PRE)])),
    dacSelPreset(std::int8_t(LCR_DEFAULT_FREQUENCY_PRE)),
    peakCoefficient(1.0f)

  //adcSampleAvgArr{}, adcSampleAvg(0UL),
  //adcSampleIdx(0U), adcNumSamples(0U)
  {
  }

  /*std::uint16_t esrFirstIdx() const volatile {
    return this->esrFilled() ? this->esrSampleIdx : 0;
  }
  bool esrFilled() const volatile {
    return this->esrNumSamples == ESR_MAX_AVG_SAMPLES;
  }
  void esrClear() volatile {
    this->esrSampleAvg = 0;
    this->esrSampleIdx = 0;
    this->esrNumSamples = 0;
  }*/

  void displayLogic();
  void logic();
  void updateRate(std::uint32_t ratems);

  void initClocks() const;
  void initUSB() const;
  void initLED() const;
  void initButtons(); // (including mode switch)
  void initGPIO() const;
  void initSerial() const;
  void initLCD();
  void initIntADC() const;
  void initExtADC() const;
  void initDAC() const;
  void initMeasure();
  void initSettings();

  void initLoop();

  void loadCal(bool firsttimecal = false);
  void saveCal() const;
  void saveSettings() const;
  void autoCal(bool interactive);

  void setrgb(int r, int g, int b) const;
  void setrgb(float r, float g, float b) const;
  void setLCDBright(int bright, bool enable = true);
  void setLCDBright(float bright, bool enable = true);
  void enableLCDBright(bool enable = true);
  void setLEDBright(int bright);
  void setLEDBright(float bright);
  
  bool comRead(char* buf, std::uint8_t& bufidx) const;
  bool comComp(const char * buf, const char * cmd, std::uint16_t len = 0) const;
  void modifyCal(CalCmd cmd, bool printOnly = true, float newValue = 0.0f);
  void manualCal(const char * cmd);
  void communicationLoop();
  void printInfo(Info type, std::uint8_t decPlaces = 0, bool hasData = false, float uData = 0.0f) const;
  void fetchButtons();
  void printButtons(const char * stockStatus = "", bool printTemp = false, float temp = 0.0f) const;
  void printInfo(const char * title, float voltage, float adcgain, float gain, bool isAtten) const;

  void genCos(std::uint16_t numSamples, float scaleFactor = 1.0f);
  void genOut(std::uint32_t frequency);
  void calcPeakCoef();

  // mode switch
  void defMode() const;
  bool setMode(Mode newmode, bool force = false);
};

namespace lcrlogic
{
  extern LCRState state;
}

// Function declarations
void modeBtnISR();
int SerialPrintf(const char* format, ...);


