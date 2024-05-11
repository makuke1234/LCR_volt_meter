#pragma once

#include "main.hpp"
#include "portAccess.hpp"
#include "clocks.hpp"

#include "algo.hpp"
#include <cassert>
#include <cstdint>

#define VOLTS_CAL_SIZE  12
#define OHMS_CAL_SIZE    4
#define IMP_CAL_SIZE    12

#define CAL_VOLTS_GAIN2X_ID          0
#define CAL_VOLTS_GAIN22X_ID         1
#define CAL_VOLTS_GAIN202X_ID        2
#define CAL_VOLTS_GAIN222X_ID        3
#define CAL_VOLTS_OFF2X_ID           4
#define CAL_VOLTS_OFF22X_ID          5
#define CAL_VOLTS_OFF202X_ID         6
#define CAL_VOLTS_OFF222X_ID         7
#define CAL_VOLTS_HVATTEN_GAIN_ID    8
#define CAL_VOLTS_HVATTEN_OFF_ID     9
#define CAL_VOLTS_SDIV_GAIN_ID      10
#define CAL_VOLTS_SDIV_OFF_ID       11

#define CAL_OHMS_GAIN3900R_ID   0
#define CAL_OHMS_GAIN47R_ID     1
#define CAL_OHMS_OFF3900R_ID    2
#define CAL_OHMS_OFF47R_ID      3

#define CAL_IMP_GAINCUR_ID   0
#define CAL_IMP_GAINVOLT_ID  1
#define CAL_IMP_OFFCUR_ID    2
#define CAL_IMP_OFFVOLT_ID   3
#define CAL_IMP_SHUNT1_ID    4
#define CAL_IMP_SHUNT2_ID    5
#define CAL_IMP_SHUNT3_ID    6
#define CAL_IMP_C1_ID        7
#define CAL_IMP_C2_ID        8
#define CAL_IMP_OFFRES_ID    9
#define CAL_IMP_OFFCAP_ID   10
#define CAL_IMP_OFFIND_ID   11

#define SLIDING_MAX_SAMPLES  21


namespace lcr
{
	using sampleFunc_t = std::int32_t (*)(void * caldata, bool precisemode, std::int32_t & instantSample);
  // gives closest gain to asked gain, returns real gain
	using gainFunc_t   = float (*)(void * caldata, float gain);

	std::int32_t autoScaleGetSample(
    void * caldata,
		sampleFunc_t sampleFunc,
		gainFunc_t gainFunc,
		float & oldgain, float maxgain, float mingain,
    std::int32_t minSample, std::int32_t maxSample,
    std::int32_t minSampleMinGain, std::int32_t maxSampleMinGain,
		bool precisemode = false
	);

  namespace volts
  {
    std::int32_t sampleVolts(void * caldata, bool precisemode, std::int32_t & instantSample);
    std::int32_t sampleAdc(void * caldata, sdadc::Channel chpos, sdadc::Channel chneg);

    // gain function also accounts for gain offset but NOT voltage offset
    float gainVolts(void * caldata, float gain);
    float gainAdc_(void * caldata, float gain, bool daisychained);
    float gainAdc(void * caldata, float gain);

    enum class Gain : std::uint8_t
    {
      g2x,
      g22x,
      g202x,
      g222x,

      size
    };

    struct MeterCalData
    {
      static constexpr lcr::sampleFunc_t getSample = &lcr::volts::sampleVolts;
      static constexpr lcr::gainFunc_t setGain_     = &lcr::volts::gainVolts;
      volts::Gain gain = volts::Gain::g2x;

      float gainf = 2.0f;
      // adc gain is located in sdadc::calData.gainSel

      // gain offsets for external instrumentation amp (4): 2x; 22x; 202x; 222x
      float igains[4] = { DEF_OHMS_GAIN_NORM, DEF_OHMS_GAIN_RG1, DEF_OHMS_GAIN_RG2, DEF_OHMS_GAIN_RG1_2 };

      // voltage offsets for gains
      float ioffsets[4];

      float hvatten_gain = DEF_VOLTAGE_ATTEN;
      float hvatten_offset = DEF_VOLTAGE_ATTEN_OFFSET;

      float supplyDiv_gain = DEF_LCR_GAIN, rsupplyDiv_gain = 1.0f / DEF_LCR_GAIN;
      float supplyDiv_offset = 0.0f;

      float zeroOffset = 0.0f;

      void setGain(lcr::volts::Gain gain);

      void loadCal(const algo::array<float, VOLTS_CAL_SIZE> & data);
      void saveCal(algo::array<float, VOLTS_CAL_SIZE> & data) const;
      
      void selfCalOffset();
      void selfCal1Volt();

    };
    extern MeterCalData calData;

    void init();

    float zeroReading(float reading, bool enable = true);
    
    bool mode();
    bool modeThermo();
    float measure(float * adcgain = nullptr);
    float measureFast(float * adcgain = nullptr);
    // mesaures voltage & cold junction temperature, returns temperature reading
    float measureThermo(float & diff, bool & suc, float * adcgain = nullptr);
    float measureThermo(float & volts, float & diff, bool & suc, float * adcgain = nullptr);
    float measureSupply(bool & suc, float * adcgain = nullptr);
    float mesaureColdJunction(bool & suc, float * adcgain = nullptr);
  }

  namespace ohms
  {
    // first, the current setting would be used to set "gain", if max current achieved, use adc gain
    float gainOhms(void * caldata, float gain);

    enum class Gain : std::uint8_t
    {
      g3900r,
      g47r,

      size
    };

    struct MeterCalData
    {
      static constexpr lcr::sampleFunc_t getSample = &lcr::volts::sampleVolts;
      static constexpr lcr::gainFunc_t   setGain   = &lcr::ohms::gainOhms;
      ohms::Gain gain = ohms::Gain::g47r;

      float gainf = 2.0f;
      float testdutyf = 0.0f;
      // current range gain offsets for generating precise current (2)
      float gains[2] = { DEF_OHMS_CUR_SHUNT2_R, DEF_OHMS_CUR_SHUNT1_R };

      // current range voltage offset
      float offsets[2];
      std::uint8_t selRange = 0;
  
      float zeroOffset = 0.0f;

      void loadCal(const algo::array<float, OHMS_CAL_SIZE> & data);
      void saveCal(algo::array<float, OHMS_CAL_SIZE> & data) const;

      void selfCal();
    };
    extern MeterCalData calData;

    void init();

    float zeroReading(float reading, bool enable = true);

    bool mode();
    float measure();

    float getcurrent(bool & suc);
    // sets test current in amps, returns real test current
    float setcurrent(float cur);
  }

  namespace imp
  {
    std::int32_t sampleCap(void * caldata, bool precisemode, std::int32_t & instantSample);
    std::int32_t sampleCurrent(void * caldata, bool precisemode, std::int32_t & instantSample);
    std::int32_t sampleOut(void * caldata, bool precisemode, std::int32_t & instantSample);

    float gainCap(void * caldata, float gain);
    float gainCurrent(void * caldata, float gain);
    float gainOut(void * caldata, float gain);

    struct MeterCalData
    {
      static constexpr lcr::sampleFunc_t getSample1 = &lcr::imp::sampleCap;
      static constexpr lcr::gainFunc_t setGain1     = &lcr::imp::gainCap;
      float gainCapf = 1.0f;

      static constexpr lcr::sampleFunc_t getSample2 = &lcr::imp::sampleCurrent;
      static constexpr lcr::gainFunc_t setGain2     = &lcr::imp::gainCurrent;
      float gainCurrentf = 1.0f;

      static constexpr lcr::sampleFunc_t getSample3 = &lcr::imp::sampleOut;
      static constexpr lcr::gainFunc_t setGain3     = &lcr::imp::gainOut;
      float gainOutMeasf = 1.0f;


      // gain is 8.5x for both current and voltage

      // current gain offset & voltage offset
      float gainCur = DEF_CURRENT_GAIN, rgainCur = 1.0f / DEF_CURRENT_GAIN, offCur = 0.0014f;

      // voltage gain offset & voltage offset
      float gainVolt = DEF_VOLTAGE_GAIN, rgainVolt = 1.0f / DEF_VOLTAGE_GAIN, offVolt = 0.00185f;

      float c1 = 11.8e-12f, c2 = 9.0e-12f;
      float zc1 = 1e9, zc2 = 1e9, zcpar = 1e9, parFactor = 1.0f;

      // shunts
      float shuntRes[3] = { LCR_DEF_RANGE1_R, LCR_DEF_RANGE2_R, LCR_DEF_RANGE3_R };
      std::uint8_t selRange = 0;

      float offRes = 0.15f;
      float offCap = 55.0e-12f;
      float offInd = 50.0e-9f;

      MeterCalData();

      void loadCal(const algo::array<float, IMP_CAL_SIZE> & data);
      void saveCal(algo::array<float, IMP_CAL_SIZE> & data) const;

      void selfCal();

      float getShuntRes() const;
      void calcParasiticImp(float frequency);
      
    };
    extern MeterCalData calData;

    void init();

    float zeroReadingRes(float reading, bool enable = true);
    float zeroReadingCap(float reading, bool enable = true);
    float zeroReadingInd(float reading, bool enable = true);

    bool mode();
    float measureVolts();
    float measureCur();
    // also measure OUT_MEAS_BUF to protect device & go into failure mode
    void measure(float & volts, float & current, float & supply, bool peaking = false);
    void calcMeas1(float voltscap, float voltscur, float supply, float & resistance, float & reactance);
    void calcMeas2(float voltscap, float voltscur, float supply, float & resistance, float & reactance);
    void setrange(std::uint8_t rangeidx);

    // Calculate capacitance in nF, inductance in microhenries
    float calcCap(float reactance, float frequency);
    float calcInd(float reactance, float frequency);
    float calcPhase(bool inductor, float resistance, float reactance);
    float calcDissipation(float resistance, float reactance);
    float calcQ(float resistance, float reactance);
  }

  bool canChangeMode();
}
