// Un-comment to disable asserts
#define NDEBUG

#include "main.hpp"
#include "ADC.hpp"
#include "display.hpp"
#include "lcr.hpp"
#include "pwm.hpp"
#include "usbheartbeat.hpp"
#include "mcp3461.hpp"
#include "DAC.hpp"
#include "settings.hpp"

LCRState lcrlogic::state;

using namespace lcrlogic;

constexpr std::uint32_t LCRState::dacFrequencyPresets[DAC_MAX_PRESETS];
constexpr float LCRState::dacPeakCoefficients[DAC_MAX_PRESETS];

const std::uint16_t LCRState::flashDacSamples[DAC_MAX_FLASH_SAMPLES] = {
  972, 972, 972, 972, 972, 972, 972, 972, 972, 971, 971, 971, 971, 971, 971, 971,
  971, 971, 970, 970, 970, 970, 970, 970, 969, 969, 969, 969, 968, 968, 968, 968,
  967, 967, 967, 967, 966, 966, 966, 965, 965, 965, 964, 964, 963, 963, 963, 962,
  962, 961, 961, 961, 960, 960, 959, 959, 958, 958, 957, 957, 956, 956, 955, 955,
  954, 954, 953, 953, 952, 951, 951, 950, 950, 949, 948, 948, 947, 946, 946, 945,
  944, 944, 943, 942, 942, 941, 940, 939, 939, 938, 937, 936, 936, 935, 934, 933,
  932, 932, 931, 930, 929, 928, 927, 927, 926, 925, 924, 923, 922, 921, 920, 919,
  918, 918, 917, 916, 915, 914, 913, 912, 911, 910, 909, 908, 907, 906, 905, 904,
  903, 901, 900, 899, 898, 897, 896, 895, 894, 893, 892, 891, 889, 888, 887, 886,
  885, 884, 882, 881, 880, 879, 878, 876, 875, 874, 873, 871, 870, 869, 868, 866,
  865, 864, 863, 861, 860, 859, 857, 856, 855, 853, 852, 851, 849, 848, 847, 845,
  844, 842, 841, 840, 838, 837, 835, 834, 833, 831, 830, 828, 827, 825, 824, 822,
  821, 819, 818, 816, 815, 813, 812, 810, 809, 807, 806, 804, 803, 801, 800, 798,
  797, 795, 793, 792, 790, 789, 787, 785, 784, 782, 781, 779, 777, 776, 774, 772,
  771, 769, 767, 766, 764, 762, 761, 759, 757, 756, 754, 752, 751, 749, 747, 745,
  744, 742, 740, 738, 737, 735, 733, 732, 730, 728, 726, 724, 723, 721, 719, 717,
  716, 714, 712, 710, 708, 707, 705, 703, 701, 699, 697, 696, 694, 692, 690, 688,
  686, 685, 683, 681, 679, 677, 675, 673, 672, 670, 668, 666, 664, 662, 660, 658,
  656, 655, 653, 651, 649, 647, 645, 643, 641, 639, 637, 635, 633, 632, 630, 628,
  626, 624, 622, 620, 618, 616, 614, 612, 610, 608, 606, 604, 602, 600, 598, 596,
  594, 593, 591, 589, 587, 585, 583, 581, 579, 577, 575, 573, 571, 569, 567, 565,
  563, 561, 559, 557, 555, 553, 551, 549, 547, 545, 543, 541, 539, 537, 535, 533,
  531, 529, 527, 525, 523, 521, 519, 517, 515, 513, 511, 509, 507, 505, 503, 501,
  499, 497, 495, 493, 491, 489, 487, 485, 483, 481, 479, 477, 475, 473, 471, 469,
  467, 465, 463, 461, 459, 457, 455, 453, 451, 449, 447, 445, 443, 441, 439, 437,
  435, 433, 431, 429, 428, 426, 424, 422, 420, 418, 416, 414, 412, 410, 408, 406,
  404, 402, 400, 398, 396, 394, 392, 391, 389, 387, 385, 383, 381, 379, 377, 375,
  373, 371, 369, 368, 366, 364, 362, 360, 358, 356, 354, 352, 351, 349, 347, 345,
  343, 341, 339, 338, 336, 334, 332, 330, 328, 326, 325, 323, 321, 319, 317, 316,
  314, 312, 310, 308, 307, 305, 303, 301, 299, 298, 296, 294, 292, 291, 289, 287,
  285, 284, 282, 280, 278, 277, 275, 273, 272, 270, 268, 266, 265, 263, 261, 260,
  258, 256, 255, 253, 251, 250, 248, 247, 245, 243, 242, 240, 238, 237, 235, 234,
  232, 230, 229, 227, 226, 224, 223, 221, 219, 218, 216, 215, 213, 212, 210, 209,
  207, 206, 204, 203, 201, 200, 198, 197, 195, 194, 193, 191, 190, 188, 187, 185,
  184, 183, 181, 180, 178, 177, 176, 174, 173, 172, 170, 169, 168, 166, 165, 164,
  162, 161, 160, 158, 157, 156, 155, 153, 152, 151, 150, 148, 147, 146, 145, 144,
  142, 141, 140, 139, 138, 137, 135, 134, 133, 132, 131, 130, 129, 127, 126, 125,
  124, 123, 122, 121, 120, 119, 118, 117, 116, 115, 114, 113, 112, 111, 110, 109,
  108, 107, 106, 105, 104, 103, 102, 101, 100, 100, 99, 98, 97, 96, 95, 94,
  93, 93, 92, 91, 90, 89, 89, 88, 87, 86, 85, 85, 84, 83, 83, 82,
  81, 80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 73, 72, 71,
  71, 70, 70, 69, 69, 68, 67, 67, 66, 66, 65, 65, 64, 64, 64, 63,
  63, 62, 62, 61, 61, 61, 60, 60, 59, 59, 59, 58, 58, 58, 57, 57,
  57, 56, 56, 56, 55, 55, 55, 55, 54, 54, 54, 54, 54, 53, 53, 53,
  53, 53, 52, 52, 52, 52, 52, 52, 52, 52, 52, 51, 51, 51, 51, 51,
  51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 51, 52, 52, 52,
  52, 52, 52, 52, 52, 52, 53, 53, 53, 53, 53, 54, 54, 54, 54, 54,
  55, 55, 55, 55, 56, 56, 56, 57, 57, 57, 58, 58, 58, 59, 59, 59,
  60, 60, 61, 61, 61, 62, 62, 63, 63, 64, 64, 64, 65, 65, 66, 66,
  67, 67, 68, 69, 69, 70, 70, 71, 71, 72, 73, 73, 74, 74, 75, 76,
  76, 77, 78, 78, 79, 80, 80, 81, 82, 83, 83, 84, 85, 85, 86, 87,
  88, 89, 89, 90, 91, 92, 93, 93, 94, 95, 96, 97, 98, 99, 100, 100,
  101, 102, 103, 104, 105, 106, 107, 108, 109, 110, 111, 112, 113, 114, 115, 116,
  117, 118, 119, 120, 121, 122, 123, 124, 125, 126, 127, 129, 130, 131, 132, 133,
  134, 135, 137, 138, 139, 140, 141, 142, 144, 145, 146, 147, 148, 150, 151, 152,
  153, 155, 156, 157, 158, 160, 161, 162, 164, 165, 166, 168, 169, 170, 172, 173,
  174, 176, 177, 178, 180, 181, 183, 184, 185, 187, 188, 190, 191, 193, 194, 195,
  197, 198, 200, 201, 203, 204, 206, 207, 209, 210, 212, 213, 215, 216, 218, 219,
  221, 223, 224, 226, 227, 229, 230, 232, 234, 235, 237, 238, 240, 242, 243, 245,
  247, 248, 250, 251, 253, 255, 256, 258, 260, 261, 263, 265, 266, 268, 270, 272,
  273, 275, 277, 278, 280, 282, 284, 285, 287, 289, 291, 292, 294, 296, 298, 299,
  301, 303, 305, 307, 308, 310, 312, 314, 316, 317, 319, 321, 323, 325, 326, 328,
  330, 332, 334, 336, 338, 339, 341, 343, 345, 347, 349, 351, 352, 354, 356, 358,
  360, 362, 364, 366, 368, 369, 371, 373, 375, 377, 379, 381, 383, 385, 387, 389,
  391, 392, 394, 396, 398, 400, 402, 404, 406, 408, 410, 412, 414, 416, 418, 420,
  422, 424, 426, 428, 429, 431, 433, 435, 437, 439, 441, 443, 445, 447, 449, 451,
  453, 455, 457, 459, 461, 463, 465, 467, 469, 471, 473, 475, 477, 479, 481, 483,
  485, 487, 489, 491, 493, 495, 497, 499, 501, 503, 505, 507, 509, 511, 513, 515,
  517, 519, 521, 523, 525, 527, 529, 531, 533, 535, 537, 539, 541, 543, 545, 547,
  549, 551, 553, 555, 557, 559, 561, 563, 565, 567, 569, 571, 573, 575, 577, 579,
  581, 583, 585, 587, 589, 591, 593, 594, 596, 598, 600, 602, 604, 606, 608, 610,
  612, 614, 616, 618, 620, 622, 624, 626, 628, 630, 632, 633, 635, 637, 639, 641,
  643, 645, 647, 649, 651, 653, 655, 656, 658, 660, 662, 664, 666, 668, 670, 672,
  673, 675, 677, 679, 681, 683, 685, 686, 688, 690, 692, 694, 696, 697, 699, 701,
  703, 705, 707, 708, 710, 712, 714, 716, 717, 719, 721, 723, 724, 726, 728, 730,
  732, 733, 735, 737, 738, 740, 742, 744, 745, 747, 749, 751, 752, 754, 756, 757,
  759, 761, 762, 764, 766, 767, 769, 771, 772, 774, 776, 777, 779, 781, 782, 784,
  785, 787, 789, 790, 792, 793, 795, 797, 798, 800, 801, 803, 804, 806, 807, 809,
  810, 812, 813, 815, 816, 818, 819, 821, 822, 824, 825, 827, 828, 830, 831, 833,
  834, 835, 837, 838, 840, 841, 842, 844, 845, 847, 848, 849, 851, 852, 853, 855,
  856, 857, 859, 860, 861, 863, 864, 865, 866, 868, 869, 870, 871, 873, 874, 875,
  876, 878, 879, 880, 881, 882, 884, 885, 886, 887, 888, 889, 891, 892, 893, 894,
  895, 896, 897, 898, 899, 900, 901, 903, 904, 905, 906, 907, 908, 909, 910, 911,
  912, 913, 914, 915, 916, 917, 918, 918, 919, 920, 921, 922, 923, 924, 925, 926,
  927, 927, 928, 929, 930, 931, 932, 932, 933, 934, 935, 936, 936, 937, 938, 939,
  939, 940, 941, 942, 942, 943, 944, 944, 945, 946, 946, 947, 948, 948, 949, 950,
  950, 951, 951, 952, 953, 953, 954, 954, 955, 955, 956, 956, 957, 957, 958, 958,
  959, 959, 960, 960, 961, 961, 961, 962, 962, 963, 963, 963, 964, 964, 965, 965,
  965, 966, 966, 966, 967, 967, 967, 967, 968, 968, 968, 968, 969, 969, 969, 969,
  970, 970, 970, 970, 970, 970, 971, 971, 971, 971, 971, 971, 971, 971, 971, 972,
  972, 972, 972, 972, 972, 972, 972
};

void setup()
{
  state.initClocks();
  state.initUSB();
  state.initLED();
  state.initButtons();
  state.initGPIO();
  state.initSerial();
  state.initLCD();
  state.initIntADC();
  state.initExtADC();
  state.initDAC();
  state.initMeasure();
  state.initSettings();

  state.initLoop();

	SERIAL.println("Initialization done!");
}
static std::uint32_t ticksabs = 0;
static bool firstTemp = true;
void loop()
{
  state.communicationLoop();
  if (state.updateLogic)
  {
    //auto start = micros();
    state.logic();
    ++state.logicUpdated;

    //lcrlogic::state.logicUpdated = micros() - start;

    /*bool suc = true;
    sample = lcr::volts::mesaureColdJunction(suc);
    if (first && suc)
    {
      temp = sample;
      first = false;
    }
    else if (suc)
    {
      temp = LCR_DEF_IIR_ALPHA * sample + (1.0f - LCR_DEF_IIR_ALPHA) * temp;
    }
    state.adcTemp = temp;*/

    state.updateLogic = 0;
  }
  if (state.updateDisp)
  {
    // stuff that needs updating every 1 ms
    state.displayLogic();
    if (state.debug)
    {
      SERIAL.print("btn: ");
      SERIAL.print(state.btn1State);
      SERIAL.print(", ");
      SERIAL.print(state.btn2State);
      SERIAL.print(", ");
      SERIAL.print(state.btn3State);
      SERIAL.print(", ");
      SERIAL.print(state.btn4State);
      SERIAL.print(", ");
      SERIAL.print(state.btn5State);
      SERIAL.print("; ");
      SERIAL.print(state.btnOhms);
      SERIAL.print("; ");

      if (!firstTemp)
      {
        SERIAL.print(state.adcTemp, 2);
        SERIAL.print(" deg. C; ");

        //SERIAL.print(sample, 2);
        //SERIAL.print(" deg. C; ");
      }

      /*sdadc::setGain(sdadc::Gain::g0x33);
      bool suc = false;*/
      // ADC_OHMS_CH
      /*auto sample = sdadc::getSample(ADC_VB0V3_CH, ADC_OHMS_CH, suc);
      auto volts = sdadc::toVolts(sample);
      //auto volts = lcr::volts::measureSupply(suc);
      SERIAL.print(sample);
      SERIAL.print("; ");
      SERIAL.print(volts, 4);
      SERIAL.print(" V; ");*/

      //auto status = sdadc::status();
      //SERIAL.print(status, HEX);
      //SERIAL.print("; ");

      SERIAL.println(ticksabs);
    }

    SERIAL.print("ESR C: ");
    SERIAL.print(state.esrCapValue);
    SERIAL.print(" Reac C: ");
    SERIAL.print(state.reacCapValue);
    SERIAL.print(" ESR L: ");
    SERIAL.print(state.esrIndValue);
    SERIAL.print(" Reac L: ");
    SERIAL.print(state.reacIndValue);
    SERIAL.print(" volts: ");
    SERIAL.print(state.lcrVolts, 4);
    SERIAL.print(" cur: ");
    SERIAL.print(state.lcrCur, 4);
    SERIAL.print(" bias: ");
    SERIAL.print(LCR_AMPLITUDE(state.lcrSupplyComp), 5);
    SERIAL.print(" ticks: ");
    SERIAL.print(state.logicUpdated);
    SERIAL.println();
    
    state.updateDisp = 0;
  }
}

void LCRState::displayLogic()
{
  switch (this->mode)
  {
  case Mode::LCR:
    lcr::imp::calcMeas1(this->lcrVolts, this->lcrCur, this->lcrSupplyComp, this->esrCapValue, this->reacCapValue);
    lcr::imp::calcMeas2(this->lcrVolts, this->lcrCur, this->lcrSupplyComp, this->esrIndValue, this->reacIndValue);
    this->impCap = sqrtf(this->esrCapValue * this->esrCapValue + this->reacCapValue * this->reacCapValue);
    this->impInd = sqrtf(this->esrIndValue * this->esrIndValue + this->reacIndValue * this->reacIndValue);
    break;
  }

  switch (this->mode)
  {
  case Mode::Volts:
    disp::screenVolts(this->volts.res, digitalRead(VOLTS_RANGE2));
    this->printInfo("volts", this->volts.res, this->volts.gain, lcr::volts::calData.igains[std::uint8_t(lcr::volts::calData.gain)], digitalRead(VOLTS_RANGE2));
    break;
  case Mode::Thermo:
    disp::screenThermo(this->volts.res, this->tctemp);
    break;
  case Mode::Shunt:
    disp::screenOhms(this->impInd, lcr::ohms::calData.gain == lcr::ohms::Gain::g3900r);
    break;
  case Mode::LCR:
    disp::screenLCR(
      this->esrCapValue, this->reacCapValue, this->impCap,
      this->esrIndValue, this->reacIndValue, this->impInd,
      this->dacFrequency,
      this->lcrIsCap, this->lcrIsInd
    );
    break;
  case Mode::Failure:
    disp::screenFail();
    break;
  }

  static float tempDisplayValue = 0.0;
  tempDisplayValue = algo::hysteresis(tempDisplayValue, this->adcTemp, 0.05f);
  
  this->printButtons("Menu", true, tempDisplayValue);
  //disp::screenStatus();
  //this->printButtons((!this->btnOhms || (this->btnOhms > 40000.0f)) ? (this->btnModeState ? "Shunt" : "Volts") : "");
}
void LCRState::logic()
{
  static std::uint8_t prevBtn = 0;

  // read buttons to know how to switch modes
  if (this->btnModeState && (this->mode != Mode::Shunt))
  {
    this->setMode(Mode::Shunt);
  }
  else if (!this->btnModeState && ((this->mode == Mode::Shunt) || (this->mode == Mode::Failure)))
  {
    this->setMode(Mode::Volts);
  }
  else if (!this->btnModeState)
  {
    if (this->btn1State)
    {
      this->setMode(Mode::LCR);
      prevBtn = 1;
    }
    else if (this->btn2State)
    {
      this->setMode(Mode::Thermo);
      prevBtn = 2;
    }
    else if (this->btn3State)
    {
      this->setMode(Mode::Volts);
      prevBtn = 3;
    }
    else if (this->btn4State)
    {
      // set shunt
      if (prevBtn != 4)
      {
        lcr::imp::setrange((lcr::imp::calData.selRange + 1) % 3);
      }
      prevBtn = 4;
    }
    else if (this->btn5State)
    {
      // set frequency
      if (prevBtn != 5)
      {
        this->dacSelPreset = (this->dacSelPreset + 1) % DAC_MAX_PRESETS;
        this->genOut(this->dacFrequencyPresets[std::uint8_t(this->dacSelPreset)]);
      }
      prevBtn = 5;
    }
    else
    {
      prevBtn = 0;
    }
  }


  const auto nextsample = [](float oldsample, float newsample, float alpha) -> float
  {
    return newsample * alpha + oldsample * (1.0f - alpha);
  };

  float thermo, thermodiff, volts, cur, supply;
  auto alpha = (this->firstMeas) ? 1.0f : this->iirAlpha;
  this->firstMeas = false;
  bool suc = false;

  // read ADC inputs depending on mode
  switch (this->mode)
  {
  case Mode::Volts:
    this->volts.res = nextsample(this->volts.res, lcr::volts::measure(&this->volts.gain), alpha);
    break;
  case Mode::Thermo:
  {
    float voltsraw;
    suc = false;
    thermo = lcr::volts::measureThermo(voltsraw, thermodiff, suc, &this->volts.gain);
    this->volts.res = nextsample(this->volts.res, voltsraw, alpha);
    this->tctemp    = nextsample(this->tctemp, thermo, alpha);
    break;
  }
  case Mode::Shunt:
    this->impInd = nextsample(this->impInd, lcr::ohms::measure(), alpha);
    break;
  case Mode::LCR:
    lcr::imp::measure(volts, cur, supply, this->dacFrequency < 2000.0f);

    // apply filter to data
    this->lcrVolts  = nextsample(this->lcrVolts, volts, alpha);
    this->lcrCur    = nextsample(this->lcrCur, cur, alpha);
    this->lcrSupply = nextsample(this->lcrSupply, supply, 0.1f * alpha);
    this->lcrSupplyComp = this->lcrSupply * this->peakCoefficient;
    break;
  }

  // calculate MCU temperature only every 100 cycles
  static std::uint8_t mcuTempCount = 0;

  if (this->mode == Mode::Thermo)
  {
    auto adctempsample = thermo - thermodiff;
    auto tempalpha = firstTemp ? 1.0f : this->iirAlphaTemp;
    this->adcTemp = nextsample(this->adcTemp, adctempsample, tempalpha);
  }
  else
  {
    /*bool suc = true;
    sample = lcr::volts::mesaureColdJunction(suc);
    if (first && suc)
    {
      temp = sample;
      first = false;
    }
    else if (suc)
    {
      temp = LCR_DEF_IIR_ALPHA * sample + (1.0f - LCR_DEF_IIR_ALPHA) * temp;
    }
    state.adcTemp = temp;*/
    if (!mcuTempCount)
    {
      bool suc = false;
      auto sample = lcr::volts::mesaureColdJunction(suc);
      auto tempalpha = firstTemp ? 1.0f : this->iirAlpha;
      this->adcTemp = nextsample(this->adcTemp, sample, tempalpha);
    }
  }
  firstTemp = false;


  /*if (!mcuTempCount)
  {
    this->mcuTemp = nextsample(this->mcuTemp, adc::getTemp(adc::sample(adc::Channel::IntTemp, true, false)), this->iirAlpha);
  }*/

  mcuTempCount = (mcuTempCount + 1) % 100;
}

void LCRState::updateRate(std::uint32_t ratems)
{
  this->updateLogicMs    = ratems;
  this->updateLogicTicks = LCR_UPDATE_MS_TICKS * this->updateLogicMs;
}

void LCRState::initClocks() const
{
  // init 8 Mhz GCLK3 from 48 Mhz PLL running from crystal for better accuracy
  clk::initGCLK(GCLK_CLKCTRL_GEN_GCLK3_Val, GCLK_GENCTRL_SRC_DFLL48M_Val, 48/8);
  // init 96 Mhz PLL from 32 kHz crystal
	clk::pll96(GCLK_CLKCTRL_GEN_GCLK1_Val, 32768U);
  // set 96 Mhz PLL as GCLK4
	clk::initGCLK(GCLK_CLKCTRL_GEN_GCLK4_Val, GCLK_GENCTRL_SRC_DPLL96M_Val);
}
void LCRState::initUSB() const
{
	heartbeat::init();
}
void LCRState::initLED() const
{
  OutModeStrong(ARDUINO_PIN_TO_PORT_PIN(LED_R_PWM));
  OutModeStrong(ARDUINO_PIN_TO_PORT_PIN(LED_G_PWM));
  OutModeStrong(ARDUINO_PIN_TO_PORT_PIN(LED_B_PWM));

	pwm::init0(LED_PWM_FREQUENCY);
	pwm::add0(LED_R_CC, LED_R_PWM, 0, true);
	pwm::add0(LED_G_CC, LED_G_PWM, 0, true);
	pwm::add0(LED_B_CC, LED_B_PWM, 0, true);
}
void LCRState::initButtons()
{
  pinMode(BTN_ADC, INPUT);
  
  pinMode(VOLTS_MODE_IN, INPUT_PULLDOWN);
  // wait for it to settle after enabling pull-down
  delay(1);
  this->btnModeState = !digitalRead(VOLTS_MODE_IN);
  // Attach interrupt to the button
	attachInterrupt(digitalPinToInterrupt(VOLTS_MODE_IN), &modeBtnISR, CHANGE);
}
void LCRState::initGPIO() const
{
  pinMode(OUT_MEAS_ADC, INPUT);

  pinMode(VOLTS_RANGE1, OUTPUT);
  pinMode(VOLTS_RANGE2, OUTPUT);

  pinMode(OHMS_PWM, OUTPUT);
  pwm::add0(OHMS_CC, OHMS_PWM);
  
  pinMode(OHMS_22X, OUTPUT);
  pinMode(OHMS_202X, OUTPUT);
  pinMode(OHMS_INVERT, OUTPUT);

  pinMode(LCR_ENABLE, OUTPUT);
  digitalWrite(LCR_ENABLE, 0);

  pinMode(LCR_RANGE3, OUTPUT);
  digitalWrite(LCR_RANGE3, 1);
  pinMode(LCR_RANGE2, OUTPUT);
  digitalWrite(LCR_RANGE2, 1);
  pinMode(LCR_RANGE1, OUTPUT);
  digitalWrite(LCR_RANGE1, 0);

  pinMode(OHMS_CUR_RANGE, OUTPUT);
  digitalWrite(OHMS_CUR_RANGE, 1);
}
void LCRState::initSerial() const
{
  SERIAL.begin(DEFAULT_BAUDRATE);
	bool connected = false;
	if (!digitalRead(VOLTS_MODE_IN))
	{
		const auto sm = millis() + 500U;
		while (!heartbeat::isConnected() && (millis() < sm));
	}
	if (heartbeat::isConnected())
	{
		this->setrgb(0, 25, 127);
		const auto sm = millis() + 300000U;
		//while (!SERIAL && (millis() < sm));
		//connected = SERIAL != 0;
		while (!SERIAL.available() && (millis() < sm));
    connected = SERIAL.available();
	}

	if (connected)
	{
		SERIAL.println("Serial connection initialized.");
		this->setrgb(0, 255, 10);
	}
	else
	{
		this->setrgb(255, 25, 0);
	}
}
void LCRState::initLCD()
{
  SERIAL.print("Initializing display...");

  pinMode(LCD_PWM, OUTPUT);
  pwm::init3(LCD_PWM_FREQUENCY);
  pwm::add3(LCD_CC, LCD_PWM);
  this->setLCDBright(this->lcdBright, this->isLcdBright);

	disp::init();

  disp::bootlogo(LCD_BOOTLOGO_TIMEOUT_MS);

	SERIAL.println(" OK");
}
void LCRState::initIntADC() const
{
  SERIAL.print("Initializing internal ADC...");

  const auto realres = adc::init(ADC_RESOLUTION_BITS, ADC_FREERUN, ADC_OVERSAMPLING_SAMPLES);
	if (realres == ADC_RESOLUTION_BITS)
	{
		SERIAL.println(" OK");
	}
	else
	{
		SERIAL.print(" Fail! Expected resolution: ");
		SERIAL.print(ADC_RESOLUTION_BITS);
		SERIAL.print("; Got: ");
		SERIAL.println(realres);

		switch (realres)
		{
		case 8:
			this->setrgb(255, 0, 0);
			break;
		case 10:
			this->setrgb(255, 0, 255);
			break;
		case 12:
			this->setrgb(255, 255, 0);
			break;
		case 13:
			this->setrgb(0, 255, 255);
			break;
		case 14:
			this->setrgb(0, 0, 255);
			break;
		case 15:
			this->setrgb(0, 255, 0);
			break;
		}
	}

	this->printInfo(Info::TempAcc);
	this->printInfo(Info::Ref);
  this->printInfo(Info::Supply);
}
void LCRState::initExtADC() const
{
  SERIAL.print("Initializing external SD-ADC...");

  sdadc::init(&sercom3, SDADC_SPI_FREQUENCY, SDADC_MOSI, SDADC_MISO, SDADC_SCK, SDADC_CS);
  sdadc::config(SDADC_OSR_RATIO, sdadc::Dataformat::f32bit_chid_sgn);
  sdadc::autozero(true);

  // print registers
  SERIAL.println();
  sdadc::printRegisters(&SerialPrintf);

  SERIAL.println(" OK");
}
void LCRState::initDAC() const
{
  SERIAL.print("Initializing DAC...");

  dac::init(clk::tmr::tTCC1);

  SERIAL.println(" OK");
}
void LCRState::initMeasure()
{
  SERIAL.print("Initializing measurement soft...");

  lcr::volts::init();
  lcr::ohms::init();
  lcr::imp::init();

  this->updateRate(this->updateLogicMs);

  SERIAL.println(" OK");
}
void LCRState::initSettings()
{
  SERIAL.print("Initializing settings... ");

  settings::init();

  // also do first time self-calibration
  this->loadCal(true);

  SERIAL.print("Loading user preferences...");
  if (!settings::loadSettings() || !settings::flashSettings.valid)
  {
    SERIAL.println(" Settings empty!");
  }
  else
  {
    // load settings
    this->setLCDBright(settings::flashSettings.dispBright, settings::flashSettings.isDispBright);
    this->setLEDBright(settings::flashSettings.ledBright);

    SERIAL.println(" OK");
  }
}

void LCRState::initLoop()
{
  SERIAL.print("Initializing tick timer...");

  // Initialize TC5 for clocking
  std::uint32_t tmrFreq = 96000000U;
  if (!clk::isInit(clk::tmr::tTC5))
  {
    clk::initTmr(clk::tmr::tTC5, GCLK_CLKCTRL_GEN_GCLK4_Val, tmrFreq);
  }
  else
  {
    tmrFreq = clk::tmrSpeed(clk::tmr::tTC5);
  }

  // LCR_UPDATE_FREQUENCY
  // calculate prescaler if desired frequency is too low
  auto prescaler = (tmrFreq / LCR_UPDATE_FREQUENCY) / 65535U + 1U;

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

  TC5->COUNT16.CTRLA.bit.PRESCALER = prescaler_reg;
  TC5->COUNT16.CTRLA.reg |=
    TC_CTRLA_MODE_COUNT16 |
    TC_CTRLA_WAVEGEN_MFRQ;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  // Set up period
  std::uint32_t period = tmrFreq / (prescaler * LCR_UPDATE_FREQUENCY) - 1;
  // TC5 is 16-bit
  period = (period > 65535U) ? 65535U : period;

  TC5->COUNT16.CC[0].reg = std::uint16_t(period);
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  // enable TC5 interrupts
  NVIC_SetPriority(TC5_IRQn, 3);
  NVIC_EnableIRQ(TC5_IRQn);
  TC5->COUNT16.INTENSET.bit.MC0 = 1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  // Start timer
  TC5->COUNT16.CTRLA.bit.ENABLE = 1;
  while (TC5->COUNT16.STATUS.bit.SYNCBUSY);

  SERIAL.print(" Prescaler: ");
  SERIAL.print(prescaler);
  SERIAL.print("; Period: ");
  SERIAL.print(period);
  SERIAL.println(" OK");

  SERIAL.print("Setting default mode...");
  this->setMode(this->mode, true);
  SERIAL.println(" OK");
}

void LCRState::loadCal(bool firsttimecal_opt)
{
  SERIAL.print("Loading CAL data...");

  if (!settings::loadCal() || !settings::flashCal.valid)
  {
    SERIAL.print(" CAL empty!");
    if (firsttimecal_opt)
    {
      SERIAL.println(" Doing first time self-calibration...");
      this->autoCal(false);
      this->saveCal();
    }
    else
    {
      SERIAL.println();
    }
  }
  else
  {
    lcr::volts::calData.loadCal(settings::flashCal.voltsCal);
    lcr::ohms::calData.loadCal(settings::flashCal.ohmsCal);
    lcr::imp::calData.loadCal(settings::flashCal.impCal);
    sdadc::calData.loadCal(settings::flashCal.adcCal);

    SERIAL.println(" CAL loaded.");
  }
}
void LCRState::saveCal() const
{
  SERIAL.print("Saving CAL data...");

  lcr::volts::calData.saveCal(settings::flashCal.voltsCal);
  lcr::ohms::calData.saveCal(settings::flashCal.ohmsCal);
  lcr::imp::calData.saveCal(settings::flashCal.impCal);
  sdadc::calData.saveCal(settings::flashCal.adcCal);

  settings::flashCal.valid = true;

  SERIAL.print(" Updating flash...");
  if (settings::saveCal())
  {
    SERIAL.println(" OK");
  }
  else
  {
    SERIAL.println(" Error saving to flash!");
  }
}
void LCRState::saveSettings() const
{
  SERIAL.print("Saving settings...");

  settings::flashSettings.dispBright   = this->lcdBright;
  settings::flashSettings.isDispBright = this->isLcdBright;
  settings::flashSettings.ledBright    = this->ledBright;

  settings::flashSettings.valid = true;

  if (settings::saveSettings())
  {
    SERIAL.println(" OK");
  }
  else
  {
    SERIAL.println(" Error saving to flash!");
  }
}
void LCRState::autoCal(bool interactive)
{
  SERIAL.println("Auto-CAL in progress...");

  disp::screenStatusOkCancel();
  SERIAL.println("Drew title!");
  disp::screenTitle("Auto-CAL");
  SERIAL.println("Drew title!");

  // calibrate everything, sequence: adc, volts, ohms, impedance
  auto oldmode = this->mode;
  this->setMode(Mode::Failure);

  SERIAL.print(" SD-ADC...");
  sdadc::calData.selfCal(ADC_VB0V3_CH);
  SERIAL.println(" OK");

  if (interactive)
  {
    SERIAL.print(" Instrumentation amp...");

    bool sel;


    // prompt to switch to volts mode (if not already)
    disp::screenTitle("Voltage offset");
    disp::screenPrompt("Please switch to voltage mode.");
    sel = true;
    while (this->btnModeState)
    {
      // read buttons to know whether to skip it
      if (this->btn2State)
      {
        sel = false;
        this->printButtons();
        while (this->btn2State);
        this->printButtons();
        break;
      }
    }

    SERIAL.print(" offset...");
    if (sel)
    {
      // prompt to short the input
      disp::screenPrompt("Please short the input.");
      while (1)
      {
        // read buttons to know whether to continue or skip
        if (this->btn2State)
        {
          sel = false;
          this->printButtons();
          while (this->btn2State);
          this->printButtons();
          break;
        }
        else if (this->btn3State)
        {
          this->printButtons();
          while (this->btn3State);
          this->printButtons();
          break;
        }
      }
    }

    if (sel)
    {
      disp::screenPrompt("In progess...");
      lcr::volts::calData.selfCalOffset();
      SERIAL.println(" OK");
    }
    else
    {
      SERIAL.println(" Skipped");
    }

    SERIAL.print(" Voltage gain...");
    disp::screenTitle("Voltage gain");
    if (sel)
    {
      disp::screenPrompt("Please add 1 V to the input.");
      // prompt to add 1 volt to the input
      while (1)
      {
        // read buttons to know whether to continue or skip
        if (this->btn2State)
        {
          sel = false;
          this->printButtons();
          while (this->btn2State);
          this->printButtons();
          break;
        }
        else if (this->btn3State)
        {
          this->printButtons();
          while (this->btn3State);
          this->printButtons();
          break;
        }
      }
    }

    if (sel)
    {
      disp::screenPrompt("In progess...");
      lcr::volts::calData.selfCal1Volt();
      SERIAL.println(" OK");
    }
    else
    {
      SERIAL.println(" Skipped");
    }
    

    SERIAL.print(" LCR ranges...");
    disp::screenTitle("LCR ranges");

    // prompt to switch to volts mode (if not already)
    disp::screenPrompt("Please switch to voltage mode.");
    sel = true;
    while (this->btnModeState)
    {
      // read buttons to know whether to skip it
      if (this->btn2State)
      {
        sel = false;
        this->printButtons();
        while (this->btn2State);
        this->printButtons();
        break;
      }
    }


    if (sel)
    {
      // prompt to add 100 ohm resistor to the input
      disp::screenPrompt("Please add 100R to the input.");
      while (1)
      {
        // read buttons to know whether to continue or skip
        if (this->btn2State)
        {
          sel = false;
          this->printButtons();
          while (this->btn2State);
          this->printButtons();
          break;
        }
        else if (this->btn3State)
        {
          this->printButtons();
          while (this->btn3State);
          this->printButtons();
          break;
        }
      }
    }

    if (sel)
    {
      disp::screenPrompt("In progess...");
      lcr::imp::calData.selfCal();
      SERIAL.println(" OK");
    }
    else
    {
      SERIAL.println(" Skipped");
    }
    
    
    
    SERIAL.print(" Current ranges...");
    disp::screenTitle("Current ranges");
    disp::screenPrompt("Please switch to shunt mode.");
    // prompt to switch to current mode (if not already)
    sel = true;
    while (!this->btnModeState)
    {
      // read buttons to know whether to skip it
      if (this->btn2State)
      {
        sel = false;
        this->printButtons();
        while (this->btn2State);
        this->printButtons();
        break;
      }
    }

    if (sel)
    {
      // prompt to add 100 ohm resistor to the input (if not already)
      disp::screenPrompt("Please add 100R to the input.");
      while (1)
      {
        // read buttons to know whether to continue or skip
        if (this->btn2State)
        {
          sel = false;
          this->printButtons();
          while (this->btn2State);
          this->printButtons();
          break;
        }
        else if (this->btn3State)
        {
          this->printButtons();
          while (this->btn3State);
          this->printButtons();
          break;
        }
      }
    }

    if (sel)
    {
      disp::screenPrompt("In progess...");
      lcr::ohms::calData.selfCal();
      SERIAL.println(" OK");
    }
    else
    {
      SERIAL.println(" Skipped");
    }
    
  }

  disp::screenPrompt("Going back to old mode...");

  this->setMode(oldmode);

  disp::screenTitle("Calibration complete!");
  SERIAL.println("Auto-CAL OK");

  delay(500);
  disp::data.lcd.clear();
}


void LCRState::setrgb(int r, int g, int b) const
{
  const auto fbright = float(this->ledBright) / 255.0f;
	pwm::duty0(LED_R_CC, (float(r) / 255.0f) * fbright);
	pwm::duty0(LED_G_CC, (float(g) / 255.0f) * fbright);
	pwm::duty0(LED_B_CC, (float(b) / 255.0f) * fbright);
}
void LCRState::setrgb(float r, float g, float b) const
{
  const auto fbright = float(this->ledBright) / 255.0f;
  pwm::duty0(LED_R_CC, r * fbright);
  pwm::duty0(LED_G_CC, g * fbright);
  pwm::duty0(LED_B_CC, b * fbright);
}
void LCRState::setLCDBright(int bright, bool enable)
{
  this->lcdBright = std::uint8_t(bright);
  this->isLcdBright = enable;
  pwm::duty3(LCD_CC, std::uint8_t(this->lcdBright * std::uint8_t(this->isLcdBright)));
}
void LCRState::setLCDBright(float bright, bool enable)
{
  this->lcdBright = std::uint8_t(bright * 255.0f + 0.5f);
  this->isLcdBright = enable;
  pwm::duty3(LCD_CC, std::uint8_t(this->lcdBright * std::uint8_t(this->isLcdBright)));
}
void LCRState::enableLCDBright(bool enable)
{
  this->isLcdBright = enable;
  pwm::duty3(LCD_CC, std::uint8_t(this->lcdBright * std::uint8_t(this->isLcdBright)));
}
void LCRState::setLEDBright(int bright)
{
  std::uint8_t r, g, b;
  r = pwm::getduty0(LED_R_CC);
  g = pwm::getduty0(LED_G_CC);
  b = pwm::getduty0(LED_B_CC);

  this->ledBright = std::uint8_t(bright);
  this->setrgb(r, g, b);
}
void LCRState::setLEDBright(float bright)
{
  std::uint8_t r, g, b;
  r = pwm::getduty0(LED_R_CC);
  g = pwm::getduty0(LED_G_CC);
  b = pwm::getduty0(LED_B_CC);

  this->ledBright = std::uint8_t(bright * 255.f + 0.5f);
  this->setrgb(r, g, b);
}

bool LCRState::comRead(char * buf, std::uint8_t & bufidx) const
{
	if (!SERIAL.available())
	{
		return false;
	}

	buf[bufidx] = SERIAL.read();
	auto & ch = buf[bufidx];
	++bufidx;

	if (bufidx >= SERIAL_READ_BUFSIZE)
	{
		bufidx = 0;
	}

	if ((ch == '\n') || (ch == '\r') || (ch == '\0') || (ch == ';'))
	{
		ch = '\0';
		bufidx = 0;
		return (buf[0] != '\0');
	}

	buf[SERIAL_READ_BUFSIZE - 1] = '\0';
	return false;
}
bool LCRState::comComp(const char * buf, const char * cmd, std::uint16_t len) const
{
  return std::strncmp(buf, cmd, !len ? std::strlen(cmd) : len) == 0;
}
void LCRState::modifyCal(CalCmd cmd, bool printOnly, float newValue)
{
  float * val = nullptr;

  switch (cmd)
  {
  case CalCmd::VoltsGain2:
    val = &lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::g2x)];
    break;
  case CalCmd::VoltsGain22:
    val = &lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::g22x)];
    break;
  case CalCmd::VoltsGain202:
    val = &lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::g202x)];
    break;
  case CalCmd::VoltsGain222:
    val = &lcr::volts::calData.igains[std::uint8_t(lcr::volts::Gain::g222x)];
    break;

  case CalCmd::VoltsOff2:
    val = &lcr::volts::calData.ioffsets[std::uint8_t(lcr::volts::Gain::g2x)];
    break;
  case CalCmd::VoltsOff22:
    val = &lcr::volts::calData.ioffsets[std::uint8_t(lcr::volts::Gain::g22x)];
    break;
  case CalCmd::VoltsOff202:
    val = &lcr::volts::calData.ioffsets[std::uint8_t(lcr::volts::Gain::g202x)];
    break;
  case CalCmd::VoltsOff222:
    val = &lcr::volts::calData.ioffsets[std::uint8_t(lcr::volts::Gain::g222x)];
    break;

  case CalCmd::VoltsAttenGain:
    val = &lcr::volts::calData.hvatten_gain;
    break;
  case CalCmd::VoltsAttenOff:
    val = &lcr::volts::calData.hvatten_offset;
    break;

  case CalCmd::VoltsSupplyGain:
    val = &lcr::volts::calData.supplyDiv_gain;
    break;
  case CalCmd::VoltsSupplyOff:
    val = &lcr::volts::calData.supplyDiv_offset;
    break;

  case CalCmd::OhmsGain3900:
    val = &lcr::ohms::calData.gains[std::uint8_t(lcr::ohms::Gain::g3900r)];
    break;
  case CalCmd::OhmsGain47:
    val = &lcr::ohms::calData.gains[std::uint8_t(lcr::ohms::Gain::g47r)];
    break;
  case CalCmd::OhmsOff3900:
    val = &lcr::ohms::calData.offsets[std::uint8_t(lcr::ohms::Gain::g3900r)];
    break;
  case CalCmd::OhmsOff47:
    val = &lcr::ohms::calData.offsets[std::uint8_t(lcr::ohms::Gain::g47r)];
    break;

  case CalCmd::ImpGainCurrent:
    val = &lcr::imp::calData.gainCur;
    break;
  case CalCmd::ImpGainVolt:
    val = &lcr::imp::calData.gainVolt;
    break;
  case CalCmd::ImpOffCurrent:
    val = &lcr::imp::calData.offCur;
    break;
  case CalCmd::ImpOffVolt:
    val = &lcr::imp::calData.offVolt;
    break;
  
  case CalCmd::ImpShunt1:
    val = &lcr::imp::calData.shuntRes[0];
    break;
  case CalCmd::ImpShunt2:
    val = &lcr::imp::calData.shuntRes[1];
    break;
  case CalCmd::ImpShunt3:
    val = &lcr::imp::calData.shuntRes[2];
    break;

  case CalCmd::ImpParC1:
    val = &lcr::imp::calData.c1;
    break;
  case CalCmd::ImpParC2:
    val = &lcr::imp::calData.c2;
    break;

  case CalCmd::ImpOffResistance:
    val = &lcr::imp::calData.offRes;
    break;
  case CalCmd::ImpOffCapacitance:
    val = &lcr::imp::calData.offCap;
    break;
  case CalCmd::ImpOffInductance:
    val = &lcr::imp::calData.offInd;
    break;

  case CalCmd::AdcGain0x33:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g0x33)];
    break;
  case CalCmd::AdcGain1:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g1x)];
    break;
  case CalCmd::AdcGain2:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g2x)];
    break;
  case CalCmd::AdcGain4:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g4x)];
    break;
  case CalCmd::AdcGain8:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g8x)];
    break;
  case CalCmd::AdcGain16:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g16x)];
    break;
  case CalCmd::AdcGain32:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g32x)];
    break;
  case CalCmd::AdcGain64:
    val = &sdadc::calData.gain[std::uint8_t(sdadc::Gain::g64x)];
    break;
  
  case CalCmd::AdcVref:
    val = &sdadc::calData.vref;
    break;

  default:
    assert(!"Impossible error to reach!");
    return;
  }
  
  // impossible to reach
  assert(val != nullptr);

  if (!printOnly)
  {
    *val = newValue;
    SERIAL.print("Set ");
  }
  
  // print new value (or just value)
  SERIAL.print(calCommands[std::underlying_type<CalCmd>::type(cmd)]);
  SERIAL.print(" value: ");
  SERIAL.println(mh::sci(*val, 6));
}
void LCRState::manualCal(const char * cmdStr)
{
  // check against load & save commands first
  if (this->comComp(cmdStr, "load"))
  {
    this->loadCal();
    return;
  }
  else if (this->comComp(cmdStr, "save"))
  {
    this->saveCal();
    return;
  }

  constexpr std::uint8_t sz = std::underlying_type<CalCmd>::type(CalCmd::size);
  
  auto cmd = CalCmd::size;
  // compare against command list
  // find
  const auto cmdLen = std::strlen(cmdStr);
  for (std::uint8_t i = 0; i < sz; ++i)
  {
    auto len = std::strlen(calCommands[i]);
    if ((len <= cmdLen) && ((cmdStr[len] == ':') || (cmdStr[len] == ',') || (cmdStr[len] == '\r') || (cmdStr[len] == '\n') || (cmdStr[len] == '\0')) && this->comComp(cmdStr, calCommands[i], len))
    {
      cmd = CalCmd(i);
      cmdStr += len;
      break;
    }
  }

  if (cmd == CalCmd::size)
  {
    SERIAL.println(" Unknown CAL command!");
    return;
  }

  if (*cmdStr != ':')
  {
    // view calibration
    this->modifyCal(cmd, true);
    return;
  }
  ++cmdStr;

  // determine arguments
  char * remain;
  errno = 0;
  float newValue = std::strtod(cmdStr, &remain);

  if (((*remain != '\0') && (*remain != '.') && (*remain != '\r') && (*remain != '\n')) || errno)
  {
    SERIAL.println(" Incorrect argument format!");
    return;
  }

  // handle different commands
  this->modifyCal(cmd, false, newValue);
}
void LCRState::communicationLoop()
{
	static char buf[SERIAL_READ_BUFSIZE];
	static std::uint8_t bufidx = 0;

	// Try to receive data
	const auto read = this->comRead(buf, bufidx);
	if (!read || !this->comComp(buf, "LCR,"))
	{
		return;
	}

  const char * cmd = buf + 4;
	
	// parse contents
	if (this->comComp(cmd, "echo"))
	{
		SERIAL.println("Echo");
	}
	else if (this->comComp(cmd, "debug"))
	{
		SERIAL.println("Toggling debug mode...");
		this->debug ^= 1;
	}
	else if (this->comComp(cmd, "temp"))
	{
		this->printInfo(Info::TempAcc);
	}
	else if (this->comComp(cmd, "ref"))
	{
		this->printInfo(Info::Ref);
	}
	else if (this->comComp(cmd, "supply"))
	{
		this->printInfo(Info::Supply);
	}
  else if (this->comComp(cmd, "cal,"))
  {
    this->manualCal(cmd + 4);
  }
  else if (this->comComp(cmd, "freq"))
  {
    SERIAL.print("Frequnecy: ");
    SERIAL.print(this->dacFrequency);
    SERIAL.println(" Hz");
  }
	else
	{
		SERIAL.println("Unknown command!");
	}
}


void LCRState::printInfo(Info type, std::uint8_t decPlaces, bool hasData, float uData) const
{
	static const char * prelist[] = {
		"Temperature: ",
		"Temperature (slow): ",
		"Reference: ",
		"Supply: ",
		"Gain 0.5x: ",
		"Gain 2x: ",
	};
	static const char * postlist[] = {
		" deg. C",
		" deg. C",
		"V",
		"V",
		"",
		"",
	};

	const auto ptype = std::underlying_type<Info>::type(type);
	SERIAL.print(prelist[ptype]);

	float data;
	std::uint16_t sample = 0;
	if (!hasData)
	{
		switch (type)
		{
		case Info::Temp:
		case Info::TempAcc:
			sample = adc::sample(adc::Channel::IntTemp, type == Info::TempAcc);
			data = adc::getTemp(sample);
			decPlaces = !decPlaces ? 2 : decPlaces;
			break;
		case Info::Ref:
			data = adc::calData.ref1VReal;
			decPlaces = !decPlaces ? 6 : decPlaces;
			break;
		case Info::Supply:
			data = adc::getSupply(true);
			decPlaces = !decPlaces ? 4 : decPlaces;
			break;
		case Info::Gain_0x5:
			data = adc::calData.gainCal[std::uint8_t(adc::Gain::g0x5)];
			decPlaces = !decPlaces ? 4 : decPlaces;
			break;
		case Info::Gain_2x:
			data = adc::calData.gainCal[std::uint8_t(adc::Gain::g2x)];
			decPlaces = !decPlaces ? 4 : decPlaces;
			break;
		default:
			assert(!"Unknown type!");
		}
	}
	else
	{
		data = uData;
	}

	SERIAL.print(data, decPlaces);
	SERIAL.println(postlist[ptype]);
}

void LCRState::fetchButtons()
{
  constexpr std::uint32_t tickSize = LCR_UPDATE_BTN_MS;

  static std::uint32_t prevticks = 0;
  static std::uint16_t ticks = 0;

  // read buttons ADC value
  adc::setGain(adc::Gain::g1x);
  adc::sampleAsync(adc::Channel::Btn, false);
  auto btnSample = (adc::adcReady == std::uint8_t(adc::Channel::Btn)) ? adc::adcResArr[std::uint8_t(adc::Channel::Btn)] : 0;
  if (!btnSample)
  {
    return;
  }
  auto volts = adc::getVolts(btnSample);

  // calculate impedance of buttons
  // volts = imp / (imp + ADC_BTN_IMP_R) * vrest
  auto imp = volts / ADC_BTN_VREST_F(adc::calData.supplyVoltage);
  
  ticksabs += tickSize;
  ticks += (ticks < (65536U - tickSize)) ? tickSize : 0;
  if (imp >= 1.0f)
  {
    ticks = 0;
    this->btn1State = 0;
    this->btn2State = 0;
    this->btn3State = 0;
    this->btn4State = 0;
    this->btn5State = 0;
    return;
  }

  imp = (float(ADC_BTN_IMP_R) * imp) / (1.0f - imp);
  state.btnOhms = imp;

  // deal with button presses
  const auto isButton = [&imp](float buttonImp, float tolerance) -> bool
  {
    const auto tolmin = buttonImp * (1.0f - tolerance), tolmax = buttonImp * (1.0f + tolerance);
    return (imp >= tolmin) && (imp <= tolmax);
  };
  std::uint8_t scan = 1;
  const auto checkButton = [&](bool oldbtnState, float btnRes, std::uint8_t btnIdx) -> bool
  {
    static std::uint8_t lastButtonPressed = 0, debounceButton = 0;

    if (!btnIdx)
    {
      if (lastButtonPressed && scan)
      {
        lastButtonPressed = 0;
        debounceButton = 0;
      }
      return false;
    }
    if (!scan)
    {
      return false;
    }

    /*if (!ticks)
    {
      return 0;
    }*/
    const auto reading = isButton(btnRes, ADC_BTN_TOL);
    auto newState = oldbtnState;

    if (reading)
    {
      if (lastButtonPressed != btnIdx)
      {
        newState = false;
        if (debounceButton != btnIdx)
        {
          ticks = 0;
          debounceButton = btnIdx;
        }
        if (ticks >= BTN_DEBOUNCE_THRESHOLD_MS)
        {
          lastButtonPressed = btnIdx;
          //prevticks = ticksabs;
          newState = true;
        }
      }
      /*else if ((ticksabs - prevticks) > BTN_DEBOUNCE_THRESHOLD_MS)
      {
        prevticks = ticksabs;
        newState = true;
      }*/
      // only the first button that matches, gets triggered
      scan = 0;
    }
    else
    {
      return false;
    }

    return newState;
  };

  this->btn1State = checkButton(this->btn1State, ADC_BTN1_R, 1);
  this->btn2State = checkButton(this->btn2State, ADC_BTN2_R, 2);
  this->btn3State = checkButton(this->btn3State, ADC_BTN3_R, 3);
  this->btn4State = checkButton(this->btn4State, ADC_BTN4_R, 4);
  this->btn5State = checkButton(this->btn5State, ADC_BTN5_R, 5);
  // final pass to reset checkButton function state
  checkButton(0, 0, 0);


  /*if (ticks <= BTN_DEBOUNCE_THRESHOLD_MS)
  {
    return;
  }

  imp = (float(ADC_BTN_IMP_R) * imp) / (1.0f - imp);
  state.btnVolts = imp;

  // deal with button presses
  const auto isButton = [&imp](float buttonImp, float tolerance) -> bool
  {
    const auto tolmin = buttonImp * (1.0f - tolerance), tolmax = buttonImp * (1.0f + tolerance);
    return (imp >= tolmin) && (imp <= tolmax);
  };
  const auto checkButton = [&](bool oldbtnState, float btnRes) -> bool
  {
    if (!ticks)
    {
      return 0;
    }
    auto newState = isButton(btnRes, ADC_BTN_TOL);
    if (newState != oldbtnState)
    {
      ticks = 0;
    }
    return newState;
  };

  this->btn1State = checkButton(this->btn1State, ADC_BTN1_R);
  this->btn2State = checkButton(this->btn2State, ADC_BTN2_R);
  this->btn3State = checkButton(this->btn3State, ADC_BTN3_R);
  this->btn4State = checkButton(this->btn4State, ADC_BTN4_R);
  this->btn5State = checkButton(this->btn5State, ADC_BTN5_R);*/

}
void LCRState::printButtons(const char * stockStatus, bool printTemp, float temp) const
{
  constexpr const char * strs[] =
  {
    "O                ",
    "O        ",
    "O",
    "         O",
    "                 O"
  };

  std::uint8_t idx = state.btn1State;
  idx += 2 * state.btn2State;
  idx += 3 * state.btn3State;
  idx += 4 * state.btn4State;
  idx += 5 * state.btn5State;
  idx = (idx > 5) ? 0 : idx;

  disp::screenStatus(idx ? strs[idx - 1] : stockStatus, printTemp, temp);
}
void LCRState::printInfo(const char * title, float voltage, float adcgain, float gain, bool isAtten) const
{
  if (!this->debug)
  {
    return;
  }

  if (isAtten)
  {
    gain *= lcr::volts::calData.hvatten_gain;
  }
  SERIAL.print("[");
  SERIAL.print(title);
  SERIAL.print("] ");
  SERIAL.print(voltage, 4);
  SERIAL.print(" V; gadc: ");
  SERIAL.print(adcgain, 4);
  SERIAL.print("; g: ");
  SERIAL.print(gain, 4);
  SERIAL.print("; atten: ");
  SERIAL.print(isAtten ? "true" : "false");
  SERIAL.println();
}

void LCRState::genCos(std::uint16_t numSamples, float factor)
{
  float fsize = float(numSamples);
  for (std::uint16_t i = 0; i < numSamples; ++i)
  {
    float arg = (factor * DAC_FACTOR * cosf(2.0f * float(M_PI) * float(i) / fsize) + 1.0f) / 2.0f;
    this->ramDacSamples[i] = std::uint16_t(arg * 1023.0f + 0.5f);
  }
}
void LCRState::genOut(std::uint32_t frequency)
{
  SERIAL.println("defmode");
  delay(50);
  dac::enable(1, false);
  analogWrite(DAC_OUT, 512);  // resting position at half supply
  if (frequency <= (DAC_MAX_SAMPLERATE / DAC_MAX_FLASH_SAMPLES))
  {
    dac::setbuf(this->flashDacSamples, DAC_MAX_FLASH_SAMPLES);
  }
  else
  {
    std::uint32_t numSamples = DAC_MAX_SAMPLERATE / frequency;
    numSamples = (numSamples > DAC_MAX_RAM_SAMPLES) ? DAC_MAX_RAM_SAMPLES : numSamples;
    // make samples odd
    numSamples |= 1;

    // generate sinewave table
    this->genCos(numSamples, frequency > 90000 ? 0.89f : 1.0f);
    dac::setbuf(this->ramDacSamples, numSamples);
  }
  SERIAL.println("defmode-enable");
  delay(50);
  this->dacFrequency = dac::enable(frequency, true);
  // calculate peak coefficient
  this->calcPeakCoef();
  // calculate parasitics data
  lcr::imp::calData.calcParasiticImp(this->dacFrequency);
}
void LCRState::calcPeakCoef()
{
  if (this->dacSelPreset == -1)
  {
    this->peakCoefficient = 1.0f;
  }
  else
  {
    this->peakCoefficient = this->dacPeakCoefficients[this->dacSelPreset];
  }
}

void LCRState::defMode() const
{
  // initialize all relevant gpios in default mode

  digitalWrite(LCR_ENABLE, 0);
  digitalWrite(LCR_RANGE3, 1);
  digitalWrite(LCR_RANGE2, 1);
  digitalWrite(LCR_RANGE1, 0);
  digitalWrite(VOLTS_RANGE1, 0);
  digitalWrite(VOLTS_RANGE2, 0);
  digitalWrite(OHMS_22X, 0);
  digitalWrite(OHMS_202X, 0);
  digitalWrite(OHMS_INVERT, 0);
  pwm::duty0(OHMS_CC, 0.0f);
  digitalWrite(OHMS_CUR_RANGE, 1);

  SERIAL.println("defmode");
  delay(50);
  dac::enable(1, false);
  analogWrite(DAC_OUT, 512);

  disp::screenPrompt("");
  disp::screenPrompt2("");
}
bool LCRState::setMode(Mode newmode, bool force)
{
  if ((newmode == this->mode) && !force)
  {
    return true;
  }
  SERIAL.println("Change mode!");
  
  if (!this->btnModeState)
  {
    mode = Mode::Shunt;
  }
  else
  {
    mode = (mode == Mode::Shunt) ? Mode::Volts : mode;
  }

  bool status = false;

  // check if it's safe to change mode
  if (lcr::canChangeMode())
  {
    // go back to default mode
    SERIAL.println("Default mode!");
    defMode();

    // change mode to new mode
    switch (newmode)
    {
    case Mode::LCR:
      status = lcr::imp::mode();
      this->genOut(this->dacFrequencyPresets[std::uint8_t(LCR_DEFAULT_FREQUENCY_PRE)]);
      break;
    case Mode::Shunt:
      status = lcr::ohms::mode();
      break;
    case Mode::Volts:
      status = lcr::volts::mode();
      break;
    case Mode::Thermo:
      status = lcr::volts::modeThermo();
      break;
    case Mode::Failure:
      status = true;
      break;
    }
  }

  if (status)
  {
    this->mode = newmode;
  }
  else
  {
    this->mode = Mode::Failure;
  }
  // else cannot go into mode, disconnect everything!
  constexpr const char * titles[std::uint8_t(Mode::size)] = {
    "LCR",
    "Shunt",
    "Volts",
    "TC-K",
    "Fail!"
  };
  if (this->mode != Mode::LCR)
  {
    disp::screenTitle(titles[std::uint8_t(this->mode)]);
  }

  this->firstMeas = true;
  return status;
}

int SerialPrintf(const char * format, ...)
{
	va_list vp;
	char buf[SERIAL_PRINTF_BUFSIZE];
	buf[SERIAL_PRINTF_BUFSIZE - 1] = '\0';

	va_start(vp, format);
	auto nchars = std::vsnprintf(buf, SERIAL_PRINTF_BUFSIZE, format, vp);
	va_end(vp);

	SERIAL.print(buf);
	return nchars;
}

void modeBtnISR()
{
	static long lastmillis = 0;
	const bool state_ = digitalRead(VOLTS_MODE_IN);
	const auto mil = millis(), delta = mil - lastmillis;
	lastmillis = mil;

	if (!state_ && (delta < BTN_DEBOUNCE_THRESHOLD_MS))
	{
		return;
	}

	const bool val = state.btnModeState;
	state.btnModeState = !state_;
	state.btnModeIntOccur = (state.btnModeState != val);
}

void TC5_Handler()
{
  static std::uint32_t ticksBtns = 1, ticksLogic = 1, ticksDisplay = 1;
  // "Game" loop
  state.update = 1;
  ++ticksLogic;
  ++ticksDisplay;
  ++ticksBtns;

  if (ticksLogic > state.updateLogicTicks)
  {
    state.updateLogic = 1;
    ticksLogic = 1;
  }
  if (ticksDisplay > (LCR_UPDATE_DISPLAY_MS * LCR_UPDATE_MS_TICKS))
  {
    state.updateDisp = 1;
    ticksDisplay = 1;
  }
  // run @100 Hz, every 10 ms
  if (ticksBtns > (LCR_UPDATE_BTN_MS * LCR_UPDATE_MS_TICKS))
  {
    state.fetchButtons();
    ticksBtns = 1;
  }

  // clear overflow flag
  TC5->COUNT16.INTFLAG.bit.MC0 = 1;
}
