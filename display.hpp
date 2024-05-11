#pragma once

#include "main.hpp"
#include <LiquidCrystal.h>

#define LCD_DEG_SYMBOL    (char(2)) // 11011111
#define LCD_MICRO_SYMBOL  (char(1)) // 11100100
#define LCD_OMEGA_SYMBOL  (char(244)) // 11110100
#define LCD_CHAR_HEIGHT   8

namespace disp
{
  extern std::uint8_t customCharOmega[LCD_CHAR_HEIGHT];
  extern std::uint8_t customCharMicro[LCD_CHAR_HEIGHT];
  extern std::uint8_t customCharDegree[LCD_CHAR_HEIGHT];


  struct LCDData
  {
    LiquidCrystal lcd = LiquidCrystal(LCD_RS, LCD_RW, LCD_EN, LCD_D4, LCD_D5, LCD_D6, LCD_D7);

    std::uint32_t timer = 0;
  };
  extern LCDData data;

	void init();

  // interface functions
  void bootlogo(std::uint32_t timeoutms);

  void screenVolts(float volts, bool range);
  void screenThermo(float volts, float temp);
  void screenOhms(float ohms, bool isshunt2);
  void screenLCR(
    float res1, float reac1, float imp1,
    float res2, float reac2, float imp2,
    float frequency, bool isCapacitor, bool isInductor
  );
  void screenFail();

  void screenStatusTemp(float temp);
  void screenStatusOkCancel();

  void screenTitle(const char * title = "");
  void screenPrompt(const char * prompt);
  void screenPrompt2(const char * prompt);
  void screenStatus(const char * text = "", bool printTemp = false, float temp = 0.0f);

  // menus

}
