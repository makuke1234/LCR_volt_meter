#include "display.hpp"
#include "lcr.hpp"
#include "algo.hpp"

std::uint8_t disp::customCharOmega[LCD_CHAR_HEIGHT] = {
  0b00000,
  0b01110,
  0b10001,
  0b10001,
  0b10001,
  0b01010,
  0b11011,
  0b00000
};
std::uint8_t disp::customCharMicro[LCD_CHAR_HEIGHT] = {
  0b00000,
  0b10001,
  0b10001,
  0b10011,
  0b11101,
  0b10000,
  0b10000,
  0b10000
};
std::uint8_t disp::customCharDegree[LCD_CHAR_HEIGHT] = {
  0b11100,
  0b10100,
  0b11100,
  0b00000,
  0b00000,
  0b00000,
  0b00000,
  0b00000
};

disp::LCDData disp::data;

static char s_lineBuf[LCD_WIDTH + 1], s_lineBuf2[LCD_WIDTH + 1];

static void s_drawNumber(
  char unit, std::uint8_t oneIdx, std::uint8_t sz,
  const uint8_t* units, const uint8_t* decPoints, const float* thres,
  float& pnum, std::uint8_t& pmultiple,
  float number, bool range, float maxValue = 0.0f, bool drawSign = true
);
static void s_drawNumberPos(
  char unit, std::uint8_t oneIdx, std::uint8_t sz,
  const std::uint8_t * units, const std::uint8_t * decPoints, const float * thres,
  float & pnum, std::uint8_t & pmultiple,
  float number, bool range, float maxValue = 0.0f
);
static void s_drawVolts(float& pv, std::uint8_t& pmult, float volts, bool range);
static void s_drawOhms(float& pohms, std::uint8_t& pmult, float ohms);
static void s_drawCap(float& pcap, std::uint8_t& pmult, float cap);
static void s_drawInd(float& pind, std::uint8_t& pmult, float ind);

static void s_drawCentered(const char* str, std::int32_t slen = -1, std::uint8_t line = 0, bool draw = true);

void disp::init()
{
  disp::data.lcd.begin(LCD_WIDTH, LCD_HEIGHT);

  // init custom characters (if any)
  //LCD_OMEGA_SYMBOL, customCharOmega
  //disp::data.lcd.createChar(LCD_OMEGA_SYMBOL, disp::customCharOmega);
  disp::data.lcd.createChar(LCD_MICRO_SYMBOL, disp::customCharMicro);
  disp::data.lcd.createChar(LCD_DEG_SYMBOL,   disp::customCharDegree);
}

void disp::bootlogo(std::uint32_t timeoutms)
{
  // clear display just in case
  disp::data.lcd.clear();

  // draw bootlogo
  s_drawCentered("LCR meter", -1, 1);
  s_drawCentered("firmware v1.0", -1, 2);

  // wait for it
  delay(timeoutms);
  // clear boot logo
  disp::data.lcd.clear();
}


static void s_drawNumber(
  char unit, std::uint8_t oneIdx, std::uint8_t sz,
  const std::uint8_t * units, const std::uint8_t * decPoints, const float * thres,
  float & pnum, std::uint8_t & pmultiple,
  float number, bool range, float maxValue, bool drawSign
)
{
  if (isnanf(number) || ((maxValue != 0.0f) && (abs(number) > maxValue)) )
  {
    snprintf(
      s_lineBuf2, LCD_WIDTH + 1,
      "%*s   ",
      8, "OL"
    );
    return;
  }

  const auto change = (fabsf(number) < (pnum * 0.5f)) || (fabsf(number) > (pnum * 2.0f));

  std::uint8_t mult = pmultiple;
  if (change)
  {
    // determine new range
    mult = 0;
    for (std::uint8_t i = 0, end = sz - 1; i < end; ++i)
    {
      mult += (fabsf(number) > thres[i]);
    }
  }
  number = (mult != oneIdx) ? (number * algo::powif(1.0f, 0.1f, 3 * (std::int8_t(mult) - oneIdx))) : number;

  std::int32_t full;
  std::uint32_t part;
  if (!drawSign && number < 0.0f)
  {
    full = 0;
    part = 0;
  }
  else
  {
    algo::floatFrac(number, full, part, decPoints[mult] - range);
  }
  
  char numBuf[13];
  if (drawSign)
  {
    snprintf(
      numBuf, 13,
      "%c%u",
      (number < 0.0f) ? '-' : '+', std::labs(full)
    );
  }
  else
  {
    snprintf(
      numBuf, 13,
      "%u", std::labs(full)
    );
  }
  snprintf(
    s_lineBuf2, LCD_WIDTH + 1,
    "%*s.%.*u %c%c",
    
    8 - (decPoints[mult] - range) - 1, numBuf,
    decPoints[mult] - range, part,
    units[mult], unit
  );

  pmultiple = mult;
}
static void s_drawNumberPos(
  char unit, std::uint8_t oneIdx, std::uint8_t sz,
  const std::uint8_t * units, const std::uint8_t * decPoints, const float * thres,
  float & pnum, std::uint8_t & pmultiple,
  float number, bool range, float maxValue
)
{
  s_drawNumber(unit, oneIdx, sz, units, decPoints, thres, pnum, pmultiple, number, range, maxValue, false);
}

static void s_drawVolts(float & pv, std::uint8_t & pmult, float volts, bool range)
{
  static const std::uint8_t units[] = { LCD_MICRO_SYMBOL, 'm', ' ', 'k' }, dec[] = { 2, 3, 4, 4 };
  static const float thres[] = { 770e-6f, 28e-3f, 9999.0f };

  s_drawNumber('V', 2, 4, units, dec, thres, pv, pmult, volts, range);
}
static void s_drawOhms(float & pohms, std::uint8_t & pmult, float ohms)
{
  static const std::uint8_t units[] = { LCD_MICRO_SYMBOL, 'm', ' ', 'k', 'M' }, dec[] = { 1, 3, 3, 3, 3 };
  static const float thres[] = { 0e-6f, 350e-3f, 999.0f, 999e3f };

  s_drawNumberPos(LCD_OMEGA_SYMBOL, 2, 5, units, dec, thres, pohms, pmult, ohms, false);
}
static void s_drawCap(float & pcap, std::uint8_t & pmult, float cap)
{
  static const std::uint8_t units[] = { 'p', 'n', LCD_MICRO_SYMBOL, 'm', ' ' }, dec[] = { 2, 3, 3, 3, 3 };
  static const float thres[] = { 999e-12f, 999e-9f, 999e-6f, 999e-3f };

  s_drawNumberPos('F', 4, 5, units, dec, thres, pcap, pmult, cap, false, LCR_MAX_CAPACITANCE);
}
static void s_drawInd(float & pind, std::uint8_t & pmult, float ind)
{
  static const std::uint8_t units[] = { 'n', LCD_MICRO_SYMBOL, 'm', ' ' }, dec[] = { 2, 3, 3, 3 };
  static const float thres[] = { 999e-9f, 999e-6f, 999e-3f };

  s_drawNumberPos('H', 3, 4, units, dec, thres, pind, pmult, ind, false, LCR_MAX_INDUCTANCE);
}

void disp::screenVolts(float volts, bool range)
{
  static float pv = 0.0f;
  static std::uint8_t pmult = 2;

  s_drawVolts(pv, pmult, volts, range);

  // draw on the second and third line with padded spaces
  s_drawCentered(s_lineBuf2, -1, 1);

  pv = fabsf(volts);
}
void disp::screenThermo(float volts, float temp)
{
  static float pv1 = 0.0f;
  static std::uint8_t pmult1 = 1;

  s_drawVolts(pv1, pmult1, volts, false);
  s_drawCentered(s_lineBuf2, -1, 1);

  pv1 = fabsf(volts);

  // draw temperature in celsius, 4 digits
  std::int32_t full;
  std::uint32_t part;
  const std::uint8_t numDigits = (temp < 1000.0f) + (temp < 100.0f);
  algo::floatFrac(temp, full, part, numDigits);

  if (numDigits)
  {
    snprintf(s_lineBuf2, LCD_WIDTH + 1, " %+*d.%.*u %cC", 6 - numDigits, full, numDigits, part, LCD_DEG_SYMBOL);
  }
  else
  {
    snprintf(s_lineBuf2, LCD_WIDTH + 1, " %+*d %cC", 7 - numDigits, full, LCD_DEG_SYMBOL);
  }
  s_drawCentered(s_lineBuf2, -1, 2);
}
void disp::screenOhms(float ohms, bool isshunt2)
{
  static float po = 0.0f;
  static std::uint8_t pmult = 0;

  if (isnan(ohms) || isinf(ohms) || (fabsf(ohms) > (isshunt2 ? OHMS_MAX_RESISTANCE_SHUNT2 : OHMS_MAX_RESISTANCE_SHUNT1)))
  {
    strcpy(s_lineBuf2, "OL");
  }
  else
  {
    s_drawOhms(po, pmult, ohms);
  }
  s_drawCentered(s_lineBuf2, -1, 1);

  po = fabsf(ohms);
}
void disp::screenLCR(
  float res1, float reac1, float imp1,
  float res2, float reac2, float imp2,
  float frequency, bool isCapacitor, bool isInductor
)
{
  float resistance, reactance, impedance;
  {
    // determine which set of values to use
    bool useSet1 = true;

    if (reac1 < abs(res1) || reac2 < abs(res2))
    {
      if (res2 < res1)
      {
        useSet1 = false;
      }
    }

    if (useSet1)
    {
      resistance = res1;
      reactance  = reac1;
      impedance  = imp1;
    }
    else
    {
      resistance = res2;
      reactance  = reac2;
      impedance  = imp2;
    }
  }
  
  static float pEsr = 0.0f, pReac = 0.0f, pCap = 0.0f, pInd = 0.0f;
  static std::uint8_t pmEsr = 0, pmReac = 0, pmCap = 0, pmInd = 0;

  // helper variables to print floats
  std::int32_t full;
  std::uint32_t part;

  if (isCapacitor)
  {
    // ESR + capacitance
    // dissipation factor + phase angle

    s_drawOhms(pEsr, pmEsr, res1);
    snprintf(
      s_lineBuf, LCD_WIDTH + 1,
      "ESR %s",
      s_lineBuf2
    );
    s_drawCap(pCap, pmCap, lcr::imp::calcCap(reactance, frequency));

    std::uint8_t len = std::uint8_t(strlen(s_lineBuf));
    memset(s_lineBuf + len, ' ', LCD_WIDTH - len);
    snprintf(
      s_lineBuf + (LCD_WIDTH / 2), LCD_WIDTH + 1 - LCD_WIDTH / 2,
      "C %s",
      s_lineBuf2
    );

    disp::data.lcd.setCursor(0, 1);
    disp::data.lcd.print(s_lineBuf);

    algo::floatFrac(lcr::imp::calcDissipation(resistance, reactance), full, part, 3);

    snprintf(
      s_lineBuf, LCD_WIDTH + 1,
      "D %d.%03u",
      full, part
    );
    algo::floatFrac(lcr::imp::calcPhase(false, resistance, reactance), full, part, 2);

    len = std::uint8_t(strlen(s_lineBuf));
    memset(s_lineBuf + len, ' ', LCD_WIDTH - len);
    snprintf(
      s_lineBuf + (LCD_WIDTH / 2), LCD_WIDTH + 1 - LCD_WIDTH / 2,
      "Ph %d.%02u%c",
      full, part,
      LCD_DEG_SYMBOL
    );

    disp::data.lcd.setCursor(0, 2);
    disp::data.lcd.print(s_lineBuf);

    return;
  }
  else if (isInductor)
  {
    // esr + inductance
    // quality factor + phase angle

    s_drawOhms(pEsr, pmEsr, res2);
    snprintf(
      s_lineBuf, LCD_WIDTH + 1,
      "ESR %s",
      s_lineBuf2
    );
    s_drawInd(pInd, pmInd, lcr::imp::calcInd(reactance, frequency));

    std::uint8_t len = std::uint8_t(strlen(s_lineBuf));
    memset(s_lineBuf + len, ' ', LCD_WIDTH - len);
    snprintf(
      s_lineBuf + (LCD_WIDTH / 2), LCD_WIDTH + 1 - LCD_WIDTH / 2,
      "L %s",
      s_lineBuf2
    );

    disp::data.lcd.setCursor(0, 1);
    disp::data.lcd.print(s_lineBuf);

    algo::floatFrac(lcr::imp::calcQ(resistance, reactance), full, part, 3);
    snprintf(
      s_lineBuf, LCD_WIDTH + 1,
      "Q %d.%03u",
      full, part
    );
    algo::floatFrac(lcr::imp::calcPhase(true, resistance, reactance), full, part, 2);

    len = std::uint8_t(strlen(s_lineBuf));
    memset(s_lineBuf + len, ' ', LCD_WIDTH - len);
    snprintf(
      s_lineBuf + (LCD_WIDTH / 2), LCD_WIDTH + 1 - LCD_WIDTH / 2,
      "Ph %d.%02u%c",
      full, part,
      LCD_DEG_SYMBOL
    );

    disp::data.lcd.setCursor(0, 2);
    disp::data.lcd.print(s_lineBuf);

    return;
  }

  // normally esr + reactance
  // capacitance + inductance
  s_drawOhms(pEsr, pmEsr, resistance);
  snprintf(
    s_lineBuf, LCD_WIDTH + 1,
    "R %s",
    s_lineBuf2
  );
  s_drawOhms(pReac, pmReac, reactance);

  /*std::uint8_t len = std::uint8_t(strlen(s_lineBuf));
  memset(s_lineBuf + len, ' ', LCD_WIDTH - len);
  snprintf(
    s_lineBuf + (LCD_WIDTH / 2), LCD_WIDTH + 1 - LCD_WIDTH / 2,
    "X %s",
    s_lineBuf2
  );*/

  disp::data.lcd.setCursor(0, 0);
  disp::data.lcd.print(s_lineBuf);

  s_drawCap(pCap, pmCap, lcr::imp::calcCap(reactance, frequency));
  snprintf(
    s_lineBuf, LCD_WIDTH + 1,
    "C %s",
    s_lineBuf2
  );
  disp::data.lcd.setCursor(0, 1);
  disp::data.lcd.print(s_lineBuf);

  s_drawInd(pInd, pmInd, lcr::imp::calcInd(reactance, frequency));

  std::uint8_t len = std::uint8_t(strlen(s_lineBuf));
  memset(s_lineBuf + len, ' ', LCD_WIDTH - len);
  snprintf(
    s_lineBuf, LCD_WIDTH + 1,
    "L %s",
    s_lineBuf2
  );

  disp::data.lcd.setCursor(0, 2);
  disp::data.lcd.print(s_lineBuf);
}
void disp::screenFail()
{
  disp::screenTitle("Failure!");
  disp::screenPrompt("Please disconnect everything");
}

void disp::screenStatusTemp(float temp)
{
  // show temperature on the last line in the right corner, 1 decimal place
  std::int32_t full;
  std::uint32_t partial;
  algo::floatFrac(temp, full, partial, 1);

  snprintf(s_lineBuf2, LCD_WIDTH + 1, "%+d.%01u%cC", full, partial, LCD_DEG_SYMBOL);
  strcpy(s_lineBuf + LCD_WIDTH - std::strlen(s_lineBuf2), s_lineBuf2);
  disp::data.lcd.setCursor(0, 3);
  disp::data.lcd.print(s_lineBuf);
}
void disp::screenStatusOkCancel()
{
  const char *str1 = "Cancel", *str2 = "OK";
  const std::uint8_t slen1 = std::strlen(str1), slen2 = std::strlen(str2);

  std::uint8_t lenLeft = slen1 + 1 + slen2 / 2, lenRight = slen2 - slen2 / 2;

  lenLeft = LCD_WIDTH / 2 - lenLeft;
  lenRight = (LCD_WIDTH - LCD_WIDTH / 2) - lenRight;

  snprintf(s_lineBuf, LCD_WIDTH + 1, "%*c%s %s%*c", lenLeft, ' ', str1, str2, lenRight, ' ');
  // draw text on last line
  disp::data.lcd.setCursor(0, 3);
  disp::data.lcd.print(s_lineBuf);
}

static void s_drawCentered(const char * str, std::int32_t slen, std::uint8_t line, bool draw)
{
  assert(str != s_lineBuf);

  // draw text on first line centered
  std::int32_t len;
  if (slen > 0)
  {
    len = snprintf(s_lineBuf, LCD_WIDTH + 1, "%.*s", slen, str);
  }
  else
  {
    len = snprintf(s_lineBuf, LCD_WIDTH + 1, "%s", str);
  }
  len = (len > LCD_WIDTH) ? LCD_WIDTH : len;
  auto space = (LCD_WIDTH - len) / 2;
  memmove(s_lineBuf + space, s_lineBuf, len);
  memset(s_lineBuf, ' ', space);
  memset(s_lineBuf + space + len, ' ', LCD_WIDTH - space - len);
  s_lineBuf[LCD_WIDTH] = '\0';

  if (draw)
  {
    disp::data.lcd.setCursor(0, line);
    disp::data.lcd.print(s_lineBuf);
  }
}
void disp::screenTitle(const char * title)
{
  // draw text on first line centered
  s_drawCentered(title, -1, 0);
}
void disp::screenPrompt(const char* prompt)
{
  // split text between second & third line, if it doesn't fit to second line
  std::uint8_t spaceIdx = 0;
  for (std::uint8_t i = 0; (prompt[i] != '\0') && (i < LCD_WIDTH); ++i)
  {
    if (prompt[i] == ' ')
    {
      spaceIdx = i;
    }
  }
  bool multi = false;
  for (std::uint8_t i = spaceIdx; (prompt[i] != '\0') && (i < (LCD_WIDTH + 1)); ++i)
  {
    if (i > (LCD_WIDTH - 1))
    {
      multi = true;
      break;
    }
  }

  if (multi)
  {
    s_drawCentered(prompt, spaceIdx, 1);
    s_drawCentered(prompt + spaceIdx + 1, -1, 2);
  }
  else
  {
    s_drawCentered(prompt, -1, 1);
  }
}
void disp::screenPrompt2(const char * prompt)
{
  s_drawCentered(prompt, -1, 2);
}
void disp::screenStatus(const char * text, bool printTemp, float temp)
{
  // draw text on last line centered
  s_drawCentered(text, -1, 3, !printTemp);
  if (printTemp)
  {
    disp::screenStatusTemp(temp);
  }
}

