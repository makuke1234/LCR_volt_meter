#include "algo.hpp"

float algo::powf(float num, float multi, std::uint8_t power)
{
  for (std::uint8_t i = 0; i < power; ++i)
  {
    num *= multi;
  }
  return num;
}
float algo::powif(float num, float multi, std::int8_t power)
{
  multi = (power < 0) ? 1.0f / multi : multi;
  power = (power < 0) ? -power : power;
  for (std::uint8_t i = 0; i < power; ++i)
  {
    num *= multi;
  }
  return num;
}

std::int32_t algo::floatFull(float f, std::uint8_t decimals)
{
  auto frac = algo::powf((f < 0.0f) ? -0.5 : 0.5f, 0.1f, decimals);

  return std::int32_t(f + frac);
}
std::uint32_t algo::floatPartial(float f, std::int32_t full, std::uint8_t decimals)
{
  auto frac = algo::powf((f < float(full)) ? -1.0 : 1.0f, 10.0f, decimals);

  return std::labs(std::int32_t((f - float(full)) * frac + 0.5f));
}
void algo::floatFrac(float f, std::int32_t & full, std::uint32_t & partial, std::uint8_t decimals)
{
  full    = algo::floatFull(f, decimals);
  partial = algo::floatPartial(f, full, decimals);
}

namespace mh
{
  static char __mathHelperBuffer[17];
}

//////////////////////////////////////////////////
//
// FLOAT REPRESENTATION HELPERS
//
const char * mh::sci(double number, int digits)
{
  int exponent = 0;
  int pos = 0;

  // Handling these costs 13 bytes RAM
  // shorten them with N, I, -I ?
  if (std::isnan(number)) 
  {
    std::strcpy(mh::__mathHelperBuffer, "nan");
    return mh::__mathHelperBuffer;
  }
  if (std::isinf(number))
  {
    if (number < 0)
    {
      std::strcpy(mh::__mathHelperBuffer, "-inf");
    }
    std::strcpy(mh::__mathHelperBuffer, "inf");
    return mh::__mathHelperBuffer;
  }

  // Handle negative numbers
  bool neg = (number < 0.0);
  if (neg)
  {
    mh::__mathHelperBuffer[pos++] = '-';
    number = -number;
  }

  while (number >= 10.0)
  {
    number /= 10;
    exponent++;
  }
  while (number < 1 && number != 0.0)
  {
    number *= 10;
    exponent--;
  }

  // Round correctly so that print(1.999, 2) prints as "2.00"
  double rounding = 0.5;
  for (uint8_t i = 0; i < digits; ++i)
  {
    rounding *= 0.1;
  }
  number += rounding;
  if (number >= 10)
  {
    exponent++;
    number /= 10;
  }


  // Extract the integer part of the number and print it
  uint8_t d = (uint8_t)number;
  double remainder = number - d;
  mh::__mathHelperBuffer[pos++] = d + '0';   // 1 digit before decimal point
  if (digits > 0)
  {
    mh::__mathHelperBuffer[pos++] = '.';  // decimal point TODO:rvdt CONFIG?
  }


  // Extract digits from the remainder one at a time to prevent missing leading zero's
  while (digits-- > 0)
  {
    remainder *= 10.0;
    d = (uint8_t)remainder;
    mh::__mathHelperBuffer[pos++] = d + '0';
    remainder -= d;
  }


  // print exponent
  mh::__mathHelperBuffer[pos++] = 'E';
  neg = exponent < 0;
  if (neg)
  {
    mh::__mathHelperBuffer[pos++] = '-';
    exponent = -exponent;
  }
  else
  {
    mh::__mathHelperBuffer[pos++] = '+';
  }


  // 3 digits for exponent;           // needed for double
  // d = exponent / 100;
  // __mathHelperBuffer[pos++] = d + '0';
  // exponent -= d * 100;

  // 2 digits for exponent
  d = exponent / 10;
  mh::__mathHelperBuffer[pos++] = d + '0';
  d = exponent - d*10;
  mh::__mathHelperBuffer[pos++] = d + '0';

  mh::__mathHelperBuffer[pos] = '\0';

  return mh::__mathHelperBuffer;
}


void mh::sci(Stream & str, float f, std::uint8_t digits)
{
  str.print(mh::sci(f, digits));
}



//////////////////////////////////////////////////
//
// TIME HELPERS
//

// (true)   00:00:00 .. 23:59:59 
// (false)  00:00 ..    23:59
const char * mh::seconds2clock(std::uint32_t seconds, bool displaySeconds)
{
  std::uint16_t days = seconds / 86400UL;  // strips the days
  seconds -= (days * 86400UL);
  std::uint8_t hours = seconds / 3600UL;
  seconds -= (hours * 3600UL);
  std::uint8_t minutes = seconds / 60UL;
  seconds -= (minutes * 60UL);

  std::uint8_t pos = 0;
  mh::__mathHelperBuffer[pos++] = hours/10 + '0';
  mh::__mathHelperBuffer[pos++] = hours%10 + '0';
  mh::__mathHelperBuffer[pos++]  = ':';
  
  mh::__mathHelperBuffer[pos++] = minutes/10 + '0';
  mh::__mathHelperBuffer[pos++] = minutes%10 + '0';
  if (displaySeconds)
  {
    mh::__mathHelperBuffer[pos++]  = ':';
    mh::__mathHelperBuffer[pos++] = seconds/10 + '0';
    mh::__mathHelperBuffer[pos++] = seconds%10 + '0';
  }
  mh::__mathHelperBuffer[pos]  = '\0';
  
  return mh::__mathHelperBuffer;
}

const char * mh::millis2clock(uint32_t millis)
{
  std::uint32_t t = millis/1000;
  mh::seconds2clock(t, true);
  std::uint16_t m = millis - t*1000;

  mh::__mathHelperBuffer[8]  = '.';
  std::uint8_t pos = 9;
  std::uint8_t d = m/100;
  mh::__mathHelperBuffer[pos++] = d + '0';
  m = m - d * 100;
  d = m/10;
  mh::__mathHelperBuffer[pos++] = d + '0';
  d = m - d * 10;
  mh::__mathHelperBuffer[pos++] = d + '0';
  mh::__mathHelperBuffer[pos]  = '\0';

  return mh::__mathHelperBuffer;
}

//////////////////////////////////////////////////
//
// HEX BIN HELPERS
//
// notes:
// - d should not exceed 16 otherwise __mathHelperBuffer overflows...
// - no 64 bit support

const char * mh::hex(std::uint32_t value, std::uint8_t d)
{
	d = (d > 16) ? 16 : d;
	mh::__mathHelperBuffer[d] = '\0';
	while (d > 0)
	{
		std::uint8_t v = value & 0x0F;
		value >>= 4;
		mh::__mathHelperBuffer[--d] = (v < 10) ? '0' + v : 'A' - 10 + v;
	}
	return mh::__mathHelperBuffer;
}


const char * mh::bin(uint32_t value, uint8_t d)
{
  d = (d > 16) ? 16 : d;
	mh::__mathHelperBuffer[d] = '\0';
	while (d > 0)
	{
		std::uint8_t v = value & 0x01;
		value >>= 1;
		mh::__mathHelperBuffer[--d] = '0' + v;
	}
	return mh::__mathHelperBuffer;
}

