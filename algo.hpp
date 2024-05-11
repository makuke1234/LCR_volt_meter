#pragma once

#include <cstdint>
#include <cassert>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <Arduino.h>

namespace algo
{
  template<typename T, std::size_t N>
  class array
  {
  private:
    T pdata[N];
  
  public:
    constexpr array() = default;

    constexpr std::size_t size() const
    {
      return N;
    }
    constexpr const T * data() const
    {
      return this->pdata;
    }
    T * data()
    {
      return this->pdata;
    }

    const T & at(std::size_t idx) const
    {
      assert(idx < N);

      return this->pdata[idx];
    }
    T & operator[](std::size_t idx)
    {
      return this->pdata[idx];
    }
    constexpr const T & operator[](std::size_t idx) const
    {
      return this->pdata[idx];
    }
    T & front()
    {
      assert((this->pdata != nullptr) && (N > 0));
      return this->pdata[0];
    }
    const T & front() const
    {
      assert((this->pdata != nullptr) && (N > 0));
      return this->pdata[0];
    }
    T & back()
    {
      assert((this->pdata != nullptr) && (N > 0));
      return this->pdata[N - 1];
    }
    const T & back() const
    {
      assert((this->pdata != nullptr) && (N > 0));
      return this->pdata[N - 1];
    }
  };



  template<typename T, typename sizeT>
  class vector_base
  {
  private:
    T * pdata = nullptr;
    sizeT psize = 0, pcap = 0;

  public:
    constexpr vector_base() = default;

    constexpr bool empty() const
    {
      return !this->psize;
    }
    constexpr sizeT size() const
    {
      return this->psize;
    }
    constexpr sizeT capacity() const
    {
      return this->pcap;
    }

    constexpr const T * data() const
    {
      return this->pdata;
    }
    T * data()
    {
      return this->pdata;
    }

    const T & at(sizeT idx) const
    {
      assert(idx < this->psize);

      return this->pdata[idx];
    }
    T & operator[](sizeT idx)
    {
      return this->pdata[idx];
    }
    constexpr const T & operator[](sizeT idx) const
    {
      return this->pdata[idx];
    }
    T & front()
    {
      assert((this->pdata != nullptr) && (this->psize > 0));
      return this->pdata[0];
    }
    const T & front() const
    {
      assert((this->pdata != nullptr) && (this->psize > 0));
      return this->pdata[0];
    }
    T & back()
    {
      assert((this->pdata != nullptr) && (this->psize > 0));
      return this->pdata[this->psize - 1];
    }
    const T & back() const
    {
      assert((this->pdata != nullptr) && (this->psize > 0));
      return this->pdata[this->psize - 1];
    }

    void push_back(const T & item)
    {
      if (this->psize >= this->pcap)
      {
        auto newcap = (this->psize + 1) * 2U;
        auto newmem = static_cast<T *>(std::realloc(this->pdata, sizeof(T) * newcap));

        if (newmem == nullptr)
        {
          assert(!"Memory error!");
          return;
        }

        this->pdata = newmem;
        this->pcap  = newcap;
      }

      this->pdata[this->psize] = item;
      ++this->psize;
    }

    void clear()
    {
      this->psize = 0;
    }
  };

  template<typename T>
  class vector : public vector_base<T, std::uint16_t>
  {
  };

  template<typename T>
  class vecmini : public vector_base<T, std::uint8_t>
  {
  };

  template<typename T>
  class veclarge : public vector_base<T, std::size_t>
  {
  };

  template<typename T>
	constexpr T clamp(T v, T min_, T max_)
	{
		return (v < min_) ? min_ : ((v > max_) ? max_ : v);
	}

  template<std::uint8_t index> constexpr float powf(float num, float multi);
  template<> constexpr float powf<0>(float num, float multi)
  {
    return num;
  }
  template<std::uint8_t index> constexpr float powf(float num, float multi)
  {
    return algo::powf<index - 1U>(num, multi) * multi;
  }
  float powf(float num, float multi, std::uint8_t power);
  float powif(float num, float multi, std::int8_t power);

  // constexpr full & partial separators
  template<std::uint8_t decimals>
  constexpr std::int32_t floatFull(float f)
  {
      return std::int32_t(f + algo::powf<decimals>((f < 0.0f) ? -0.5f : 0.5f, 0.1f));
  }
  template<std::uint8_t decimals>
  constexpr std::uint32_t floatPartial(float f, std::int32_t full)
  {
    return std::labs(std::int32_t((f - float(full)) * algo::powf<decimals>((f < float(full)) ? -1.0f : 1.0f, 10.0f) + 0.5f));
  }
  template<std::uint8_t decimals>
  constexpr void floatFrac(float f, std::int32_t & full, std::uint32_t & partial)
  {
    full    = algo::floatFull<decimals>(f);
    partial = algo::floatPartial<decimals>(f, full);
  }

  // runtime full & partial separators
  std::int32_t floatFull(float f, std::uint8_t decimals = 2);
  std::uint32_t floatPartial(float f, std::int32_t full, std::uint8_t decimals = 2);
  void floatFrac(float f, std::int32_t & full, std::uint32_t & partial, std::uint8_t decimals = 2);

  template<typename T>
  constexpr T hysteresis(T prevhyst, T newvalue, T hyst)
  {
    return (std::labs(newvalue - prevhyst) < hyst) ? prevhyst : newvalue;
  }

  template<> constexpr std::int64_t hysteresis(std::int64_t prevhyst, std::int64_t newvalue, std::int64_t hyst)
  {
    return (std::llabs(newvalue - prevhyst) < hyst) ? prevhyst : newvalue;
  }
  template<> constexpr std::uint64_t hysteresis(std::uint64_t prevhyst, std::uint64_t newvalue, std::uint64_t hyst)
  {
    return (std::llabs(newvalue - prevhyst) < hyst) ? prevhyst : newvalue;
  }
  template<> constexpr float hysteresis<float>(float prevhyst, float newvalue, float hyst)
  {
    return (fabsf(newvalue - prevhyst) < hyst) ? prevhyst : newvalue;
  }
  template<> constexpr double hysteresis<double>(double prevhyst, double newvalue, double hyst)
  {
    return (fabs(newvalue - prevhyst) < hyst) ? prevhyst : newvalue;
  }

  // Complex dividing function
  template<typename T>
  void cdiv(T real1, T imag1, T real2, T imag2, T & realrec, T & imagrec)
  {
    // (a+bi)/(c+di) = (ac+bd)/(c^2+d^2) + (bc-ad)/(c^2+d^2)i

    const auto divisor = real2 * real2 + imag2 * imag2;

    realrec = (real1 * real2 + imag1 * imag2) / divisor;
    imagrec = (imag1 * real2 - real1 * imag2) / divisor;
  }
  // Complex multiplication function
  template<typename T>
  void cmul(T real1, T imag1, T real2, T imag2, T & realrec, T & imagrec)
  {
    realrec = real1 * real2 - imag1 * imag2;
    imagrec = real1 * imag2 + imag1 * real2;
  }
}

//
//    FILE: MathHelpers.h
//  AUTHOR: Rob Tillaart
//    DATE: 2018-01-21
// VERSION: 0.1.1
//
//
// PUPROSE: misc functions for math and time
//
#ifndef MATHHELPERS
#define MATHHELPERS

#define MATHHELPERS_VERSION (F("0.1.1"))

namespace mh
{
  //////////////////////////////////////////////////
  //
  // FLOAT REPRESENTATION HELPERS
  //
  const char * sci(double number, int digits);


  void sci(Stream & str, float f, std::uint8_t digits);



  //////////////////////////////////////////////////
  //
  // TIME HELPERS
  //

  // (true)   00:00:00 .. 23:59:59 
  // (false)  00:00 ..    23:59
  const char * seconds2clock(std::uint32_t seconds, bool displaySeconds=false);

  const char * millis2clock(std::uint32_t millis);

  constexpr float weeks(std::uint32_t seconds)
  {
    return seconds * 1.653439153439e-6;  //  /604800
  }

  constexpr float days(std::uint32_t seconds)
  {
    return seconds * 1.157407407407e-5;  //  /86400
  }

  constexpr float hours(std::uint32_t seconds)
  {
    return seconds * 2.777777777778e-4;  //  /3600
  }

  constexpr float minutes(std::uint32_t seconds)
  {
    return seconds * 1.666666666667e-2;  //  /60
  }

  //////////////////////////////////////////////////
  //
  // HEX BIN HELPERS
  //
  // notes:
  // - d should not exceed 16 otherwise __mathHelperBuffer overflows...
  // - no 64 bit support

  const char * hex(std::uint32_t value, std::uint8_t d = 8);


  const char * bin(std::uint32_t value, std::uint8_t d = 8);

}


#endif  // MATHHELPERS

// END OF FILE
