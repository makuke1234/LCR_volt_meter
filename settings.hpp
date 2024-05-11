#pragma once


#include "lcr.hpp"
#include "mcp3461.hpp"

#include "algo.hpp"
#include <cstdint>
#include <cassert>
#include <cstring>
#include <type_traits>

#include <Arduino.h>

#define SETTINGS_MAX_SIZE         256U

#define SETTINGS_IDX_FLASHCAL       0U
#define SETTINGS_IDX_FLASHSETTINGS  1U


namespace settings
{
  struct FlashCal
  {
    bool valid = false;

    algo::array<float, VOLTS_CAL_SIZE> voltsCal;
    algo::array<float, OHMS_CAL_SIZE>  ohmsCal;
    algo::array<float, IMP_CAL_SIZE>   impCal;

    algo::array<float, MCP3461_CAL_SIZE> adcCal;

  };
  extern FlashCal flashCal;

  struct FlashSettings
  {
    bool valid : 1;

    bool isDispBright : 1;
    std::uint8_t dispBright, ledBright;

    constexpr FlashSettings()
      : valid(false),

      isDispBright(false), dispBright(0), ledBright(0)
    {
    }
  };
  extern FlashSettings flashSettings;


  void init();
  bool hasData();

  bool loadCal();
  bool saveCal(bool force = false);

  bool loadSettings();
  bool saveSettings();

  void save();

  // operations

  bool write(std::uint16_t idx, const void * data, std::uint16_t bytes);
  template<typename T, typename = typename std::enable_if<!std::is_pointer<T>::value>::type >
  bool write(std::uint16_t idx, const T & data)
  {
    return settings::write(idx, &data, sizeof(T));
  }
  bool write(std::uint16_t idx, const char * msg);

  // returns number of bytes copied
  std::uint16_t read(std::uint16_t idx, void * data, std::uint16_t expectedSize = 0);
  template<typename T, typename = typename std::enable_if<!std::is_pointer<T>::value && std::is_arithmetic<T>::value>::type >
  T read(std::uint16_t idx, const T & def = 0)
  {
    T data;
    if (!settings::read(idx, &data, sizeof(T)))
    {
      return def;
    }
    return data;
  }
  template<typename T, typename = typename std::enable_if<!std::is_pointer<T>::value>::type >
  std::uint16_t read(std::uint16_t idx, T & data = 0)
  {
    return settings::read(idx, &data, sizeof(T));
  }

  // return number of bytes available at index, 0 means index doesn't exist
  std::uint16_t size(std::uint16_t idx);
  // returns whether the requested item exists or not
  bool exist(std::uint16_t idx);
}
