#include "settings.hpp"

#define FLASH_DEBUG 0

#ifdef SETTINGS_MAX_SIZE
  #define EEPROM_EMULATION_SIZE (SETTINGS_MAX_SIZE)
#endif

#include "FlashStorage_SAMD.h"

settings::FlashCal settings::flashCal;
settings::FlashSettings settings::flashSettings;

static algo::vector<std::uint32_t> s_idxVec;

static std::uint16_t s_writesize(std::uint16_t idx, std::uint16_t size);
static std::uint16_t s_readsize(std::uint16_t idx, std::uint16_t & sz);
static bool s_readsize_addr(std::uint32_t addr, std::uint16_t & sz);

// returns data write address
static std::uint16_t s_writesize(std::uint16_t idx, std::uint16_t size)
{
  SERIAL.println("assert #1");
  // the element must be either the last element or not exist at all
  assert((idx + 1 + 1) > s_idxVec.size());
  SERIAL.println("assert #2");
  assert(size > 0);
  if ((idx + 2 + size + 1) > EEPROM_EMULATION_SIZE)
  {
    return UINT16_MAX;
  }

  auto addr = 0;
  if (s_idxVec.size() > 0)
  {
    if (s_idxVec.size() == (idx + 1))
    {
      addr = s_idxVec.at(idx);
    }
    else
    {
      assert(s_idxVec.size() == idx);

      std::uint16_t sz;
      if (!s_readsize_addr(s_idxVec.back(), sz))
      {
        return UINT16_MAX;
      }
      addr += 1 + (sz > 0x7F) + sz;
      s_idxVec.push_back(addr);
      //SERIAL.println("getting size");
      //auto sz = settings::size(s_idxVec.size() - 1);
      //addr += (sz > 0x7F);
      //addr += sz;
    }
  }
  else
  {
    // addr = 0;
    s_idxVec.push_back(addr);
  }


  SERIAL.println("writing...");
  if (size < 0x7F)
  {
    EEPROM.write(addr, size);
    ++addr;
  }
  else
  {
    assert(size <= 0x7EFF);

    EEPROM.write(addr, ((size >> 8) & 0xFF) | 0x80);
    ++addr;
    EEPROM.write(addr, size & 0xFF);
    ++addr;
  }

  return addr;
}
// return data read address
static std::uint16_t s_readsize(std::uint16_t idx, std::uint16_t & sz)
{
  assert(idx < s_idxVec.size());

  //sz = settings::size(idx);
  auto addr = s_idxVec.at(idx);
  if (!s_readsize_addr(addr, sz))
  {
    return UINT16_MAX;
  }
  addr += 1 + (sz > 0x7F);
  return addr;
}
static bool s_readsize_addr(std::uint32_t addr, std::uint16_t & sz)
{
  assert((addr + 1) < EEPROM_EMULATION_SIZE);

  auto size1 = EEPROM.read(addr);
  if ((!size1) || (size1 == 0xFF))
  {
    return false;
  }
  ++addr;


  if (size1 & 0x80)
  {
    size1 &= 0x7F;
    auto size2 = EEPROM.read(addr);
    ++addr;
    sz = (std::uint16_t(size1) << 8) | std::uint16_t(size2);
  }
  else
  {
    sz = size1;
  }

  return true;
}

void settings::init()
{
  if (!EEPROM.isValid())
  {
    return;
  }

  // read through eeprom, until first 0 is reached
  std::uint32_t addr = 0;
  std::uint16_t sz = 0;
  while (s_readsize_addr(addr, sz))
  {
    SERIAL.print("addr: ");
    SERIAL.print(addr);
    SERIAL.print("; sz: ");
    SERIAL.println(sz);
    s_idxVec.push_back(addr);
    addr += 1 + (sz > 0x7F);
    addr += sz;
  }

  if (!settings::hasData())
  {
    SERIAL.println("Doesn't have data!");
    // initialize slots for CAL & settings
    settings::flashCal.valid = false;
    saveCal(true);

    settings::flashSettings.valid = false;
    saveSettings();
  }
}
bool settings::hasData()
{
  return s_idxVec.size() > 0;
}

bool settings::loadCal()
{
  if (!settings::hasData())
  {
    return false;
  }

  std::uint16_t ret = settings::read(SETTINGS_IDX_FLASHCAL, settings::flashCal) == sizeof(settings::FlashCal);
  if (!ret)
  {
    settings::flashCal.valid = false;
  }
  return ret;
}
bool settings::saveCal(bool force)
{
  if (!settings::flashCal.valid && !force)
  {
    return false;
  }

  // save data to EEPROM
  SERIAL.print(sizeof(settings::FlashCal));
  SERIAL.println("WRITE");
  auto ret = settings::write(SETTINGS_IDX_FLASHCAL, settings::flashCal);
  if (!ret)
  {
    return false;
  }

  SERIAL.println("SAVE");
  settings::save();
  return true;
}

bool settings::loadSettings()
{
  if (!settings::hasData())
  {
    return false;
  }

  auto ret = settings::read(SETTINGS_IDX_FLASHSETTINGS, settings::flashSettings) == sizeof(settings::FlashSettings);
  return ret;
}
bool settings::saveSettings()
{
  auto ret = settings::write(SETTINGS_IDX_FLASHSETTINGS, settings::flashSettings);
  if (!ret)
  {
    return false;
  }

  settings::save();
  return true;
}

void settings::save()
{
  EEPROM.commit();
}

bool settings::write(std::uint16_t idx, const void * data, std::uint16_t bytes)
{
  assert(data != nullptr);
  assert(bytes > 0);

  std::uint32_t addr;
  SERIAL.print("getting size [");
  SERIAL.print(idx);
  SERIAL.print("]: ");
  SERIAL.print(settings::size(idx));
  SERIAL.print(" ? ");
  SERIAL.println(bytes);
  if (settings::size(idx) == bytes)
  {
    addr = s_idxVec.at(idx) + 1 + (bytes > 0x7F);
  }
  else
  {
    SERIAL.println("writesize");
    addr = s_writesize(idx, bytes);
    if (addr == UINT16_MAX)
    {
      return false;
    }
  }

  SERIAL.println("before for");
  auto data8 = static_cast<const std::uint8_t *>(data);
  for (std::uint16_t i = 0; i < bytes; ++i)
  {
    //SERIAL.println("writing byte");
    EEPROM.write(addr, data8[i]);
    ++addr;
  }

  return true;
}
bool settings::write(std::uint16_t idx, const char * msg)
{
  assert(msg != nullptr);
  return settings::write(idx, msg, std::strlen(msg) + 1);
}

std::uint16_t settings::read(std::uint16_t idx, void * data, std::uint16_t expectedSize)
{
  if (idx >= s_idxVec.size())
  {
    return 0;
  }
  assert(data != nullptr);

  std::uint16_t sz;
  auto addr = s_readsize(idx, sz);
  if (expectedSize && (sz != expectedSize))
  {
    return 0;
  }

  auto data8 = static_cast<std::uint8_t *>(data);
  for (std::uint16_t i = 0; i < sz; ++i)
  {
    data8[i] = EEPROM.read(addr);
    ++addr;
  }
  return sz;
}

std::uint16_t settings::size(std::uint16_t idx)
{
  if (idx >= s_idxVec.size())
  {
    return 0;
  }

  auto addr = s_idxVec.at(idx);
  // read byte from EEPROM

  std::uint16_t sz;
  if (!s_readsize_addr(addr, sz))
  {
    return 0;
  }
  return sz;
}
bool settings::exist(std::uint16_t idx)
{
  return settings::size(idx) > 0;
}
