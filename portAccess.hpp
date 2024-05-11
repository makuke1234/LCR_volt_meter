#pragma once

#include "main.hpp"
#include <Arduino.h>
#include <variant.h>

#ifdef PA_OUTPUT_READ
  #define PORT_OUT_READ  PORT_PINCFG_INEN
#else
  #define PORT_OUT_READ  0
#endif

#define ARDUINO_PORT_TO_GROUP(x)    ((g_APinDescription[x].ulPort == PORTA) ? PAGrp : ((g_APinDescription[x].ulPort == PORTB) ? PBGrp : -1))
#define ARDUINO_PIN_TO_PORT_PIN(x)  (g_APinDescription[x].ulPin | ARDUINO_PORT_TO_GROUP(x))
#define ARDUINO_PIN_TO_EIC(x)       (g_APinDescription[x].ulExtInt)

#define PAGrp        0
#define PBGrp       (1 << 5)
#define PAPin(x)    (x | PAGrp)
#define PBPin(x)    (x | PBGrp)
#define PBMask(_x)  (1 << ((_x) & 0x1f))
#define PGrp(_x)    ((_x) >> 5)

#define InMode_impl(x, cfg) do { \
  PORT->Group[PGrp(x)].PINCFG[x & 0x1f].reg = cfg; \
  PORT->Group[PGrp(x)].DIRCLR.reg = PBMask(x); \
} while (0)
#define InMode(x)    InMode_impl(x, PORT_PINCFG_INEN)
#define InModePD(x)  do { \
  InMode_impl(x, PORT_PINCFG_INEN | PORT_PINCFG_PULLEN); \
  OutClr(x); \
} while (0)
#define InModePU(x)  do { \
  InMode_impl(x, PORT_PINCFG_INEN | PORT_PINCFG_PULLEN); \
  OutSet(x); \
} while (0)
#define OutMode_impl(x) (PORT->Group[PGrp(x)].DIRSET.reg = PBMask(x))

#ifdef PA_OUTPUT_READ
  #define OutMode(x)  do { \
    InMode_impl(x, PORT_OUT_READ); \
    OutMode_impl(x); \
  } while (0)
#else
  #define OutMode(x)  OutMode_impl(x)
#endif

#define OutModeStrong(x)  do { \
  InMode_impl(x, PORT_PINCFG_DRVSTR | PORT_OUT_READ); \
  OutMode_impl(x); \
} while (0)

#define ResetMode(x)  InMode_impl(x, 0)

#define OutSet(_x)  (PORT_IOBUS->Group[PGrp(_x)].OUTSET.reg = PBMask(_x))
#define OutClr(_x)  (PORT_IOBUS->Group[PGrp(_x)].OUTCLR.reg = PBMask(_x))
#define OutTgl(_x)  (PORT_IOBUS->Group[PGrp(_x)].OUTTGL.reg = PBMask(_x))

#define OutPin(_x, _val)  do { if (_val) { OutSet(_x); } else { OutClr(_x); } } while (0)

#define disableInterrupt_impl(x)  EIC->INTFLAG.reg |= (1 << (x));
#define disableInterrupt(pin)     disableInterrupt_impl(ARDUINO_PIN_TO_EIC(pin))
