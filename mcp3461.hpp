// got inspiration from https://github.com/nerdyscout/Arduino_MCP3x6x_Library/blob/main/lib/MCP3x6x/MCP3x6x.h

#pragma once

#include "algo.hpp"

#include <cstdint>
#include <cassert>

#include <Arduino.h>
#include <SPI.h>
#include <wiring_private.h>


// Define this for 16-bit ADC
#define MCP346x

// Define this for 24-bit ADC
//#define MCP356x

// defaults to 16-bit ADC
#if !defined(MCP346x) && !defined(MCP356x)
  #define MCP346x
#endif


/*
  if temp_diode_p is vin+ &
  temp_diode_m is vin-

  samples:
  temp in celsius = 0.102646 * sample * vref - 269.13

  voltages:
  temp in celsius = volts*(32768.0 * 0.102646) - 269.13
*/

#define MCP3461_CAL_SIZE  9

#define CAL_ADC_GAIN_0_33X_ID   0
#define CAL_ADC_GAIN_1X_ID      1
#define CAL_ADC_GAIN_2X_ID      2
#define CAL_ADC_GAIN_4X_ID      3
#define CAL_ADC_GAIN_8X_ID      4
#define CAL_ADC_GAIN_16X_ID     5
#define CAL_ADC_GAIN_32X_ID     6
#define CAL_ADC_GAIN_64X_ID     7
#define CAL_ADC_VREF_ID         8


#define MCP3461_MAX_COUNT_17BIT  34405L
#define MCP3461_MAX_COUNT_16BIT  32767L
#define MCP3561_MAX_COUNT_24BIT  8388607L
#define MCP3561_MAX_COUNT_25BIT  8808037L


#define MCP3x6x_DEVICE_ADDRESS       0x1
#define MCP3x6x_SPI_ADDR_POS         6
#define MCP3x6x_DEVICE_ADDRESS_MASK  (MCP3x6x_DEVICE_ADDRESS << MCP3x6x_SPI_ADDR_POS)

#define MCP3x6x_CMD_ADDR_POS  2

#define MCP3x6x_DATA_READY_SMASK   (0x04) // 0b00000100 // Tells us whether data is ready from an SPI transaction
#define MCP3x6x_ADDRESS_MASK       (0x38) // 0b00111000
#define MCP3x6x_WRITE_COMMAND_MASK (0x02) // 0b00000010
#define MCP3x6x_WRITE_COMMAND      (MCP3x6x_WRITE_COMMAND_MASK | MCP3x6x_DEVICE_ADDRESS_MASK)
#define MCP3x6x_IREAD_COMMAND_MASK (0x03) // 0b00000011 // Incremental read command
#define MCP3x6x_IREAD_COMMAND      (MCP3x6x_IREAD_COMMAND_MASK | MCP3x6x_DEVICE_ADDRESS_MASK)
#define MCP3x6x_SREAD_COMMAND_MASK (0x01) // 0b1 // Static read command
#define MCP3x6x_SREAD_DATA_COMMAND (MCP3x6x_SREAD_COMMAND_MASK | MCP3x6x_DEVICE_ADDRESS_MASK)

#define MCP3x6x_CMD_ADC_START_MASK    0x28
#define MCP3x6x_CMD_ADC_START         (MCP3x6x_DEVICE_ADDRESS_MASK | MCP3x6x_CMD_ADC_START_MASK)
#define MCP3x6x_CMD_ADC_STANDBY_MASK  0x2C
#define MCP3x6x_CMD_ADC_STANDBY       (MCP3x6x_DEVICE_ADDRESS_MASK | MCP3x6x_CMD_ADC_STANDBY_MASK)
#define MCP3x6x_CMD_ADC_SHDN_MASK     0x30
#define MCP3x6x_CMD_ADC_SHDN          (MCP3x6x_DEVICE_ADDRESS_MASK | MCP3x6x_CMD_ADC_SHDN_MASK)
#define MCP3x6x_CMD_FULL_SHDN_MASK    0x34
#define MCP3x6x_CMD_FULL_SHDN         (MCP3x6x_DEVICE_ADDRESS_MASK | MCP3x6x_CMD_FULL_SHDN_MASK)
#define MCP3x6x_CMD_FULL_RST_MASK     0x38
#define MCP3x6x_CMD_FULL_RST          (MCP3x6x_DEVICE_ADDRESS_MASK | MCP3x6x_CMD_FULL_RST_MASK)

#define MCP3x6x_CMD_READ(reg)       (MCP3x6x_SREAD_DATA_COMMAND | (reg << MCP3x6x_CMD_ADDR_POS))
#define MCP3x6x_CMD_WRITE_INC(reg)  (MCP3x6x_WRITE_COMMAND      | (reg << MCP3x6x_CMD_ADDR_POS))
#define MCP3x6x_CMD_READ_INC(reg)   (MCP3x6x_IREAD_COMMAND      | (reg << MCP3x6x_CMD_ADDR_POS))
#define MCP3x6x_CMD_RESET_REG       MCP3x6x_CMD_WRITE_INC(0xE)

#define MCP3461_DEVICE_TYPE   0x0008
#define MCP3462_DEVICE_TYPE   0x0009
#define MCP3464_DEVICE_TYPE   0x000B
#define MCP3561_DEVICE_TYPE   0x000C
#define MCP3562_DEVICE_TYPE   0x000D
#define MCP3564_DEVICE_TYPE   0x000F


#define MCP3x6x_SPI_ORDER  MSBFIRST
#define MCP3x6x_SPI_MODE   SPI_MODE0


#define MCP3x6x_ADCDATA_ADDR    0x00

#define MCP3x6x_CONFIG0_ADDR              0x01
#define MCP3x6xR_CONFIG0_REF_POS          7
#define MCP3x6xR_CONFIG0_REF_EXT          (0x00 << MCP3x6xR_CONFIG0_REF_POS)
#define MCP3x6xR_CONFIG0_REF_INT          (0x01 << MCP3x6xR_CONFIG0_REF_POS)
#define MCP3x6x_CONFIG0_SHDN_POS          6
#define MCP3x6x_CONFIG0_SHDN_ON           (0x00 << MCP3x6x_CONFIG0_SHDN_POS)
#define MCP3x6x_CONFIG0_SHDN_OFF          (0x01 << MCP3x6x_CONFIG0_SHDN_POS)
#define MCP3x6x_CONFIG0_CLK_SEL_MASK      0x30
#define MCP3x6x_CONFIG0_CLK_SEL_POS       4
#define MCP3x6x_CONFIG0_CLK_SEL_INT_O     (0x03 << MCP3x6x_CONFIG0_CLK_SEL_POS)
#define MCP3x6x_CONFIG0_CLK_SEL_INT       (0x02 << MCP3x6x_CONFIG0_CLK_SEL_POS)
#define MCP3x6x_CONFIG0_CLK_SEL_EXT       (0x00 << MCP3x6x_CONFIG0_CLK_SEL_POS)
#define MCP3x6x_CONFIG0_CS_SEL_POS        2
#define MCP3x6x_CONFIG0_CS_SEL_15uA       (0x03 << MCP3x6x_CONFIG0_CS_SEL_POS)
#define MCP3x6x_CONFIG0_CS_SEL_3_7uA      (0x02 << MCP3x6x_CONFIG0_CS_SEL_POS)
#define MCP3x6x_CONFIG0_CS_SEL_0_9uA      (0x01 << MCP3x6x_CONFIG0_CS_SEL_POS)
#define MCP3x6x_CONFIG0_CS_SEL_NONE       (0x00 << MCP3x6x_CONFIG0_CS_SEL_POS)
#define MCP3x6x_CONFIG0_ADC_MODE_POS      0
#define MCP3x6x_CONFIG0_ADC_MODE_CONV     (0x03 << MCP3x6x_CONFIG0_ADC_MODE_POS)
#define MCP3x6x_CONFIG0_ADC_MODE_STANDBY  (0x02 << MCP3x6x_CONFIG0_ADC_MODE_POS)
#define MCP3x6x_CONFIG0_ADC_MODE_OFF      (0x00 << MCP3x6x_CONFIG0_ADC_MODE_POS)

#define MCP3x6x_CONFIG1_ADDR        0x02
#define MCP3x6x_CONFIG1_AMCLK_POS   6
#define MCP3x6x_CONFIG1_AMCLK_DIV8  (0x03 << MCP3x6x_CONFIG1_AMCLK_POS)
#define MCP3x6x_CONFIG1_AMCLK_DIV4  (0x02 << MCP3x6x_CONFIG1_AMCLK_POS)
#define MCP3x6x_CONFIG1_AMCLK_DIV2  (0x01 << MCP3x6x_CONFIG1_AMCLK_POS)
#define MCP3x6x_CONFIG1_AMCLK_DIV0  (0x00 << MCP3x6x_CONFIG1_AMCLK_POS)
#define MCP3x6x_CONFIG1_OSR_POS     2
#define MCP3x6x_CONFIG1_OSR_32      (0x00 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_64      (0x01 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_128     (0x02 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_256     (0x03 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_512     (0x04 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_1024    (0x05 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_2048    (0x06 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_4096    (0x07 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_8192    (0x08 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_16384   (0x09 << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_20480   (0x0A << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_24576   (0x0B << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_40960   (0x0C << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_49152   (0x0D << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_81920   (0x0E << MCP3x6x_CONFIG1_OSR_POS)
#define MCP3x6x_CONFIG1_OSR_98304   (0x0F << MCP3x6x_CONFIG1_OSR_POS)

#define MCP3x6x_CONFIG2_ADDR         0x03
#define MCP3x6x_CONFIG2_BOOST_POS    6
#define MCP3x6x_CONFIG2_BOOST_x2     (0x03 << MCP3x6x_CONFIG2_BOOST_POS)
#define MCP3x6x_CONFIG2_BOOST_x1     (0x02 << MCP3x6x_CONFIG2_BOOST_POS)
#define MCP3x6x_CONFIG2_BOOST_2DIV3  (0x01 << MCP3x6x_CONFIG2_BOOST_POS)
#define MCP3x6x_CONFIG2_BOOST_DIV2   (0x00 << MCP3x6x_CONFIG2_BOOST_POS)
#define MCP3x6x_CONFIG2_GAIN_POS     3
#define MCP3x6x_CONFIG2_GAIN_x64     (0x07 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_x32     (0x06 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_x16     (0x05 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_x8      (0x04 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_x4      (0x03 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_x2      (0x02 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_x1      (0x01 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_DIV3    (0x00 << MCP3x6x_CONFIG2_GAIN_POS)
#define MCP3x6x_CONFIG2_GAIN_VCM     MCP3x6x_CONFIG2_GAIN_x1
#define MCP3x6x_CONFIG2_GAIN_AVDD    MCP3x6x_CONFIG2_GAIN_DIV3
#define MCP3x6x_CONFIG2_GAIN_TEMP    MCP3x6x_CONFIG2_GAIN_x1
#define MCP3x6x_CONFIG2_AZ_MUX_POS   2
#define MCP3x6x_CONFIG2_AZ_MUX_ON    (0x01 << MCP3x6x_CONFIG2_AZ_MUX_POS)
#define MCP3x6x_CONFIG2_AZ_MUX_OFF   (0x00 << MCP3x6x_CONFIG2_AZ_MUX_POS)
#define MCP3x6xR_CONFIG2_AZ_REF_POS  1
#define MCP3x6xR_CONFIG2_AZ_REF_ON   (0x01 << MCP3x6xR_CONFIG2_AZ_REF_POS)
#define MCP3x6xR_CONFIG2_AZ_REF_OFF  (0x00 << MCP3x6xR_CONFIG2_AZ_REF_POS)

#define MCP3x6x_CONFIG3_ADDR                        0x04
#define MCP3x6x_CONFIG3_CONV_MODE_POS               6
#define MCP3x6x_CONFIG3_CONV_MODE_CONTINUOUS        (0x03 << MCP3x6x_CONFIG3_CONV_MODE_POS)
#define MCP3x6x_CONFIG3_CONV_MODE_ONE_SHOT_STANDBY  (0x02 << MCP3x6x_CONFIG3_CONV_MODE_POS)
#define MCP3x6x_CONFIG3_CONV_MODE_ONE_SHOT_OFF      (0x00 << MCP3x6x_CONFIG3_CONV_MODE_POS)
#define MCP3x6x_CONFIG3_DATA_FORMAT_POS             4
#define MCP3x6x_CONFIG3_DATA_FORMAT_32BIT_CHID_SGN  (0x03 << MCP3x6x_CONFIG3_DATA_FORMAT_POS)
#define MCP3x6x_CONFIG3_DATA_FORMAT_32BIT_SGN       (0x02 << MCP3x6x_CONFIG3_DATA_FORMAT_POS)
#define MCP3x6x_CONFIG3_DATA_FORMAT_32BIT           (0x01 << MCP3x6x_CONFIG3_DATA_FORMAT_POS)
#define MCP356x_CONFIG3_DATA_FORMAT_24BIT           (0x00 << MCP3x6x_CONFIG3_DATA_FORMAT_POS)
#define MCP346x_CONFIG3_DATA_FORMAT_16BIT           (0x00 << MCP3x6x_CONFIG3_DATA_FORMAT_POS)
#define MCP3x6x_CONFIG3_CRC_POS                     3
#define MCP3x6x_CONFIG3_CRC_FORMAT_32               (0x01 << MCP3x6x_CONFIG3_CRC_POS)
#define MCP3x6x_CONFIG3_CRC_FORMAT_16               (0x00 << MCP3x6x_CONFIG3_CRC_POS)
#define MCP3x6x_CONFIG3_CRCCOM_POS                  2
#define MCP3x6x_CONFIG3_CRCCOM_ON                   (0x01 << MCP3x6x_CONFIG3_CRCCOM_POS)
#define MCP3x6x_CONFIG3_CRCCOM_OFF                  (0x00 << MCP3x6x_CONFIG3_CRCCOM_POS)
#define MCP3x6x_CONFIG3_OFFCAL_POS                  1
#define MCP3x6x_CONFIG3_OFFCAL_ON                   (0x01 << MCP3x6x_CONFIG3_OFFCAL_POS)
#define MCP3x6x_CONFIG3_OFFCAL_OFF                  (0x00 << MCP3x6x_CONFIG3_OFFCAL_POS)
#define MCP3x6x_CONFIG3_GAINCAL_POS                 0
#define MCP3x6x_CONFIG3_GAINCAL_ON                  (0x01 << MCP3x6x_CONFIG3_GAINCAL_POS)
#define MCP3x6x_CONFIG3_GAINCAL_OFF                 (0x00 << MCP3x6x_CONFIG3_GAINCAL_POS)

#define MCP3x6x_IRQ_ADDR             0x05
#define MCP3x6x_IRQ_DR_POS           6
#define MCP3x6x_IRQ_CRCCFG_POS       5
#define MCP3x6x_IRQ_POR_POS          4
#define MCP3x6x_STATUS_DR_POS        2
#define MCP3x6x_STATUS_CRCCFG_POS    1
#define MCP3x6x_STATUS_POR_POS       0
#define MCP3x6x_IRQ_MODE_POS         3
#define MCP3x6x_IRQ_MODE_MDAT_HIGHZ  (0x02 << MCP3x6x_IRQ_MODE_POS)
#define MCP3x6x_IRQ_MODE_MDAT_HIGH   (0x03 << MCP3x6x_IRQ_MODE_POS)
#define MCP3x6x_IRQ_MODE_IRQ_HIGHZ   (0x00 << MCP3x6x_IRQ_MODE_POS)
#define MCP3x6x_IRQ_MODE_IRQ_HIGH    (0x01 << MCP3x6x_IRQ_MODE_POS)
#define MCP3x6x_IRQ_FASTCMD_POS      1
#define MCP3x6x_IRQ_FASTCMD_ON       (0x01 << MCP3x6x_IRQ_FASTCMD_POS)
#define MCP3x6x_IRQ_FASTCMD_OFF      (0x00 << MCP3x6x_IRQ_FASTCMD_POS)
#define MCP3x6x_IRQ_STP_POS          0
#define MCP3x6x_IRQ_STP_ON           (0x01 << MCP3x6x_IRQ_STP_POS)
#define MCP3x6x_IRQ_STP_OFF          (0x00 << MCP3x6x_IRQ_STP_POS)

#define MCP3x6x_MUX_ADDR          0x06
#define MCP3x6x_MUX_VIN_P_POS     4
#define MCP3x6x_MUX_VIN_N_POS     0
#define MCP3x6x_MUX_CH_IntVcm     0xF
#define MCP3x6x_MUX_CH_IntTemp_M  0xE
#define MCP3x6x_MUX_CH_IntTemp_P  0xD
#define MCP3x6x_MUX_CH_REFIN_n    0xC
#define MCP3x6x_MUX_CH_REFIN_p    0xB
#define MCP3x6x_MUX_CH_RESERVED1  0xA
#define MCP3x6x_MUX_CH_AVDD       0x9
#define MCP3x6x_MUX_CH_AGND       0x8
#define MCP3x64_MUX_CH7           0x7
#define MCP3x64_MUX_CH6           0x6
#define MCP3x64_MUX_CH5           0x5
#define MCP3x64_MUX_CH4           0x4
#define MCP3x62_MUX_CH3           0x3
#define MCP3x62_MUX_CH2           0x2
#define MCP3x6x_MUX_CH1           0x1
#define MCP3x6x_MUX_CH0           0x0

#define MCP3x6x_MUX_PRE_CH0_SE      0x08
#define MCP3x6x_MUX_PRE_CH1_SE      0x18
#define MCP3x62_MUX_PRE_CH2_SE      0x28
#define MCP3x62_MUX_PRE_CH3_SE      0x38
#define MCP3x64_MUX_PRE_CH4_SE      0x48
#define MCP3x64_MUX_PRE_CH5_SE      0x58
#define MCP3x64_MUX_PRE_CH6_SE      0x68
#define MCP3x64_MUX_PRE_CH7_SE      0x78
#define MCP3x6x_MUX_PRE_CH0_1_DIF   0x01
#define MCP3x62_MUX_PRE_CH0_2_DIF   0x02
#define MCP3x62_MUX_PRE_CH0_3_DIF   0x03
#define MCP3x6x_MUX_PRE_CH1_0_DIF   0x10
#define MCP3x62_MUX_PRE_CH1_2_DIF   0x12
#define MCP3x62_MUX_PRE_CH1_3_DIF   0x13
#define MCP3x62_MUX_PRE_CH2_0_DIF   0x20
#define MCP3x62_MUX_PRE_CH2_1_DIF   0x21
#define MCP3x62_MUX_PRE_CH2_3_DIF   0x23
#define MCP3x64_MUX_PRE_CH4_5_DIF   0x45
#define MCP3x64_MUX_PRE_CH4_6_DIF   0x46
#define MCP3x64_MUX_PRE_CH4_7_DIF   0x47
#define MCP3x64_MUX_PRE_CH6_4_DIF   0x64
#define MCP3x64_MUX_PRE_CH6_5_DIF   0x65
#define MCP3x64_MUX_PRE_CH6_7_DIF   0x67
#define MCP3x6x_MUX_PRE_TEMP        0xDE
#define MCP3x6x_MUX_PRE_AVDD        0x98
#define MCP3x6x_MUX_PRE_VCM         0xF8
#define MCP3x6x_MUX_PRE_OFFSET      0x88

#define MCP3x6x_SCAN_ADDR            0x07
#define MCP3x6x_SCAN_DLY_POS         5 // bit 21
#define MCP3x6x_SCAN_DLY_BYTE        2 // 3rd byte, index of 2
#define MCP3x6x_SCAN_DLY_512         (0x07 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_256         (0x06 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_128         (0x05 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_64          (0x04 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_32          (0x03 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_16          (0x02 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_8           (0x01 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_DLY_NONE        (0x00 << MCP3x6x_SCAN_DLY_POS)
#define MCP3x6x_SCAN_OFFSET_POS      7  // bit 15
#define MCP3x6x_SCAN_OFFSET_BYTE     1
#define MCP3x6x_SCAN_VCM_POS         6  // bit 14
#define MCP3x6x_SCAN_VCM_BYTE        1
#define MCP3x6x_SCAN_AVDD_POS        5  // bit 13
#define MCP3x6x_SCAN_AVDD_BYTE       1
#define MCP3x6x_SCAN_TEMP_POS        4  // bit 12
#define MCP3x6x_SCAN_TEMP_BYTE       1
#define MCP3x6x_SCAN_DIFF_POS        0  // bit 8
#define MCP3x6x_SCAN_DIFF_BYTE       1
#define MCP3x64_SCAN_DIFF_CH6_7_POS  3
#define MCP3x64_SCAN_DIFF_CH4_5_POS  2
#define MCP3x62_SCAN_DIFF_CH2_3_POS  1
#define MCP3x6x_SCAN_DIFF_CH0_1_POS  0
#define MCP3x6x_SCAN_SE_POS          0  // bit 0
#define MCP3x6x_SCAN_SE_BYTE         0
#define MCP3x64_SCAN_SE_CH7          7
#define MCP3x64_SCAN_SE_CH6          6
#define MCP3x64_SCAN_SE_CH5          5
#define MCP3x64_SCAN_SE_CH4          4
#define MCP3x62_SCAN_SE_CH3          3
#define MCP3x62_SCAN_SE_CH2          2
#define MCP3x6x_SCAN_SE_CH1          1
#define MCP3x6x_SCAN_SE_CH0          0
#define MCP3x6x_SCAN_CHID_OFFSET     0xF
#define MCP3x6x_SCAN_CHID_VCM        0xE
#define MCP3x6x_SCAN_CHID_AVDD       0xD
#define MCP3x6x_SCAN_CHID_TEMP       0xC
#define MCP3x64_SCAN_CHID_CH6_7      0xB
#define MCP3x64_SCAN_CHID_CH4_5      0xA
#define MCP3x62_SCAN_CHID_CH2_3      0x9
#define MCP3x6x_SCAN_CHID_CH0_1      0x8
#define MCP3x64_SCAN_CHID_CH7        0x7
#define MCP3x64_SCAN_CHID_CH6        0x6
#define MCP3x64_SCAN_CHID_CH5        0x5
#define MCP3x64_SCAN_CHID_CH4        0x4
#define MCP3x62_SCAN_CHID_CH3        0x3
#define MCP3x62_SCAN_CHID_CH2        0x2
#define MCP3x6x_SCAN_CHID_CH1        0x1
#define MCP3x6x_SCAN_CHID_CH0        0x0
#define MCP3x6x_SCAN_NUM_IDS         16

#define MCP3x6x_TIMER_ADDR      0x08
#define MCP3x6x_OFFSET_ADDR     0x09
#define MCP3x6x_GAIN_ADDR       0x0A
#define MCP3x6x_RESERVED1_ADDR  0x0B
#define MCP3x6x_RESERVED2_ADDR  0x0C
#define MCP3x6x_LOCK_ADDR       0x0D
#define MCP3x6x_RESERVED3_ADDR  0x0E
#define MCP3x6x_CRCCFG_ADDR     0x0F

#define MCP3461_DEFAULT_CONFIG0   0xC0
#define MCP3461_DEFAULT_CONFIG1   0x0C
#define MCP3461_DEFAULT_CONFIG2   0x8B
#define MCP3461_DEFAULT_CONFIG3   0x00
#define MCP3461_DEFAULT_IRQ       0x73 
#define MCP3461_DEFAULT_MUX       0x01
#define MCP3461_DEFAULT_SCAN1     0x00
#define MCP3461_DEFAULT_SCAN2     0x00
#define MCP3461_DEFAULT_SCAN3     0x00
#define MCP3461_DEFAULT_TIMER1    0x00
#define MCP3461_DEFAULT_TIMER2    0x00
#define MCP3461_DEFAULT_TIMER3    0x00
#define MCP3461_DEFAULT_OFFSET1   0x00
#define MCP3461_DEFAULT_OFFSET2   0x00
#define MCP3461_DEFAULT_OFFSET3   0x00
#define MCP3461_DEFAULT_GAIN1     0x80
#define MCP3461_DEFAULT_GAIN2     0x00
#define MCP3461_DEFAULT_GAIN3     0x00
#define MCP3461_DEFAULT_LOCK      0xA5
#define MCP3461_DEFAULT_CRCCFG1   0x00
#define MCP3461_DEFAULT_CRCCFG2   0x00

namespace sdadc
{
  struct __attribute__((packed)) Config0
  {
    std::uint8_t adcMode : 2;
    std::uint8_t bias    : 2;
    std::uint8_t clk     : 2;
    std::uint8_t shdwn   : 1;
    std::uint8_t vrefSel : 1;
  };
  struct __attribute__((packed)) Config1
  {
    std::uint8_t reserved     : 2;
    std::uint8_t osr          : 4;
    std::uint8_t clkPrescaler : 2;
  };
  struct __attribute__((packed)) Config2
  {
    std::uint8_t reserved : 1;
    std::uint8_t azRef    : 1;
    std::uint8_t azMu     : 1;
    std::uint8_t gain     : 3;
    std::uint8_t boost    : 2;
  };
  struct __attribute__((packed)) Config3
  {
    std::uint8_t enGaincal  : 1;
    std::uint8_t enOffcal   : 1;
    std::uint8_t enCrccom   : 1;
    std::uint8_t crcFormat  : 1;
    std::uint8_t dataFormat : 2;
    std::uint8_t convMode   : 2;
  };

  template<typename T>
  union __attribute__((packed)) Conf
  {
    T bit;
    std::uint8_t reg;

    constexpr Conf(std::uint8_t val)
      : reg(val)
    {}
  };

  struct Configs
  {
    Conf<Config0> config0;
    Conf<Config1> config1;
    Conf<Config2> config2;
    Conf<Config3> config3;
    std::uint8_t mux;

    static constexpr std::uint8_t c_chids[MCP3x6x_SCAN_NUM_IDS] = {
      MCP3x6x_MUX_PRE_CH0_SE,
      MCP3x6x_MUX_PRE_CH1_SE,
      MCP3x62_MUX_PRE_CH2_SE,
      MCP3x62_MUX_PRE_CH3_SE,
      MCP3x64_MUX_PRE_CH4_SE,
      MCP3x64_MUX_PRE_CH5_SE,
      MCP3x64_MUX_PRE_CH6_SE,
      MCP3x64_MUX_PRE_CH7_SE,
      MCP3x6x_MUX_PRE_CH0_1_DIF,
      MCP3x62_MUX_PRE_CH2_3_DIF,
      MCP3x64_MUX_PRE_CH4_5_DIF,
      MCP3x64_MUX_PRE_CH6_7_DIF,
      MCP3x6x_MUX_PRE_TEMP,
      MCP3x6x_MUX_PRE_AVDD,
      MCP3x6x_MUX_PRE_VCM,
      MCP3x6x_MUX_PRE_OFFSET
    };
  };
  extern Configs configs;

  enum class Clock : std::uint8_t
  {
    external = 0,
    internal = 2,
    internalOutput = 3
  };

  enum class Channel : std::uint8_t
  {
    ch0,
    ch1,
    ch2,
    ch3,
    ch4,
    ch5,
    ch6,
    ch7,
    agnd,
    avdd,
    reserved1,
    refin_pos,
    refin_neg,
    temp_diode_p,
    temp_diode_m,
    vcm_int
  };

  enum class OSR : std::uint8_t
  {
    osr32,
    osr64,
    osr128,
    osr256,
    osr512,
    osr1024,
    osr2048,
    osr4096,
    osr8192,
    osr16384,
    osr20480,
    osr24576,
    osr40960,
    osr49152,
    osr81920,
    osr98304
  };
  
  enum class Dataformat : std::uint8_t
  {
    f16bit = 0,
    f24bit = 0,
    f32bit = 1,
    f32bit_sgn = 2,
    f32bit_chid_sgn = 3,
    
    size
  };

  enum class Gain : std::uint8_t
  {
    g0x33,
    g1x,
    g2x,
    g4x,
    g8x,
    g16x,
    g32x,
    g64x,

    size,

    vcm  = Gain::g1x,
    avdd = Gain::g0x33,
    temp = Gain::g1x
  };

  struct AdcCalData
  {
    static constexpr float c_vref = 2.4f;

    float gain[std::uint8_t(Gain::size)] = { 0.33f, 1.0f, 2.0f, 4.0f, 8.0f, 16.0f, 32.0f, 64.0f };
    
    Dataformat format = Dataformat::f32bit_chid_sgn;
    Gain gainSel = Gain::g1x;
    OSR osrRatio = OSR::osr32;
    float vref = AdcCalData::c_vref;

    void loadCal(const algo::array<float, MCP3461_CAL_SIZE> & data);
    void saveCal(algo::array<float, MCP3461_CAL_SIZE> & data) const;

    void selfCal(sdadc::Channel mpos, sdadc::Channel mneg = sdadc::Channel::agnd);
  };
  extern AdcCalData calData;

  void printRegisters(int (*pfunc)(const char * fmt, ...));
  void init(SERCOM * module, std::uint32_t frequency, std::int8_t mosi, std::int8_t miso, std::int8_t sck, std::int8_t cs);
  void setFreq(std::uint32_t frequency);
  void config(sdadc::OSR oversampling = sdadc::OSR::osr32, sdadc::Dataformat format = sdadc::Dataformat::f32bit_chid_sgn);
  void reset();
  void standby();
  void setClock();
  
  void spiTransfer(std::uint8_t * tx, std::uint8_t * rx, std::uint16_t size);
  void spiWrite(std::uint8_t * data, std::uint16_t sz);
  std::uint8_t spiRead(std::uint8_t cmd);
  std::uint8_t status();

  void fastcmd(std::uint8_t cmd);
  void read(std::int32_t * data, std::uint8_t * chid);
  void writeReg(std::uint8_t byte, std::uint8_t addr);
  void startConversion();
  bool isContReady();
  
  void autozero(bool enable = true);
  void setGain(sdadc::Gain gain);

  std::int32_t getSample(sdadc::Channel chpos, sdadc::Channel chneg, bool & suc);
  float toVolts(std::int32_t sample);
  float toTemp(std::int32_t sample);
  float toTemp(float volts);
}
