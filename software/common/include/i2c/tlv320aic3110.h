/*
 * I2C driver for TLV320 audio codec
 * https://www.ti.com/lit/ds/symlink/TLV320.pdf
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

// I2C device details
#define TLV320_ADDR 0x18

// Page map
#define TLV320_PAGE_0    0x00
#define TLV320_PAGE_1     0x01
#define TLV320_PAGE_3    0x03
#define TLV320_PAGE_4  0x04
#define TLV320_PAGE_8     0x08
#define TLV320_PAGE_9     0x09
#define TLV320_PAGE_12     0x0C
#define TLV320_PAGE_13     0x0D

// Register map

// Page 0
#define TLV320_REG_PAGECTRL        0x00
#define TLV320_REG_SW_RESET        0x01
#define TLV320_REG_OT_FLAG         0x03
#define TLV320_REG_CLKMUX          0x04
#define TLV320_REG_PLL_PR          0x05
#define TLV320_REG_PLL_J           0x06
#define TLV320_REG_PLL_D_MSB       0x07
#define TLV320_REG_PLL_D_LSB       0x08
#define TLV320_REG_NDAC            0x0B
#define TLV320_REG_MDAC            0x0C
#define TLV320_REG_DOSR_MSB        0x0D
#define TLV320_REG_DOSR_LSB        0x0E
#define TLV320_REG_NADC            0x12
#define TLV320_REG_MADC            0x13
#define TLV320_REG_AOSR            0x14
#define TLV320_REG_CLKOUT_MUX      0x19
#define TLV320_REG_CLKOUT_MEN      0x1A
#define TLV320_REG_CODEC_IF_1      0x1B
#define TLV320_REG_SLOTOFFSET      0x1C
#define TLV320_REG_CODEC_IF_2      0x1D
#define TLV320_REG_BCLKN_EN        0x1E
#define TLV320_REG_CODEC_IF_3      0x1F
#define TLV320_REG_CODEC_IF_4      0x20