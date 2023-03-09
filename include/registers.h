// This file is part of Teensy_VS1053b.
// Copyright (C) 2023 Florian Rau.
//
// Teensy_VS1053b is free software: you can redistribute it and/or modify it
// under the terms of the GNU Lesser General Public License as published by the
// Free Software Foundation, either version 3 of the License, or any later
// version.
//
// Teensy_VS1053b is distributed in the hope that it will be useful, but WITHOUT
// ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
// FOR A PARTICULAR PURPOSE. See the GNU Lesser General Public License for more
// details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with Teensy_VS1053b. If not, see <https://www.gnu.org/licenses/>.

// This file is based on vs10xx_uc.h (v1.01) in vs1053an_playrec.zip by courtesy
// of VLSI Solution Oy, Tampere, Finland - https://vlsi.fi
//
// See "Example microcontroller code" at https://vlsi.fi/en/products/vs1053.html

#pragma once

// SCI opcodes
#define SCI_WRITE            0x02
#define SCI_READ             0x03

// SCI registers
#define SCI_MODE             0x00
#define SCI_STATUS           0x01
#define SCI_BASS             0x02
#define SCI_CLOCKF           0x03
#define SCI_DECODE_TIME      0x04
#define SCI_AUDATA           0x05
#define SCI_WRAM             0x06
#define SCI_WRAMADDR         0x07
#define SCI_HDAT0            0x08
#define SCI_HDAT1            0x09
#define SCI_AIADDR           0x0A
#define SCI_VOL              0x0B
#define SCI_AICTRL0          0x0C
#define SCI_AICTRL1          0x0D
#define SCI_AICTRL2          0x0E
#define SCI_AICTRL3          0x0F

// SCI_MODE
#define SM_DIFF_B               0
#define SM_LAYER12_B            1
#define SM_RESET_B              2
#define SM_CANCEL_B             3
#define SM_OUTOFWAV_B           3
#define SM_EARSPEAKER_LO_B      4
#define SM_TESTS_B              5
#define SM_STREAM_B             6
#define SM_EARSPEAKER_HI_B      7
#define SM_DACT_B               8
#define SM_SDIORD_B             9
#define SM_SDISHARE_B          10
#define SM_SDINEW_B            11
#define SM_ADPCM_B             12
#define SM_ADPCM_HP_B          13
#define SM_LINE1_B             14
#define SM_CLK_RANGE_B         15

#define SM_DIFF          (1 <<  0)
#define SM_LAYER12       (1 <<  1)
#define SM_RESET         (1 <<  2)
#define SM_CANCEL        (1 <<  3)
#define SM_EARSPEAKER_LO (1 <<  4)
#define SM_TESTS         (1 <<  5)
#define SM_STREAM        (1 <<  6)
#define SM_EARSPEAKER_HI (1 <<  7)
#define SM_DACT          (1 <<  8)
#define SM_SDIORD        (1 <<  9)
#define SM_SDISHARE      (1 << 10)
#define SM_SDINEW        (1 << 11)
#define SM_ADPCM         (1 << 12)
#define SM_LINE1         (1 << 14)
#define SM_CLK_RANGE     (1 << 15)

#define SM_EARSPEAKER_1103_BITS 2
#define SM_EARSPEAKER_1103_MASK 0x3000

// SCI_STATUS
#define SS_REFERENCE_SEL_B      0
#define SS_AD_CLOCK_B           1
#define SS_APDOWN1_B            2
#define SS_APDOWN2_B            3
#define SS_VER_B                4
#define SS_VUMETER_B            9 // cf. section 1.2 of vs1053b-patches.pdf
#define SS_VCM_DISABLE_B       10
#define SS_VCM_OVERLOAD_B      11
#define SS_SWING_B             12
#define SS_DO_NOT_JUMP_B       15

#define SS_REFERENCE_SEL (1 <<  0)
#define SS_AD_CLOCK      (1 <<  1)
#define SS_APDOWN1       (1 <<  2)
#define SS_APDOWN2       (1 <<  3)
#define SS_VER           (1 <<  4)
#define SS_VUMETER       (1 <<  9)
#define SS_VCM_DISABLE   (1 << 10)
#define SS_VCM_OVERLOAD  (1 << 11)
#define SS_SWING         (1 << 12)
#define SS_DO_NOT_JUMP   (1 << 15)

#define SS_SWING_BITS           3
#define SS_SWING_MASK      0x7000
#define SS_VER_BITS             4
#define SS_VER_MASK        0x00f0
#define SS_AVOL_BITS            2
#define SS_AVOL_MASK       0x0003

#define SS_VER_VS1001        0x00
#define SS_VER_VS1011        0x10
#define SS_VER_VS1002        0x20
#define SS_VER_VS1003        0x30
#define SS_VER_VS1053        0x40
#define SS_VER_VS8053        0x40
#define SS_VER_VS1033        0x50
#define SS_VER_VS1063        0x60
#define SS_VER_VS1103        0x70

// SCI_BASS
#define ST_AMPLITUDE_B         12
#define ST_FREQLIMIT_B          8
#define SB_AMPLITUDE_B          4
#define SB_FREQLIMIT_B          0

#define ST_AMPLITUDE     (1 << 12)
#define ST_FREQLIMIT     (1 <<  8)
#define SB_AMPLITUDE     (1 <<  4)
#define SB_FREQLIMIT     (1 <<  0)

#define ST_AMPLITUDE_BITS       4
#define ST_AMPLITUDE_MASK  0xf000
#define ST_FREQLIMIT_BITS       4
#define ST_FREQLIMIT_MASK  0x0f00
#define SB_AMPLITUDE_BITS       4
#define SB_AMPLITUDE_MASK  0x00f0
#define SB_FREQLIMIT_BITS       4
#define SB_FREQLIMIT_MASK  0x000f

// SCI_CLOCKF bits
#define SC_MULT_B              13
#define SC_ADD_B               11
#define SC_FREQ_B               0

#define SC_MULT          (1 << 13)
#define SC_ADD           (1 << 11)
#define SC_FREQ          (1 <<  0)

#define SC_MULT_BITS            3
#define SC_ADD_BITS             2
#define SC_FREQ_BITS           11

#define SC_MULT_MASK       0xe000
#define SC_ADD_MASK        0x1800
#define SC_FREQ_MASK       0x07ff

#define SC_MULT_53_10X     0x0000
#define SC_MULT_53_20X     0x2000
#define SC_MULT_53_25X     0x4000
#define SC_MULT_53_30X     0x6000
#define SC_MULT_53_35X     0x8000
#define SC_MULT_53_40X     0xa000
#define SC_MULT_53_45X     0xc000
#define SC_MULT_53_50X     0xe000

#define SC_ADD_53_00X      0x0000
#define SC_ADD_53_10X      0x0800
#define SC_ADD_53_15X      0x1000
#define SC_ADD_53_20X      0x1800

// SCI_WRAMADDR
#define SCI_WRAM_X_START   0x0000
#define SCI_WRAM_Y_START   0x4000
#define SCI_WRAM_I_START   0x8000
#define SCI_WRAM_IO_START  0xC000

#define SCI_WRAM_X_OFFSET  0x0000
#define SCI_WRAM_Y_OFFSET  0x4000
#define SCI_WRAM_I_OFFSET  0x8000
#define SCI_WRAM_IO_OFFSET 0x0000  // I/O addresses are @0xC000 -> no offset

// SCI_VOL
#define SV_LEFT_B               8
#define SV_RIGHT_B              0

#define SV_LEFT           (1 << 8)
#define SV_RIGHT          (1 << 0)

#define SV_LEFT_BITS            8
#define SV_LEFT_MASK       0xFF00
#define SV_RIGHT_BITS           8
#define SV_RIGHT_MASK      0x00FF

// Extra Parameters (cf. section 10.11 of vs1053.pdf)
#define PAR_CHIP_ID        0x1e00 // 32 bits
#define PAR_VERSION        0x1e02
#define PAR_CONFIG1        0x1e03
#define PAR_PLAY_SPEED     0x1e04
#define PAR_BYTERATE       0x1e05
#define PAR_END_FILL_BYTE  0x1e06
#define PAR_PLAY_MODE      0x1e09 // cf. section 1.6 of vs1053b-patches.pdf
#define PAR_JUMP_POINTS    0x1e16
#define PAR_LATEST_JUMP    0x1e26
#define PAR_POSITION_MSEC  0x1e27 // 32 bits
#define PAR_RESYNC         0x1e29

#define PAR_RESERVED_00    0x1e07
#define PAR_RESERVED_01    0x1e08
#define PAR_RESERVED_02    0x1e09 // PAR_PLAY_MODE, see above
#define PAR_RESERVED_03    0x1e0a
#define PAR_RESERVED_04    0x1e0b
#define PAR_RESERVED_05    0x1e0c
#define PAR_RESERVED_06    0x1e0d
#define PAR_RESERVED_07    0x1e0e
#define PAR_RESERVED_08    0x1e0f
#define PAR_RESERVED_09    0x1e10
#define PAR_RESERVED_10    0x1e11
#define PAR_RESERVED_11    0x1e12
#define PAR_RESERVED_12    0x1e13
#define PAR_RESERVED_13    0x1e14
#define PAR_RESERVED_14    0x1e15

#define PAR_PLAY_MODE_RESAMPLER_48KHZ_ONLY_B     11
#define PAR_PLAY_MODE_DUAL_CHANNEL_SEL_B         10
#define PAR_PLAY_MODE_DUAL_CHANNEL_B              9
#define PAR_PLAY_MODE_RESAMPLER_TEST_B            8
#define PAR_PLAY_MODE_RESAMPLER_ON_B              7
#define PAR_PLAY_MODE_PAUSE_ON_B                  1
#define PAR_PLAY_MODE_MONO_OUTPUT_B               0

#define PAR_PLAY_MODE_RESAMPLER_48KHZ_ONLY (1 << 11)
#define PAR_PLAY_MODE_DUAL_CHANNEL_SEL     (1 << 10)
#define PAR_PLAY_MODE_DUAL_CHANNEL         (1 <<  9)
#define PAR_PLAY_MODE_RESAMPLER_TEST       (1 <<  8)
#define PAR_PLAY_MODE_RESAMPLER_ON         (1 <<  7)
#define PAR_PLAY_MODE_PAUSE_ON             (1 <<  1)
#define PAR_PLAY_MODE_MONO_OUTPUT          (1 <<  0)

// GPIO
#define GPIO_DDR           0xc017
#define GPIO_IDATA         0xc018
#define GPIO_ODATA         0xc019

#define DREQ_ADDRESS       0x5b17
