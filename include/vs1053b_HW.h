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

#pragma once

#define VS1053B___INCLUDE_HW
#include "vs1053b_base.h"
#include <SPI.h>

template <uint8_t pinReset, uint8_t pinCS, uint8_t pinDCS, uint8_t pinDREQ>
struct VS1053b_HW : public VS1053b_Base<pinReset, pinCS, pinDCS, pinDREQ> {

  // constructors
  VS1053b_HW(uint32_t maxClock = 14E6)
      : VS1053b_Base<pinReset, pinCS, pinDCS, pinDREQ>(maxClock) {}

private:
  const SPISettings _SPIConfR = SPISettings(this->_clockR, MSBFIRST, SPI_MODE0);
  const SPISettings _SPIConfW = SPISettings(this->_clockW, MSBFIRST, SPI_MODE0);

  void initSPI() override { SPI.begin(); }

  inline void transfer(const void *buf, void *retbuf, size_t count) override {
    return SPI.transfer(buf, retbuf, count);
  }

  inline uint16_t transfer16(uint16_t data) override {
    return SPI.transfer16(data);
  }

  inline uint16_t transfer16(uint8_t byte1, uint8_t byte0) {
    return SPI.transfer16(((uint16_t)byte1 << 8) | byte0);
  }

  inline uint32_t transfer32(uint32_t data) override {
#if defined(__IMXRT1052__) || defined(__IMXRT1062__)
    return SPI.transfer32(data);
#else
    uint16_t word1 = transfer16((data >> 16) & 0xFFFFUL);
    uint16_t word0 = transfer16(data & 0xFFFFUL);
    return ((uint32_t)word1 << 16) + word0;
#endif
  }

  inline uint32_t transfer32(uint8_t byte3, uint8_t byte2,
                             uint16_t word0) override {
#if (defined(__IMXRT1052__) || defined(__IMXRT1062__))
    return SPI.transfer32(((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) |
                          word0);
#else
    uint16_t out1 = SPI.transfer16(byte3, byte2);
    uint16_t out0 = SPI.transfer16(word0);
    return ((uint32_t)out1 << 16) + out0;
#endif
  }

  inline void beginTransaction(const uint32_t clock) override {
    SPI.beginTransaction(SPISettings(clock, MSBFIRST, SPI_MODE0));
  }

  inline void endTransaction() override { SPI.endTransaction(); }
};
