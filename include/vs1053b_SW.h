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
#include "vs1053b_base.h"

template <uint8_t pinReset, uint8_t pinCS, uint8_t pinDCS, uint8_t pinDREQ, uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinCLK>
struct VS1053b_SW : public VS1053b_Base<pinReset, pinCS, pinDCS, pinDREQ> {

  // constructors
  VS1053b_SW(uint32_t maxClock = 14E6) : VS1053b_Base<pinReset, pinCS, pinDCS, pinDREQ>(maxClock) {
    pinMode(pinMOSI, OUTPUT);
    pinMode(pinCLK, OUTPUT);
    pinMode(pinMISO, INPUT_PULLUP);
    digitalWriteFast(pinMOSI, HIGH);
  };

  private:
  uint32_t _delay = 0;

  void initSPI() {}

  inline void transfer(const void *buf, void *retbuf, size_t count) {

  }

  inline uint16_t transfer16(uint16_t data) {
    noInterrupts();
    for (int8_t i = 15; i > -1; i--) {
      digitalWriteFast(pinMOSI, bitRead(data, i));
      digitalWriteFast(pinCLK, HIGH);
      if (_delay > 0)
        delayNanoseconds(_delay);
      bitWrite(data, i, digitalReadFast(pinMISO));
      digitalWriteFast(pinCLK, LOW);
      if (_delay > 0)
        delayNanoseconds(_delay);
    }
    digitalWriteFast(pinMOSI, HIGH);
    interrupts();
    return data;
  }

  inline uint16_t transfer16(uint8_t byte1, uint8_t byte0) {
    return 0;
  }

  inline uint32_t transfer32(uint32_t data) {
    noInterrupts();
    for (int8_t i = 31; i > -1; i--) {
      digitalWriteFast(pinMOSI, bitRead(data, i));
      digitalWriteFast(pinCLK, HIGH);
      if (_delay > 0)
        delayNanoseconds(_delay);
      bitWrite(data, i, digitalReadFast(pinMISO));
      digitalWriteFast(pinCLK, LOW);
      if (_delay > 0)
        delayNanoseconds(_delay);
    }
    digitalWriteFast(pinMOSI, HIGH);
    interrupts();
    return data;
  }

  inline void beginTransaction(const uint32_t clock) {
    _delay = 5E8 / clock;
  }

  inline void endTransaction() { }

  // SPI wrappers
  // void transfer(const void *buf, void *retbuf, size_t count);
  // uint16_t transfer16(uint16_t data);
  // uint32_t transfer32(uint32_t data);
};
