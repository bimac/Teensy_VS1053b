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
#include <Arduino.h>

class softSPI {
  public:
    constexpr softSPI(const uint8_t pinMOSI, const uint8_t pinMISO, const uint8_t pinCLK)
      : _pinMOSI(pinMOSI)
      , _pinMISO(pinMISO)
      , _pinCLK(pinCLK)
      , _delay(0)
    {
      pinMode(_pinMOSI, OUTPUT);
      pinMode(_pinCLK, OUTPUT);
      pinMode(_pinMISO, INPUT_PULLUP);
      digitalWriteFast(_pinMOSI, HIGH);
    }
    constexpr softSPI(const uint8_t pinMOSI, const uint8_t pinMISO, const uint8_t pinCLK, const uint32_t delay)
      : _pinMOSI(pinMOSI)
      , _pinMISO(pinMISO)
      , _pinCLK(pinCLK)
      , _delay(delay)
    {
      pinMode(_pinMOSI, OUTPUT);
      pinMode(_pinCLK, OUTPUT);
      pinMode(_pinMISO, INPUT_PULLUP);
      digitalWriteFast(_pinMOSI, HIGH);
    }
    void transfer(const void *, void *, size_t);
    uint8_t transfer(uint8_t);
    uint16_t transfer16(uint16_t);
    uint32_t transfer32(uint32_t);

private:
    const uint8_t _pinMOSI, _pinMISO, _pinCLK;
    const uint32_t _delay;
};
