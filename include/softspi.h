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

class softspi {

  public:
    constexpr softspi(uint8_t pinMOSI, uint8_t pinMISO, uint8_t pinCLK, uint32_t delay)
      : _pinMOSI(pinMOSI)
      , _pinMISO(pinMISO)
      , _pinCLK(pinCLK)
      , _delay(delay)
    {
      pinMode(_pinMOSI, OUTPUT);
      pinMode(_pinCLK,  OUTPUT);
      pinMode(_pinMISO, INPUT_PULLUP);
      digitalWriteFast(_pinMOSI, HIGH);
    }
    uint16_t transfer16(uint16_t);
    uint32_t transfer32(uint32_t);

  private:
    const uint8_t _pinMOSI, _pinMISO, _pinCLK;
    const uint32_t _delay;
};
