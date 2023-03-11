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

#include "softspi.h"

FASTRUN
uint8_t softSPI::transfer(uint8_t data) {
  noInterrupts();
  for (int8_t i = 7; i > -1; i--) {
    digitalWriteFast(_pinMOSI, bitRead(data, i));
    digitalWriteFast(_pinCLK, HIGH);
    if (_delay > 0)
      delayNanoseconds(_delay);
    bitWrite(data, i, digitalReadFast(_pinMISO));
    digitalWriteFast(_pinCLK, LOW);
    if (_delay > 0)
      delayNanoseconds(_delay);
  }
  digitalWriteFast(_pinMOSI, HIGH);
  interrupts();
  return data;
}

FASTRUN
void softSPI::transfer(const void *buf, void *retbuf, size_t count) {

}

FASTRUN
uint16_t softSPI::transfer16(uint16_t data) {
  noInterrupts();
  for (int8_t i = 15; i > -1; i--) {
    digitalWriteFast(_pinMOSI, bitRead(data, i));
    digitalWriteFast(_pinCLK, HIGH);
    if (_delay > 0)
      delayNanoseconds(_delay);
    bitWrite(data, i, digitalReadFast(_pinMISO));
    digitalWriteFast(_pinCLK, LOW);
    if (_delay > 0)
      delayNanoseconds(_delay);
  }
  digitalWriteFast(_pinMOSI, HIGH);
  interrupts();
  return data;
}

FASTRUN
uint32_t softSPI::transfer32(uint32_t data) {
  noInterrupts();
  for (int8_t i = 31; i > -1; i--) {
    digitalWriteFast(_pinMOSI, bitRead(data, i));
    digitalWriteFast(_pinCLK, HIGH);
    if (_delay > 0)
      delayNanoseconds(_delay);
    bitWrite(data, i, digitalReadFast(_pinMISO));
    digitalWriteFast(_pinCLK, LOW);
    if (_delay > 0)
      delayNanoseconds(_delay);
  }
  digitalWriteFast(_pinMOSI, HIGH);
  interrupts();
  return data;
}
