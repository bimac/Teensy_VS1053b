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

// inline void softspi::transfer(const void *buf, void *retbuf, size_t count) {
//   // return SPI.transfer(buf, retbuf, count);
// }

uint16_t softspi::transfer16(uint16_t data) {
  noInterrupts();
  for (int8_t i = 15; i > -1; i--) {
    digitalWriteFast(_pinMOSI, bitRead(data, i));
    digitalWriteFast(_pinCLK, HIGH);
    delayNanoseconds(_delay);
    bitWrite(data, i, digitalReadFast(_pinMISO));
    digitalWriteFast(_pinCLK, LOW);
    delayNanoseconds(_delay);
  }
  digitalWriteFast(_pinMOSI, HIGH);
  interrupts();
  return data;
}

uint32_t softspi::transfer32(uint32_t data) {
  noInterrupts();
  for (int8_t i = 31; i > -1; i--) {
    digitalWriteFast(_pinMOSI, bitRead(data, i));
    digitalWriteFast(_pinCLK, HIGH);
    delayNanoseconds(_delay);
    bitWrite(data, i, digitalReadFast(_pinMISO));
    digitalWriteFast(_pinCLK, LOW);
    delayNanoseconds(_delay);
  }
  digitalWriteFast(_pinMOSI, HIGH);
  interrupts();
  return data;
}
