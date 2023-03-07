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
// along with Foobar. If not, see <https://www.gnu.org/licenses/>.

#pragma once
#include <SD.h>
#include <SPI.h>

#define VS1053B___INIT_FAIL_SPI_COMM 1
#define VS1053B___INIT_FAIL_UNKNOWN_IC 2
#define VS1053B___INIT_FAIL_CLK_RAISE 3
#define VS1053B___INIT_FAIL_SD_CARD 4
#define VS1053B___INIT_FAIL_PATCH 4

class VS1053b {

  public:

  VS1053b(uint8_t, uint8_t, uint8_t, uint8_t, uint8_t);
  uint8_t begin(void);
  uint8_t begin(const uint32_t);
  bool setClock(const uint32_t);
  uint8_t playFile(const char *);
  void resetHW(void);
  void resetSW(void);

  void volume(uint8_t);
  void volume(uint8_t, uint8_t);

  // methods for assessing state of buffers
  // see http://www.vsdsp-forum.com/phpbb/viewtopic.php?p=6679#p6679
  int16_t streamBufferFillWords(void);
  int16_t streamBufferFreeWords(void);
  int16_t audioBufferFillWords(void);
  int16_t audioBufferFreeWords(void);
  uint16_t audioBufferUnderflow(void);

  bool isPatched(void);
  bool loadPatch(const uint16_t *, uint16_t);


  private:

  const uint8_t _pinReset, _pinCS, _pinDCS, _pinDREQ, _pinSDCS, _irqDREQ;
  const uint32_t _XTALI = 12288E3;
  uint32_t _CLKI = _XTALI;
  IntervalTimer * _timer;
  uint32_t _maxClock = _XTALI / 7;

  SPISettings _SPIConfR;
  SPISettings _SPIConfW;

  bool connectionTest(void);
  bool readbackTest(void);


  public:

  void pinMode(uint8_t, uint8_t);
  void digitalWrite(uint8_t, uint8_t);
  uint8_t digitalRead(uint8_t);

  void writeWRAM16(uint16_t addr, uint16_t data);
  void writeWRAM32(uint16_t addr, uint32_t data);
  uint16_t readWRAM16(uint16_t addr);
  uint32_t readWRAM32(uint16_t addr);
  uint32_t readWRAM32Counter(uint16_t addr);

  // SCI & SDI operations
  void writeSci(uint8_t, uint16_t);
  inline void writeSciMultiple(uint16_t); // see section 7.4.3 of data-sheet
  uint16_t readSci(uint8_t);
  void writeSdi(const uint8_t *, size_t);

  private:

  // handling of DREQ
  inline bool getDREQ(void);
  inline void waitForDREQ(void);

  // SPI helpers
  inline void beginTransaction(SPISettings, uint8_t);
  inline void endTransaction(uint8_t);
  inline uint16_t transfer16(uint8_t, uint8_t);
  inline uint32_t transfer32(uint8_t, uint8_t, uint16_t);

  // SPI wrappers
  inline void transfer(const void *, void *, size_t);
  inline uint16_t transfer16(uint16_t);
  inline uint32_t transfer32(uint32_t);

  // fill buffer function
  // http://www.vsdsp-forum.com/phpbb/viewtopic.php?p=6977#p6977
};
