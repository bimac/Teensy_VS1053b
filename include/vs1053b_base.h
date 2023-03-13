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

#if (!defined(VS1053B___INCLUDE_HW) && !defined(VS1053B___INCLUDE_SW))
#error Include either "vs1053b_HW.h" or "vs1053b_SW.h" - not "vs1053b_base.h".
#endif

#if (!defined(__arm__) || !defined(TEENSYDUINO))
#error This library is designed for Teensy 3.x / 4.x boards.
#endif

#if __has_include("vs1053b-patches.plg")
#define VS1053B___PATCHES_PLG_FOUND
#endif

#include <Arduino.h>
#include <SD.h>
#include "registers.h"

#define VS1053B___INIT_FAIL_SPI_COMM   1
#define VS1053B___INIT_FAIL_UNKNOWN_IC 2
#define VS1053B___INIT_FAIL_CLK_RAISE  3
#define VS1053B___INIT_FAIL_SD_CARD    4

template <uint8_t pinReset, uint8_t pinCS, uint8_t pinDCS, uint8_t pinDREQ>
struct VS1053b_Base {

  // --- VARIOUS VARIABLES -----------------------------------------------------
  private:
  static constexpr uint32_t _XTALI = 12288E3;         // clock frequency [Hz]
  static constexpr uint32_t _dXTALIns = 1E9 / _XTALI; // clock interval [ns]
  static constexpr uint32_t _CLKI = _XTALI * 4.5;
  static constexpr uint16_t _Hz2SC(uint32_t Hz) { return (Hz - 8E6) / 4E3; }
  const uint32_t _maxClock;

  protected:
  uint32_t _clockW;
  uint32_t _clockR;


  // --- CONSTRUCTOR (ONLY FOR DESCENDANTS) ------------------------------------
  protected:

  VS1053b_Base(uint32_t maxClock)
    : _maxClock(maxClock)
    , _clockW(min(_CLKI / 4, _maxClock))
    , _clockR(min(_CLKI / 7, _maxClock)) {}


  // --- INITIALIZATION ROUTINE  -----------------------------------------------
  public:

  uint8_t begin(void) {
    // configure pins
    ::pinMode(pinReset, OUTPUT);
    ::pinMode(pinCS, OUTPUT);
    ::pinMode(pinDCS, OUTPUT);
    ::pinMode(pinDREQ, INPUT);
    digitalWriteFast(pinCS, HIGH);
    digitalWriteFast(pinDCS, HIGH);

    // initialize VS1053b
    initSPI();         // initialize SPI
    resetHW();         // hardware reset to set all output pins
    readSci(SCI_MODE); // dummy read to bring SCI bus to a known state
    resetSW();         // software reset (without changing clock)

    // check if SPI communication works
    if (connectionTest())
      return VS1053B___INIT_FAIL_SPI_COMM;

    // check if we're actually talking to a VS1053b
    if ((readSci(SCI_STATUS) & SS_VER_MASK) != SS_VER_VS1053)
      return VS1053B___INIT_FAIL_UNKNOWN_IC;

    // set clock multiplier & adjust speed of SPI bus
    if (setClock())
      return VS1053B___INIT_FAIL_CLK_RAISE;

    // if vs1053b-patches.plg has been included: load it
#if defined(VS1053B___PATCHES_PLG_FOUND)
    loadPatch(plugin, sizeof(plugin) / sizeof(plugin[0]));
#endif

    return false;
  }


  // --- PLAYBACK --------------------------------------------------------------
  public:

  uint8_t playFile(const char *filename) {
    return 0; // TODO
  }


  // --- GPIO CONTROL ----------------------------------------------------------
  public:

  void pinMode(uint8_t pin, uint8_t mode) {
    if (pin > 7)
      return;
    uint16_t ddr = readWRAM16(GPIO_DDR);
    bitWrite(ddr, pin, mode);
    writeWRAM16(GPIO_DDR, ddr);
  }

  void digitalWrite(uint8_t pin, uint8_t val) {
    if (pin > 7)
      return;
    uint16_t odata = readWRAM16(GPIO_ODATA);
    bitWrite(odata, pin, val);
    writeWRAM16(GPIO_ODATA, odata);
  }

  uint8_t digitalRead(uint8_t pin) {
    if (pin > 7)
      return 0;
    uint16_t idata = readWRAM16(GPIO_IDATA);
    return bitRead(idata, pin);
  }


  // --- VOLUME CONTROL --------------------------------------------------------
  public:

  void volume(uint8_t value) {
    volume(value, value);
  }

  void volume(uint8_t left, uint8_t right) {
    writeWRAM16(SCI_VOL, ((uint16_t)left << 8) | right);
  }


  // --- STREAM & AUDIO BUFFERS ------------------------------------------------
  //
  // see http://www.vsdsp-forum.com/phpbb/viewtopic.php?p=6679#p6679
  public:

  int16_t streamBufferFillWords(void) {
    int16_t bufSize = (readSci(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
    uint16_t wrp = readWRAM16(0x5A7D);
    uint16_t rdp = readSci(SCI_WRAM);
    int16_t res = wrp - rdp;
    if (res < 0)
      return res + bufSize;
    return res;
  }

  int16_t streamBufferFreeWords(void) {
    int16_t bufSize = (readSci(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
    int16_t res = bufSize - streamBufferFillWords();
    if (res < 2)
      return 0;
    return res - 2;
  }

  int16_t audioBufferFillWords(void) {
    uint16_t wrp = readWRAM16(0x5A80);
    uint16_t rdp = readSci(SCI_WRAM);
    return (wrp - rdp) & 4095;
  }

  int16_t audioBufferFreeWords(void) {
    int16_t res = 4096 - audioBufferFillWords();
    if (res < 2)
      return 0;
    return res - 2;
  }

  uint16_t audioBufferUnderflow(void) {
    uint16_t uFlow = readWRAM16(0x5A82);
    if (uFlow)
      writeWRAM16(0x5A82, 0);
    return uFlow;
  }

  // TODO: fill buffer function
  // http://www.vsdsp-forum.com/phpbb/viewtopic.php?p=6977#p6977


  // --- handling of DREQ ------------------------------------------------------
  public:
  inline __attribute__((always_inline)) void waitForDREQ(bool state = HIGH) {
    while (digitalReadFast(pinDREQ) != state) { yield(); }
  }


  // --- reset functions, clock set & tests ------------------------------------
  private:

  void resetHW(void) {
    digitalWriteFast(pinReset, LOW);
    digitalWriteFast(pinCS, HIGH);
    digitalWriteFast(pinDCS, HIGH);
    delayNanoseconds(2 * _dXTALIns);
    digitalWriteFast(pinReset, HIGH);
    _clockR = min(_XTALI / 7, _maxClock);
    _clockW = min(_XTALI / 4, _maxClock);
    waitForDREQ(HIGH);
  }

  void resetSW(void) {
    writeSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);
    waitForDREQ(HIGH);
  }

  bool setClock() {
    writeSci(SCI_CLOCKF, _Hz2SC(_XTALI) | SC_MULT_53_45X);
    _clockR = min(_CLKI / 7, _maxClock);
    _clockW = min(_CLKI / 4, _maxClock);
    waitForDREQ(HIGH);
    if (readbackTest())
      return true;
    return false;
  }

  bool connectionTest() {
    uint16_t status = readSci(SCI_STATUS);
    if (status == 0 || status == 0xFFFF)
      return true;
    return readbackTest();
  }

  bool readbackTest(void) {
    writeSci(SCI_AICTRL1, 0xABAD);
    if (readSci(SCI_AICTRL1) != 0xABAD)
      return true;
    writeSci(SCI_AICTRL2, 0x7E57);
    if (readSci(SCI_AICTRL2) != 0x7E57)
      return true;
    writeSci(SCI_AICTRL1, 0);
    writeSci(SCI_AICTRL2, 0);
    return false;
  }


  // --- loading of patches ----------------------------------------------------
  public:

  void loadPatch(const uint16_t *patch, const uint16_t size) {
    uint8_t addr;
    uint16_t n, val, i = 0;
    beginTransaction(_clockW);
    while (i < size) {
      addr = patch[i++];
      n = patch[i++];

      // RLE run, replicate n samples
      if (n & 0x8000U) {
        n &= 0x7FFF;
        val = patch[i++];

        // write address and, if possible, first value
        waitForDREQ(HIGH);
        digitalWriteFast(pinCS, LOW);
        if (n == 0) {
          transfer16(SCI_WRITE, addr);
        } else {
          transfer32(SCI_WRITE, addr, val);
          n--;
        }
        waitForDREQ(LOW);

        // write remaining values to same address
        while (n--) {
          waitForDREQ(HIGH);
          transfer16(val);
          waitForDREQ(LOW);
        }
        digitalWriteFast(pinCS, HIGH);

        // copy run, copy n samples
      } else {

        // write address and, if possible, first value
        waitForDREQ(HIGH);
        digitalWriteFast(pinCS, LOW);
        if (n == 0) {
          transfer16(SCI_WRITE, addr);
        } else {
          transfer32(SCI_WRITE, addr, patch[i++]);
          n--;
        }
        waitForDREQ(LOW);

        // write remaining values to same address
        while (n--) {
          waitForDREQ(HIGH);
          transfer16(patch[i++]);
          waitForDREQ(LOW);
        }
        digitalWriteFast(pinCS, HIGH);
      }
    }
    endTransaction();
  }


  // --- READING AND WRITING OF SCI_WRAM -----------------------------------------
  public:

  // write 16-bit value to given address
  void writeWRAM16(uint16_t addr, uint16_t data) {
    writeSci(SCI_WRAMADDR, addr);
    writeSci(SCI_WRAM, data);
  }

  // write 32-bit value to given address
  void writeWRAM32(uint16_t addr, uint32_t data) {
    writeSci(SCI_WRAMADDR, addr);
    writeSci(SCI_WRAM, (data >> 16) & 0xFFFFUL);
    writeSci(SCI_WRAM, data & 0xFFFFUL);
  }

  // read 16-bit value from address
  uint16_t readWRAM16(uint16_t addr) {
    writeSci(SCI_WRAMADDR, addr);
    return readSci(SCI_WRAM);
  }

  // read 32-bit non-changing value from address
  uint32_t readWRAM32(uint16_t addr) {
    uint16_t lsb;
    writeSci(SCI_WRAMADDR, addr);
    lsb = readSci(SCI_WRAM);
    return lsb | ((uint32_t)readSci(SCI_WRAM) << 16);
  }

  // read 32-bit increasing counter value from address
  uint32_t readWRAM32Counter(uint16_t addr) {
    uint16_t msbV1, lsb, msbV2;

    msbV1 = readWRAM16(addr + 1);
    lsb = readWRAM16(addr);
    msbV2 = readSci(SCI_WRAM);
    if (lsb < 0x8000U) {
      msbV1 = msbV2;
    }
    return ((uint32_t)msbV1 << 16) | lsb;
  }


  // --- SCI & SDI OPERATIONS ----------------------------------------------------
  public:

  FASTRUN
  void writeSci(uint8_t addr, uint16_t data) {
    beginTransaction(_clockW);
    waitForDREQ(HIGH);
    digitalWriteFast(pinCS, LOW);
    transfer32(SCI_WRITE, addr, data);
    digitalWriteFast(pinCS, HIGH);
    endTransaction();
    waitForDREQ(LOW);
  }

  FASTRUN
  uint16_t readSci(uint8_t addr) {
    uint16_t data;
    beginTransaction(_clockR);
    waitForDREQ(HIGH);
    digitalWriteFast(pinCS, LOW);
    data = transfer32(SCI_READ, addr, 0x0) & 0xFFFFUL;
    digitalWriteFast(pinCS, HIGH);
    endTransaction();
    waitForDREQ(LOW);
    return data;
  }

  FASTRUN
  void writeSdi(const uint8_t *data, size_t bytes) {
    if (bytes > 32 || bytes == 0)
      return;
    beginTransaction(_clockW);
    waitForDREQ(HIGH);
    digitalWriteFast(pinDCS, LOW);
    transfer(data, nullptr, bytes);
    digitalWriteFast(pinDCS, HIGH);
    endTransaction();
    waitForDREQ(LOW);
  }


  // --- SPI: pure virtual methods to be implemented by descendants ------------
  protected:
  virtual void initSPI() = 0;
  virtual void beginTransaction(uint32_t clock) = 0;
  virtual void endTransaction() = 0;
  virtual void transfer(const void *, void *, size_t) = 0;
  virtual uint16_t transfer16(uint16_t) = 0;
  virtual uint32_t transfer32(uint32_t) = 0;


  // --- SPI: helper functions -------------------------------------------------
  protected:
  uint16_t transfer16(uint8_t byte1, uint8_t byte0) {
    return transfer16(((uint16_t)byte1 << 8) | byte0);
  }
  virtual uint32_t transfer32(uint8_t byte3, uint8_t byte2, uint16_t word0) {
    return transfer32(((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) | word0);
  }
};
