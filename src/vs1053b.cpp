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

#if (!defined(__arm__) && defined(TEENSYDUINO))
#error THIS LIBRARY IS DESIGNED FOR TEENSY 3.X / 4.X BOARDS
#endif

#include "vs1053b.h"
#include "registers.h"

namespace {
constexpr uint16_t Hz2SC_FREQ(uint32_t Hz) { return (Hz - 8E6) / 4E3; }
} // namespace

VS1053b::VS1053b(uint8_t pinReset, uint8_t pinCS, uint8_t pinDCS, uint8_t pinDREQ, uint8_t pinSDCS)
    : _pinReset(pinReset), _pinCS(pinCS), _pinDCS(pinDCS), _pinDREQ(pinDREQ), _pinSDCS(pinSDCS), _irqDREQ(digitalPinToInterrupt(pinDREQ)) {

  // configure pins
  ::pinMode(_pinReset, OUTPUT);
  ::pinMode(_pinCS, OUTPUT);
  ::pinMode(_pinDCS, OUTPUT);
  ::pinMode(_pinDREQ, INPUT);
  digitalWriteFast(_pinCS, HIGH);
  digitalWriteFast(_pinDCS, HIGH);

  // register interrupts with SPI
  // SPI.usingInterrupt(_irqDREQ);
}

uint8_t VS1053b::begin(void) {
  return begin(_maxClock);
}

uint8_t VS1053b::begin(const uint32_t maxClock) {

  _SPIConfR = SPISettings(min(_XTALI / 7, maxClock), MSBFIRST, SPI_MODE0);
  _SPIConfW = SPISettings(min(_XTALI / 4, maxClock), MSBFIRST, SPI_MODE0);

  // initialize VS1053b
  resetHW();         // hardware reset to set all output pins
  readSci(SCI_MODE); // dummy read to bring SCI bus to a known state
  resetSW();         // software reset

  // check if SPI communication works
  if (connectionTest() || readbackTest())
    return VS1053B___INIT_FAIL_SPI_COMM;

  // check if we're actually talking to a VS1053b
  if ((readSci(SCI_STATUS) & SS_VER_MASK) != SS_VER_VS1053)
    return VS1053B___INIT_FAIL_UNKNOWN_IC;

  // set clock multiplier & adjust speed of SPI bus
  if (setClock(maxClock))
    return VS1053B___INIT_FAIL_CLK_RAISE;

  // store clock setting
  _maxClock = maxClock;

  // we're all set
  return 0;
}

bool VS1053b::setClock(const uint32_t maxClock) {
  uint16_t multiplierBits;

  if        (maxClock > (_XTALI / 7 * 4.0)) {
    multiplierBits = SC_MULT_53_45X;
    _CLKI = _XTALI * 4.5;
  } else if (maxClock > (_XTALI / 7 * 3.5)) {
    multiplierBits = SC_MULT_53_40X;
    _CLKI = _XTALI * 4.0;
  } else if (maxClock > (_XTALI / 7 * 3.0)) {
    multiplierBits = SC_MULT_53_35X;
    _CLKI = _XTALI * 3.5;
  } else if (maxClock > (_XTALI / 7 * 2.5)) {
    multiplierBits = SC_MULT_53_30X;
    _CLKI = _XTALI * 3.0;
  } else if (maxClock > (_XTALI / 7 * 2.0)) {
    multiplierBits = SC_MULT_53_25X;
    _CLKI = _XTALI * 2.5;
  } else if (maxClock > (_XTALI / 7 * 1.0)) {
    multiplierBits = SC_MULT_53_20X;
    _CLKI = _XTALI * 2.0;
  } else {
    multiplierBits = SC_MULT_53_10X;
    _CLKI = _XTALI;
  }

  writeSci(SCI_CLOCKF, Hz2SC_FREQ(_XTALI) | multiplierBits);
  _SPIConfR = SPISettings(min(_CLKI / 7, maxClock), MSBFIRST, SPI_MODE0);
  _SPIConfW = SPISettings(min(_CLKI / 4, maxClock), MSBFIRST, SPI_MODE0);
  waitForDREQ();

  return readbackTest();
}

uint8_t VS1053b::playFile(const char *filename) {
  return 0; // TODO
}

void VS1053b::resetHW(void) {
  digitalWriteFast(_pinCS, HIGH);
  digitalWriteFast(_pinDCS, HIGH);
  digitalWriteFast(_pinReset, LOW);
  delay(1);
  digitalWriteFast(_pinReset, HIGH);
  waitForDREQ();
}

void VS1053b::resetSW(void) {
  writeSci(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);
  waitForDREQ();
}

void VS1053b::volume(uint8_t value) {
  volume(value, value);
}

void VS1053b::volume(uint8_t left, uint8_t right) {
  writeWRAM16(SCI_VOL, ((uint16_t)left << 8) | right);
}

int16_t VS1053b::streamBufferFillWords(void) {
  int16_t bufSize = (readSci(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
  uint16_t wrp = readWRAM16(0x5A7D);
  uint16_t rdp = readSci(SCI_WRAM);
  int16_t res = wrp - rdp;
  if (res < 0)
    return res + bufSize;
  return res;
}

int16_t VS1053b::streamBufferFreeWords(void) {
  int16_t bufSize = (readSci(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
  int16_t res = bufSize - streamBufferFillWords();
  if (res < 2)
    return 0;
  return res - 2;
}

int16_t VS1053b::audioBufferFillWords(void) {
  uint16_t wrp = readWRAM16(0x5A80);
  uint16_t rdp = readSci(SCI_WRAM);
  return (wrp - rdp) & 4095;
}

int16_t VS1053b::audioBufferFreeWords(void) {
  int16_t res = 4096 - audioBufferFillWords();
  if (res < 2)
    return 0;
  return res - 2;
}

//  Function returns:
//  - 0 if there have been no audio buffer overflows since last call
//  - non-0 if there has been at least one audio buffer overflow since last call
//
//  Before calling this function for the 1st time, you have to do the following:
//  1) Load and start the VS1053b Patches set 2.20 or newer, from
//     http://www.vlsi.fi/en/support/software/vs10xxpatches.html
//  2) After the Patches set has started, write 0x60 to register SCI_AIADDR
uint16_t VS1053b::audioBufferUnderflow(void) {
  uint16_t uFlow = readWRAM16(0x5A82);
  if (uFlow)
    writeWRAM16(0x5A82, 0);
  return uFlow;
}

bool VS1053b::connectionTest(void) {
  uint16_t status = readSci(SCI_STATUS);
  return (status == 0 || status == 0xFFFF);
}

bool VS1053b::readbackTest(void) {
  writeSci(SCI_AICTRL1, 0xABAD);
  writeSci(SCI_AICTRL2, 0x7E57);
  if (readSci(SCI_AICTRL1) != 0xABAD || readSci(SCI_AICTRL2) != 0x7E57)
    return true;
  writeSci(SCI_AICTRL1, 0);
  writeSci(SCI_AICTRL2, 0);
  return false;
}

bool VS1053b::loadPlugin(const uint16_t *d, uint16_t len) {
  uint8_t addr, n, val;
  uint16_t i = 0;
  while (i < len) {
    addr = d[i++];
    n = d[i++];
    beginTransaction(_SPIConfW, _pinCS);
    transfer16(SCI_WRITE, addr);
    if (n & 0x8000U) {
      n &= 0x7FFF;
      val = d[i++];
      while (n--)
        writeSciMultiple(val);
    } else {
      while (n--) {
        val = d[i++];
        writeSciMultiple(val);
        i++;
      }
    }
    endTransaction(_pinCS); // end transaction
  }
  return !isPatched();
}

bool VS1053b::isPatched(void) {
  return readSci(SCI_AIADDR) != 0;
}


// --- GPIO CONTROL ------------------------------------------------------------

void VS1053b::pinMode(uint8_t pin, uint8_t mode) {
  if (pin > 7)
    return;
  uint16_t ddr = readWRAM16(GPIO_DDR);
  bitWrite(ddr, pin, mode);
  writeWRAM16(GPIO_DDR, ddr);
}

void VS1053b::digitalWrite(uint8_t pin, uint8_t val) {
  if (pin > 7)
    return;
  uint16_t odata = readWRAM16(GPIO_ODATA);
  bitWrite(odata, pin, val);
  writeWRAM16(GPIO_ODATA, odata);
}

uint8_t VS1053b::digitalRead(uint8_t pin) {
  if (pin > 7)
    return 0;
  uint16_t idata = readWRAM16(GPIO_IDATA);
  return bitRead(idata, pin);
}


// --- READING AND WRITING OF SCI_WRAM -----------------------------------------

// write 16-bit value to given address
void VS1053b::writeWRAM16(uint16_t addr, uint16_t data) {
  writeSci(SCI_WRAMADDR, addr);
  writeSci(SCI_WRAM, data);
}

// write 32-bit value to given address
void VS1053b::writeWRAM32(uint16_t addr, uint32_t data) {
  writeSci(SCI_WRAMADDR, addr);
  writeSci(SCI_WRAM, (data >> 16) & 0xFFFFUL);
  writeSci(SCI_WRAM, data & 0xFFFFUL);
}

// read 16-bit value from address
uint16_t VS1053b::readWRAM16(uint16_t addr) {
  writeSci(SCI_WRAMADDR, addr);
  return readSci(SCI_WRAM);
}

// read 32-bit non-changing value from address
uint32_t VS1053b::readWRAM32(uint16_t addr) {
  uint16_t lsb;
  writeSci(SCI_WRAMADDR, addr);
  lsb = readSci(SCI_WRAM);
  return lsb | ((uint32_t)readSci(SCI_WRAM) << 16);
}

// read 32-bit increasing counter value from address
uint32_t VS1053b::readWRAM32Counter(uint16_t addr) {
  uint16_t msbV1, lsb, msbV2;

  msbV1 = readWRAM16(addr + 1);
  lsb   = readWRAM16(addr);
  msbV2 = readSci(SCI_WRAM);
  if (lsb < 0x8000U) {
    msbV1 = msbV2;
  }
  return ((uint32_t)msbV1 << 16) | lsb;
}


// --- SCI & SDI OPERATIONS ----------------------------------------------------

void VS1053b::writeSci(uint8_t addr, uint16_t data) {
  beginTransaction(_SPIConfW, _pinCS);
  transfer32(SCI_WRITE, addr, data);
  endTransaction(_pinCS);
  while (getDREQ()) {;}
}

// prior to calling this function in a loop:
// - wait for DREQ
// - begin the SPI transaction
// - pull _pinCS low
// - transfer the address to write to
//
// afterwards:
// - bring _pinCS back up
// - end the SPI transaction
inline __attribute__((always_inline)) void VS1053b::writeSciMultiple(uint16_t data) {
  waitForDREQ();    // wait until DREQ is high
  transfer16(data); // write data
  while (getDREQ()) {;}
}

uint16_t VS1053b::readSci(uint8_t addr) {
  uint16_t data;
  beginTransaction(_SPIConfR, _pinCS);
  data = transfer32(SCI_READ, addr, 0x0) & 0xFFFFUL;
  endTransaction(_pinCS);
  while (getDREQ()) {;}
  return data;
}

void VS1053b::writeSdi(const uint8_t *data, size_t bytes) {
  if (bytes > 32 || bytes == 0)
    return;
  beginTransaction(_SPIConfW, _pinDCS);
  transfer(data, nullptr, bytes);
  endTransaction(_pinDCS);
}


// --- HANDLING OF DREQ --------------------------------------------------------

inline __attribute__((always_inline)) bool VS1053b::getDREQ(void) {
  return digitalReadFast(_pinDREQ);
}

inline __attribute__((always_inline)) void VS1053b::waitForDREQ(void) {
  while (!getDREQ()) { yield(); }
}


// --- SPI HELPERS -------------------------------------------------------------

inline __attribute__((always_inline)) void VS1053b::beginTransaction(SPISettings settings, uint8_t pin) {
  waitForDREQ();                  // wait until DREQ is high
  SPI.beginTransaction(settings); // begin transaction
  digitalWriteFast(pin, LOW);     // activate CS
}

inline __attribute__((always_inline)) void VS1053b::endTransaction(uint8_t pin) {
  digitalWriteFast(pin, HIGH); // deactivate CS
  SPI.endTransaction();        // end transaction
}

inline __attribute__((always_inline)) uint16_t VS1053b::transfer16(uint8_t byte1, uint8_t byte0) {
  return transfer16(((uint16_t)byte1 << 8) | byte0);
}

inline __attribute__((always_inline)) uint32_t VS1053b::transfer32(uint8_t byte3, uint8_t byte2, uint16_t word0) {
#if (defined(__IMXRT1052__) || defined(__IMXRT1062__))
  return transfer32(((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) | word0);
#else
  uint16_t out1 = transfer16(byte3, byte2);
  uint16_t out0 = transfer16(word0);
  return ((uint32_t)out1 << 16) + out0;
#endif
}


// --- SPI WRAPPERS ------------------------------------------------------------
// TODO: add bit-banging

inline __attribute__((always_inline)) void VS1053b::transfer(const void *buf, void *retbuf, size_t count) {
  return SPI.transfer(buf, retbuf, count);
}

inline __attribute__((always_inline)) uint16_t VS1053b::transfer16(uint16_t data) {
  return SPI.transfer16(data);
}

inline __attribute__((always_inline)) uint32_t VS1053b::transfer32(uint32_t data) {
#if (defined(__IMXRT1052__) || defined(__IMXRT1062__))
  return SPI.transfer32(data);
#else
  uint16_t word1 = transfer16((data >> 16) & 0xFFFFUL);
  uint16_t word0 = transfer16(data & 0xFFFFUL);
  return ((uint32_t)word1 << 16) + word0;
#endif
}
