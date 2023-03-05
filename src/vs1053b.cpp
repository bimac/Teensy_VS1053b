// This file is part of Teensy_VS1053b.
// Copyright (C) 2023 Florian Rau
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
constexpr uint32_t LOWER_WORD{0x0000FFFF};

typedef struct {
  union {
    struct {
      uint8_t byte3;
      uint8_t byte2;
      uint8_t byte1;
      uint8_t byte0;
    };
    struct {
      uint16_t word1;
      uint16_t word0;
    };
    uint32_t dword;
  };
} dword_t;

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
  return begin(_XTALI / 7);
}

uint8_t VS1053b::begin(const uint32_t maxClock) {

  _SPIConfR = SPISettings(min(_XTALI / 7, maxClock), MSBFIRST, SPI_MODE0);
  _SPIConfW = SPISettings(min(_XTALI / 7, maxClock), MSBFIRST, SPI_MODE0);

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

  // set clock, adjust speed of SPI bus and confirm with readback
  setClock(maxClock);
  if (readbackTest())
    return VS1053B___INIT_FAIL_CLK_RAISE;

  // we're all set
  return 0;
}

void VS1053b::setClock(const uint32_t maxClock) {
  uint16_t multiplierBits;

  if        (maxClock > _XTALI * 4.0 / 7) {
    multiplierBits = SC_MULT_53_45X;
    _CLKI = _XTALI * 4.5;
  } else if (maxClock > _XTALI * 3.5 / 7) {
    multiplierBits = SC_MULT_53_40X;
    _CLKI = _XTALI * 4.0;
  } else if (maxClock > _XTALI * 3.0 / 7) {
    multiplierBits = SC_MULT_53_35X;
    _CLKI = _XTALI * 3.5;
  } else if (maxClock > _XTALI * 2.5 / 7) {
    multiplierBits = SC_MULT_53_30X;
    _CLKI = _XTALI * 3.0;
  } else if (maxClock > _XTALI * 2.0 / 7) {
    multiplierBits = SC_MULT_53_25X;
    _CLKI = _XTALI * 2.5;
  } else if (maxClock > _XTALI * 1.0 / 7) {
    multiplierBits = SC_MULT_53_20X;
    _CLKI = _XTALI * 2.0;
  } else {
    multiplierBits = SC_MULT_53_10X;
    _CLKI = _XTALI;
  }

  writeSci16(SCI_CLOCKF, Hz2SC_FREQ(_XTALI) | multiplierBits);
  _SPIConfR = SPISettings(min(_CLKI / 7, maxClock), MSBFIRST, SPI_MODE0);
  _SPIConfW = SPISettings(min(_CLKI / 4, maxClock), MSBFIRST, SPI_MODE0);
  waitForDREQ();
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
  writeSci16(SCI_MODE, SM_SDINEW | SM_TESTS | SM_RESET);
  waitForDREQ();
}

void VS1053b::volume(uint8_t value) {
  volume(value, value);
}

void VS1053b::volume(uint8_t left, uint8_t right) {
  writeMem16(SCI_VOL, ((uint16_t)left << 8) | right);
}

int16_t VS1053b::streamBufferFillWords(void) {
  int16_t bufSize = (readSci(SCI_HDAT1) == 0x664C) ? 0x1800 : 0x400;
  uint16_t wrp = readMem16(0x5A7D);
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
  uint16_t wrp = readMem16(0x5A80);
  uint16_t rdp = readSci(SCI_WRAM);
  return (wrp - rdp) & 4095;
}

int16_t VS1053b::audioBufferFreeWords(void) {
  int16_t res = 4096 - audioBufferFillWords();
  if (res < 2)
    return 0;
  return res - 2;
}

uint16_t VS1053b::audioBufferUnderflow(void) {
  uint16_t uFlow = readMem16(0x5A82);
  if (uFlow)
    writeMem16(0x5A82, 0);
  return uFlow;
}

bool VS1053b::connectionTest(void) {
  uint16_t status = readSci(SCI_STATUS);
  return (status == 0 || status == 0xFFFF);
}

bool VS1053b::readbackTest(void) {
  writeSci16(SCI_AICTRL1, 0xABAD);
  writeSci16(SCI_AICTRL2, 0x7E57);
  if (readSci(SCI_AICTRL1) != 0xABAD || readSci(SCI_AICTRL2) != 0x7E57)
    return true;
  writeSci16(SCI_AICTRL1, 0);
  writeSci16(SCI_AICTRL2, 0);
  return false;
}

// bool VS1053b::loadPlugin(const uint16_t *d, uint16_t len) {
//   uint8_t addr, n, val;
//   uint16_t i = 0;
//   while (i < len) {
//     addr = d[i++];
//     n = d[i++];
//     if (n & 0x8000U) {
//       n &= 0x7FFF;
//       val = d[i++];
//       writeSciStart(addr);
//       while (n--) {
//         writeSciMid(val);
//       }
//       writeSciEnd();
//     } else {
//       writeSciStart(addr);
//       while (n--) {
//         val = d[i++];
//         writeSciMid(val);
//         i++;
//       }
//       writeSciEnd();
//     }
//   }
//   return !isPatched();
// }

bool VS1053b::loadPlugin(const uint16_t *d, uint16_t len) {
  uint8_t addr, n, val;
  uint16_t i = 0;
  while (i < len) {
    addr = d[i++];
    n = d[i++];
    if (n & 0x8000U) {
      n &= 0x7FFF;
      val = d[i++];
      writeSciStart(addr);
      while (n--) {
        writeSciMid(val);
      }
      writeSciEnd();
    } else {
      writeSciStart(addr);
      while (n--) {
        val = d[i++];
        writeSciMid(val);
        i++;
      }
      writeSciEnd();
    }
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
  uint16_t ddr = readMem16(GPIO_DDR);
  bitWrite(ddr, pin, mode);
  writeMem16(GPIO_DDR, ddr);
}

void VS1053b::digitalWrite(uint8_t pin, uint8_t val) {
  if (pin > 7)
    return;
  uint16_t odata = readMem16(GPIO_ODATA);
  bitWrite(odata, pin, val);
  writeMem16(GPIO_ODATA, odata);
}

uint8_t VS1053b::digitalRead(uint8_t pin) {
  if (pin > 7)
    return 0;
  uint16_t idata = readMem16(GPIO_IDATA);
  return bitRead(idata, pin);
}


// --- READING AND WRITING OF SCI_WRAM -----------------------------------------

// read 16-bit value from address
uint16_t VS1053b::readMem16(uint16_t addr) {
  writeSci16(SCI_WRAMADDR, addr);
  return readSci(SCI_WRAM);
}

// read 32-bit non-changing value from address
uint32_t VS1053b::readMem32(uint16_t addr) {
  uint16_t lsb;
  writeSci16(SCI_WRAMADDR, addr);
  lsb = readSci(SCI_WRAM);
  return lsb | ((uint32_t)readSci(SCI_WRAM) << 16);
}

// read 32-bit increasing counter value from address
uint32_t VS1053b::readMem32Counter(uint16_t addr) {
  uint16_t msbV1, lsb, msbV2;

  msbV1 = readMem16(addr + 1);
  lsb   = readMem16(addr);
  msbV2 = readSci(SCI_WRAM);
  if (lsb < 0x8000U) {
    msbV1 = msbV2;
  }
  return ((uint32_t)msbV1 << 16) | lsb;
}

// write 16-bit value to given address
void VS1053b::writeMem16(uint16_t addr, uint16_t data) {
  writeSci16(SCI_WRAMADDR, addr);
  writeSci16(SCI_WRAM, data);
}

// write 32-bit value to given address
void VS1053b::writeMem32(uint16_t addr, uint32_t data) {
  writeSci16(SCI_WRAMADDR, addr);
  writeSci32(SCI_WRAM, data);
}


// --- SCI & SDI OPERATIONS ----------------------------------------------------

void VS1053b::writeSci16(uint8_t addr, uint16_t data) {
  beginTransaction(_SPIConfW, _pinCS);
  transfer32(SCI_WRITE, addr, data);
  endTransaction(_pinCS);
  while (getDREQ()) {;}
}

void VS1053b::writeSci32(uint8_t addr, uint32_t data) {
  writeSciStart(addr);
  writeSciMid((uint16_t)data);
  writeSciMid((uint16_t)(data >> 16));
  writeSciEnd();
}

uint16_t VS1053b::readSci(uint8_t addr) {
  uint16_t data;
  beginTransaction(_SPIConfR, _pinCS);
  data = transfer32(SCI_READ, addr, 0x0000) & 0xFFFFUL;
  endTransaction(_pinCS);
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
  while (!getDREQ()) { asm volatile("nop"); }
}


// --- SCI MULTIPLE WRITES -----------------------------------------------------

inline __attribute__((always_inline)) void VS1053b::writeSciStart(uint8_t addr) {
  beginTransaction(_SPIConfW, _pinCS); // begin transaction
  transfer16(SCI_WRITE, addr);         // write opcode & SCI address
}

inline __attribute__((always_inline)) void VS1053b::writeSciMid(uint16_t data) {
  waitForDREQ();    // wait until DREQ is high
  transfer16(data); // write data
}

inline __attribute__((always_inline)) void VS1053b::writeSciEnd() {
  endTransaction(_pinCS);
}


// --- SPI WRAPPERS ------------------------------------------------------------

inline __attribute__((always_inline)) void VS1053b::beginTransaction(SPISettings settings, uint8_t pin) {
  waitForDREQ();                  // wait until DREQ is high
  SPI.beginTransaction(settings); // begin transaction
  digitalWriteFast(pin, LOW);     // activate CS
}

inline __attribute__((always_inline)) void VS1053b::endTransaction(uint8_t pin) {
  digitalWriteFast(pin, HIGH); // deactivate CS
  SPI.endTransaction();        // end transaction
}

inline __attribute__((always_inline)) uint16_t VS1053b::transfer16(uint16_t data) {
  return SPI.transfer16(data);
}

inline __attribute__((always_inline)) uint16_t VS1053b::transfer16(uint8_t msb, uint8_t lsb) {
  return transfer16(((uint16_t)msb << 8) | lsb);
}

// inline __attribute__((always_inline)) uint32_t VS1053b::transfer32(uint16_t word1, uint16_t word0) {
// #if (defined(__IMXRT1052__) || defined(__IMXRT1062__))
//   return transfer32(((uint32_t)word1 << 16) | word0);
// #else
//   uint16_t word1 = this.transfer16(word1);
//   uint16_t word0 = this.transfer16(word0);
//   return ((uint32_t)word1 << 16) | word0;
// #endif
// }

inline __attribute__((always_inline)) uint32_t VS1053b::transfer32(uint8_t byte3, uint8_t byte2, uint16_t word0) {
#if (defined(__IMXRT1052__) || defined(__IMXRT1062__))
  return transfer32(((uint32_t)byte3 << 24) | ((uint32_t)byte2 << 16) | word0);
#else
  uint16_t out1 = transfer16(byte3, byte2);
  uint16_t out0 = transfer16(word0);
  return ((uint32_t)out1 << 16) + out0;
#endif
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

inline __attribute__((always_inline)) void VS1053b::transfer(const void *buf, void *retbuf, size_t count) {
  SPI.transfer(buf, retbuf, count);
}
