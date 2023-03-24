// TI TSC2046 touchscreen library.
// Author: Qyriad <qyriad@qyriad.me>
// License: MIT

#ifndef TSC2046_H
#define TSC2046_H

#if ARDUINO >= 100
#include "Arduino.h"
#include "Print.h"
#else
#include "WProgram.h"
#endif

#include <Adafruit_BusIO_Register.h>
#include <Adafruit_SPIDevice.h>
#include <stdint.h>

class TSPoint {
public:
  TSPoint(int16_t x, int16_t y, int16_t z1, int16_t z2);

  bool operator==(TSPoint rhs);
  bool operator!=(TSPoint rhs);

  int16_t raw_x;
  int16_t raw_y;
  int16_t raw_z1;
  int16_t raw_z2;

  float x();
  float y();
  float z();
};

class Adafruit_TSC2046 {
public:
  Adafruit_TSC2046(int32_t sensorId);

  bool begin(int spiChipSelect, uint16_t rDiffX, SPIClass &spi = SPI,
             uint32_t spiFrequency = 2L * 1000L * 1000L);

  TSPoint getPoint();

private:
  SPIClass *_spi;
  int _spiCS;
  int32_t _sensorId;
  int64_t _spiFrequency;

  uint16_t readDfr(Adafruit_SPIDevice &spiDev, uint8_t channelSelect);
  uint16_t readSer(uint8_t channelSelect);
  static uint16_t parse12BitValue(uint8_t spiUpperByte, uint8_t spiLowerByte);
};

class CommandBits {
public:
  CommandBits(uint8_t value = 0);
  void addBit(bool value);
  void addBits(uint8_t value, uint8_t length);

  uint8_t command;
};

#endif
