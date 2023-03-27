#include <math.h>

#include "TSC2046.h"

// Multiplexer addresses for the various outputs in single-ended reference mode.
#define ADDR_SER_TEMP0 (0b000)
#define ADDR_SER_Y_POS (0b001)
#define ADDR_SER_VBAT (0b010)
#define ADDR_SER_Z1_POS (0b011)
#define ADDR_SER_Z2_POS (0b100)
#define ADDR_SER_X_POS (0b101)
#define ADDR_SER_AUX (0b110)
#define ADDR_SER_TEMP1 (0b111)

// Multiplexer addresses for the various outputs in differential reference mode.
#define ADDR_DFR_Y_POS (0b001)
#define ADDR_DFR_Z1_POS (0b011)
#define ADDR_DFR_Z2_POS (0b100)
#define ADDR_DFR_X_POS (0b101)

TSPoint::TSPoint(int16_t x, int16_t y, float z) {
  this->x = x;
  this->y = y;
  this->z = z;
}

float TSPoint::xPercent() { return x / 4096.f; }

float TSPoint::yPercent() { return y / 4096.f; }

void Adafruit_TSC2046::begin(int spiChipSelect, uint32_t xResistance,
                             SPIClass &spi, uint32_t spiFrequency) {
  _spiCS = spiChipSelect;
  _spi = &spi;
  _spiFrequency = spiFrequency;
  _xResistance = xResistance;

  return true;
}

TSPoint Adafruit_TSC2046::getPoint() {

  // Regarding SPI mode, timing diagrams on the datasheet show DCLK idling LOW,
  // which means the leading edge is a rising edge, which means CPOL = 0.
  // For DOUT (MISO/CIPO) the datasheet says "data are shifted on the falling
  // edge of DCLK", and for DIN it says "data are latched on the rising edge
  // of DCLK", which means OUT side changes on the trailing edge of the clock
  // and the IN side changes on/after the leading edge of the clock, which
  // means CPHA = 0.
  // Therefore, our SPI mode is 0.
  Adafruit_SPIDevice spiDev =
      Adafruit_SPIDevice(_spiCS,                // cspin
                         _spiFrequency,         // freq
                         SPI_BITORDER_MSBFIRST, // dataOrder
                         SPI_MODE0,             // dataMode
                         _spi                   // theSPI
      );

  spiDev.begin();

  int16_t xResult = readDfr(spiDev, ADDR_DFR_X_POS);
  int16_t yResult = readDfr(spiDev, ADDR_DFR_Y_POS);
  int16_t z1Result = readDfr(spiDev, ADDR_DFR_Z1_POS);
  int16_t z2Result = readDfr(spiDev, ADDR_DFR_Z2_POS);

  // The datasheet gives two ways to calculate pressure. We're going to use the
  // one that requires the least information from the user:
  //
  // R_TOUCH = R_X_PLATE * (X_POSITON / 4096) * (Z_2 / Z_1 - 1)
  // So this requires knowing the X-Plate resistance, which thankfully we got
  // from the user back at Adafruit_TSC2046::begin().

  float rTouch = _xResistance * (xResult / 4096.f) *
                 (((float)z2Result / (float)z1Result) - 1.f);

  return TSPoint(xResult, yResult, rTouch);
}

bool Adafruit_TSC2046::isTouched() {
  TSPoint point = getPoint();

  // If the resistance is not infinity, NaN, some other non-finite number,
  // or 0, then the touchscreen is probably being touched.
  return isfinite(point.z) && point.z != 0;
}

void Adafruit_TSC2046::enableInterrupts(bool enable) {

  // NOTE: On my hardware, PENIRQ' goes low when the touchscreen is touched
  // *regardless* of the PD0 value that enables and disables interrupts.

  _interruptsEnabled = enable;

  // Perform any read so we can get the control byte over there with the new
  // PD0 value which enables or disables the PENIRQ' output.
  Adafruit_SPIDevice spiDev = Adafruit_SPIDevice(
      _spiCS, _spiFrequency, SPI_BITORDER_MSBFIRST, SPI_MODE0, _spi);
  spiDev.begin();
  readDfr(spiDev, 0);
}

uint16_t Adafruit_TSC2046::readDfr(Adafruit_SPIDevice &spiDev,
                                   uint8_t channelSelect) {

  CommandBits controlCmd;

  // START bit, always 1.
  controlCmd.addBit(1);

  // A2:A0: the channel select/"address" bits, which control the multiplexer
  // output. This will be one of the `ADDR_` values near the top of the file.
  controlCmd.addBits(channelSelect, 3);

  // ADC conversion mode: LOW for 12-bit mode, and HIGH for 8-bit mode.
  controlCmd.addBit(0);

  // SER/DFR': what to use for VREF. HIGH for single-ended reference mode,
  // which uses the internal 2.5 VREF in the TSC2046. LOW for differential
  // reference mode. We're using differential reference mode, which requires
  // connecting the Arduino Vcc to both Vcc and VREF on the TSC2046.
  controlCmd.addBit(0);

  // PD1: Enable/disable' internal VREF. We're using differential reference
  // mode, so turn off the internal VREF.
  controlCmd.addBit(0);

  // PD0: This bit is ADC on/off', technically, but when both PD1 and PD0 are
  // 0, then it leaves the ADC off *between* conversions, but powers it on
  // *during* conversions. According to the datasheet the ADC is able to power
  // up instantly and there are no delays incured by leaving the ADC powered
  // off between conversions. Leaving the ADC on is intended for certain
  // strategies that use external capacitors to filter out touchscreen noise.
  // This doesn't apply to us, but there is one more consideration, which is
  // that the PENIRQ' output used to trigger interrupts is disabled if
  // this bit is HIGH (1). Since that's the only functionality of this bit
  // we care about, we'll make it correspond directly to the user's
  // IRQ setting.
  controlCmd.addBit(!_interruptsEnabled);

  Adafruit_BusIO_Register controlReg = Adafruit_BusIO_Register(
      &spiDev,
      controlCmd.command,   // reg_addr
      ADDRBIT8_HIGH_TOREAD, // reg_type
      2,                    // Width: 2, to get the 12-bits we need.
      // It's a 12-bit value with the most-significant BIT first.
      // Therefore, the first 1-byte read will read the 8 most-significant bits,
      // and the next 1-byte read will read the 4 least-significant bits.
      MSBFIRST,
      1 // address_with
  );

  uint8_t spiOutputBuffer[2];
  controlReg.read(spiOutputBuffer, 2);

  // Will contain bits 11:5. See parse12BitValue for why.
  uint8_t upperByte = spiOutputBuffer[0];
  // Will contain bits 4:0. See parse12BitValue for why.
  uint8_t lowerByte = spiOutputBuffer[1];

  uint16_t result = parse12BitValue(upperByte, lowerByte);

  return result;
}

static uint16_t Adafruit_TSC2046::parse12BitValue(uint8_t spiUpperByte,
                                                  uint8_t spiLowerByte) {

  // `spiUpperByte` will contain bits 11:5.
  // `spiLowerByte` will contain bits 4:0.
  // See below for why.

  // clang-format off
  /*
                                 Upper Byte                                         Lower Byte
                 -------------------------------------------------  -------------------------------------------------
    SPI Read:    |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    12-bit Map:  |  X  |  11 |  10 |  9  |  8  |  7  |  6  |  5  |  |  4  |  3  |  2  |  1  |  0  |  X  |  X  |  X  |

    Want:        |  X  |  X  |  X  |  12 |  11 |  10 |  9  |  8  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    Xs are "don't care" values that we don't want in our result, so we should mask them out.

--
    "12-bit Map" are the where each bit of the 12-bit number we actually want are placed *in* the data we read
    over SPI. So the bit 11 (0-indexed) of the 12-bit number is found in bit 6 of `spiUpperByte`.
    Why is bit 7 of the Upper Byte an X? Because there is *one* extra clock cycle after we finish sending the
    control byte before the TSC2046 starts sending out the converted data. BusIO captures *immediately* after
    the SPI command is finished, though, so the very first bit (the most-significant bit, in BusIO's "eyes")
    is not the most-significant bit of the 12-bit value, but instead an "empty" value that
    1) we need to mask out, and 2) pads the start of the 12-bit value by one bit index.

    "Want" is what we need to bit-shift and mask to get, with the numbers being the indexes of the overall 12-bit
    value.
   */
  // clang-format on

  // Mask out bit 7 of `spiUpperByte`, which is an "X".
  spiUpperByte &= 0x7F; // 0b0111_1111;

  // Mask out the least significant 3 bits (bits 2, 1, and 0) of `spiLowerByte`,
  // which are also "X"s.
  spiLowerByte &= 0xF8; // 0b1111_1000;

  // Bit 6 of `spiUpperByte` needs to become bit 11 of the uint16_t,
  // so we shift `spiUpperByte` LEFT by 5 (11 - 6 = 5).
  // Meanwhile, bit 3 of `spiLowerByte` needs to become bit 0 of the uint16_t,
  // so we shift that RIGHT by 3 (3 - 0 = 3).
  uint16_t finalResult = (spiUpperByte << 5) | (spiLowerByte >> 3);

  // Finally, we have the 12-bit number we read as a uint16_t.
  return finalResult;
}

CommandBits::CommandBits(uint8_t value) { command = value; }

void CommandBits::addBit(bool value) {
  uint8_t bit = value & 1;
  command = (command << 1) | bit;
}

void CommandBits::addBits(uint8_t value, uint8_t length) {
  uint8_t mask = (1 << length) - 1;
  uint8_t bits = value & mask;

  command = (command << length) | bits;
}
