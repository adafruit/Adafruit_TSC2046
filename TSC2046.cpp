#include "TSC2046.h"

#define REG_SER_TEMP0   (0b000)
#define REG_SER_Y_POS   (0b001)
#define REG_SER_VBAT    (0b010)
#define REG_SER_Z1_POS  (0b011)
#define REG_SER_Z2_POS  (0b100)
#define REG_SER_X_POS   (0b101)
#define REG_SER_AUX     (0b110)
#define REG_SER_TEMP1   (0b111)

#define REG_DFR_Y_POS   (0b001)
#define REG_DFR_Z1_POS  (0b011)
#define REG_DFR_Z2_POS  (0b100)
#define REG_DFR_X_POS   (0b101)

#define REG_POS_Z1_REF_OFF_ADC_ON_DIFF (0b10111001)

/*! @brief The index of the bit of the command word that indicates the start of a control byte.
 * Must always be 1.
 */
#define CMD_START_INDEX   (7)
#define CMD_START_LEN     (1)

#define CMD_ADDR_INDEX    (6)
#define CMD_ADDR_LEN      (3)

#define CMD_MODE_INDEX    (3)
#define CMD_MODE_LEN      (1)

#define CMD_SER_DFR_INDEX (2)
#define CMD_SER_DFR_LEN   (1)

#define CMD_POWER_INDEX   (1)
#define CMD_POWER_LEN     (2)

// LSB_SIZE = V_REF / (2 ** ADC_BITS)
// In 12-bit conversion mode: LSB_SIZE = V_REF / 4096
// In  8-bit conversion mode: LSB_SIZE = V_REF / 256


TSPoint::TSPoint(int16_t x, int16_t y, int16_t z1, int16_t z2) {
  raw_x = x;
  raw_y = y;
  raw_z1 = z1;
  raw_z2 = z2;
}

float TSPoint::x() {
  return (raw_x * 3.3) / 4096;
}

float TSPoint::y() {
  return (raw_y * 3.3) / 4096;
}

float TSPoint::z() {
  return raw_z1 / 4096;
}


Adafruit_TSC2046::Adafruit_TSC2046(int32_t sensorId) {
  _sensorId = sensorId;
}

bool Adafruit_TSC2046::begin(int spiChipSelect, uint16_t rDiffX, SPIClass &spi, uint32_t spiFrequency) {
  _spiCS = spiChipSelect;
  _spi = &spi;
  _spiFrequency = spiFrequency;

  return true;
}

TSPoint Adafruit_TSC2046::getPoint() {

  Adafruit_SPIDevice spiDev = Adafruit_SPIDevice(
    _spiCS, // cspin
    _spiFrequency, // freq
    SPI_BITORDER_MSBFIRST, // dataOrder
    SPI_MODE0, // dataMode FIXME: document why
    _spi // theSPI
  );

  spiDev.begin();

  int16_t xResult = readDfr(spiDev, REG_DFR_X_POS);
  delay(100);
  int16_t yResult = readDfr(spiDev, REG_DFR_Y_POS);
  delay(100);
  int16_t z1Result = readDfr(spiDev, REG_DFR_Z1_POS);
  delay(100);
  int16_t z2Result = readDfr(spiDev, REG_DFR_Z2_POS);

  // FIXME: Calculate pressure.
  return TSPoint(xResult, yResult, z1Result, z2Result);
}

uint16_t Adafruit_TSC2046::readDfr(Adafruit_SPIDevice &spiDev, uint8_t channelSelect) {

  CommandBits controlCmd;
  controlCmd.addBit(1); // START bit, always 1.
  controlCmd.addBits(channelSelect, 3); // A2:A0.
  controlCmd.addBit(0); // 8/12': conversion mode.
  controlCmd.addBit(0); // SER/DFR': what to use for VREF.
  controlCmd.addBit(0); // PD1: Enable/disable' internal VREF.
  controlCmd.addBit(1); // PD0: ADC on/off'.


  Adafruit_BusIO_Register controlReg = Adafruit_BusIO_Register (
    &spiDev,
    controlCmd.command, // reg_addr
    ADDRBIT8_HIGH_TOREAD, // reg_type
    2, // Width: 2, to get the 12-bits we need.
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

static uint16_t Adafruit_TSC2046::parse12BitValue(uint8_t spiUpperByte, uint8_t spiLowerByte) {

  // `spiUpperByte` will contain bits 11:5.
  // `spiLowerByte` will contain bits 4:0.
  // See below for why.

  /*
                                 Upper Byte                                         Lower Byte
                 -------------------------------------------------  -------------------------------------------------
    SPI Read:    |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    12-bit Map:  |  X  |  11 |  10 |  9  |  8  |  7  |  6  |  5  |  |  4  |  3  |  2  |  1  |  0  |  X  |  X  |  X  |

    Want:        |  X  |  X  |  X  |  12 |  11 |  10 |  9  |  8  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    Xs are "don't care" values that we don't want in our result, so we should mask them out.

    "SPI Read" are the raw `spiUpperByte` and `spiLowerByte` we get over SPI.

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

  // Mask out bit 7 of `spiUpperByte`, which is an "X".
  spiUpperByte &= 0x7F; // 0b0111_1111;

  // Mask out the least significant 3 bits (bits 2, 1, and 0) of `spiLowerByte`, which are also "X"s.
  spiLowerByte &= 0xF8; // 0b1111_1000;

  // Bit 6 of `spiUpperByte` needs to become bit 11 of the uint16_t,
  // so we shift `spiUpperByte` LEFT by 5 (11 - 6 = 5).
  // Meanwhile, bit 3 of `spiLowerByte` needs to become bit 0 of the uint16_t,
  // so we shift that RIGHT by 3 (3 - 0 = 3).
  uint16_t finalResult = (spiUpperByte << 5) | (spiLowerByte >> 3);

  // Finally, we have the 12-bit number we read as a uint16_t.
  return finalResult;
}

CommandBits::CommandBits(uint8_t value) {
  command = value;
}

void CommandBits::addBit(bool value) {
  uint8_t bit = value & 1;
  command = (command << 1) | bit;
}

void CommandBits::addBits(uint8_t value, uint8_t length) {
  uint8_t mask = (1 << length) - 1;
  uint8_t bits = value & mask;

  command = (command << length) | bits;
}
