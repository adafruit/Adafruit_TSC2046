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

// spi_dev = new Adafruit_SPIDevice(chipSelect, freq, bitorder, spimode, SPIClass);

TSPoint Adafruit_TSC2046::getPoint() {

  //CommandBits readXCmd;
  //readXCmd.addBit(1); // Start bit.
  //readXCmd.addBits(REG_DFR_X_POS, 3); // A2:A0: read X position in DFR mode.
  //readXCmd.addBit(1); // MODE: use 12-bit conversion mode.
  //readXCmd.addBit(0); // SER/DFR': use differential reference mode.
  //readXCmd.addBit(0); // PD1: Internal reference voltage off.
  //readXCmd.addBit(1); // PD0: ADC on.

  //auto spiDev = new Adafruit_SPIDevice(_spiCS, _spiFrequency, SPI_BITORDER_MSBFIRST, SPI_MODE0, _spi);

  //Adafruit_BusIO_Register readXReg = Adafruit_BusIO_Register(
    //spiDev,
    //[> regAddr <] readXCmd.command,
    //[> regType <] ADDRBIT8_HIGH_TOREAD,
    //[> width <] 2, // For 12-bit reads.
    //[> byteorder <] LSBFIRST,
    //[> addressWidth <] 1
  //);

  //Adafruit_BusIO_RegisterBits resultRange = Adafruit_BusIO_RegisterBits(
    //[> reg <] &readXReg,
    //[> bits <] 12,
    //[> shift <] 0
  //);

  //uint16_t xResult = resultRange.read();

  //delete spiDev;

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

  SPISettings settings = SPISettings(_spiFrequency, MSBFIRST, SPI_MODE0);

  _spi->beginTransaction(settings);
  pinMode(_spiCS, OUTPUT);
  digitalWrite(_spiCS, LOW);

  // SPI here.
  _spi->transfer(controlCmd.command);
  uint16_t upper_8 = (_spi->transfer(0) & 0x7F);
  uint16_t lower_4 = (_spi->transfer(0) & 0xF8);
  uint16_t res = ((upper_8 << 5) | (lower_4 >> 3));

  // First 8 are 12:4.
  // Next byte's 7:4 contain 3:0.

  digitalWrite(_spiCS, HIGH);
  _spi->endTransaction();


  /*
     | 15 | 14 | 13 | 12 | 11 | 10 |  9 |  8 |  7 |  6 |  5 |  4 |  3 |  2 |  1 |  0 |

                             Upper Byte                                         Lower Byte
           -------------------------------------------------  -------------------------------------------------
    Bytes: |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |  |  7  |  6  |  5  |  4  |  3  |  2  |  1  |  0  |

    Map:   |  X  |  11 |  10 |  9  |  8  |  7  |  6  |  5  |  |  4  |  3  |  2  |  1  |  0  |  X  |  X  |  X  |


    Xs are values we should mask out.
   */


  return res;

  CommandBits readCmd;
  readCmd.addBit(1); // START bit, always 1.
  readCmd.addBits(channelSelect, 3); // A2:A0.
  //readCmd.addBit(0); // MODE: use 12-bit conversion mode.
  readCmd.addBit(1); // 8/12': conversion mode.
  readCmd.addBit(0); // SER/DFR': use differential reference mode.
  //readCmd.addBit(0); // PD1: internal reference voltage off.
  readCmd.addBit(0); // PD1: internal reference voltage on/off'.
  //readCmd.addBit(1); // PD0: ADC on.
  readCmd.addBit(1); // PD0: ADC on/off'

  //Serial.print("CMD: ");
  //Serial.println(readCmd.command, HEX);

  Adafruit_BusIO_Register cmdReg = Adafruit_BusIO_Register(
    &spiDev,
    readCmd.command, // red_addr
    ADDRBIT8_HIGH_TOREAD, // reg_type
    //2, // Width: 2, for 12-bit reads.
    1, // Width: XXX
    // It's a 12-bit value with the most-significant BIT first.
    // Therefore, the first 1-byte read will read the 8 most-significant bits,
    // and the next 1-byte read will read the 4 least-significant bits.
    MSBFIRST,
    1 // address_width
  );

  Adafruit_BusIO_RegisterBits resultRange = Adafruit_BusIO_RegisterBits(
    &cmdReg, // reg
    8, // bits // XXX
    0 // shift
  );

  uint16_t result = resultRange.read();

  //uint16_t result = 0;
  //uint8_t buffer[2];
  //cmdReg.read(buffer, 2);
  //result |= (buffer[0] << 16);
  //result |= buffer[1];
  //Serial.print("Buffer: ");
  //Serial.print(buffer[0], HEX);
  //Serial.print("\t");
  //Serial.print(buffer[1], HEX);
  //Serial.print("\n");

  if (result != (result & ~(0xFF << 12))) {
    Serial.print("Mismatch:\t");
    Serial.print(result, HEX);
    Serial.print("\t");
    Serial.print((result & ~(0xFF << 12)), HEX);
    Serial.print("\n");
  }

  return result;
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
